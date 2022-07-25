/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * Driver for the I2C attached PiJuice
 */

#include "pijuice.h"


PiJuice::PiJuice(const I2CSPIDriverConfig &config, int battery_index) :
	I2C(config),
    ModuleParams(nullptr),
	I2CSPIDriver(config),
    _battery_index((uint8_t)battery_index),
	_sample_perf(perf_alloc(PC_ELAPSED, "pijuice_read")),
	_comms_errors(perf_alloc(PC_COUNT, "pijuice_com_err"))
{
    _battery_status_pub.advertise();
}

PiJuice::~PiJuice()
{
	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int PiJuice::init()
{
	int ret = PX4_ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != PX4_OK) {
		return ret;
	}

    // Read battery threshold params on startup.
    param_get(param_find("BAT_CRIT_THR"), &_crit_thr);
    param_get(param_find("BAT_LOW_THR"), &_low_thr);
    param_get(param_find("BAT_EMERGEN_THR"), &_emergency_thr);

    ScheduleOnInterval(PIJUICE_SAMPLE_INTERVAL_US);

	return PX4_OK;
}

void PiJuice::RunImpl() {

    if (should_exit()) {
        PX4_INFO("stopping");
        return;
    }

    perf_begin(_sample_perf);

    battery_status_s report = {};
    int ret = PX4_OK;
    pijuice_status_t status;
    uint16_t raw_voltage{0};
    int16_t raw_current{0};
    int8_t raw_temp{0};
    uint8_t charge_level{0};

    report.id = _battery_index;
    report.source = battery_status_s::BATTERY_SOURCE_EXTERNAL;
    report.timestamp = hrt_absolute_time();
    report.connected = true;

    report.scale = -1;
    report.cell_count = 1;

    ret |= read_battery_voltage(&raw_voltage);
    float voltage = raw_voltage / 1000.f;
    report.voltage_cell_v[0] = voltage;
    report.voltage_v = voltage;
    report.voltage_filtered_v = voltage;

    ret |= read_battery_current(&raw_current);
    float current = raw_current / 1000.f;
    report.current_a = current;
    report.current_filtered_a = current;
    report.current_average_a = -1;
    report.discharged_mah = -1;

    ret |= read_battery_charge_level(&charge_level);
    float remaining = charge_level / 100.f;
    report.remaining = remaining;

    report.time_remaining_s = NAN;

    ret |= read_battery_temperature(&raw_temp);
    report.temperature = raw_temp;

    ret |= read_battery_status(&status);
    if (status.battery_status == BATTERY_NOT_PRESENT) {
        report.warning = battery_status_s::BATTERY_STATE_UNHEALTHY;
    } else {
        ChargeStatus charge_source = status.charge_status;
        if (status.charge_status_5v != POWER_NOT_PRESENT) {
            charge_source = status.charge_status_5v;
        }
        switch (charge_source) {
            case POWER_BAD:
            case POWER_WEAK:
            case POWER_NOT_PRESENT:
                if (remaining > _low_thr) {
                    report.warning = battery_status_s::BATTERY_WARNING_NONE;
                } else if (remaining > _crit_thr) {
                    report.warning = battery_status_s::BATTERY_WARNING_LOW;
                } else if (remaining > _emergency_thr) {
                    report.warning = battery_status_s::BATTERY_WARNING_CRITICAL;
                } else {
                    report.warning = battery_status_s::BATTERY_WARNING_EMERGENCY;
                }
                break;
            case POWER_PRESENT:
                report.warning = battery_status_s::BATTERY_STATE_CHARGING;
                break;
        }
    }

    if (ret != PX4_OK) {
        PX4_INFO("error reading from PiJuice!");
        perf_count(_comms_errors);
    } else {
        _battery_status_pub.publish(report);
    }

    perf_end(_sample_perf);
}

int PiJuice::read_battery_status(pijuice_status_t *result) {
    uint8_t output[1] = {};
    int ret;
    if ((ret = read_data(Command::Status, output, 1)) != PX4_OK) {
        return ret;
    }
    uint8_t d = output[0];
    result->is_fault = (d & 0x01) == 0x01;
    result->is_button = (d & 0x02) == 0x02;
    result->battery_status = static_cast<BatteryStatus>((d >> 2) & 0x03);
    result->charge_status = static_cast<ChargeStatus>((d >> 4) & 0x03);
    result->charge_status_5v = static_cast<ChargeStatus>((d >> 6) & 0x03);
    return PX4_OK;
}

int PiJuice::read_battery_charge_level(uint8_t *result) {
    uint8_t output[1] = {};
    int ret;
    if ((ret = read_data(Command::ChargeLevel, output, 1)) != PX4_OK) {
        return ret;
    }
    *result = output[0];
    return PX4_OK;
}

int PiJuice::read_battery_voltage(uint16_t *result) {
    uint8_t output[2] = {};
    int ret;
    if ((ret = read_data(Command::BatteryVoltage, output, 2)) != PX4_OK) {
        return ret;
    }
    *result = (output[1] << 8) | output[0];
    return PX4_OK;
}

int PiJuice::read_battery_current(int16_t *result) {
    uint8_t output[2] = {};
    int ret;
    if ((ret = read_data(Command::BatteryCurrent, output, 2)) != PX4_OK) {
        return ret;
    }
    int32_t current = (output[1] << 8) | output[0];
    if (current & (1 << 15)) {
        current -= (1 << 16);
    }
    *result = current;
    return PX4_OK;
}

int PiJuice::read_battery_temperature(int8_t *result) {
    uint8_t output[2] = {};
    int ret;
    if ((ret = read_data(Command::BatteryTemperature, output, 2)) != PX4_OK) {
        return ret;
    }
    int32_t temp = output[0];
    if (temp & (1 << 7)) {
        temp -= (1 << 8);
    }
    *result = temp;
    return PX4_OK;
}

int PiJuice::read_data(Command command, uint8_t *output, const unsigned output_len) {
    uint8_t write_buf[1] = { (uint8_t)command };

    int ret;

    if ((ret = transfer(write_buf, 1, output, output_len)) != PX4_OK) {
        return ret;
    }

    // TODO: pijuice.py does checksumming, should we?

    return PX4_OK;
}

void PiJuice::print_status()
{
	I2CSPIDriverBase::print_status();

    perf_print_counter(_sample_perf);
    perf_print_counter(_comms_errors);

    printf("poll interval:  %u \n", PIJUICE_SAMPLE_INTERVAL_US);
}
