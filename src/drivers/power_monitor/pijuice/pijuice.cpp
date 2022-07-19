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
	_sample_perf(perf_alloc(PC_ELAPSED, "pijuice_read")),
	_comms_errors(perf_alloc(PC_COUNT, "pijuice_com_err")),
	_collection_errors(perf_alloc(PC_COUNT, "pijuice_collection_err")),
    // Err this should really be EXTERNAL, but it seems px4 isn't really set-up to support that right now
	_battery(battery_index, this, PIJUICE_SAMPLE_INTERVAL_US, battery_status_s::BATTERY_SOURCE_POWER_MODULE)
{
    _battery.setConnected(false);
    _battery.updateVoltage(0.f);
    _battery.updateCurrent(0.f);
    _battery.updateAndPublishBatteryStatus(hrt_absolute_time());
}

PiJuice::~PiJuice()
{
	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_collection_errors);
}

int PiJuice::init()
{
	int ret = PX4_ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != PX4_OK) {
		return ret;
	}

    ScheduleOnInterval(PIJUICE_SAMPLE_INTERVAL_US);

	return PX4_OK;
}

void PiJuice::RunImpl() {

    if (should_exit()) {
        PX4_INFO("stopping");
        return;
    }

    perf_begin(_sample_perf);

    bool success{true};
    int16_t voltage{0};
    int16_t current{0};
    success = success && (read_battery_voltage(&voltage) == PX4_OK);
    success = success && (read_battery_current(&current) == PX4_OK);

    if (!success) {
        PX4_INFO("error reading from PiJuice!");
        voltage = current = 0;
        perf_count(_comms_errors);
    } else {
        _battery.setConnected(success);
        _battery.updateVoltage(static_cast<float>(voltage));
        _battery.updateCurrent(static_cast<float>(current));
        _battery.updateAndPublishBatteryStatus(hrt_absolute_time());
    }

    perf_end(_sample_perf);
}

int PiJuice::read_battery_charge_level(int16_t *result) {
    uint8_t output[1] = {};
    int ret;
    if ((ret = read_data(Command::ChargeLevel, output, 1)) != PX4_OK) {
        return ret;
    }
    *result = output[0];
    return PX4_OK;
}

int PiJuice::read_battery_voltage(int32_t *result) {
    uint8_t output[2] = {};
    int ret;
    if ((ret = read_data(Command::BatteryVoltage, output, 2)) != PX4_OK) {
        return ret;
    }
    *result = (output[1] << 8) | output[0];
    return PX4_OK;
}

int PiJuice::read_battery_current(int32_t *result) {
    uint8_t output[2] = {};
    int ret;
    if ((ret = read_data(Command::BatteryCurrent, output, 2)) != PX4_OK) {
        return ret;
    }
    int32_t current = (output[1] << 8) | output[0];
    if (current & (1 << 15)) {
        i -= (1 << 16);
    }
    *result = current;
    return PX4_OK;
}

int PiJuice::read_battery_temperature(int32_t *result) {
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
