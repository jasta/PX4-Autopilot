/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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


#pragma once


#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <battery/battery.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/SubscriptionInterval.hpp>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace time_literals;

#define PIJUICE_SAMPLE_FREQUENCY_HZ           1
#define PIJUICE_SAMPLE_INTERVAL_US            (1_s / PIJUICE_SAMPLE_FREQUENCY_HZ)

enum class Command : uint8_t {
    Status = 0x40,
    ChargeLevel = 0x41,
    BatteryTemperature = 0x47,
    BatteryVoltage = 0x49,
    BatteryCurrent = 0x4b,
};

enum BatteryStatus {
    BATTERY_NORMAL = 0,
    CHARGING_FROM_IN,
    CHARGING_FROM_5V_IO,
    BATTERY_NOT_PRESENT,
};

enum ChargeStatus {
    POWER_NOT_PRESENT = 0,
    POWER_BAD,
    POWER_WEAK,
    POWER_PRESENT,
};

typedef struct {
    bool is_fault;
    bool is_button;
    BatteryStatus battery_status;
    ChargeStatus charge_status;
    ChargeStatus charge_status_5v;
} pijuice_status_t;

class PiJuice : public device::I2C, ModuleParams, public I2CSPIDriver<PiJuice>
{
public:
	PiJuice(const I2CSPIDriverConfig &config, int battery_index);
	virtual ~PiJuice();

	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);
	static void print_usage();

	void RunImpl();

	int init() override;

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void print_status() override;

private:
	uint8_t _battery_index;

	perf_counter_t _sample_perf;
	perf_counter_t _comms_errors;

    /** @param _crit_thr Critical battery threshold param. */
    float _crit_thr{0.f};

    /** @param _emergency_thr Emergency battery threshold param. */
    float _emergency_thr{0.f};

    /** @param _low_thr Low battery threshold param. */
    float _low_thr{0.f};

    uORB::PublicationMulti<battery_status_s> _battery_status_pub{ORB_ID(battery_status)};

    int read_battery_status(pijuice_status_t *result);
    int read_battery_charge_level(uint8_t *result);
    int read_battery_voltage(uint16_t *result);
    int read_battery_current(int16_t *result);
    int read_battery_temperature(int8_t *result);

    int read_data(Command command, uint8_t *output, const unsigned output_len);
};
