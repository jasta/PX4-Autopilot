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
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace time_literals;

/* Configuration Constants */
#define PIJUICE_BASEADDR 	            0x12

#define PIJUICE_SAMPLE_FREQUENCY_HZ           1
#define PIJUICE_SAMPLE_INTERVAL_US            (1_s / PIJUICE_SAMPLE_FREQUENCY_HZ)

enum class Command : uint8_t {
    ChargeLevel = 0x41,
    BatteryTemperature = 0x47,
    BatteryVoltage = 0x49,
    BatteryCurrent = 0x4b,
};

class PiJuice : public device::I2C, public I2CSPIDriver<PiJuice>
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
	unsigned int _measure_interval{0};

	perf_counter_t _sample_perf;
	perf_counter_t _comms_errors;

	Battery _battery;

    int read_battery_charge_level(int16_t *result);
    int read_battery_voltage(int32_t *result);
    int read_battery_current(int32_t *result);
    int read_battery_temperature(int32_t *result);

    int read_data(Command command, uint8_t *output, const unsigned output_len);
};
