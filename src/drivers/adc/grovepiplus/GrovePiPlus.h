/****************************************************************************
 *
 *   Copyright (C) 2022 PX4 Development Team. All rights reserved.
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

#include <stdint.h>
#include <drivers/device/i2c.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/parameters/param.h>
#include <uORB/topics/adc_report.h>
#include <uORB/Publication.hpp>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>

enum class Register : uint8_t {
    ANALOG_READ = 0x03,
    ANALOG_WRITE = 0x04,
    PIN_MODE = 0x05,
};

enum class PinMode : uint8_t {
    INPUT = 0x00,
    OUTPUT = 0x01,
};

#define DATA_NOT_AVAILABLE 0x17 // 23

// Some other condition observed in grovepi.py???
#define DATA_NOT_AVAILABLE2 0xff

#define MAX_I2C_RETRIES_PER_CYCLE 2
#define MAX_DATA_AVAILABLE_TRIES_PER_CYCLE 3

// 10-bit resolution
#define ADC_RESOLUTION 1023

// I think this is hardcoded by the GrovePi+ FW, but not really sure.  Value comes from Seeed studio
// wiki.
#define REFERENCE_VOLTAGE 4.980f

using namespace time_literals;

/*
 * This driver enables the GrovePi+ Raspberry Pi HAT to publish
 * ADC reports (e.g. for battery voltage readings).
 */
class GrovePiPlus : public device::I2C, public I2CSPIDriver<GrovePiPlus>
{
public:
	GrovePiPlus(const I2CSPIDriverConfig &config);
	~GrovePiPlus() override;

	int init() override;

	static void print_usage();

	void RunImpl();

protected:

	void print_status() override;

	void exit_and_cleanup() override;

private:

	uORB::Publication<adc_report_s>		_to_adc_report{ORB_ID(adc_report)};

	static const hrt_abstime	SAMPLE_INTERVAL{20_ms};

	adc_report_s _adc_report{};

	perf_counter_t			_cycle_perf;

    uint8_t _voltage_port_selection = -1;
    uint8_t _current_port_selection = -1;

    int readAndPopulateAdcReport(uint8_t port, adc_report_s *value_out);
    int pinMode(uint8_t port, PinMode mode);
    int analogRead(uint8_t port, uint16_t *value_out);
};
