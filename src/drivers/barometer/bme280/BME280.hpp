/****************************************************************************
 *
 *   Copyright (c) 2016-2019 PX4 Development Team. All rights reserved.
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

#include "bme280.h"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_baro.h>
#include <lib/perf/perf_counter.h>

class BME280 : public I2CSPIDriver<BME280>
{
public:
	BME280(const I2CSPIDriverConfig &config, bme280::IBME280 *interface);
	virtual ~BME280();

	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);
	static void print_usage();

	int			init();
	void			print_status();

	void			RunImpl();
private:
	void			Start();

	int			measure(); //start measure
	int			collect(); //get results and publish

	uORB::PublicationMulti<sensor_baro_s> _sensor_baro_pub{ORB_ID(sensor_baro)};

	bme280::IBME280		*_interface;

	// set config, recommended settings
	static constexpr uint8_t	_curr_ctrl{BME280_CTRL_P16 | BME280_CTRL_T2};
	static constexpr uint32_t	_measure_interval{BME280_MT_INIT + BME280_MT *(16 - 1 + 2 - 1)};

	bool			_collect_phase{false};

	perf_counter_t		_sample_perf;
	perf_counter_t		_measure_perf;
	perf_counter_t		_comms_errors;

	bme280::calibration_s	*_cal{nullptr}; //stored calibration constants
	bme280::fcalibration_s	_fcal{}; //pre processed calibration constants
};
