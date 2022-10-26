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

#include "GrovePiPlus.h"
#include <cassert>

int GrovePiPlus::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		PX4_ERR("I2C init failed");
		return ret;
	}

    int32_t voltage_port_param = 0;
    param_get(param_find("ADC_PORT_VOLTAGE"), &voltage_port_param);

    int32_t current_port_param = 0;
    param_get(param_find("ADC_PORT_CURRENT"), &current_port_param);

    _voltage_port_selection = voltage_port_param;
    _current_port_selection = current_port_param;

    PX4_INFO("Using port %d for voltage", voltage_port_param);
    pinMode(voltage_port_param, PinMode::INPUT);

    PX4_INFO("Using port %d for current", current_port_param);
    pinMode(current_port_param, PinMode::INPUT);

	ScheduleOnInterval(SAMPLE_INTERVAL, SAMPLE_INTERVAL);

	return PX4_OK;
}

int GrovePiPlus::pinMode(uint8_t port, PinMode mode) {
    uint8_t config[4] = {};
    config[0] = (uint8_t)Register::PIN_MODE;
    config[1] = port;
    config[2] = (uint8_t)PinMode::INPUT;
    config[3] = 0; // padding

    uint8_t dummy = 0;
    int ret = transfer(config, 4, &dummy, 1);

    if (ret != PX4_OK) {
        PX4_ERR("failed to set INPUT pin mode on port %d", port);
        return ret;
    }

    PX4_INFO("dummy read=%d", dummy);

    return ret;
}

int GrovePiPlus::analogRead(uint8_t port, uint16_t *value_out) {
    uint8_t write_buf[4] = {};
    write_buf[0] = (uint8_t)Register::ANALOG_READ;
    write_buf[1] = port;
    write_buf[2] = 0;
    write_buf[3] = 0;

    int write_cnt = 0;
    int ret = PX4_ERROR;
    while (ret != PX4_OK && write_cnt < MAX_I2C_RETRIES_PER_CYCLE) {
        ret = transfer(write_buf, 4, nullptr, 0);
        write_cnt++;
    }

    if (ret != PX4_OK) {
        PX4_WARN("failed to send ANALOG_READ command on port %d after %d retries", port, write_cnt);
        return ret;
    }

    uint8_t read_buf[3] = {};
    read_buf[0] = DATA_NOT_AVAILABLE;
    int read_failure_cnt = 0;
    int read_cnt = 0;

    while ((read_buf[0] == DATA_NOT_AVAILABLE || read_buf[0] == DATA_NOT_AVAILABLE2) &&
           read_failure_cnt < MAX_I2C_RETRIES_PER_CYCLE &&
           read_cnt < MAX_DATA_AVAILABLE_TRIES_PER_CYCLE) {
        ret = transfer(nullptr, 0, read_buf, 3);
        if (ret != PX4_OK) {
            read_failure_cnt++;
        } else {
            read_failure_cnt = 0;
        }
        read_cnt++;
    }

    if (ret != PX4_OK) {
        PX4_WARN("failed to read ANALOG_READ on port %d after %d failures", port, read_failure_cnt);
        return ret;
    }

    if (read_buf[0] == DATA_NOT_AVAILABLE || read_buf[0] == DATA_NOT_AVAILABLE2) {
        PX4_WARN("data not available from GrovePi on port %d after %d tries", port, read_cnt);
        return PX4_ERROR;
    }

    *value_out = read_buf[1] << 8 | read_buf[2];

    return PX4_OK;
}
