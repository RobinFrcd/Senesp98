/*
 * THIS FILE IS AUTOMATICALLY GENERATED
 *
 * Generator:     sensirion-driver-generator 1.0.1
 * Product:       sen66
 * Model-Version: 1.3.1
 */
/*
 * Copyright (c) 2024, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "sen66_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c.h"
#include "sensirion_i2c_hal.h"

#define sensirion_hal_sleep_us sensirion_i2c_hal_sleep_usec

static uint8_t communication_buffer[48] = {0};

static uint8_t _i2c_address;

void sen66_init(uint8_t i2c_address) {
    _i2c_address = i2c_address;
}

uint16_t sen66_signal_co2(uint16_t co2_raw) {
    uint16_t co2 = 0;
    co2 = co2_raw;
    return co2;
}

int16_t sen66_start_continuous_measurement() {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x21);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(50 * 1000);
    return local_error;
}

int16_t sen66_stop_measurement() {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x104);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(1000 * 1000);
    return local_error;
}

int16_t sen66_get_data_ready(uint8_t* padding, bool* data_ready) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x202);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(20 * 1000);
    local_error = sensirion_i2c_read_data_inplace(_i2c_address, buffer_ptr, 2);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    *padding = (uint8_t)buffer_ptr[0];
    *data_ready = (bool)buffer_ptr[1];
    return local_error;
}

int16_t sen66_read_measured_values_as_integers(
    uint16_t* mass_concentration_pm1p0, uint16_t* mass_concentration_pm2p5,
    uint16_t* mass_concentration_pm4p0, uint16_t* mass_concentration_pm10p0,
    int16_t* ambient_humidity, int16_t* ambient_temperature, int16_t* voc_index,
    int16_t* nox_index, uint16_t* co2) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x300);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(20 * 1000);
    local_error = sensirion_i2c_read_data_inplace(_i2c_address, buffer_ptr, 18);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    *mass_concentration_pm1p0 =
        sensirion_common_bytes_to_uint16_t(&buffer_ptr[0]);
    *mass_concentration_pm2p5 =
        sensirion_common_bytes_to_uint16_t(&buffer_ptr[2]);
    *mass_concentration_pm4p0 =
        sensirion_common_bytes_to_uint16_t(&buffer_ptr[4]);
    *mass_concentration_pm10p0 =
        sensirion_common_bytes_to_uint16_t(&buffer_ptr[6]);
    *ambient_humidity = sensirion_common_bytes_to_int16_t(&buffer_ptr[8]);
    *ambient_temperature = sensirion_common_bytes_to_int16_t(&buffer_ptr[10]);
    *voc_index = sensirion_common_bytes_to_int16_t(&buffer_ptr[12]);
    *nox_index = sensirion_common_bytes_to_int16_t(&buffer_ptr[14]);
    *co2 = sensirion_common_bytes_to_uint16_t(&buffer_ptr[16]);
    return local_error;
}

int16_t sen66_read_number_concentration_values_as_integers(
    uint16_t* number_concentration_pm0p5, uint16_t* number_concentration_pm1p0,
    uint16_t* number_concentration_pm2p5, uint16_t* number_concentration_pm4p0,
    uint16_t* number_concentration_pm10p0) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x316);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(20 * 1000);
    local_error = sensirion_i2c_read_data_inplace(_i2c_address, buffer_ptr, 10);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    *number_concentration_pm0p5 =
        sensirion_common_bytes_to_uint16_t(&buffer_ptr[0]);
    *number_concentration_pm1p0 =
        sensirion_common_bytes_to_uint16_t(&buffer_ptr[2]);
    *number_concentration_pm2p5 =
        sensirion_common_bytes_to_uint16_t(&buffer_ptr[4]);
    *number_concentration_pm4p0 =
        sensirion_common_bytes_to_uint16_t(&buffer_ptr[6]);
    *number_concentration_pm10p0 =
        sensirion_common_bytes_to_uint16_t(&buffer_ptr[8]);
    return local_error;
}

int16_t sen66_read_measured_raw_values(int16_t* raw_humidity,
                                       int16_t* raw_temperature,
                                       uint16_t* raw_voc, uint16_t* raw_nox,
                                       uint16_t* raw_co2) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x405);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(20 * 1000);
    local_error = sensirion_i2c_read_data_inplace(_i2c_address, buffer_ptr, 10);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    *raw_humidity = sensirion_common_bytes_to_int16_t(&buffer_ptr[0]);
    *raw_temperature = sensirion_common_bytes_to_int16_t(&buffer_ptr[2]);
    *raw_voc = sensirion_common_bytes_to_uint16_t(&buffer_ptr[4]);
    *raw_nox = sensirion_common_bytes_to_uint16_t(&buffer_ptr[6]);
    *raw_co2 = sensirion_common_bytes_to_uint16_t(&buffer_ptr[8]);
    return local_error;
}

int16_t sen66_start_fan_cleaning() {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x5607);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(20 * 1000);
    return local_error;
}

int16_t sen66_set_temperature_offset_parameters(int16_t offset, int16_t slope,
                                                uint16_t time_constant,
                                                uint16_t slot) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x60b2);
    local_offset =
        sensirion_i2c_add_int16_t_to_buffer(buffer_ptr, local_offset, offset);
    local_offset =
        sensirion_i2c_add_int16_t_to_buffer(buffer_ptr, local_offset, slope);
    local_offset = sensirion_i2c_add_uint16_t_to_buffer(
        buffer_ptr, local_offset, time_constant);
    local_offset =
        sensirion_i2c_add_uint16_t_to_buffer(buffer_ptr, local_offset, slot);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(20 * 1000);
    return local_error;
}

int16_t sen66_set_voc_algorithm_tuning_parameters(
    int16_t index_offset, int16_t learning_time_offset_hours,
    int16_t learning_time_gain_hours, int16_t gating_max_duration_minutes,
    int16_t std_initial, int16_t gain_factor) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x60d0);
    local_offset = sensirion_i2c_add_int16_t_to_buffer(buffer_ptr, local_offset,
                                                       index_offset);
    local_offset = sensirion_i2c_add_int16_t_to_buffer(
        buffer_ptr, local_offset, learning_time_offset_hours);
    local_offset = sensirion_i2c_add_int16_t_to_buffer(
        buffer_ptr, local_offset, learning_time_gain_hours);
    local_offset = sensirion_i2c_add_int16_t_to_buffer(
        buffer_ptr, local_offset, gating_max_duration_minutes);
    local_offset = sensirion_i2c_add_int16_t_to_buffer(buffer_ptr, local_offset,
                                                       std_initial);
    local_offset = sensirion_i2c_add_int16_t_to_buffer(buffer_ptr, local_offset,
                                                       gain_factor);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(20 * 1000);
    return local_error;
}

int16_t sen66_get_voc_algorithm_tuning_parameters(
    int16_t* index_offset, int16_t* learning_time_offset_hours,
    int16_t* learning_time_gain_hours, int16_t* gating_max_duration_minutes,
    int16_t* std_initial, int16_t* gain_factor) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x60d0);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(20 * 1000);
    local_error = sensirion_i2c_read_data_inplace(_i2c_address, buffer_ptr, 12);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    *index_offset = sensirion_common_bytes_to_int16_t(&buffer_ptr[0]);
    *learning_time_offset_hours =
        sensirion_common_bytes_to_int16_t(&buffer_ptr[2]);
    *learning_time_gain_hours =
        sensirion_common_bytes_to_int16_t(&buffer_ptr[4]);
    *gating_max_duration_minutes =
        sensirion_common_bytes_to_int16_t(&buffer_ptr[6]);
    *std_initial = sensirion_common_bytes_to_int16_t(&buffer_ptr[8]);
    *gain_factor = sensirion_common_bytes_to_int16_t(&buffer_ptr[10]);
    return local_error;
}

int16_t sen66_set_nox_algorithm_tuning_parameters(
    int16_t index_offset, int16_t learning_time_offset_hours,
    int16_t learning_time_gain_hours, int16_t gating_max_duration_minutes,
    int16_t std_initial, int16_t gain_factor) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x60e1);
    local_offset = sensirion_i2c_add_int16_t_to_buffer(buffer_ptr, local_offset,
                                                       index_offset);
    local_offset = sensirion_i2c_add_int16_t_to_buffer(
        buffer_ptr, local_offset, learning_time_offset_hours);
    local_offset = sensirion_i2c_add_int16_t_to_buffer(
        buffer_ptr, local_offset, learning_time_gain_hours);
    local_offset = sensirion_i2c_add_int16_t_to_buffer(
        buffer_ptr, local_offset, gating_max_duration_minutes);
    local_offset = sensirion_i2c_add_int16_t_to_buffer(buffer_ptr, local_offset,
                                                       std_initial);
    local_offset = sensirion_i2c_add_int16_t_to_buffer(buffer_ptr, local_offset,
                                                       gain_factor);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(20 * 1000);
    return local_error;
}

int16_t sen66_get_nox_algorithm_tuning_parameters(
    int16_t* index_offset, int16_t* learning_time_offset_hours,
    int16_t* learning_time_gain_hours, int16_t* gating_max_duration_minutes,
    int16_t* std_initial, int16_t* gain_factor) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x60e1);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(20 * 1000);
    local_error = sensirion_i2c_read_data_inplace(_i2c_address, buffer_ptr, 12);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    *index_offset = sensirion_common_bytes_to_int16_t(&buffer_ptr[0]);
    *learning_time_offset_hours =
        sensirion_common_bytes_to_int16_t(&buffer_ptr[2]);
    *learning_time_gain_hours =
        sensirion_common_bytes_to_int16_t(&buffer_ptr[4]);
    *gating_max_duration_minutes =
        sensirion_common_bytes_to_int16_t(&buffer_ptr[6]);
    *std_initial = sensirion_common_bytes_to_int16_t(&buffer_ptr[8]);
    *gain_factor = sensirion_common_bytes_to_int16_t(&buffer_ptr[10]);
    return local_error;
}

int16_t sen66_set_temperature_acceleration_parameters(uint16_t k, uint16_t p,
                                                      uint16_t t1,
                                                      uint16_t t2) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x6100);
    local_offset =
        sensirion_i2c_add_uint16_t_to_buffer(buffer_ptr, local_offset, k);
    local_offset =
        sensirion_i2c_add_uint16_t_to_buffer(buffer_ptr, local_offset, p);
    local_offset =
        sensirion_i2c_add_uint16_t_to_buffer(buffer_ptr, local_offset, t1);
    local_offset =
        sensirion_i2c_add_uint16_t_to_buffer(buffer_ptr, local_offset, t2);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(20 * 1000);
    return local_error;
}

int16_t sen66_set_voc_algorithm_state(const uint8_t* state,
                                      uint16_t state_size) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x6181);
    sensirion_i2c_add_bytes_to_buffer(buffer_ptr, local_offset, state,
                                      state_size);

    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    return local_error;
}

int16_t sen66_get_voc_algorithm_state(uint8_t* state, uint16_t state_size) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x6181);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(20 * 1000);
    local_error = sensirion_i2c_read_data_inplace(_i2c_address, buffer_ptr, 8);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_common_copy_bytes(&buffer_ptr[0], (uint8_t*)state, state_size);
    return local_error;
}

int16_t
sen66_perform_forced_co2_recalibration(uint16_t target_co2_concentration,
                                       uint16_t* correction) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x6707);
    local_offset = sensirion_i2c_add_uint16_t_to_buffer(
        buffer_ptr, local_offset, target_co2_concentration);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(500 * 1000);
    local_error = sensirion_i2c_read_data_inplace(_i2c_address, buffer_ptr, 2);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    *correction = sensirion_common_bytes_to_uint16_t(&buffer_ptr[0]);
    return local_error;
}

int16_t sen66_set_co2_sensor_automatic_self_calibration(uint16_t status) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x6711);
    local_offset =
        sensirion_i2c_add_uint16_t_to_buffer(buffer_ptr, local_offset, status);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(20 * 1000);
    return local_error;
}

int16_t sen66_get_co2_sensor_automatic_self_calibration(uint8_t* padding,
                                                        bool* status) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x6711);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(20 * 1000);
    local_error = sensirion_i2c_read_data_inplace(_i2c_address, buffer_ptr, 2);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    *padding = (uint8_t)buffer_ptr[0];
    *status = (bool)buffer_ptr[1];
    return local_error;
}

int16_t sen66_set_ambient_pressure(uint16_t ambient_pressure) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x6720);
    local_offset = sensirion_i2c_add_uint16_t_to_buffer(
        buffer_ptr, local_offset, ambient_pressure);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(20 * 1000);
    return local_error;
}

int16_t sen66_get_ambient_pressure(uint16_t* ambient_pressure) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x6720);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(20 * 1000);
    local_error = sensirion_i2c_read_data_inplace(_i2c_address, buffer_ptr, 2);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    *ambient_pressure = sensirion_common_bytes_to_uint16_t(&buffer_ptr[0]);
    return local_error;
}

int16_t sen66_set_sensor_altitude(uint16_t altitude) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x6736);
    local_offset = sensirion_i2c_add_uint16_t_to_buffer(buffer_ptr,
                                                        local_offset, altitude);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(20 * 1000);
    return local_error;
}

int16_t sen66_get_sensor_altitude(uint16_t* altitude) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x6736);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(20 * 1000);
    local_error = sensirion_i2c_read_data_inplace(_i2c_address, buffer_ptr, 2);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    *altitude = sensirion_common_bytes_to_uint16_t(&buffer_ptr[0]);
    return local_error;
}

int16_t sen66_activate_sht_heater() {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0x6765);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(1300 * 1000);
    return local_error;
}

int16_t sen66_get_product_name(uint8_t* product_name,
                               uint16_t product_name_size) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0xd014);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(20 * 1000);
    local_error = sensirion_i2c_read_data_inplace(_i2c_address, buffer_ptr, 32);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_common_copy_bytes(&buffer_ptr[0], (uint8_t*)product_name,
                                product_name_size);
    return local_error;
}

int16_t sen66_get_serial_number(uint8_t* serial_number,
                                uint16_t serial_number_size) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0xd033);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(20 * 1000);
    local_error = sensirion_i2c_read_data_inplace(_i2c_address, buffer_ptr, 32);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_common_copy_bytes(&buffer_ptr[0], (uint8_t*)serial_number,
                                serial_number_size);
    return local_error;
}

int16_t sen66_read_device_status(sen66_device_status* device_status) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0xd206);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(20 * 1000);
    local_error = sensirion_i2c_read_data_inplace(_i2c_address, buffer_ptr, 4);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    (*device_status).value = sensirion_common_bytes_to_uint32_t(&buffer_ptr[0]);
    return local_error;
}

int16_t sen66_read_and_clear_device_status(sen66_device_status* device_status) {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0xd210);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(20 * 1000);
    local_error = sensirion_i2c_read_data_inplace(_i2c_address, buffer_ptr, 4);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    (*device_status).value = sensirion_common_bytes_to_uint32_t(&buffer_ptr[0]);
    return local_error;
}

int16_t sen66_device_reset() {
    int16_t local_error = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset =
        sensirion_i2c_add_command16_to_buffer(buffer_ptr, local_offset, 0xd304);
    local_error =
        sensirion_i2c_write_data(_i2c_address, buffer_ptr, local_offset);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    sensirion_i2c_hal_sleep_usec(1200 * 1000);
    return local_error;
}
