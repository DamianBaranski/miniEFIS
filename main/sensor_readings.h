#pragma once

#include "adc_ads1115.h"
#include "esp_err.h"
#include "imu_qmi8658.h"
#include "mag_qmc5883.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * The latest raw reading from every sensor, for display.
 *
 * The IMU and the magnetometer are sampled by the IMU task, which
 * feeds them to the attitude filter, and their readings are recorded
 * here as they go past. The ADC is polled by a task of its own, at a
 * low priority: an ADS1115 conversion takes 9 ms to wait for, which
 * neither the IMU task nor the LVGL task can spare.
 *
 * Nothing here is filtered or calibrated - it is what the parts report.
 * The barometer is not repeated here; navigation_barometer() has it.
 */

typedef struct {
    bool imu_valid;
    imu_sample_t imu;           /* sensor axes, accel offset removed */
    uint32_t imu_errors;        /* failed reads since start-up */

    bool mag_valid;
    mag_sample_t mag;           /* sensor axes, uncalibrated */
    uint32_t mag_errors;

    bool adc_valid[ADS1115_CHANNELS];
    int16_t adc_raw[ADS1115_CHANNELS];
    float adc_volts[ADS1115_CHANNELS];
    uint32_t adc_errors;
} sensor_readings_t;

/* Starts the ADC polling task, if there is an ADC. */
void sensor_readings_start(void);

/* Call once per IMU sample, from the IMU task. */
void sensor_readings_record_imu(const imu_sample_t *sample,
                                esp_err_t read_status);

/* Call with each magnetometer read, from the IMU task. */
void sensor_readings_record_mag(const mag_sample_t *sample,
                                esp_err_t read_status);

/* A consistent copy of everything. Safe from any task. */
void sensor_readings_get(sensor_readings_t *out);
