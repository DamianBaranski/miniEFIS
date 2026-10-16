#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * QMI8658 6-axis IMU.
 *
 * Accelerometer and gyroscope are read in one burst so that both belong
 * to the same sample instant, which matters for the attitude filter -
 * two separate transfers would pair an accelerometer sample with a gyro
 * sample taken later.
 */

typedef struct {
    float x;
    float y;
    float z;
} imu_vec3_t;

typedef struct {
    imu_vec3_t accel_g;         /* g */
    imu_vec3_t gyro_dps;        /* deg/s */
} imu_sample_t;

typedef enum {
    IMU_ACCEL_RANGE_2G = 0,
    IMU_ACCEL_RANGE_4G,
    IMU_ACCEL_RANGE_8G,
    IMU_ACCEL_RANGE_16G
} imu_accel_range_t;

typedef enum {
    IMU_GYRO_RANGE_16DPS = 0,
    IMU_GYRO_RANGE_32DPS,
    IMU_GYRO_RANGE_64DPS,
    IMU_GYRO_RANGE_128DPS,
    IMU_GYRO_RANGE_256DPS,
    IMU_GYRO_RANGE_512DPS,
    IMU_GYRO_RANGE_1024DPS
} imu_gyro_range_t;

/* Output data rate, applied to both sensors. */
typedef enum {
    IMU_ODR_8000HZ = 0,
    IMU_ODR_4000HZ,
    IMU_ODR_2000HZ,
    IMU_ODR_1000HZ,
    IMU_ODR_500HZ,
    IMU_ODR_250HZ,
    IMU_ODR_120HZ,
    IMU_ODR_60HZ,
    IMU_ODR_30HZ
} imu_odr_t;

/* Anti-alias filter bandwidth, as a fraction of the output data rate. */
typedef enum {
    IMU_LPF_2_66_PCT = 0,
    IMU_LPF_3_63_PCT,
    IMU_LPF_5_39_PCT,
    IMU_LPF_13_37_PCT
} imu_lpf_t;

typedef struct {
    imu_accel_range_t accel_range;
    imu_gyro_range_t gyro_range;
    imu_odr_t odr;
    imu_lpf_t lpf;

    /*
     * Accelerometer zero-g offset, subtracted from every reading.
     *
     * Per unit, so treat the default as a starting point rather than a
     * constant of nature. To measure your own: rest the board with each
     * axis pointing down and then up in turn, note the readings from the
     * serial log, and take the offset as (down + up) / 2 - the gravity
     * term cancels and what is left is the offset.
     *
     * Worth doing: on the board this was written for, an uncorrected
     * 0.05 g offset showed as 2.9 degrees of pitch while sitting level,
     * and made the measured gravity magnitude swing between 0.95 g and
     * 1.04 g depending on attitude.
     *
     * There is no equivalent for the gyro: the attitude filter measures
     * and tracks that offset itself at every start-up.
     */
    imu_vec3_t accel_offset_g;
} imu_config_t;

/*
 * Defaults suited to an attitude display polled at 100 Hz:
 *
 *   512 dps  - a brisk roll passes 64 dps easily, and a saturated rate
 *              corrupts the attitude integration.
 *   1000 Hz  - with the filter below this puts the anti-alias corner at
 *              27 Hz, comfortably under the 50 Hz Nyquist limit of a
 *              100 Hz polling loop. At the chip's 8 kHz default, noise
 *              well above that limit folds straight into the samples.
 */
void imu_config_default(imu_config_t *config);

/* Pass NULL for the defaults above. */
esp_err_t imu_init(const imu_config_t *config);

esp_err_t imu_read(imu_sample_t *sample);

#ifdef __cplusplus
}
#endif
