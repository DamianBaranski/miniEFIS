#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * QMC5883 three-axis magnetometer.
 *
 * Optional, like the barometer: mag_init() reporting ESP_ERR_NOT_FOUND
 * is a normal outcome. Two parts share the name and nothing else - the
 * register maps, addresses and scale factors all differ - so both are
 * probed:
 *
 *   QMC5883L  at 0x0D, chip id 0xFF in register 0x0D
 *   QMC5883P  at 0x2C, chip id 0x80 in register 0x00
 *
 * Breakout boards sold as "QMC5883" carry either; the I2C scan in the
 * boot log says which one is fitted.
 *
 * Readings are in the sensor's own axes, as printed on the chip. They
 * are raw: no hard or soft iron correction is applied here.
 */

#define MAG_ADDR_QMC5883L       0x0D
#define MAG_ADDR_QMC5883P       0x2C

typedef enum {
    MAG_CHIP_NONE = 0,
    MAG_CHIP_QMC5883L,
    MAG_CHIP_QMC5883P
} mag_chip_t;

typedef struct {
    int16_t raw[3];             /* counts, x y z */
    float x_uT;
    float y_uT;
    float z_uT;
    bool overflow;              /* a field beyond the range was measured */
} mag_sample_t;

/* ESP_ERR_NOT_FOUND if nothing answers at either address. */
esp_err_t mag_init(void);

bool mag_present(void);
mag_chip_t mag_chip(void);
const char *mag_chip_name(void);

/* Which address answered, or 0 if none did. */
uint8_t mag_address(void);

/* The most recent measurement; the part converts continuously at 50 Hz. */
esp_err_t mag_read(mag_sample_t *sample);

#ifdef __cplusplus
}
#endif
