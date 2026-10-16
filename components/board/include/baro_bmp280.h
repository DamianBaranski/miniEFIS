#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * BMP280 barometric pressure and temperature sensor.
 *
 * Optional: the board does not carry one, so baro_init() reporting
 * ESP_ERR_NOT_FOUND is a normal outcome and everything else keeps
 * working. Use baro_present() to decide whether to show altitude.
 *
 * The part sits at 0x76 or 0x77 depending on how its SDO pin is
 * strapped; both are probed. A BME280 answers the same registers for
 * pressure and temperature and is accepted too - it is the same die with
 * humidity added, which this driver ignores.
 */

#define BARO_ADDR_PRIMARY       0x76    /* SDO low  */
#define BARO_ADDR_SECONDARY     0x77    /* SDO high */

/* Standard atmosphere at sea level, for altitude until a local QNH is
 * known. Set this from the airfield's reported pressure and the altitude
 * becomes absolute rather than relative. */
#define BARO_SEA_LEVEL_STANDARD_PA 101325.0f

typedef struct {
    float pressure_pa;
    float temperature_c;
} baro_sample_t;

/*
 * Oversampling. Higher settings average more internally: quieter, but
 * slower. x16 pressure gives about 1.3 Pa of noise - some 11 cm - at
 * roughly 25 Hz, which is the right trade for an altimeter.
 */
typedef enum {
    BARO_OVERSAMPLE_SKIP = 0,
    BARO_OVERSAMPLE_X1,
    BARO_OVERSAMPLE_X2,
    BARO_OVERSAMPLE_X4,
    BARO_OVERSAMPLE_X8,
    BARO_OVERSAMPLE_X16
} baro_oversample_t;

/* On-chip IIR filter, which smooths the short term noise that
 * oversampling alone leaves. Coefficient 16 is the strongest. */
typedef enum {
    BARO_FILTER_OFF = 0,
    BARO_FILTER_2,
    BARO_FILTER_4,
    BARO_FILTER_8,
    BARO_FILTER_16
} baro_filter_t;

typedef struct {
    baro_oversample_t pressure_oversample;
    baro_oversample_t temperature_oversample;
    baro_filter_t filter;
} baro_config_t;

void baro_config_default(baro_config_t *config);

/* Pass NULL for the defaults. ESP_ERR_NOT_FOUND if nothing answers. */
esp_err_t baro_init(const baro_config_t *config);

bool baro_present(void);

/* Which address answered, or 0 if none did. */
uint8_t baro_address(void);

/* Chip identifier that was read: 0x58 is BMP280, 0x60 is BME280. */
uint8_t baro_chip_id(void);

esp_err_t baro_read(baro_sample_t *sample);

/*
 * Pressure altitude by the international standard atmosphere. Pass the
 * local sea level pressure for absolute altitude, or the standard value
 * above for the pressure altitude.
 */
float baro_altitude_m(float pressure_pa, float sea_level_pa);

#ifdef __cplusplus
}
#endif
