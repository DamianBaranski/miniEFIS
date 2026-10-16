#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ADS1115 four channel, 16-bit I2C ADC.
 *
 * Optional: ads1115_init() reporting ESP_ERR_NOT_FOUND is a normal
 * outcome. The address is set by strapping the ADDR pin, so all four
 * possibilities are probed:
 *
 *   ADDR to GND 0x48, VDD 0x49, SDA 0x4A, SCL 0x4B
 *
 * Channels are read single-ended, against GND, one conversion at a
 * time. The part has no chip id register, so anything answering at those
 * addresses is taken to be an ADS1115.
 */

#define ADS1115_ADDR_FIRST      0x48
#define ADS1115_ADDR_LAST       0x4B
#define ADS1115_CHANNELS        4

/*
 * Full scale range. The inputs can never go beyond the supply rail, so
 * on a 3.3 V supply the 4.096 V range is the finest that still covers
 * every possible input.
 */
typedef enum {
    ADS1115_RANGE_6V144 = 0,
    ADS1115_RANGE_4V096,
    ADS1115_RANGE_2V048,
    ADS1115_RANGE_1V024,
    ADS1115_RANGE_0V512,
    ADS1115_RANGE_0V256
} ads1115_range_t;

/* ESP_ERR_NOT_FOUND if nothing answers. */
esp_err_t ads1115_init(ads1115_range_t range);

bool ads1115_present(void);

/* Which address answered, or 0 if none did. */
uint8_t ads1115_address(void);

/*
 * Convert one channel, 0..3, and wait for the result: about 9 ms at the
 * 128 samples per second used here. Blocks the calling task, so call it
 * from a task that can afford to wait - not from the LVGL or IMU task.
 */
esp_err_t ads1115_read(uint8_t channel, int16_t *raw);

/* Counts to volts at the configured range. */
float ads1115_to_volts(int16_t raw);

#ifdef __cplusplus
}
#endif
