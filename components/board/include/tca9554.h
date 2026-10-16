#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * TCA9554 8-bit I2C IO expander.
 *
 * On this board it drives the display reset and chip select, the touch
 * controller reset and the IMU enable - see board_pins.h. Pins are
 * numbered 1..8 to match the datasheet's P0..P7 as printed on the
 * schematic.
 */

#define TCA9554_PIN_MIN     1
#define TCA9554_PIN_MAX     8

typedef enum {
    TCA9554_OUTPUT = 0,
    TCA9554_INPUT = 1
} tca9554_mode_t;

typedef enum {
    TCA9554_LOW = 0,
    TCA9554_HIGH = 1
} tca9554_level_t;

/*
 * modes is a bit mask, one bit per pin, 0 = output and 1 = input.
 * 0x00 therefore makes every line an output, which is what this board
 * wants.
 */
esp_err_t tca9554_init(uint8_t modes);

esp_err_t tca9554_set_mode(uint8_t pin, tca9554_mode_t mode);
esp_err_t tca9554_write_pin(uint8_t pin, tca9554_level_t level);
esp_err_t tca9554_read_pin(uint8_t pin, tca9554_level_t *level);
esp_err_t tca9554_toggle_pin(uint8_t pin);

esp_err_t tca9554_write_port(uint8_t levels);
esp_err_t tca9554_read_port(uint8_t *levels);

#ifdef __cplusplus
}
#endif
