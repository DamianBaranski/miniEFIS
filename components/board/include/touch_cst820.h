#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * CST820 capacitive touch controller, single touch point.
 *
 * The controller's interrupt line is configured but not used: the panel
 * is polled from the LVGL input callback, which is simpler and costs
 * nothing at LVGL's read rate.
 */

typedef enum {
    TOUCH_GESTURE_NONE = 0x00,
    TOUCH_GESTURE_SWIPE_UP = 0x01,
    TOUCH_GESTURE_SWIPE_DOWN = 0x02,
    TOUCH_GESTURE_SWIPE_LEFT = 0x03,
    TOUCH_GESTURE_SWIPE_RIGHT = 0x04,
    TOUCH_GESTURE_SINGLE_CLICK = 0x05,
    TOUCH_GESTURE_DOUBLE_CLICK = 0x0B,
    TOUCH_GESTURE_LONG_PRESS = 0x0C
} touch_gesture_t;

typedef struct {
    bool pressed;
    uint16_t x;
    uint16_t y;
    touch_gesture_t gesture;
} touch_state_t;

esp_err_t touch_init(void);

/* Reads the controller. On an I2C error the state comes back released. */
esp_err_t touch_read(touch_state_t *state);

const char *touch_gesture_name(touch_gesture_t gesture);

#ifdef __cplusplus
}
#endif
