#pragma once

#include "lvgl.h"

/*
 * Heading indicator: a compass card that turns under a fixed lubber
 * line and aircraft symbol, as on an aircraft's directional gyro, with
 * the heading as a number below the symbol.
 *
 * The heading is the attitude filter's: tilt compensated, smoothed by
 * the gyro, and only available once the magnetometer is calibrated -
 * which the screen says, rather than showing a heading it does not have.
 */
void compass_screen_create(lv_obj_t *screen);

/* What the status line says, in order of what to fix first. */
typedef enum {
    COMPASS_STATE_OK = 0,
    COMPASS_STATE_NO_MAGNETOMETER,
    COMPASS_STATE_NOT_CALIBRATED,
    COMPASS_STATE_CALIBRATING,
    COMPASS_STATE_NO_HEADING        /* calibrated, but the filter has none */
} compass_state_t;

/* Safe from any task. */
compass_state_t compass_state(bool heading_valid);
