#pragma once

#include "lvgl.h"

/*
 * Altitude and vertical speed readout.
 *
 * A compact panel at the right of the screen: altitude, vertical speed,
 * and a vertical speed bar. Hides itself when no barometer is fitted, so
 * it can be created unconditionally.
 *
 * Runs in the LVGL task, like every other widget.
 */

typedef enum {
    ALTITUDE_UNITS_METRIC,      /* m and m/s      */
    ALTITUDE_UNITS_AVIATION     /* ft and ft/min  */
} altitude_units_t;

void altitude_widget_create(lv_obj_t *parent);
void altitude_widget_close(void);

void altitude_widget_set_units(altitude_units_t units);

/* Cost of drawing this panel in the last frame, microseconds. */
uint32_t altitude_widget_render_cost(void);
