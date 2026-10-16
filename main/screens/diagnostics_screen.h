#pragma once

#include "lvgl.h"

/*
 * Raw sensor readings, live: IMU, magnetometer, ADC and barometer.
 *
 * Numbers as the parts report them, before any filtering - the on-screen
 * counterpart of the serial diagnostics. Refreshes only while it is the
 * loaded screen, and while it is, the same readings stream out of the
 * serial port for plotting on a PC - see diag_stream.h.
 */
void diagnostics_screen_create(lv_obj_t *screen);
