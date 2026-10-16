#pragma once

#include "lvgl.h"

/*
 * Altimeter and vertical speed.
 *
 * A two pointer altimeter dial - the long pointer hundreds, a turn per
 * 1000, the short one thousands - with the altitude as a number too, and
 * the vertical speed as a bar along the left edge, climb up and sink
 * down, with its number under the altitude.
 *
 * The QNH window below the hub is the altimeter setting: - and + step
 * it a hectopascal at a time (hold to repeat), STD sets the standard
 * 1013.25. It applies at once, to every altitude shown, and is kept
 * across restarts. Units follow the setting on the settings screen.
 *
 * Both readings come from the vertical channel (navigation.h): barometric
 * altitude blended with the accelerometer, so the vertical speed is
 * steady without the lag of differentiating the barometer.
 */

void altimeter_screen_create(lv_obj_t *screen);

/* Units changed on the settings screen. */
void altimeter_screen_units_changed(void);
