#pragma once

#include "lvgl.h"

/*
 * Settings: units and backlight, and the magnetometer's mounting axes
 * and calibration. All of it is kept across restarts.
 *
 * The screen scrolls - a round panel has little room - and the
 * calibration procedure runs on a screen of its own, which also keeps
 * the double tap menu out of the way until it is finished or cancelled.
 */
void settings_screen_create(lv_obj_t *screen);
