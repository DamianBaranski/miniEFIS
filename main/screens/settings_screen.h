#pragma once

#include "lvgl.h"

/*
 * Settings, a page at a time: the main page lists Display (units,
 * backlight), Screens (which instruments are shown), Engine (its type,
 * and the custom engine's limits, a page for each value), Mounting (Set
 * level), Wi-Fi debug and Compass (the magnetometer, its calibration,
 * and the raw sensor stream for tools/sensor_plot.py while it is open).
 * Each page has a Back button; leaving the screen goes back to the main
 * page. All of it is kept across restarts.
 *
 * Only the page showing is built - LVGL's heap has little room - and
 * the screen scrolls, a round panel having little room either. The
 * calibration procedure runs on a screen of its own, which also keeps
 * the double tap menu out of the way until it is finished or cancelled.
 */
void settings_screen_create(lv_obj_t *screen);
