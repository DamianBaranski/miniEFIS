#pragma once

#include "lvgl.h"

/*
 * Turn coordinator: a miniature aircraft, seen from behind, that banks
 * with the rate of turn - wings on the lower marks is a standard rate
 * turn, 3 deg/s, two minutes for a full circle - and the slip ball
 * below it.
 *
 * The numbers come from the navigation snapshot (navigation.h), like
 * every other screen's. Like the real instrument it says nothing about
 * pitch, and nothing about bank either: the symbol banks with the rate
 * of turn, not with the wings.
 */

void turn_coordinator_screen_create(lv_obj_t *screen);
