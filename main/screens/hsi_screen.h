#pragma once

#include "lvgl.h"

/*
 * HSI: navigation along the route loaded from a GPX file
 * (route_store.h), from the CAN bus GPS.
 *
 * A compass card turns under a fixed lubber line, track up: the GPS
 * ground track, true, is always at the top. On the card, the magenta
 * pointer is the bearing to the active waypoint and the green one the
 * leg's desired track. In the middle, the magenta bars are the course
 * deviation - the vertical bar cross track, the horizontal bar the
 * vertical path between the waypoints' heights, if the file had them -
 * each against three dots a side; fly towards the bar. Around them, the
 * active waypoint, the bearing and distance to it and the cross track
 * error, L or R of the leg.
 *
 * No route, or no GPS fix, is said in words; the needles go.
 */
void hsi_screen_create(lv_obj_t *screen);

/* The route menu - the waypoints, for direct to one or onto the leg into
 * it - over the HSI, if it is the screen up. */
void hsi_screen_open_menu(void);
