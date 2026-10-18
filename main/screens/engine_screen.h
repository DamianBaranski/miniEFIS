#pragma once

#include "lvgl.h"

/*
 * Engine 1's values from the CAN bus engine unit (can_bus.h), coloured
 * by the limits of the engine picked in settings (engine_limits.h):
 * white in the green range, amber or red outside it.
 *
 * Engine: the RPM dial over the top, its ranges round the arc and the
 * manifold pressure under the number; below it, a bar each for oil
 * pressure and temperature, the hottest CHT, the coolant (liquid
 * cooled) or hottest EGT, fuel pressure, flow and quantity, and the bus
 * voltage, each with its ranges along it and a mark at the value.
 *
 * Cylinders: EGT and CHT of each cylinder as bars side by side, the
 * numbers under them, the hottest cylinder's number lit; and around
 * them the carburettor air, the battery current, the endurance, the
 * fuel used and the engine time.
 *
 * A value not sent, or not sent lately, shows as dashes; with nothing
 * from the engine unit at all the screen says so.
 *
 * What they show - the EGT and CHT bars, the manifold pressure and which values get the engine screen's eight
 * bars - is set in their menu, opened by a swipe down on either.
 */
void engine_screen_create(lv_obj_t *screen);
void cylinders_screen_create(lv_obj_t *screen);

/* The engine screens' menu, if one of them is the screen up. */
void engine_screen_open_menu(void);
