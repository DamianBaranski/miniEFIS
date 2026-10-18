#pragma once

#include "route.h"

#include <stddef.h>
#include <stdint.h>

/*
 * The route being flown, kept across restarts, and the guidance along it
 * from the CAN bus GPS (can_bus.h) - for the HSI screen and the web page.
 *
 * The route comes from a GPX file sent to the debug web page. It is
 * kept in NVS with the active waypoint, so a restart in flight carries
 * on with the same leg. A task of its own steps the guidance five times
 * a second whatever screen is up, so waypoints are passed and the next
 * leg taken even with the HSI not showing.
 *
 * All functions are safe from any task.
 */

/* Loads the saved route and starts the guidance task. After NVS and the
 * CAN bus are up. */
void route_store_start(void);

/*
 * A GPX file (route_parse_gpx()): replaces the route and saves it, then
 * direct to its first waypoint. text needn't be NUL terminated. True
 * with a summary in message, or false with the reason.
 */
bool route_store_load_gpx(const char *text, size_t length, char *message,
                          size_t message_size);

/* No route at all. */
void route_store_clear(void);

/* Straight from where the aircraft is now to waypoint index. */
bool route_store_direct_to(int index);

/* Back onto the leg into waypoint index from the one before it; for the
 * first waypoint, which has none, direct to it. */
bool route_store_activate_leg(int index);

typedef struct {
    uint32_t version;           /* changes with the route or its active waypoint */
    int count;                  /* waypoints; 0: no route */
    RouteGuidance guidance;     /* valid: a route and a fix */

    char to_name[ROUTE_NAME_BYTES];

    bool have_fix;              /* a current 2D or 3D GPS fix */
    bool have_track;            /* fast enough for the track to mean anything */
    float track_deg;            /* true; the last good one when not have_track */
    float ground_speed_ms;
    float altitude_m;           /* GPS, MSL; NAN without a 3D fix */
} route_status_t;

void route_store_status(route_status_t *out);

/* The whole route, for the web page. */
void route_store_get(Route *out);
