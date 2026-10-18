#pragma once

#include <stddef.h>

/*
 * A route from a GPX file, and guidance along it from GPS fixes - what
 * the HSI screen shows.
 *
 * The route is a list of waypoints, flown in order. Each leg runs from
 * one waypoint to the next along the great circle; the first leg, and
 * any "direct to", run from where the aircraft was when it began. The
 * earth is a sphere here: over a leg of a few hundred kilometres that
 * is good to a fraction of a percent, far finer than the needles show.
 *
 * Bearings and tracks are true, as GPS gives them and GPX holds them:
 * no magnetic variation anywhere.
 *
 * Portable: no ESP-IDF dependency, host tested (test/test_route.cpp).
 */

#define ROUTE_MAX_WAYPOINTS 100
#define ROUTE_NAME_BYTES    16      /* UTF-8, with the terminator */

struct RouteWaypoint
{
    char name[ROUTE_NAME_BYTES];
    double lat_deg;
    double lon_deg;
    float elevation_m;              /* above MSL; NAN: the file had none */
};

struct Route
{
    int count;
    RouteWaypoint waypoint[ROUTE_MAX_WAYPOINTS];
};

/* Which of the file's point lists the route was made from. */
enum RouteSource
{
    ROUTE_SOURCE_RTE = 0,           /* <rte><rtept>: the first route */
    ROUTE_SOURCE_WPT,               /* <wpt>, in file order */
    ROUTE_SOURCE_TRK,               /* <trk><trkseg><trkpt> */
};

/*
 * Reads a GPX file: the first route if it has one, else its waypoints,
 * else its track. Names come from <name> (WP1, WP2... when missing),
 * elevations from <ele>. False with a reason in error if there are no
 * points, more than ROUTE_MAX_WAYPOINTS, or a point without a valid
 * lat and lon.
 */
bool route_parse_gpx(const char *text, size_t length, Route *out,
                     RouteSource *source, char *error, size_t error_size);

/* Great circle distance, m, and initial true bearing, deg 0-360. */
double route_distance_m(double lat1_deg, double lon1_deg,
                        double lat2_deg, double lon2_deg);
double route_bearing_deg(double lat1_deg, double lon1_deg,
                         double lat2_deg, double lon2_deg);

struct RoutePosition
{
    double lat_deg;
    double lon_deg;
    float altitude_m;               /* MSL; NAN: none */
};

struct RouteGuidance
{
    bool valid;                     /* a route, and a fix to guide from */
    int to;                         /* the active waypoint; -1: no route */
    bool direct;                    /* the leg starts where "direct to" was */

    double distance_m;              /* straight to the active waypoint */
    float bearing_deg;              /* to the active waypoint */
    float desired_track_deg;        /* the leg's, at its start */
    float cross_track_m;            /* off the leg: + right of it */
    float along_track_to_go_m;      /* to abeam the waypoint; <= 0 passed */

    bool vertical_valid;            /* both ends of the leg have heights */
    float desired_altitude_m;       /* on the straight path between them */
    float vertical_deviation_m;     /* + above that path */
};

/* Within this of a waypoint it counts as reached, whatever the leg. */
#define ROUTE_ARRIVAL_RADIUS_M 500.0

/*
 * Where the aircraft is on a route, and moving along it: a waypoint
 * counts as passed once the aircraft is abeam of it - across the line
 * square to the leg through it - or within ROUTE_ARRIVAL_RADIUS_M of it,
 * and the next leg starts there. The radius catches a waypoint flown
 * past at an angle to a short leg, which may never come abeam: a
 * "direct to" begun just beside it, say. The last waypoint stays active
 * once passed.
 *
 * One task only; the caller locks.
 */
class RouteNavigator
{
public:
    RouteNavigator();

    /* A new route: direct to its first waypoint from the next fix. An
     * empty route clears it. */
    void setRoute(const Route &route);

    /* Straight from the next fix to waypoint index. */
    bool directTo(int index);

    /* The leg into waypoint index from the one before it; the first
     * waypoint has none, so for it this is directTo(). */
    bool activateLeg(int index);

    /*
     * Guidance from a fix, or none (NULL): valid then stays false, but
     * to and direct are still filled. True when the active waypoint
     * moved on because this fix passed it.
     */
    bool update(const RoutePosition *fix, RouteGuidance *out);

    const Route &route() const { return route_; }
    int active() const { return active_; }
    bool direct() const { return direct_; }

private:
    void guide(const RoutePosition &fix, RouteGuidance *out) const;

    Route route_;
    int active_;
    bool direct_;
    bool origin_pending_;           /* direct: taken from the next fix */
    RoutePosition origin_;
};
