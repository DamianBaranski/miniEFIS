/*
 * Host tests for the GPX reader and RouteNavigator.
 *
 * Small GPX files exercise what real ones do - a route among waypoints,
 * self-closing points, names with entities, CDATA and Polish letters,
 * tracks too long to fly - and simple routes along the equator and a
 * meridian, where the right answers are easy to work out by hand,
 * check the guidance: distance, bearing, cross track, sequencing and
 * the vertical path.
 */

#include "route.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* One degree along a great circle on the mean earth. */
static const double DEGREE_M = 6371008.8 * 3.14159265358979323846 / 180.0;

static int g_failures = 0;
static int g_checks = 0;

static void check_near(double actual, double expected, double tolerance,
                       const char *what)
{
    g_checks++;

    if(fabs(actual - expected) > tolerance)
    {
        g_failures++;
        printf("  FAIL %-50s %+12.3f, expected %+.3f +- %.3f\n",
               what, actual, expected, tolerance);
    }
    else
    {
        printf("  ok   %-50s %+12.3f\n", what, actual);
    }
}

static void check_true(bool condition, const char *what)
{
    g_checks++;

    if(!condition)
    {
        g_failures++;
        printf("  FAIL %s\n", what);
    }
    else
    {
        printf("  ok   %s\n", what);
    }
}

static bool parse(const char *gpx, Route *route, RouteSource *source,
                  char *error)
{
    return route_parse_gpx(gpx, strlen(gpx), route, source, error, 96);
}

static void test_route_among_waypoints()
{
    printf("a route among waypoints\n");

    const char *gpx =
        "<?xml version=\"1.0\"?>\n"
        "<gpx version=\"1.1\" creator=\"test\">\n"
        " <wpt lat=\"1\" lon=\"1\"><name>IGNORED</name></wpt>\n"
        " <rte><name>Trip</name>\n"
        "  <rtept lat=\"52.1657\" lon=\"20.9671\">\n"
        "   <ele>110.5</ele><name>EPWA</name></rtept>\n"
        "  <rtept lon='19.7848' lat='50.0777'><name>A &amp; B</name></rtept>\n"
        "  <rtept lat=\"50.5\" lon=\"20.0\"><name><![CDATA[ <X> ]]></name></rtept>\n"
        " </rte>\n"
        " <rte><rtept lat=\"0\" lon=\"0\"/></rte>\n"
        "</gpx>\n";

    Route route;
    RouteSource source;
    char error[96] = "";

    check_true(parse(gpx, &route, &source, error), "parsed");
    check_true(source == ROUTE_SOURCE_RTE, "from the route");
    check_near(route.count, 3, 0, "only the first route's points");
    check_true(strcmp(route.waypoint[0].name, "EPWA") == 0, "name");
    check_near(route.waypoint[0].lat_deg, 52.1657, 1e-9, "lat");
    check_near(route.waypoint[0].lon_deg, 20.9671, 1e-9, "lon");
    check_near(route.waypoint[0].elevation_m, 110.5, 1e-4, "elevation");
    check_near(route.waypoint[1].lon_deg, 19.7848, 1e-9,
               "single quotes, lon before lat");
    check_true(strcmp(route.waypoint[1].name, "A & B") == 0,
               "entity decoded");
    check_true(isnan(route.waypoint[1].elevation_m), "no elevation: NAN");
    check_true(strcmp(route.waypoint[2].name, "<X>") == 0,
               "CDATA unwrapped and trimmed");
}

static void test_waypoints()
{
    printf("waypoints only\n");

    const char *gpx =
        "<gpx><wpt lat=\"10\" lon=\"20\"/>\n"
        "<wpt lat=\"11\" lon=\"21\"><name>Zażółć gęślą jaźń</name></wpt>\n"
        "<wptExtension lat=\"0\" lon=\"0\"/></gpx>";

    Route route;
    RouteSource source;
    char error[96] = "";

    check_true(parse(gpx, &route, &source, error), "parsed");
    check_true(source == ROUTE_SOURCE_WPT, "from the waypoints");
    check_near(route.count, 2, 0, "a longer tag name is not a waypoint");
    check_true(strcmp(route.waypoint[0].name, "WP1") == 0,
               "no name: numbered");

    /* 15 bytes fit: "Zażółć gę" is 14, and the "ś" after it two more. */
    const char *name = route.waypoint[1].name;
    check_true(strlen(name) <= ROUTE_NAME_BYTES - 1, "cut to fit");
    check_true(strcmp(name, "Zażółć gę") == 0,
               "cut at a character boundary");
}

static void test_refused()
{
    printf("files refused\n");

    Route route;
    RouteSource source;
    char error[96];

    check_true(!parse("<gpx></gpx>", &route, &source, error),
               "no points");
    printf("       %s\n", error);

    check_true(!parse("<gpx><wpt lat=\"91\" lon=\"0\"/></gpx>", &route,
                      &source, error), "latitude out of range");
    printf("       %s\n", error);

    check_true(!parse("<gpx><wpt lon=\"0\"/></gpx>", &route, &source,
                      error), "no latitude");
    printf("       %s\n", error);

    static char track[64 * 1024];
    size_t used = (size_t)snprintf(track, sizeof(track), "<gpx><trk><trkseg>");

    for(int i = 0; i < ROUTE_MAX_WAYPOINTS + 1; i++)
        used += (size_t)snprintf(track + used, sizeof(track) - used,
                                 "<trkpt lat=\"0\" lon=\"%d.0\"/>", i % 180);

    snprintf(track + used, sizeof(track) - used, "</trkseg></trk></gpx>");

    check_true(!parse(track, &route, &source, error), "too many points");
    printf("       %s\n", error);
}

static void test_great_circle()
{
    printf("great circles\n");

    check_near(route_distance_m(0, 0, 1, 0), DEGREE_M, 0.01,
               "a degree of latitude, m");
    check_near(route_distance_m(0, 0, 0, 1), DEGREE_M, 0.01,
               "a degree of longitude on the equator, m");
    check_near(route_distance_m(60, 0, 60, 0.001), DEGREE_M * 0.001 * 0.5,
               0.01, "short and at 60 N: haversine, not cosines");
    check_near(route_bearing_deg(0, 0, 1, 0), 0.0, 1e-9, "north");
    check_near(route_bearing_deg(0, 0, 0, 1), 90.0, 1e-9, "east");
    check_near(route_bearing_deg(0, 0, -1, 0), 180.0, 1e-9, "south");
    check_near(route_bearing_deg(0, 0, 0, -1), 270.0, 1e-9, "west");

    /* Warsaw to Krakow, both airports: 2.088 deg south (232.2 km) and
     * 1.182 deg west at about 51.1 N (82.5 km) - 246.4 km at 200 deg,
     * near enough flat. */
    check_near(route_distance_m(52.1657, 20.9671, 50.0777, 19.7848) / 1000.0,
               246.4, 0.5, "EPWA to EPKK, km");
    check_near(route_bearing_deg(52.1657, 20.9671, 50.0777, 19.7848),
               200.0, 0.5, "EPWA to EPKK, deg");
}

static Route square_route()
{
    /* East along the equator, then north, climbing. */
    Route route;
    route.count = 3;

    const struct { const char *name; double lat, lon; float ele; } points[] = {
        { "A", 0.0, 0.0, 0.0f },
        { "B", 0.0, 1.0, 1000.0f },
        { "C", 1.0, 1.0, NAN },
    };

    for(int i = 0; i < 3; i++)
    {
        snprintf(route.waypoint[i].name, ROUTE_NAME_BYTES, "%s", points[i].name);
        route.waypoint[i].lat_deg = points[i].lat;
        route.waypoint[i].lon_deg = points[i].lon;
        route.waypoint[i].elevation_m = points[i].ele;
    }

    return route;
}

static void test_direct_to()
{
    printf("direct to the first waypoint\n");

    RouteNavigator navigator;
    navigator.setRoute(square_route());

    RouteGuidance guidance;

    check_true(!navigator.update(NULL, &guidance) && !guidance.valid &&
                   guidance.to == 0 && guidance.direct,
               "no fix: nothing to show, but the waypoint is known");

    /* Starts 0.1 deg west of A, at 500 m. */
    const RoutePosition start = { 0.0, -0.1, 500.0f };
    navigator.update(&start, &guidance);

    check_true(guidance.valid, "valid");
    check_near(guidance.distance_m, 0.1 * DEGREE_M, 0.5, "distance to A");
    check_near(guidance.bearing_deg, 90.0, 0.01, "bearing to A");
    check_near(guidance.desired_track_deg, 90.0, 0.01,
               "desired track: from where it started");
    check_near(guidance.cross_track_m, 0.0, 0.01, "on track");

    /* Drifted north half way there: left of the track. */
    const RoutePosition drifted = { 0.01, -0.05, 250.0f };
    navigator.update(&drifted, &guidance);

    check_near(guidance.cross_track_m, -0.01 * DEGREE_M, 1.0,
               "north of an eastbound track: left, negative");
    check_near(guidance.desired_track_deg, 90.0, 0.01,
               "desired track unchanged");
    check_true(guidance.bearing_deg > 90.0 && guidance.bearing_deg < 180.0,
               "bearing to A: to the right of east");
    check_near(guidance.desired_altitude_m, 250.0, 1.0,
               "vertical: from the height at the start");
    check_near(guidance.vertical_deviation_m, 0.0, 1.0, "on the path");
}

static void test_legs()
{
    printf("legs and sequencing\n");

    RouteNavigator navigator;
    navigator.setRoute(square_route());
    navigator.activateLeg(1);

    RouteGuidance guidance;

    /* Half way along A-B, 0.01 deg south, at 600 m. */
    const RoutePosition half = { -0.01, 0.5, 600.0f };
    check_true(!navigator.update(&half, &guidance), "no waypoint passed");

    check_true(guidance.to == 1 && !guidance.direct, "leg A-B");
    check_near(guidance.desired_track_deg, 90.0, 0.01, "desired track east");
    check_near(guidance.cross_track_m, 0.01 * DEGREE_M, 1.0,
               "south of an eastbound leg: right, positive");
    check_near(guidance.along_track_to_go_m, 0.5 * DEGREE_M, 2.0,
               "half the leg to go");
    check_true(guidance.vertical_valid, "both ends have heights");
    check_near(guidance.desired_altitude_m, 500.0, 0.5,
               "half way from 0 to 1000 m");
    check_near(guidance.vertical_deviation_m, 100.0, 0.5,
               "100 m above the path");

    /* Just past abeam B, well off the leg: still passed. */
    const RoutePosition past = { 0.05, 1.0001, 600.0f };
    check_true(navigator.update(&past, &guidance), "B passed");
    check_true(navigator.active() == 2 && guidance.to == 2, "on to C");
    check_near(guidance.desired_track_deg, 0.0, 0.01,
               "leg B-C: north");
    check_true(!guidance.vertical_valid, "C has no height: no path");

    /* Past C: the last stays active. */
    const RoutePosition beyond = { 1.1, 1.0, 600.0f };
    check_true(!navigator.update(&beyond, &guidance) &&
                   navigator.active() == 2,
               "the last waypoint stays");
    check_true(guidance.along_track_to_go_m < 0.0f, "and reads passed");

    /* Direct back to A from here. */
    check_true(navigator.directTo(0), "direct to A");
    navigator.update(&beyond, &guidance);
    check_near(guidance.cross_track_m, 0.0, 0.01, "on the new track");
    check_near(guidance.desired_track_deg,
               route_bearing_deg(1.1, 1.0, 0.0, 0.0), 0.01,
               "its track: straight to A");

    check_true(!navigator.directTo(3) && !navigator.activateLeg(-1),
               "no such waypoint");
}

static void test_short_leg()
{
    printf("two waypoints in one place\n");

    Route route = square_route();
    route.waypoint[1].lat_deg = route.waypoint[0].lat_deg;
    route.waypoint[1].lon_deg = route.waypoint[0].lon_deg;

    RouteNavigator navigator;
    navigator.setRoute(route);
    navigator.activateLeg(1);

    RouteGuidance guidance;
    const RoutePosition near = { 0.0, -0.2, 100.0f };

    check_true(navigator.update(&near, &guidance) && navigator.active() == 2,
               "a leg with no length is passed at once");
}

static void test_flown_past_close()
{
    printf("a waypoint flown past close by, never abeam\n");

    RouteNavigator navigator;
    navigator.setRoute(square_route());

    RouteGuidance guidance;

    /* Direct to A begun 800 m south of it, then off east: at right
     * angles to that leg, short of abeam A. */
    const double m = 1.0 / DEGREE_M;
    const RoutePosition start = { -800.0 * m, 0.0, 0.0f };
    navigator.update(&start, &guidance);

    const RoutePosition far_past = { -900.0 * m, 2000.0 * m, 0.0f };
    check_true(!navigator.update(&far_past, &guidance) &&
                   navigator.active() == 0,
               "well clear: not passed");

    navigator.directTo(0);
    navigator.update(&start, &guidance);

    /* 424 m from A, 300 m short of abeam it. */
    const RoutePosition close_by = { -300.0 * m, 300.0 * m, 0.0f };
    check_true(navigator.update(&close_by, &guidance) &&
                   navigator.active() == 1,
               "500 m of it: passed");
}

static void test_cleared()
{
    printf("route cleared\n");

    RouteNavigator navigator;
    navigator.setRoute(square_route());

    Route empty;
    empty.count = 0;
    navigator.setRoute(empty);

    RouteGuidance guidance;
    const RoutePosition here = { 0.0, 0.0, 0.0f };

    check_true(!navigator.update(&here, &guidance) && !guidance.valid &&
                   guidance.to == -1,
               "no route: nothing");
}

int main()
{
    test_route_among_waypoints();
    test_waypoints();
    test_refused();
    test_great_circle();
    test_direct_to();
    test_legs();
    test_short_leg();
    test_flown_past_close();
    test_cleared();

    printf("\n%d checks, %d failed\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
