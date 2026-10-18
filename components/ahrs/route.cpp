#include "route.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const double EARTH_RADIUS_M = 6371008.8;    /* mean */
static const double PI = 3.14159265358979323846;
static const double DEG = PI / 180.0;

/* A leg shorter than this has no direction: it is passed at once. */
static const double SHORTEST_LEG_M = 1.0;

/* ---- GPX ----------------------------------------------------------------- */

/* The text is not NUL terminated: every search stops at end. */
static const char *find(const char *from, const char *end, const char *what)
{
    const size_t length = strlen(what);

    for(const char *p = from; p + length <= end; p++)
        if(memcmp(p, what, length) == 0)
            return p;

    return NULL;
}

/*
 * The next start tag <name ...> or <name> or <name/>, not a longer name
 * that begins the same way (<wpt> is not <wptExtension>).
 */
static const char *find_tag(const char *from, const char *end,
                            const char *name)
{
    char open[24];
    snprintf(open, sizeof(open), "<%s", name);

    const size_t length = strlen(open);

    for(const char *p = find(from, end, open); p != NULL;
        p = find(p + 1, end, open))
    {
        if(p + length >= end)
            return NULL;

        const char next = p[length];

        if(next == '>' || next == '/' || next == ' ' || next == '\t' ||
           next == '\r' || next == '\n')
            return p;
    }

    return NULL;
}

/* An attribute's value in the start tag [tag, tag_end): name="value". */
static bool attribute(const char *tag, const char *tag_end, const char *name,
                      char *value, size_t size)
{
    const size_t length = strlen(name);

    /* From after the '<', so there is always a character before. */
    for(const char *p = tag + 1; p + length < tag_end; p++)
    {
        const char before = p[-1];

        if((before != ' ' && before != '\t' && before != '\r' &&
            before != '\n') || memcmp(p, name, length) != 0)
            continue;

        const char *q = p + length;

        while(q < tag_end && (*q == ' ' || *q == '\t'))
            q++;

        if(q >= tag_end || *q != '=')
            continue;

        q++;

        while(q < tag_end && (*q == ' ' || *q == '\t'))
            q++;

        if(q >= tag_end || (*q != '"' && *q != '\''))
            continue;

        const char quote = *q++;
        const char *close = (const char *)memchr(q, quote, (size_t)(tag_end - q));

        if(close == NULL || (size_t)(close - q) >= size)
            return false;

        memcpy(value, q, (size_t)(close - q));
        value[close - q] = '\0';
        return true;
    }

    return false;
}

/* Appends one UTF-8 character, whole or not at all. */
static void append(char *out, size_t size, size_t *used, const char *bytes,
                   size_t count)
{
    if(*used + count < size)
    {
        memcpy(out + *used, bytes, count);
        *used += count;
    }
    else
    {
        *used = size;               /* full: nothing more fits */
    }
}

/*
 * An element's text, entities decoded, CDATA unwrapped, spaces at either
 * end dropped, cut at a character boundary to fit.
 */
static bool element_text(const char *from, const char *end, const char *name,
                         char *out, size_t size)
{
    const char *start = find_tag(from, end, name);

    if(start == NULL)
        return false;

    start = (const char *)memchr(start, '>', (size_t)(end - start));

    if(start == NULL || start[-1] == '/')
        return false;

    start++;

    char close[24];
    snprintf(close, sizeof(close), "</%s>", name);

    const char *stop = find(start, end, close);

    if(stop == NULL)
        return false;

    const char *cdata = find(start, stop, "<![CDATA[");

    if(cdata != NULL)
    {
        start = cdata + 9;
        const char *cdata_end = find(start, stop, "]]>");

        if(cdata_end != NULL)
            stop = cdata_end;
    }

    while(start < stop && (unsigned char)*start <= ' ')
        start++;

    while(stop > start && (unsigned char)stop[-1] <= ' ')
        stop--;

    static const struct { const char *entity; char c; } ENTITIES[] = {
        { "&amp;", '&' }, { "&lt;", '<' }, { "&gt;", '>' },
        { "&quot;", '"' }, { "&apos;", '\'' },
    };

    size_t used = 0;

    for(const char *p = start; p < stop && used < size;)
    {
        if(*p == '&')
        {
            bool decoded = false;

            for(const auto &e : ENTITIES)
            {
                const size_t length = strlen(e.entity);

                if(p + length <= stop && memcmp(p, e.entity, length) == 0)
                {
                    append(out, size, &used, &e.c, 1);
                    p += length;
                    decoded = true;
                    break;
                }
            }

            if(decoded)
                continue;
        }

        /* A whole UTF-8 sequence at a time, so a cut never splits one. */
        const unsigned char lead = (unsigned char)*p;
        size_t count = (lead < 0x80) ? 1 : (lead >= 0xF0) ? 4 :
                       (lead >= 0xE0) ? 3 : (lead >= 0xC0) ? 2 : 1;

        if(p + count > stop)
            count = (size_t)(stop - p);

        append(out, size, &used, p, count);
        p += count;
    }

    if(used >= size)
    {
        /* Cut: step back to the last whole character that fits. */
        used = size - 1;

        while(used > 0 && ((unsigned char)out[used] & 0xC0) == 0x80)
            used--;
    }

    out[used] = '\0';
    return used > 0;
}

/* Counts tags up to end. */
static int count_tags(const char *from, const char *end, const char *name)
{
    int count = 0;

    for(const char *p = find_tag(from, end, name); p != NULL;
        p = find_tag(p + 1, end, name))
        count++;

    return count;
}

bool route_parse_gpx(const char *text, size_t length, Route *out,
                     RouteSource *source, char *error, size_t error_size)
{
    const char *end = text + length;

    out->count = 0;

    /*
     * A route says the order to fly in; failing that, the waypoints, in
     * file order; failing that, a track. Only the first route: a file
     * with several is a library of them, not one to join together.
     */
    const char *tag = NULL;
    const char *close = NULL;
    const char *from = text;
    const char *until = end;

    if(find_tag(text, end, "rtept") != NULL)
    {
        tag = "rtept";
        close = "</rtept>";
        *source = ROUTE_SOURCE_RTE;

        from = find_tag(text, end, "rte");
        const char *rte_end = (from != NULL) ? find(from, end, "</rte>") : NULL;

        if(from == NULL)
            from = text;

        if(rte_end != NULL)
            until = rte_end;
    }
    else if(find_tag(text, end, "wpt") != NULL)
    {
        tag = "wpt";
        close = "</wpt>";
        *source = ROUTE_SOURCE_WPT;
    }
    else if(find_tag(text, end, "trkpt") != NULL)
    {
        tag = "trkpt";
        close = "</trkpt>";
        *source = ROUTE_SOURCE_TRK;
    }
    else
    {
        snprintf(error, error_size, "no <rtept>, <wpt> or <trkpt> in the file");
        return false;
    }

    const int total = count_tags(from, until, tag);

    if(total > ROUTE_MAX_WAYPOINTS)
    {
        snprintf(error, error_size,
                 "%d points, at most %d: %s", total, ROUTE_MAX_WAYPOINTS,
                 (*source == ROUTE_SOURCE_TRK)
                     ? "a track is too detailed, make a route of it"
                     : "split the route");
        return false;
    }

    for(const char *p = find_tag(from, until, tag); p != NULL;
        p = find_tag(p + 1, until, tag))
    {
        const char *tag_end = (const char *)memchr(p, '>', (size_t)(until - p));

        if(tag_end == NULL)
            break;

        RouteWaypoint &waypoint = out->waypoint[out->count];
        const int number = out->count + 1;

        char lat[32];
        char lon[32];

        if(!attribute(p, tag_end, "lat", lat, sizeof(lat)) ||
           !attribute(p, tag_end, "lon", lon, sizeof(lon)))
        {
            snprintf(error, error_size, "point %d has no lat or lon", number);
            return false;
        }

        char *lat_end;
        char *lon_end;
        waypoint.lat_deg = strtod(lat, &lat_end);
        waypoint.lon_deg = strtod(lon, &lon_end);

        if(lat_end == lat || lon_end == lon ||
           !(fabs(waypoint.lat_deg) <= 90.0) ||
           !(fabs(waypoint.lon_deg) <= 180.0))
        {
            snprintf(error, error_size, "point %d: lat \"%s\" lon \"%s\"",
                     number, lat, lon);
            return false;
        }

        /* Its children, unless it closed itself: <wpt lat=.. lon=../> */
        const char *body_end = tag_end;

        if(tag_end[-1] != '/')
        {
            const char *element_end = find(tag_end, until, close);
            body_end = (element_end != NULL) ? element_end : tag_end;
        }

        if(!element_text(tag_end, body_end, "name", waypoint.name,
                         sizeof(waypoint.name)))
            snprintf(waypoint.name, sizeof(waypoint.name), "WP%d", number);

        char elevation[32];
        waypoint.elevation_m = NAN;

        if(element_text(tag_end, body_end, "ele", elevation,
                        sizeof(elevation)))
        {
            char *elevation_end;
            const double value = strtod(elevation, &elevation_end);

            if(elevation_end != elevation && isfinite(value))
                waypoint.elevation_m = (float)value;
        }

        out->count++;
    }

    if(out->count == 0)
    {
        snprintf(error, error_size, "no usable points in the file");
        return false;
    }

    return true;
}

/* ---- great circles ------------------------------------------------------- */

/* Angular distance, rad - haversine, sound at short range. */
static double angular_distance(double lat1, double lon1,
                               double lat2, double lon2)
{
    const double s_lat = sin((lat2 - lat1) * 0.5);
    const double s_lon = sin((lon2 - lon1) * 0.5);
    const double a = s_lat * s_lat + cos(lat1) * cos(lat2) * s_lon * s_lon;

    return 2.0 * atan2(sqrt(a), sqrt(fmax(0.0, 1.0 - a)));
}

/* Initial bearing, rad, -pi to pi. */
static double bearing(double lat1, double lon1, double lat2, double lon2)
{
    const double d_lon = lon2 - lon1;

    return atan2(sin(d_lon) * cos(lat2),
                 cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(d_lon));
}

static float degrees_360(double radians)
{
    double d = fmod(radians / DEG, 360.0);

    if(d < 0.0)
        d += 360.0;

    return (float)d;
}

double route_distance_m(double lat1_deg, double lon1_deg,
                        double lat2_deg, double lon2_deg)
{
    return EARTH_RADIUS_M * angular_distance(lat1_deg * DEG, lon1_deg * DEG,
                                             lat2_deg * DEG, lon2_deg * DEG);
}

double route_bearing_deg(double lat1_deg, double lon1_deg,
                         double lat2_deg, double lon2_deg)
{
    return degrees_360(bearing(lat1_deg * DEG, lon1_deg * DEG,
                               lat2_deg * DEG, lon2_deg * DEG));
}

/* ---- navigator ----------------------------------------------------------- */

RouteNavigator::RouteNavigator()
    : active_(-1), direct_(false), origin_pending_(false)
{
    route_.count = 0;
    origin_.lat_deg = 0.0;
    origin_.lon_deg = 0.0;
    origin_.altitude_m = NAN;
}

void RouteNavigator::setRoute(const Route &route)
{
    route_ = route;

    if(route_.count > 0)
        directTo(0);
    else
        active_ = -1;
}

bool RouteNavigator::directTo(int index)
{
    if(index < 0 || index >= route_.count)
        return false;

    active_ = index;
    direct_ = true;
    origin_pending_ = true;
    return true;
}

bool RouteNavigator::activateLeg(int index)
{
    if(index == 0)
        return directTo(0);

    if(index < 0 || index >= route_.count)
        return false;

    active_ = index;
    direct_ = false;
    origin_pending_ = false;
    return true;
}

void RouteNavigator::guide(const RoutePosition &fix, RouteGuidance *out) const
{
    const RouteWaypoint &to = route_.waypoint[active_];

    RoutePosition from = origin_;

    if(!direct_)
    {
        const RouteWaypoint &previous = route_.waypoint[active_ - 1];
        from.lat_deg = previous.lat_deg;
        from.lon_deg = previous.lon_deg;
        from.altitude_m = previous.elevation_m;
    }

    const double lat1 = from.lat_deg * DEG, lon1 = from.lon_deg * DEG;
    const double lat2 = to.lat_deg * DEG, lon2 = to.lon_deg * DEG;
    const double lat3 = fix.lat_deg * DEG, lon3 = fix.lon_deg * DEG;

    const double to_go = angular_distance(lat3, lon3, lat2, lon2);
    const double to_bearing = bearing(lat3, lon3, lat2, lon2);

    out->distance_m = EARTH_RADIUS_M * to_go;
    out->bearing_deg = degrees_360(to_bearing);

    const double leg = angular_distance(lat1, lon1, lat2, lon2);
    double along = 0.0;

    if(leg * EARTH_RADIUS_M < SHORTEST_LEG_M)
    {
        /* No direction to be off: steer straight at it. */
        out->desired_track_deg = out->bearing_deg;
        out->cross_track_m = 0.0f;
        out->along_track_to_go_m = 0.0f;
        along = leg;
    }
    else
    {
        /*
         * The spherical right triangle from the leg's start: d13 from
         * there to the aircraft at angle a off the leg. Then
         *   sin(xt) = sin(d13) sin(a)
         *   tan(at) = tan(d13) cos(a)   - kept as atan2, sound near 0
         */
        const double leg_track = bearing(lat1, lon1, lat2, lon2);
        const double d13 = angular_distance(lat1, lon1, lat3, lon3);
        const double a = bearing(lat1, lon1, lat3, lon3) - leg_track;

        const double cross = asin(sin(d13) * sin(a));
        along = atan2(sin(d13) * cos(a), cos(d13));

        out->desired_track_deg = degrees_360(leg_track);
        out->cross_track_m = (float)(EARTH_RADIUS_M * cross);
        out->along_track_to_go_m = (float)(EARTH_RADIUS_M * (leg - along));
    }

    out->vertical_valid = isfinite(from.altitude_m) &&
                          isfinite(to.elevation_m) &&
                          isfinite(fix.altitude_m);

    if(out->vertical_valid)
    {
        double fraction = (leg > 0.0) ? along / leg : 1.0;

        if(fraction < 0.0)
            fraction = 0.0;
        if(fraction > 1.0)
            fraction = 1.0;

        out->desired_altitude_m = (float)(from.altitude_m +
            (to.elevation_m - from.altitude_m) * fraction);
        out->vertical_deviation_m = fix.altitude_m - out->desired_altitude_m;
    }
    else
    {
        out->desired_altitude_m = NAN;
        out->vertical_deviation_m = NAN;
    }
}

bool RouteNavigator::update(const RoutePosition *fix, RouteGuidance *out)
{
    memset(out, 0, sizeof(*out));
    out->to = active_;
    out->direct = direct_;

    if(active_ < 0 || fix == NULL)
        return false;

    if(direct_ && origin_pending_)
    {
        /* Where "direct to" was pressed - with the height then, so the
         * vertical path runs from there to the waypoint's. */
        origin_ = *fix;
        origin_pending_ = false;
    }

    guide(*fix, out);
    out->valid = true;

    const bool reached = out->along_track_to_go_m <= 0.0f ||
                         out->distance_m < ROUTE_ARRIVAL_RADIUS_M;

    if(!reached || active_ >= route_.count - 1)
        return false;

    /* Abeam or close: on to the next leg, and guidance along that. */
    active_++;
    direct_ = false;

    out->to = active_;
    out->direct = false;
    guide(*fix, out);

    return true;
}
