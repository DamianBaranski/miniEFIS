#include "route_store.h"
#include "can_bus.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <math.h>
#include <new>
#include <stdio.h>
#include <string.h>

static const char *TAG = "route";

static const char *NVS_NAMESPACE = "route";
static const char *NVS_KEY_WAYPOINTS = "wpts";
static const char *NVS_KEY_ACTIVE = "active";

static const TickType_t UPDATE_PERIOD = pdMS_TO_TICKS(200);
static const uint32_t TASK_STACK_BYTES = 4096;
static const UBaseType_t TASK_PRIORITY = 2;
static const BaseType_t TASK_CORE = 0;

/* Below this the GPS track wanders with the noise: the last good one is
 * kept instead. 2 m/s is about 4 kt - taxiing, not flying. */
static const float MIN_TRACK_SPEED_MS = 2.0f;

/* GPS fix types (docs/CAN_PROTOCOL.md, 0x201). */
static const uint8_t FIX_2D = 2;
static const uint8_t FIX_3D = 3;

/* A waypoint as saved: positions in 1e-7 deg, as on the CAN bus. */
typedef struct {
    char name[ROUTE_NAME_BYTES];
    int32_t lat_e7;
    int32_t lon_e7;
    float elevation_m;
} stored_waypoint_t;

static SemaphoreHandle_t s_lock = NULL;

/* Guarded by s_lock. */
static RouteNavigator *s_navigator = NULL;
static route_status_t s_status;

static void lock(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
}

static void unlock(void)
{
    xSemaphoreGive(s_lock);
}

/* ---- NVS ------------------------------------------------------------------- */

static void save_active(int active, bool direct)
{
    nvs_handle_t handle;

    if(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK)
        return;

    /* Direct to: -1 - index, so a restart knows to go straight there
     * again rather than back onto a leg it never flew. */
    const int32_t value = direct ? -1 - active : active;

    if(nvs_set_i32(handle, NVS_KEY_ACTIVE, value) == ESP_OK)
        nvs_commit(handle);

    nvs_close(handle);
}

static bool save_route(const Route &route)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);

    if(err != ESP_OK)
        return false;

    if(route.count == 0)
    {
        nvs_erase_all(handle);
    }
    else
    {
        const size_t bytes = sizeof(stored_waypoint_t) * (size_t)route.count;
        stored_waypoint_t *stored = (stored_waypoint_t *)
            heap_caps_calloc(1, bytes, MALLOC_CAP_SPIRAM);

        if(stored == NULL)
        {
            nvs_close(handle);
            return false;
        }

        for(int i = 0; i < route.count; i++)
        {
            const RouteWaypoint &w = route.waypoint[i];

            memcpy(stored[i].name, w.name, sizeof(stored[i].name));
            stored[i].lat_e7 = (int32_t)lround(w.lat_deg * 1e7);
            stored[i].lon_e7 = (int32_t)lround(w.lon_deg * 1e7);
            stored[i].elevation_m = w.elevation_m;
        }

        err = nvs_set_blob(handle, NVS_KEY_WAYPOINTS, stored, bytes);
        free(stored);
    }

    if(err == ESP_OK)
        err = nvs_commit(handle);

    nvs_close(handle);

    if(err != ESP_OK)
        ESP_LOGE(TAG, "could not save the route: %s", esp_err_to_name(err));

    return err == ESP_OK;
}

/* The saved route into s_navigator, on the leg it was on. */
static void load_saved(void)
{
    nvs_handle_t handle;

    if(nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK)
        return;

    size_t bytes = 0;
    Route *route = (Route *)heap_caps_calloc(1, sizeof(Route), MALLOC_CAP_SPIRAM);
    stored_waypoint_t *stored = NULL;

    if(route != NULL &&
       nvs_get_blob(handle, NVS_KEY_WAYPOINTS, NULL, &bytes) == ESP_OK &&
       bytes > 0 && bytes % sizeof(stored_waypoint_t) == 0 &&
       bytes / sizeof(stored_waypoint_t) <= ROUTE_MAX_WAYPOINTS)
    {
        stored = (stored_waypoint_t *)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);

        if(stored != NULL &&
           nvs_get_blob(handle, NVS_KEY_WAYPOINTS, stored, &bytes) == ESP_OK)
        {
            route->count = (int)(bytes / sizeof(stored_waypoint_t));

            for(int i = 0; i < route->count; i++)
            {
                RouteWaypoint &w = route->waypoint[i];

                memcpy(w.name, stored[i].name, sizeof(w.name));
                w.name[sizeof(w.name) - 1] = '\0';
                w.lat_deg = stored[i].lat_e7 * 1e-7;
                w.lon_deg = stored[i].lon_e7 * 1e-7;
                w.elevation_m = stored[i].elevation_m;
            }
        }
    }

    int32_t active = 0;
    nvs_get_i32(handle, NVS_KEY_ACTIVE, &active);
    nvs_close(handle);

    if(route != NULL && route->count > 0)
    {
        s_navigator->setRoute(*route);

        if(active < 0)
            s_navigator->directTo(-1 - active);
        else
            s_navigator->activateLeg(active);

        ESP_LOGI(TAG, "%d waypoints, to %d %s", route->count,
                 s_navigator->active() + 1,
                 route->waypoint[s_navigator->active()].name);
    }

    free(stored);
    free(route);
}

/* ---- guidance -------------------------------------------------------------- */

/* With s_lock held: what the screen shows of the route itself. */
static void route_changed(void)
{
    s_status.version++;
    s_status.count = s_navigator->route().count;

    const int active = s_navigator->active();

    if(active >= 0)
        snprintf(s_status.to_name, sizeof(s_status.to_name), "%s",
                 s_navigator->route().waypoint[active].name);
    else
        s_status.to_name[0] = '\0';
}

static void update(void)
{
    /* The whole bus: too big for the stack, and only this task uses it. */
    static can_data_t can;
    can_bus_get(&can);

    const int64_t now_us = esp_timer_get_time();
    const can_gps_t &gps = can.gps;

    const bool velocity_fresh =
        can_fresh(&gps.velocity_stamp, now_us, CAN_PERIOD_GPS_MS);

    const bool have_fix =
        velocity_fresh && gps.fix_type >= FIX_2D &&
        can_fresh(&gps.position_stamp, now_us, CAN_PERIOD_GPS_MS) &&
        isfinite(gps.latitude_deg) && isfinite(gps.longitude_deg);

    const bool have_altitude =
        have_fix && gps.fix_type >= FIX_3D &&
        can_fresh(&gps.altitude_stamp, now_us, CAN_PERIOD_GPS_MS) &&
        isfinite(gps.altitude_msl_m);

    RoutePosition fix;
    fix.lat_deg = gps.latitude_deg;
    fix.lon_deg = gps.longitude_deg;
    fix.altitude_m = have_altitude ? gps.altitude_msl_m : NAN;

    lock();

    RouteGuidance guidance;
    const bool passed = s_navigator->update(have_fix ? &fix : NULL, &guidance);

    s_status.guidance = guidance;
    s_status.have_fix = have_fix;
    s_status.altitude_m = fix.altitude_m;
    s_status.ground_speed_ms = (have_fix && isfinite(gps.ground_speed_ms))
                                   ? gps.ground_speed_ms : NAN;
    s_status.have_track = have_fix && isfinite(gps.track_deg) &&
                          gps.ground_speed_ms >= MIN_TRACK_SPEED_MS;

    if(s_status.have_track)
        s_status.track_deg = gps.track_deg;

    if(passed)
        route_changed();

    const int active = s_navigator->active();
    const bool direct = s_navigator->direct();

    char name[ROUTE_NAME_BYTES];
    memcpy(name, s_status.to_name, sizeof(name));

    unlock();

    if(passed)
    {
        ESP_LOGI(TAG, "waypoint passed, now to %d %s", active + 1, name);
        save_active(active, direct);
    }
}

static void route_task(void *argument)
{
    (void)argument;

    TickType_t wake = xTaskGetTickCount();

    for(;;)
    {
        update();
        vTaskDelayUntil(&wake, UPDATE_PERIOD);
    }
}

/* ---- API ------------------------------------------------------------------- */

void route_store_start(void)
{
    s_lock = xSemaphoreCreateMutex();

    /* 4 KB of waypoints: PSRAM, not the internal heap. */
    void *memory = heap_caps_malloc(sizeof(RouteNavigator), MALLOC_CAP_SPIRAM);

    if(s_lock == NULL || memory == NULL)
    {
        ESP_LOGE(TAG, "out of memory");
        return;
    }

    s_navigator = new(memory) RouteNavigator();
    s_status.track_deg = 0.0f;
    s_status.altitude_m = NAN;
    s_status.ground_speed_ms = NAN;

    load_saved();
    route_changed();

    if(xTaskCreatePinnedToCore(route_task, "route", TASK_STACK_BYTES, NULL,
                               TASK_PRIORITY, NULL, TASK_CORE) != pdPASS)
        ESP_LOGE(TAG, "could not start the guidance task");
}

bool route_store_load_gpx(const char *text, size_t length, char *message,
                          size_t message_size)
{
    if(s_navigator == NULL)
    {
        snprintf(message, message_size, "route store not running");
        return false;
    }

    Route *route = (Route *)heap_caps_calloc(1, sizeof(Route), MALLOC_CAP_SPIRAM);

    if(route == NULL)
    {
        snprintf(message, message_size, "out of memory");
        return false;
    }

    RouteSource source;
    char error[96];

    if(!route_parse_gpx(text, length, route, &source, error, sizeof(error)))
    {
        snprintf(message, message_size, "%s", error);
        free(route);
        return false;
    }

    if(!save_route(*route))
    {
        snprintf(message, message_size, "could not save it (NVS full?)");
        free(route);
        return false;
    }

    save_active(0, true);

    lock();
    s_navigator->setRoute(*route);
    route_changed();
    unlock();

    static const char *const SOURCES[] = { "route", "waypoints", "track" };

    snprintf(message, message_size, "%d waypoints from the file's %s, "
             "direct to %s", route->count, SOURCES[source],
             route->waypoint[0].name);

    ESP_LOGI(TAG, "%s", message);

    free(route);
    return true;
}

void route_store_clear(void)
{
    if(s_navigator == NULL)
        return;

    Route *empty = (Route *)heap_caps_calloc(1, sizeof(Route), MALLOC_CAP_SPIRAM);

    if(empty == NULL)
        return;

    save_route(*empty);

    lock();
    s_navigator->setRoute(*empty);
    route_changed();
    unlock();

    free(empty);
    ESP_LOGI(TAG, "route cleared");
}

bool route_store_direct_to(int index)
{
    if(s_navigator == NULL)
        return false;

    char name[ROUTE_NAME_BYTES];

    lock();
    const bool done = s_navigator->directTo(index);

    if(done)
        route_changed();

    memcpy(name, s_status.to_name, sizeof(name));
    unlock();

    if(done)
    {
        save_active(index, true);
        ESP_LOGI(TAG, "direct to %d %s", index + 1, name);
    }

    return done;
}

bool route_store_activate_leg(int index)
{
    if(s_navigator == NULL)
        return false;

    char name[ROUTE_NAME_BYTES];

    lock();
    const bool done = s_navigator->activateLeg(index);
    const bool direct = s_navigator->direct();

    if(done)
        route_changed();

    memcpy(name, s_status.to_name, sizeof(name));
    unlock();

    if(done)
    {
        save_active(index, direct);
        ESP_LOGI(TAG, "leg to %d %s", index + 1, name);
    }

    return done;
}

void route_store_status(route_status_t *out)
{
    if(s_navigator == NULL)
    {
        memset(out, 0, sizeof(*out));
        out->guidance.to = -1;
        return;
    }

    lock();
    *out = s_status;
    unlock();
}

void route_store_get(Route *out)
{
    out->count = 0;

    if(s_navigator == NULL)
        return;

    lock();
    *out = s_navigator->route();
    unlock();
}
