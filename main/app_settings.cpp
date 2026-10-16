#include "app_settings.h"
#include "baro_bmp280.h"

#include "esp_log.h"
#include "lvgl.h"
#include "nvs.h"

#include <string.h>

static const char *TAG = "settings";

static const char *NVS_NAMESPACE = "app";
static const char *NVS_KEY = "settings";

/* Bump when the layout of Stored changes; an old blob is then ignored
 * and the defaults used. */
static const uint32_t STORED_VERSION = 1;

/* How long after the last change to write it out. */
static const uint32_t SAVE_DELAY_MS = 3000;

static const uint8_t BRIGHTNESS_MIN = 5;

struct Stored
{
    uint32_t version;
    uint8_t screen;
    uint8_t units;
    uint8_t brightness;
    uint8_t wifi_debug;     /* was reserved, 0: older blobs read as off */
    float qnh_pa;
};

static app_settings_t s_settings = {
    0,                              /* the horizon */
    UNITS_METRIC,
    BARO_SEA_LEVEL_STANDARD_PA,
    80,
    false
};

static lv_timer_t *s_save_timer = NULL;

static void save_now(lv_timer_t *timer)
{
    (void)timer;
    s_save_timer = NULL;            /* one shot: LVGL deletes it */

    Stored stored;
    memset(&stored, 0, sizeof(stored));

    stored.version = STORED_VERSION;
    stored.screen = s_settings.screen;
    stored.units = (uint8_t)s_settings.units;
    stored.brightness = s_settings.brightness;
    stored.wifi_debug = s_settings.wifi_debug ? 1 : 0;
    stored.qnh_pa = s_settings.qnh_pa;

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);

    if(err == ESP_OK)
    {
        err = nvs_set_blob(handle, NVS_KEY, &stored, sizeof(stored));

        if(err == ESP_OK)
            err = nvs_commit(handle);

        nvs_close(handle);
    }

    if(err != ESP_OK)
        ESP_LOGE(TAG, "could not save: %s", esp_err_to_name(err));
}

static void save_later(void)
{
    if(s_save_timer != NULL)
    {
        lv_timer_reset(s_save_timer);
        return;
    }

    s_save_timer = lv_timer_create(save_now, SAVE_DELAY_MS, NULL);
    lv_timer_set_repeat_count(s_save_timer, 1);
}

void app_settings_init(void)
{
    nvs_handle_t handle;

    if(nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK)
    {
        ESP_LOGI(TAG, "none saved, using defaults");
        return;
    }

    Stored stored;
    size_t length = sizeof(stored);

    const esp_err_t err = nvs_get_blob(handle, NVS_KEY, &stored, &length);

    nvs_close(handle);

    if(err != ESP_OK || length != sizeof(stored) ||
       stored.version != STORED_VERSION)
    {
        ESP_LOGW(TAG, "saved settings unreadable, using defaults");
        return;
    }

    s_settings.screen = stored.screen;
    s_settings.units = (stored.units == UNITS_AVIATION) ? UNITS_AVIATION
                                                        : UNITS_METRIC;
    s_settings.wifi_debug = (stored.wifi_debug == 1);

    if(stored.brightness >= BRIGHTNESS_MIN && stored.brightness <= 100)
        s_settings.brightness = stored.brightness;

    /* The same window navigation accepts. */
    if(stored.qnh_pa >= 87000.0f && stored.qnh_pa <= 108500.0f)
        s_settings.qnh_pa = stored.qnh_pa;

    ESP_LOGI(TAG, "screen %u, %s, QNH %.1f hPa, backlight %u%%, "
             "Wi-Fi debug %s",
             (unsigned)s_settings.screen,
             s_settings.units == UNITS_AVIATION ? "ft" : "m",
             s_settings.qnh_pa / 100.0f, (unsigned)s_settings.brightness,
             s_settings.wifi_debug ? "on" : "off");
}

app_settings_t app_settings(void)
{
    return s_settings;
}

void app_settings_set_screen(uint8_t screen)
{
    if(screen == s_settings.screen)
        return;

    s_settings.screen = screen;
    save_later();
}

void app_settings_set_units(units_t units)
{
    if(units == s_settings.units)
        return;

    s_settings.units = units;
    save_later();
}

void app_settings_set_qnh(float pascals)
{
    if(pascals == s_settings.qnh_pa)
        return;

    s_settings.qnh_pa = pascals;
    save_later();
}

void app_settings_set_brightness(uint8_t percent)
{
    if(percent < BRIGHTNESS_MIN)
        percent = BRIGHTNESS_MIN;
    if(percent > 100)
        percent = 100;

    if(percent == s_settings.brightness)
        return;

    s_settings.brightness = percent;
    save_later();
}

void app_settings_set_wifi_debug(bool enabled)
{
    if(enabled == s_settings.wifi_debug)
        return;

    s_settings.wifi_debug = enabled;
    save_later();
}
