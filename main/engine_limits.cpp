#include "engine_limits.h"
#include "app_settings.h"

#include "esp_log.h"
#include "lvgl.h"
#include "nvs.h"

#include <math.h>
#include <string.h>

static const char *TAG = "engine";

#define NONE NAN

/*
 * The usual limits for each type, from its operator's manual. A
 * particular installation may differ: check before relying on them.
 */
static const engine_limits_t LIMITS[ENGINE_PRESET_COUNT] = {
    /* Rotax 912 ULS: oil 0.8-7 bar, normal 2-5; oil 50-130 C, normal
     * 90-110; CHT 135 C; coolant 120 C; EGT 880 C; fuel 0.15-0.5 bar;
     * 5800 rpm for 5 minutes, 5500 continuous. */
    {
        "Rotax 912", 4, true,
        500.0f, 1000.0f, 1000.0f, { {
        /* rpm */           { 0, 6000, NONE, NONE, 5500, 5800 },
        /* manifold */      { 10, 110, NONE, NONE, NONE, NONE },
        /* oil pressure */  { 0, 800, 80, 200, 500, 700 },
        /* oil temp */      { 20, 150, NONE, 90, 110, 130 },
        /* CHT */           { 20, 150, NONE, NONE, 120, 135 },
        /* coolant */       { 20, 150, NONE, NONE, 110, 120 },
        /* EGT */           { 300, 950, NONE, NONE, 850, 880 },
        /* fuel pressure */ { 0, 60, 15, NONE, 40, 50 },
        /* fuel flow */     { 0, 30, NONE, NONE, NONE, NONE },
        /* fuel */          { 0, 70, 10, 20, NONE, NONE },
        /* volts */         { 10, 16, 11.5f, 12.4f, 14.8f, 15.5f },
        /* amps */          { -30, 30, NONE, -1, NONE, NONE },
        /* carb */          { -20, 40, NONE, NONE, NONE, NONE },
    } } },
    /* Lycoming O-320: oil 25-100 psi, normal 60-90; oil 245 F; CHT
     * 500 F; fuel 0.5-8 psi; 2700 rpm. */
    {
        "Lycoming O-320", 4, false,
        100.0f, 500.0f, 100.0f, { {
        /* rpm */           { 0, 3000, NONE, NONE, NONE, 2700 },
        /* manifold */      { 10, 110, NONE, NONE, NONE, NONE },
        /* oil pressure */  { 0, 800, 172, 414, 621, 690 },
        /* oil temp */      { 20, 140, NONE, 38, NONE, 118 },
        /* CHT */           { 50, 280, NONE, NONE, 224, 260 },
        /* coolant */       { 20, 150, NONE, NONE, NONE, NONE },
        /* EGT */           { 300, 900, NONE, NONE, NONE, NONE },
        /* fuel pressure */ { 0, 70, 3.4f, NONE, NONE, 55 },
        /* fuel flow */     { 0, 50, NONE, NONE, NONE, NONE },
        /* fuel */          { 0, 150, 15, 37, NONE, NONE },
        /* volts */         { 10, 16, 11.5f, 12.4f, 14.8f, 15.5f },
        /* amps */          { -60, 60, NONE, -1, NONE, NONE },
        /* carb */          { -20, 40, NONE, NONE, NONE, NONE },
    } } },
    /* Lycoming IO-540: oil 25-115 psi, normal 55-95; oil 245 F; CHT
     * 500 F; injected, fuel 14-45 psi; 2700 rpm. */
    {
        "Lycoming IO-540", 6, false,
        100.0f, 500.0f, 100.0f, { {
        /* rpm */           { 0, 3000, NONE, NONE, NONE, 2700 },
        /* manifold */      { 10, 110, NONE, NONE, NONE, NONE },
        /* oil pressure */  { 0, 900, 172, 379, 655, 793 },
        /* oil temp */      { 20, 140, NONE, 38, NONE, 118 },
        /* CHT */           { 50, 280, NONE, NONE, 224, 260 },
        /* coolant */       { 20, 150, NONE, NONE, NONE, NONE },
        /* EGT */           { 300, 900, NONE, NONE, NONE, NONE },
        /* fuel pressure */ { 0, 350, 97, NONE, NONE, 310 },
        /* fuel flow */     { 0, 100, NONE, NONE, NONE, NONE },
        /* fuel */          { 0, 340, 30, 80, NONE, NONE },
        /* volts */         { 10, 16, 11.5f, 12.4f, 14.8f, 15.5f },
        /* amps */          { -60, 60, NONE, -1, NONE, NONE },
        /* carb */          { -20, 40, NONE, NONE, NONE, NONE },
    } } },
};

/* How each value is shown and set: pressures in bar, the rest as sent. */
static const engine_param_info_t PARAMS[ENGINE_PARAM_COUNT] = {
    { "RPM",             "rpm",  1.0f,   50.0f, 0 },
    { "Manifold",        "kPa",  1.0f,    1.0f, 0 },
    { "Oil pressure",    "bar",  0.01f,   0.1f, 1 },
    { "Oil temperature", "\xC2\xB0" "C", 1.0f, 1.0f, 0 },
    { "CHT",             "\xC2\xB0" "C", 1.0f, 1.0f, 0 },
    { "Coolant",         "\xC2\xB0" "C", 1.0f, 1.0f, 0 },
    { "EGT",             "\xC2\xB0" "C", 1.0f, 5.0f, 0 },
    { "Fuel pressure",   "bar",  0.01f,  0.01f, 2 },
    { "Fuel flow",       "L/h",  1.0f,    1.0f, 0 },
    { "Fuel quantity",   "L",    1.0f,    5.0f, 0 },
    { "Volts",           "V",    1.0f,    0.1f, 1 },
    { "Battery amps",    "A",    1.0f,    1.0f, 0 },
    { "Carb air",        "\xC2\xB0" "C", 1.0f, 1.0f, 0 },
};

/* ---- the custom engine ------------------------------------------------------ */

static const char *NVS_NAMESPACE = "engine";
static const char *NVS_KEY_CUSTOM = "custom";

/* Bump when Stored changes: an older one is then ignored. */
static const uint32_t STORED_VERSION = 1;

/* How long after the last change to save it: a held +/- steps often. */
static const uint32_t SAVE_DELAY_MS = 3000;

struct Stored
{
    uint32_t version;
    uint8_t cylinders;
    uint8_t coolant;
    uint8_t reserved[2];
    engine_range_t range[ENGINE_PARAM_COUNT];
};

static engine_limits_t s_custom;
static lv_timer_t *s_save_timer = NULL;

/* The dial's ticks for the custom engine's top RPM: a Rotax's 1000s
 * past 4000, a Lycoming's 500s below. */
static void derive_dial(engine_limits_t *l)
{
    if(l->rpm.max > 4000.0f)
    {
        l->rpm_tick = 500.0f;
        l->rpm_label = 1000.0f;
        l->rpm_label_divisor = 1000.0f;
    }
    else
    {
        l->rpm_tick = 100.0f;
        l->rpm_label = 500.0f;
        l->rpm_label_divisor = 100.0f;
    }
}

void engine_custom_copy(engine_type_t from)
{
    const engine_type_t preset =
        (from < ENGINE_PRESET_COUNT) ? from : ENGINE_TYPE_ROTAX_912;

    s_custom = LIMITS[preset];
    s_custom.name = "Custom";
    derive_dial(&s_custom);
}

static void save_now(lv_timer_t *timer)
{
    (void)timer;
    s_save_timer = NULL;            /* one shot: LVGL deletes it */

    Stored stored;
    memset(&stored, 0, sizeof(stored));

    stored.version = STORED_VERSION;
    stored.cylinders = (uint8_t)s_custom.cylinders;
    stored.coolant = s_custom.coolant ? 1 : 0;
    memcpy(stored.range, s_custom.range, sizeof(stored.range));

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);

    if(err == ESP_OK)
    {
        err = nvs_set_blob(handle, NVS_KEY_CUSTOM, &stored, sizeof(stored));

        if(err == ESP_OK)
            err = nvs_commit(handle);

        nvs_close(handle);
    }

    if(err != ESP_OK)
        ESP_LOGE(TAG, "could not save the custom engine: %s",
                 esp_err_to_name(err));
}

void engine_custom_changed(void)
{
    derive_dial(&s_custom);

    if(s_save_timer != NULL)
    {
        lv_timer_reset(s_save_timer);
        return;
    }

    s_save_timer = lv_timer_create(save_now, SAVE_DELAY_MS, NULL);
    lv_timer_set_repeat_count(s_save_timer, 1);
}

engine_limits_t *engine_custom(void)
{
    return &s_custom;
}

void engine_limits_init(void)
{
    /* Until one is saved: a Rotax's. */
    engine_custom_copy(ENGINE_TYPE_ROTAX_912);

    nvs_handle_t handle;

    if(nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK)
        return;

    Stored stored;
    size_t length = sizeof(stored);

    const esp_err_t err = nvs_get_blob(handle, NVS_KEY_CUSTOM, &stored,
                                       &length);
    nvs_close(handle);

    if(err != ESP_OK || length != sizeof(stored) ||
       stored.version != STORED_VERSION)
        return;

    if(stored.cylinders >= 1 && stored.cylinders <= ENGINE_MAX_CYLINDERS)
        s_custom.cylinders = stored.cylinders;

    s_custom.coolant = stored.coolant != 0;
    memcpy(s_custom.range, stored.range, sizeof(s_custom.range));
    derive_dial(&s_custom);

    ESP_LOGI(TAG, "custom engine: %d cylinders, %s cooled, %.0f rpm max",
             s_custom.cylinders, s_custom.coolant ? "liquid" : "air",
             s_custom.rpm.max);
}

const engine_param_info_t *engine_param_info(engine_param_t param)
{
    return &PARAMS[(param < ENGINE_PARAM_COUNT) ? param : 0];
}

static const engine_limits_t *limits_of(engine_type_t type)
{
    if(type == ENGINE_TYPE_CUSTOM)
        return &s_custom;

    return &LIMITS[(type < ENGINE_PRESET_COUNT) ? type : 0];
}

/* ---- what is shown ---------------------------------------------------------- */

static const char *const ITEM_NAMES[ENGINE_ITEM_COUNT] = {
    "Oil pressure",
    "Oil temperature",
    "CHT (hottest)",
    "EGT (hottest)",
    "Coolant",
    "Fuel pressure",
    "Fuel flow",
    "Fuel quantity",
    "Volts",
    "Battery amps",
    "Carb air",
};

#define ITEM(i) ((uint16_t)(1u << (i)))

const char *engine_item_name(engine_item_t item)
{
    return (item < ENGINE_ITEM_COUNT) ? ITEM_NAMES[item] : "";
}

uint16_t engine_default_items(engine_type_t type)
{
    const uint16_t common =
        ITEM(ENGINE_ITEM_OIL_PRESSURE) | ITEM(ENGINE_ITEM_OIL_TEMP) |
        ITEM(ENGINE_ITEM_CHT) | ITEM(ENGINE_ITEM_FUEL_PRESSURE) |
        ITEM(ENGINE_ITEM_FUEL_FLOW) | ITEM(ENGINE_ITEM_FUEL_QUANTITY) |
        ITEM(ENGINE_ITEM_VOLTS);

    /* Liquid cooled: the coolant; air cooled: the hottest EGT. */
    const bool coolant = limits_of(type)->coolant;

    return common | ITEM(coolant ? ENGINE_ITEM_COOLANT : ENGINE_ITEM_EGT);
}

int engine_cylinders(void)
{
    return engine_limits()->cylinders;
}

uint16_t engine_items(void)
{
    const uint16_t set = app_settings().engine_items &
                         (uint16_t)((1u << ENGINE_ITEM_COUNT) - 1u);

    if(set != 0)
        return set;

    const uint8_t type = app_settings().engine_type;

    return engine_default_items((engine_type_t)
                                    ((type < ENGINE_TYPE_COUNT) ? type : 0));
}

const char *engine_type_name(engine_type_t type)
{
    return (type < ENGINE_TYPE_COUNT) ? limits_of(type)->name : "";
}

const engine_limits_t *engine_limits(void)
{
    const uint8_t type = app_settings().engine_type;

    return limits_of((engine_type_t)type);
}

engine_zone_t engine_zone(const engine_range_t *range, float value)
{
    if(isnan(value))
        return ENGINE_ZONE_NONE;

    /* Comparisons with NAN are false: a limit that isn't never trips. */
    if(value < range->red_low || value > range->red_high)
        return ENGINE_ZONE_RED;

    if(value < range->amber_low || value > range->amber_high)
        return ENGINE_ZONE_AMBER;

    return ENGINE_ZONE_GREEN;
}

float engine_fraction(const engine_range_t *range, float value)
{
    if(isnan(value))
        return NAN;

    const float fraction = (value - range->min) / (range->max - range->min);

    return fminf(1.0f, fmaxf(0.0f, fraction));
}
