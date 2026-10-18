#pragma once

#include <stdint.h>

/*
 * The user's settings, kept in NVS so they survive a restart: the screen
 * last shown, the screens hidden, the engine type, units, the altimeter setting (QNH),
 * the backlight and whether Wi-Fi debug is on.
 * The magnetometer's mounting and calibration are kept separately, by
 * magnetometer.cpp.
 *
 * Changes are saved a few seconds after the last one, not at once:
 * holding the QNH button down, or flicking through screens, would
 * otherwise write flash on every step. A restart inside that window
 * loses only the last few seconds of changes.
 *
 * All functions from the LVGL task, apart from app_settings_init().
 */

typedef enum {
    UNITS_METRIC = 0,       /* m, m/s */
    UNITS_AVIATION          /* ft, ft/min */
} units_t;

typedef struct {
    uint8_t screen;         /* screen_id_t */
    units_t units;
    float qnh_pa;
    uint8_t brightness;     /* percent */
    bool wifi_debug;        /* wifi_debug.h */
    uint8_t hidden_screens; /* bit per screen_id_t: left out of the swipes and menu */
    uint8_t engine_type;    /* engine_type_t, engine_limits.h */
    uint16_t engine_items;  /* bit per engine_item_t shown; 0: the type's */
    uint8_t engine_flags;   /* ENGINE_HIDE_* */
} app_settings_t;

/* Load, or fall back to defaults. Call once nvs_flash_init() has run. */
void app_settings_init(void);

app_settings_t app_settings(void);

void app_settings_set_screen(uint8_t screen);
void app_settings_set_units(units_t units);
void app_settings_set_qnh(float pascals);
void app_settings_set_brightness(uint8_t percent);
void app_settings_set_wifi_debug(bool enabled);
void app_settings_set_hidden_screens(uint8_t mask);
void app_settings_set_engine_type(uint8_t type);
void app_settings_set_engine_items(uint16_t items);
void app_settings_set_engine_flags(uint8_t flags);
