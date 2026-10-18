#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * The engine's limits: the green, amber and red ranges the engine
 * screens colour each value by, and their scales.
 *
 * The engine unit on the CAN bus only sends values (docs/CAN_PROTOCOL.md);
 * whether one is out of range is decided here, on the EFIS. The engine
 * is picked in settings from a few types, or is a custom one whose every
 * scale and limit is set there and kept in NVS. The types' limits are
 * the usual ones - check them against the engine's own manual before
 * relying on them.
 *
 * Values in the units on the wire: kPa, deg C, L, L/h, V, A.
 *
 * Also what the engine screens show, as set in their menu: which
 * values get a bar, and the rest of ENGINE_HIDE_*.
 *
 * From the LVGL task, apart from engine_limits_init().
 */

typedef enum {
    ENGINE_TYPE_ROTAX_912 = 0,
    ENGINE_TYPE_O320,
    ENGINE_TYPE_IO540,
    ENGINE_TYPE_CUSTOM,
    ENGINE_TYPE_COUNT
} engine_type_t;

/* The types with limits of their own, not set by the user. */
#define ENGINE_PRESET_COUNT ENGINE_TYPE_CUSTOM

typedef enum {
    ENGINE_ZONE_NONE = 0,       /* no value */
    ENGINE_ZONE_GREEN,
    ENGINE_ZONE_AMBER,
    ENGINE_ZONE_RED,
} engine_zone_t;

/*
 * A value's scale and ranges: red below red_low, amber below amber_low,
 * amber above amber_high, red above red_high, green between. NAN for a
 * limit the value hasn't.
 */
typedef struct {
    float min;
    float max;
    float red_low;
    float amber_low;
    float amber_high;
    float red_high;
} engine_range_t;

/* The values with a range, in engine_limits_t's order. */
typedef enum {
    ENGINE_PARAM_RPM = 0,
    ENGINE_PARAM_MANIFOLD,
    ENGINE_PARAM_OIL_PRESSURE,
    ENGINE_PARAM_OIL_TEMP,
    ENGINE_PARAM_CHT,
    ENGINE_PARAM_COOLANT,
    ENGINE_PARAM_EGT,
    ENGINE_PARAM_FUEL_PRESSURE,
    ENGINE_PARAM_FUEL_FLOW,
    ENGINE_PARAM_FUEL,
    ENGINE_PARAM_VOLTS,
    ENGINE_PARAM_AMPS,
    ENGINE_PARAM_CARB,
    ENGINE_PARAM_COUNT
} engine_param_t;

typedef struct {
    const char *name;
    int cylinders;
    bool coolant;               /* liquid cooled: a coolant temperature */

    /* The RPM dial: a tick every rpm_tick, a number every rpm_label
     * showing rpm / rpm_label_divisor. */
    float rpm_tick;
    float rpm_label;
    float rpm_label_divisor;

    /* Indexed by engine_param_t. */
    union {
        engine_range_t range[ENGINE_PARAM_COUNT];
        struct {
            engine_range_t rpm;
            engine_range_t manifold_kpa;
            engine_range_t oil_pressure_kpa;
            engine_range_t oil_temp_c;
            engine_range_t cht_c;
            engine_range_t coolant_c;
            engine_range_t egt_c;
            engine_range_t fuel_pressure_kpa;
            engine_range_t fuel_flow_lph;
            engine_range_t fuel_l;      /* all tanks together */
            engine_range_t volts;
            engine_range_t amps;        /* battery, charging positive */
            engine_range_t carb_c;      /* carburettor or intake air */
        };
    };
} engine_limits_t;

/*
 * How a value is shown and set: shown = value * scale, with decimals
 * places, in unit; set in steps of step, as shown.
 */
typedef struct {
    const char *name;
    const char *unit;
    float scale;
    float step;
    int decimals;
} engine_param_info_t;

const engine_param_info_t *engine_param_info(engine_param_t param);

/* Loads the custom engine. Once NVS is up. */
void engine_limits_init(void);

const char *engine_type_name(engine_type_t type);

/* The engine picked in settings. */
const engine_limits_t *engine_limits(void);

/*
 * The custom engine, to change. After a change, engine_custom_changed()
 * - it works out the RPM dial's ticks again and saves, a few seconds
 * after the last change.
 */
engine_limits_t *engine_custom(void);
void engine_custom_changed(void);

/* The custom engine made a copy of a type, to start from. */
void engine_custom_copy(engine_type_t from);

/* The values that can have a bar on the engine screen, in its order. */
typedef enum {
    ENGINE_ITEM_OIL_PRESSURE = 0,
    ENGINE_ITEM_OIL_TEMP,
    ENGINE_ITEM_CHT,            /* the hottest */
    ENGINE_ITEM_EGT,            /* the hottest */
    ENGINE_ITEM_COOLANT,
    ENGINE_ITEM_FUEL_PRESSURE,
    ENGINE_ITEM_FUEL_FLOW,
    ENGINE_ITEM_FUEL_QUANTITY,
    ENGINE_ITEM_VOLTS,
    ENGINE_ITEM_AMPS,
    ENGINE_ITEM_CARB,
    ENGINE_ITEM_COUNT
} engine_item_t;

/* The engine screen has room for this many bars. */
#define ENGINE_MAX_ITEMS 8

/* app_settings().engine_flags */
#define ENGINE_HIDE_EGT_BARS    0x01    /* cylinders screen */
#define ENGINE_HIDE_CHT_BARS    0x02
#define ENGINE_HIDE_MANIFOLD    0x04    /* under the RPM */

#define ENGINE_MAX_CYLINDERS 6

const char *engine_item_name(engine_item_t item);

/* The engine type's cylinders: a preset's, or as set for the custom
 * engine. */
int engine_cylinders(void);

/* As set in the engine screens' menu, or the engine type's own. */
uint16_t engine_items(void);    /* bit per engine_item_t */

/* The type's own items: its usual eight. */
uint16_t engine_default_items(engine_type_t type);

/* Which range a value is in; ENGINE_ZONE_NONE for NAN. */
engine_zone_t engine_zone(const engine_range_t *range, float value);

/* Where a value is on its scale, 0-1, clamped; NAN for NAN. */
float engine_fraction(const engine_range_t *range, float value);
