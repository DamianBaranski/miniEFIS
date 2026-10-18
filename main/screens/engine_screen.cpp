#include "engine_screen.h"
#include "app_settings.h"
#include "can_bus.h"
#include "engine_limits.h"
#include "fast_draw.h"
#include "fonts.h"

#include "esp_timer.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Engine instruments needn't be quicker, and each redraw is the whole
 * screen. */
static const uint32_t UPDATE_PERIOD_MS = 200;

static const uint32_t TEXT_COLOUR = 0xFFFFFF;
static const uint32_t CAPTION_COLOUR = 0xA0A8B0;
static const uint32_t NONE_COLOUR = 0x606870;
static const uint32_t TRACK_COLOUR = 0x2A3440;
static const uint32_t GREEN_COLOUR = 0x20B040;
static const uint32_t AMBER_COLOUR = 0xFFB000;
static const uint32_t RED_COLOUR = 0xFF3030;
static const uint32_t EGT_COLOUR = 0x60A8FF;
static const uint32_t HOTTEST_COLOUR = 0xFFD400;

/* The RPM dial: half a circle over the top, from the left. */
static const float DIAL_CX = 240.0f;
static const float DIAL_CY = 222.0f;
static const float DIAL_BAND_RADIUS = 166.0f;       /* middle of the band */
static const uint16_t DIAL_BAND_WIDTH = 12;
static const float DIAL_TICK_OUTER = 158.0f;
static const float DIAL_TICK_SHORT = 8.0f;
static const float DIAL_TICK_LONG = 16.0f;
static const float DIAL_LABEL_RADIUS = 126.0f;
/* Outside the numbers, so it never hides one. */
static const float NEEDLE_INNER = 140.0f;
static const float NEEDLE_OUTER = 172.0f;

/* The bars under it: two columns of four. */
static const int CELLS = 8;
static const float CELL_X[2] = { 92.0f, 252.0f };
static const float CELL_WIDTH = 136.0f;
static const float CELL_Y0 = 252.0f;                /* first text line */
static const float CELL_PITCH = 48.0f;
static const float CELL_BAR_DY = 17.0f;             /* text line to bar */
static const uint16_t CELL_BAR_HEIGHT = 6;

/* The cylinder bars. */
static const float CYL_PITCH = 56.0f;
static const float CYL_BAR_WIDTH = 18.0f;
static const float CYL_TOP = 108.0f;
static const float CYL_BOTTOM = 328.0f;

static const float KPA_PER_BAR = 100.0f;

typedef struct {
    char caption[12];
    char value[12];
    engine_zone_t zone;
    float fraction;             /* NAN: no mark */
    const engine_range_t *range;
} cell_t;

typedef struct {
    bool have_data;
    float rpm_fraction;         /* NAN: no needle */
    engine_zone_t rpm_zone;
    char rpm[8];
    char manifold[20];
    cell_t cell[CELLS];
} engine_view_t;

typedef struct {
    char egt[6];
    char cht[6];
    float egt_fraction;
    float cht_fraction;
    engine_zone_t cht_zone;
} cylinder_t;

typedef struct {
    bool have_data;
    bool show_egt;
    bool show_cht;
    int cylinders;
    int hottest;                /* by CHT, or EGT without CHT; -1 none */
    cylinder_t cylinder[CAN_CYLINDERS];
    char top[40];
    char endurance[40];
    char engine_time[24];
} cylinders_view_t;

static lv_obj_t *s_engine_screen = NULL;
static lv_obj_t *s_engine_canvas = NULL;
static engine_view_t s_engine;

static lv_obj_t *s_cylinders_screen = NULL;
static lv_obj_t *s_cylinders_canvas = NULL;
static cylinders_view_t s_cylinders;

/* ---- drawing ------------------------------------------------------------------ */

static lv_color_t zone_colour(engine_zone_t zone)
{
    switch(zone)
    {
    case ENGINE_ZONE_GREEN: return lv_color_hex(GREEN_COLOUR);
    case ENGINE_ZONE_AMBER: return lv_color_hex(AMBER_COLOUR);
    case ENGINE_ZONE_RED:   return lv_color_hex(RED_COLOUR);
    default:                return lv_color_hex(TRACK_COLOUR);
    }
}

/* What a value's number is shown in: white when all is well. */
static lv_color_t text_colour(engine_zone_t zone)
{
    switch(zone)
    {
    case ENGINE_ZONE_AMBER: return lv_color_hex(AMBER_COLOUR);
    case ENGINE_ZONE_RED:   return lv_color_hex(RED_COLOUR);
    case ENGINE_ZONE_NONE:  return lv_color_hex(NONE_COLOUR);
    default:                return lv_color_hex(TEXT_COLOUR);
    }
}

/* A range with no limits at all is just a scale: no green. */
static bool has_limits(const engine_range_t *r)
{
    return !isnan(r->red_low) || !isnan(r->amber_low) ||
           !isnan(r->amber_high) || !isnan(r->red_high);
}

/* The colour of the scale at a fraction along it. */
static engine_zone_t zone_at(const engine_range_t *r, float fraction)
{
    if(!has_limits(r))
        return ENGINE_ZONE_NONE;

    return engine_zone(r, r->min + (r->max - r->min) * fraction);
}

static void dial_point(float fraction, float radius, float *x, float *y)
{
    const float angle = (float)M_PI * (1.0f - fraction);

    *x = DIAL_CX + radius * cosf(angle);
    *y = DIAL_CY - radius * sinf(angle);
}

static void draw_dial(lv_draw_ctx_t *ctx, float ox, float oy)
{
    const engine_limits_t *limits = engine_limits();
    const engine_range_t *r = &limits->rpm;

    /* The ranges round the arc, a degree at a time. */
    for(int i = 0; i < 180; i++)
    {
        const float f1 = i / 180.0f;
        const float f2 = (i + 1) / 180.0f;
        float x1, y1, x2, y2;

        dial_point(f1, DIAL_BAND_RADIUS, &x1, &y1);
        dial_point(f2, DIAL_BAND_RADIUS, &x2, &y2);
        fast_draw_line(ctx, ox + x1, oy + y1, ox + x2, oy + y2,
                       zone_colour(zone_at(r, (f1 + f2) * 0.5f)),
                       DIAL_BAND_WIDTH);
    }

    const lv_color_t white = lv_color_white();
    /* Down: a custom scale may end between ticks, and no tick or
     * number goes past its end. */
    const int ticks = (int)floorf((r->max - r->min) / limits->rpm_tick +
                                  0.001f);
    const int per_label = (int)lroundf(limits->rpm_label / limits->rpm_tick);

    for(int i = 0; i <= ticks; i++)
    {
        const float rpm = r->min + limits->rpm_tick * (float)i;
        const float f = engine_fraction(r, rpm);
        const bool labelled = (i % per_label) == 0;
        float x1, y1, x2, y2;

        dial_point(f, DIAL_TICK_OUTER, &x1, &y1);
        dial_point(f, DIAL_TICK_OUTER -
                          (labelled ? DIAL_TICK_LONG : DIAL_TICK_SHORT),
                   &x2, &y2);
        fast_draw_line(ctx, ox + x1, oy + y1, ox + x2, oy + y2, white,
                       labelled ? 3 : 2);

        if(labelled)
        {
            char text[8];
            float x, y;

            snprintf(text, sizeof(text), "%.0f",
                     rpm / limits->rpm_label_divisor);
            dial_point(f, DIAL_LABEL_RADIUS, &x, &y);
            fast_draw_text(ctx, ox + x, oy + y, text, FONT_LARGE, white);
        }
    }

    if(!isnan(s_engine.rpm_fraction))
    {
        float x1, y1, x2, y2;

        dial_point(s_engine.rpm_fraction, NEEDLE_INNER, &x1, &y1);
        dial_point(s_engine.rpm_fraction, NEEDLE_OUTER, &x2, &y2);
        /* White always: amber on the amber band would vanish. The
         * number carries the colour. */
        fast_draw_line(ctx, ox + x1, oy + y1, ox + x2, oy + y2,
                       lv_color_white(), 7);
    }

    fast_draw_text(ctx, ox + DIAL_CX, oy + DIAL_CY - 52.0f, "RPM",
                   FONT_SMALL, lv_color_hex(CAPTION_COLOUR));
    fast_draw_text(ctx, ox + DIAL_CX, oy + DIAL_CY - 26.0f, s_engine.rpm,
                   FONT_HUGE, text_colour(s_engine.rpm_zone));
    fast_draw_text(ctx, ox + DIAL_CX, oy + DIAL_CY + 2.0f, s_engine.manifold,
                   FONT_SMALL, lv_color_hex(CAPTION_COLOUR));
}

/* A scale's ranges as a strip from x0 to x0 + width. */
static void draw_strip(lv_draw_ctx_t *ctx, const engine_range_t *r,
                       float x0, float y, float width, uint16_t height)
{
    int start = 0;
    engine_zone_t zone = zone_at(r, 0.0f);
    const int pixels = (int)width;

    for(int i = 1; i <= pixels; i++)
    {
        const engine_zone_t here =
            (i < pixels) ? zone_at(r, (i + 0.5f) / width) : ENGINE_ZONE_NONE;

        if(i < pixels && here == zone)
            continue;

        fast_draw_line(ctx, x0 + start, y, x0 + i - 1, y, zone_colour(zone),
                       height);
        start = i;
        zone = here;
    }
}

static void draw_cell(lv_draw_ctx_t *ctx, const cell_t *cell, float x0,
                      float y)
{
    fast_draw_text(ctx, x0 + 36.0f, y, cell->caption, FONT_SMALL,
                   lv_color_hex(CAPTION_COLOUR));
    fast_draw_text(ctx, x0 + 104.0f, y, cell->value, FONT_LARGE,
                   text_colour(cell->zone));

    const float bar_y = y + CELL_BAR_DY;

    draw_strip(ctx, cell->range, x0, bar_y, CELL_WIDTH, CELL_BAR_HEIGHT);

    if(!isnan(cell->fraction))
    {
        const float x = x0 + cell->fraction * (CELL_WIDTH - 1.0f);

        fast_draw_line(ctx, x, bar_y - 8.0f, x, bar_y + 6.0f,
                       lv_color_white(), 3);
    }
}

/* When the engine unit says nothing: at y, over what it hides. */
static void draw_no_data(lv_draw_ctx_t *ctx, float ox, float oy, float y)
{
    fast_draw_line(ctx, ox + 120.0f, oy + y, ox + 360.0f, oy + y,
                   lv_color_black(), 40);
    fast_draw_text(ctx, ox + 240.0f, oy + y, "NO ENGINE DATA",
                   FONT_LARGE, lv_color_hex(RED_COLOUR));
}

static void draw_engine(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *ctx = lv_event_get_draw_ctx(e);

    lv_area_t area;
    lv_obj_get_coords(obj, &area);

    const float ox = (float)area.x1;
    const float oy = (float)area.y1;

    draw_dial(ctx, ox, oy);

    /* Filled in order; fewer items than cells leaves the last empty. */
    for(int i = 0; i < CELLS && s_engine.cell[i].range != NULL; i++)
        draw_cell(ctx, &s_engine.cell[i], ox + CELL_X[i % 2],
                  oy + CELL_Y0 + CELL_PITCH * (float)(i / 2));

    /* In the dial, over the dashes for RPM. */
    if(!s_engine.have_data)
        draw_no_data(ctx, ox, oy, DIAL_CY - 26.0f);
}

static void draw_cylinders(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *ctx = lv_event_get_draw_ctx(e);

    lv_area_t area;
    lv_obj_get_coords(obj, &area);

    const float ox = (float)area.x1;
    const float oy = (float)area.y1;
    const engine_limits_t *limits = engine_limits();
    const cylinders_view_t &v = s_cylinders;

    const lv_color_t caption = lv_color_hex(CAPTION_COLOUR);
    const lv_color_t track = lv_color_hex(TRACK_COLOUR);

    fast_draw_text(ctx, ox + 240.0f, oy + 52.0f, v.top, FONT_SMALL, caption);
    const bool both = v.show_egt && v.show_cht;

    /* The legend: whichever bars there are. */
    if(v.show_egt)
        fast_draw_text(ctx, ox + (both ? 200.0f : 240.0f), oy + 82.0f, "EGT",
                       FONT_BODY, lv_color_hex(EGT_COLOUR));
    if(v.show_cht)
        fast_draw_text(ctx, ox + (both ? 280.0f : 240.0f), oy + 82.0f, "CHT",
                       FONT_BODY, lv_color_hex(GREEN_COLOUR));

    const float left = 240.0f - CYL_PITCH * (float)v.cylinders * 0.5f;
    const float height = CYL_BOTTOM - CYL_TOP;

    /* One kind of bar: wider, in the middle of its column. */
    const float width = both ? CYL_BAR_WIDTH : CYL_BAR_WIDTH * 1.8f;
    const float offset = both ? CYL_BAR_WIDTH * 0.5f + 2.0f : 0.0f;

    /* The CHT limit across the CHT bars. */
    const float cht_red = engine_fraction(&limits->cht_c,
                                          limits->cht_c.red_high);

    /* The rows of numbers under the bars, as many as there are bars. */
    const float egt_row = 370.0f;
    const float cht_row = v.show_egt ? 394.0f : 370.0f;

    for(int i = 0; i < v.cylinders; i++)
    {
        const cylinder_t &c = v.cylinder[i];
        const float mid = ox + left + CYL_PITCH * ((float)i + 0.5f);
        const float egt_x = mid - offset;
        const float cht_x = mid + offset;
        const float bottom = oy + CYL_BOTTOM;

        if(v.show_egt)
        {
            fast_draw_line(ctx, egt_x, bottom, egt_x, bottom - height, track,
                           (uint16_t)width);

            if(!isnan(c.egt_fraction))
                fast_draw_line(ctx, egt_x, bottom, egt_x,
                               bottom - height * c.egt_fraction,
                               lv_color_hex(EGT_COLOUR), (uint16_t)width);

            fast_draw_text(ctx, mid, oy + egt_row, c.egt, FONT_SMALL,
                           lv_color_hex(c.egt[0] == '-' ? NONE_COLOUR
                                                        : EGT_COLOUR));
        }

        if(v.show_cht)
        {
            fast_draw_line(ctx, cht_x, bottom, cht_x, bottom - height, track,
                           (uint16_t)width);

            if(!isnan(c.cht_fraction))
                fast_draw_line(ctx, cht_x, bottom, cht_x,
                               bottom - height * c.cht_fraction,
                               zone_colour(c.cht_zone), (uint16_t)width);

            if(!isnan(cht_red))
            {
                const float y = bottom - height * cht_red;

                fast_draw_line(ctx, cht_x - width * 0.5f - 2.0f, y,
                               cht_x + width * 0.5f + 2.0f, y,
                               lv_color_hex(RED_COLOUR), 3);
            }

            fast_draw_text(ctx, mid, oy + cht_row, c.cht, FONT_SMALL,
                           text_colour(c.cht_zone));
        }

        char number[12];
        snprintf(number, sizeof(number), "%d", i + 1);

        fast_draw_text(ctx, mid, oy + 346.0f, number, FONT_BODY,
                       lv_color_hex(i == v.hottest ? HOTTEST_COLOUR
                                                   : TEXT_COLOUR));
    }

    fast_draw_text(ctx, ox + 240.0f, oy + 424.0f, v.endurance, FONT_SMALL,
                   caption);
    fast_draw_text(ctx, ox + 240.0f, oy + 448.0f, v.engine_time, FONT_SMALL,
                   caption);

    if(!v.have_data)
        draw_no_data(ctx, ox, oy, 218.0f);
}

/* ---- the values ----------------------------------------------------------------- */

static void set_cell(cell_t *cell, const char *caption,
                     const engine_range_t *range, float value,
                     float shown, const char *format)
{
    memset(cell, 0, sizeof(*cell));
    snprintf(cell->caption, sizeof(cell->caption), "%s", caption);
    cell->range = range;
    cell->zone = engine_zone(range, value);
    cell->fraction = engine_fraction(range, value);

    if(isnan(value))
        snprintf(cell->value, sizeof(cell->value), "---");
    else
        snprintf(cell->value, sizeof(cell->value), format, shown);
}

/* The hottest of the cylinders' values; NAN if none. */
static float hottest(const float *values, int count, int *which)
{
    float best = NAN;

    if(which != NULL)
        *which = -1;

    for(int i = 0; i < count; i++)
    {
        if(!isnan(values[i]) && (isnan(best) || values[i] > best))
        {
            best = values[i];
            if(which != NULL)
                *which = i;
        }
    }

    return best;
}

/* A value, or NAN once its message has stopped coming. */
static float fresh(float value, bool is_fresh)
{
    return is_fresh ? value : NAN;
}

typedef struct {
    can_engine_t e;
    bool fast, slow, egt, cht, cyl56, fuel, totals;
} engine_now_t;

static void read_engine(engine_now_t *out)
{
    static can_data_t data;     /* large: kept off the LVGL task's stack */

    can_bus_get(&data);

    const int64_t now = esp_timer_get_time();
    const can_engine_t &e = data.engine[0];

    out->e = e;
    out->fast = can_fresh(&e.fast_stamp, now, CAN_PERIOD_ENGINE_FAST_MS);
    out->slow = can_fresh(&e.slow_stamp, now, CAN_PERIOD_ENGINE_SLOW_MS);
    out->egt = can_fresh(&e.egt_stamp, now, CAN_PERIOD_ENGINE_SLOW_MS);
    out->cht = can_fresh(&e.cht_stamp, now, CAN_PERIOD_ENGINE_SLOW_MS);
    out->cyl56 = can_fresh(&e.cyl56_stamp, now, CAN_PERIOD_ENGINE_SLOW_MS);
    out->fuel = can_fresh(&e.fuel_stamp, now, CAN_PERIOD_FUEL_MS);
    out->totals = can_fresh(&e.totals_stamp, now, CAN_PERIOD_TOTALS_MS);
}

/* Each cylinder's EGT and CHT, NAN where not fresh. */
static void cylinder_values(const engine_now_t &n, float *egt, float *cht)
{
    for(int i = 0; i < CAN_CYLINDERS; i++)
    {
        const bool high = i >= 4;

        egt[i] = fresh(n.e.egt_c[i], high ? n.cyl56 : n.egt);
        cht[i] = fresh(n.e.cht_c[i], high ? n.cyl56 : n.cht);
    }
}

static float fuel_total(const engine_now_t &n)
{
    if(!n.fuel)
        return NAN;

    const float left = n.e.fuel_left_l;
    const float right = n.e.fuel_right_l;

    if(isnan(left))
        return right;
    if(isnan(right))
        return left;
    return left + right;
}

static void make_engine_view(const engine_now_t &n, engine_view_t *v)
{
    const engine_limits_t *l = engine_limits();
    const can_engine_t &e = n.e;

    memset(v, 0, sizeof(*v));
    v->have_data = n.fast || n.slow || n.egt || n.cht || n.fuel;

    const float rpm = fresh(e.rpm, n.fast);

    v->rpm_fraction = engine_fraction(&l->rpm, rpm);
    v->rpm_zone = engine_zone(&l->rpm, rpm);

    if(isnan(rpm))
        snprintf(v->rpm, sizeof(v->rpm), "---");
    else
        snprintf(v->rpm, sizeof(v->rpm), "%.0f", rpm);

    const float manifold = fresh(e.manifold_kpa, n.fast);

    if(!isnan(manifold) &&
       (app_settings().engine_flags & ENGINE_HIDE_MANIFOLD) == 0)
        snprintf(v->manifold, sizeof(v->manifold), "MAP %.1f kPa", manifold);

    float egt[CAN_CYLINDERS], cht[CAN_CYLINDERS];
    cylinder_values(n, egt, cht);

    const int cylinders = engine_cylinders();
    const uint16_t items = engine_items();
    int next = 0;

    for(int item = 0; item < ENGINE_ITEM_COUNT && next < CELLS; item++)
    {
        if((items & (1u << item)) == 0)
            continue;

        cell_t *cell = &v->cell[next++];

        switch((engine_item_t)item)
        {
        case ENGINE_ITEM_OIL_PRESSURE:
        {
            const float value = fresh(e.oil_pressure_kpa, n.fast);
            set_cell(cell, "OIL bar", &l->oil_pressure_kpa, value,
                     value / KPA_PER_BAR, "%.1f");
            break;
        }
        case ENGINE_ITEM_OIL_TEMP:
            set_cell(cell, "OIL \xC2\xB0" "C", &l->oil_temp_c,
                     fresh(e.oil_temp_c, n.slow), e.oil_temp_c, "%.0f");
            break;

        case ENGINE_ITEM_CHT:
        {
            const float value = hottest(cht, cylinders, NULL);
            set_cell(cell, "CHT \xC2\xB0" "C", &l->cht_c, value, value,
                     "%.0f");
            break;
        }
        case ENGINE_ITEM_EGT:
        {
            const float value = hottest(egt, cylinders, NULL);
            set_cell(cell, "EGT \xC2\xB0" "C", &l->egt_c, value, value,
                     "%.0f");
            break;
        }
        case ENGINE_ITEM_COOLANT:
            set_cell(cell, "COOL \xC2\xB0" "C", &l->coolant_c,
                     fresh(e.coolant_temp_c, n.slow), e.coolant_temp_c,
                     "%.0f");
            break;

        case ENGINE_ITEM_FUEL_PRESSURE:
        {
            /* Low fuel pressures read in tenths of a bar: two places. */
            const float value = fresh(e.fuel_pressure_kpa, n.fast);
            set_cell(cell, "FUEL bar", &l->fuel_pressure_kpa, value,
                     value / KPA_PER_BAR,
                     (l->fuel_pressure_kpa.max < 100.0f) ? "%.2f" : "%.1f");
            break;
        }
        case ENGINE_ITEM_FUEL_FLOW:
            set_cell(cell, "FLOW L/h", &l->fuel_flow_lph,
                     fresh(e.fuel_flow_lph, n.slow), e.fuel_flow_lph, "%.1f");
            break;

        case ENGINE_ITEM_FUEL_QUANTITY:
        {
            const float value = fuel_total(n);
            set_cell(cell, "FUEL L", &l->fuel_l, value, value, "%.0f");
            break;
        }
        case ENGINE_ITEM_VOLTS:
            set_cell(cell, "VOLTS", &l->volts, fresh(e.bus_volts, n.slow),
                     e.bus_volts, "%.1f");
            break;

        case ENGINE_ITEM_AMPS:
            set_cell(cell, "AMPS", &l->amps, fresh(e.battery_amps, n.fuel),
                     e.battery_amps, "%+.1f");
            break;

        case ENGINE_ITEM_CARB:
            set_cell(cell, "CARB \xC2\xB0" "C", &l->carb_c,
                     fresh(e.intake_temp_c, n.fuel), e.intake_temp_c,
                     "%.0f");
            break;

        default:
            break;
        }
    }
}

static void format_temperature(char *text, size_t size, float value)
{
    if(isnan(value))
        snprintf(text, size, "--");
    else
        snprintf(text, size, "%.0f", value);
}

static void make_cylinders_view(const engine_now_t &n, cylinders_view_t *v)
{
    const engine_limits_t *l = engine_limits();
    const can_engine_t &e = n.e;

    memset(v, 0, sizeof(*v));
    v->have_data = n.fast || n.slow || n.egt || n.cht || n.fuel;
    v->cylinders = engine_cylinders();

    const uint8_t flags = app_settings().engine_flags;

    v->show_egt = (flags & ENGINE_HIDE_EGT_BARS) == 0;
    v->show_cht = (flags & ENGINE_HIDE_CHT_BARS) == 0 || !v->show_egt;

    float egt[CAN_CYLINDERS], cht[CAN_CYLINDERS];
    cylinder_values(n, egt, cht);
    hottest(v->show_cht ? cht : egt, v->cylinders, &v->hottest);

    for(int i = 0; i < v->cylinders; i++)
    {
        cylinder_t &c = v->cylinder[i];

        format_temperature(c.egt, sizeof(c.egt), egt[i]);
        format_temperature(c.cht, sizeof(c.cht), cht[i]);
        c.egt_fraction = engine_fraction(&l->egt_c, egt[i]);
        c.cht_fraction = engine_fraction(&l->cht_c, cht[i]);
        c.cht_zone = engine_zone(&l->cht_c, cht[i]);
    }

    /* The carburettor air - icing - and whether the battery charges. */
    char carb[16] = "";
    char amps[20] = "";

    if(n.fuel && !isnan(e.intake_temp_c))
        snprintf(carb, sizeof(carb), "CARB %.0f\xC2\xB0" "C", e.intake_temp_c);

    if(n.fuel && !isnan(e.battery_amps))
        snprintf(amps, sizeof(amps), "BATT %+.1f A", e.battery_amps);

    snprintf(v->top, sizeof(v->top), "%s%s%s", carb,
             (carb[0] != '\0' && amps[0] != '\0') ? "   " : "", amps);

    if(n.totals && !isnan(e.endurance_min))
    {
        const int minutes = (int)e.endurance_min;

        snprintf(v->endurance, sizeof(v->endurance),
                 "ENDUR %d:%02d   USED %.1f L", minutes / 60, minutes % 60,
                 e.fuel_used_l);
    }
    else if(n.totals && !isnan(e.fuel_used_l))
    {
        snprintf(v->endurance, sizeof(v->endurance), "USED %.1f L",
                 e.fuel_used_l);
    }

    if(n.totals && e.engine_time_s != 0xFFFFFFFFu)
        snprintf(v->engine_time, sizeof(v->engine_time), "ENGINE %.1f h",
                 (double)e.engine_time_s / 3600.0);
}

static bool menu_is_open(void);
static void menu_close(void);

static void update(lv_timer_t *timer)
{
    (void)timer;

    lv_obj_t *const active = lv_scr_act();

    if(active != s_engine_screen && active != s_cylinders_screen)
    {
        if(menu_is_open())
            menu_close();
        return;
    }

    engine_now_t now;
    read_engine(&now);

    /* Redrawn only when what is shown changes; the views are memset, so
     * the comparison sees no stray padding. */
    if(active == s_engine_screen)
    {
        engine_view_t view;
        make_engine_view(now, &view);

        if(memcmp(&view, &s_engine, sizeof(view)) != 0)
        {
            s_engine = view;
            lv_obj_invalidate(s_engine_canvas);
        }
    }
    else
    {
        cylinders_view_t view;
        make_cylinders_view(now, &view);

        if(memcmp(&view, &s_cylinders, sizeof(view)) != 0)
        {
            s_cylinders = view;
            lv_obj_invalidate(s_cylinders_canvas);
        }
    }
}

/* ---- building it ---------------------------------------------------------------- */

static lv_obj_t *make_canvas(lv_obj_t *screen, lv_event_cb_t draw)
{
    lv_obj_t *canvas = lv_obj_create(screen);

    lv_obj_remove_style_all(canvas);
    lv_obj_set_size(canvas, LV_PCT(100), LV_PCT(100));
    lv_obj_center(canvas);

    /* Taps go through to the screen, where the double tap is caught. */
    lv_obj_clear_flag(canvas, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(canvas, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(canvas, draw, LV_EVENT_DRAW_MAIN, NULL);

    return canvas;
}

static void start_timer(void)
{
    static bool started = false;

    if(started)
        return;

    started = true;
    lv_timer_create(update, UPDATE_PERIOD_MS, NULL);
}

void engine_screen_create(lv_obj_t *screen)
{
    s_engine_screen = screen;
    s_engine_canvas = make_canvas(screen, draw_engine);

    engine_now_t none;
    memset(&none, 0, sizeof(none));
    make_engine_view(none, &s_engine);

    start_timer();
}

void cylinders_screen_create(lv_obj_t *screen)
{
    s_cylinders_screen = screen;
    s_cylinders_canvas = make_canvas(screen, draw_cylinders);

    engine_now_t none;
    memset(&none, 0, sizeof(none));
    make_cylinders_view(none, &s_cylinders);

    start_timer();
}

/* ---- the menu ---------------------------------------------------------------------- */

/*
 * What the two screens show, opened by a swipe down on either
 * (screen_manager.cpp): the bars on the cylinders screen, and which
 * values get one of the engine screen's
 * eight bars. Each change shows at once, and is kept (app_settings.h).
 *
 * The toggles are button matrices - one object a grid, where a switch
 * each would be three - and the whole menu is made when it opens and
 * deleted when it closes: LVGL's heap is mostly taken by the screens.
 * Over the screen on the top layer, like the HSI's route menu, with a
 * backdrop that closes it on a tap anywhere else.
 */

static const uint32_t MENU_BG_COLOUR = 0x10161C;
static const uint32_t MENU_BORDER_COLOUR = 0x3A4A5A;
static const uint32_t BUTTON_COLOUR = 0x2A3440;
static const uint32_t CHECKED_COLOUR = 0x1987D1;
static const uint32_t SECTION_COLOUR = 0xFFD400;

/* The engine screen's toggles: the manifold pressure, then the items. */
static const char *const VALUES_MAP[] = {
    "MAP", "OIL P", "OIL T", "\n",
    "CHT", "EGT", "COOL", "\n",
    "FUEL P", "FLOW", "FUEL", "\n",
    "VOLTS", "AMPS", "CARB", ""
};

static const char *const BARS_MAP[] = { "EGT bars", "CHT bars", "" };

static lv_obj_t *s_menu_backdrop = NULL;
static lv_obj_t *s_menu = NULL;
static lv_obj_t *s_values_caption = NULL;
static lv_obj_t *s_values = NULL;
static lv_obj_t *s_bars = NULL;

static bool menu_is_open(void)
{
    return s_menu != NULL;
}

static void menu_close(void)
{
    if(s_menu == NULL)
        return;

    /* Async: this is often the menu's own Close button's click. */
    lv_obj_del_async(s_menu);
    s_menu = NULL;
    lv_obj_add_flag(s_menu_backdrop, LV_OBJ_FLAG_HIDDEN);
}

static int count_bits(uint32_t bits)
{
    int count = 0;

    for(; bits != 0; bits &= bits - 1)
        count++;

    return count;
}

static void set_checked(lv_obj_t *matrix, uint16_t button, bool on)
{
    if(on)
        lv_btnmatrix_set_btn_ctrl(matrix, button, LV_BTNMATRIX_CTRL_CHECKED);
    else
        lv_btnmatrix_clear_btn_ctrl(matrix, button, LV_BTNMATRIX_CTRL_CHECKED);
}

/* The menu as the settings are now. */
static void menu_show_settings(void)
{
    char text[40];

    const uint16_t items = engine_items();
    const uint8_t flags = app_settings().engine_flags;

    snprintf(text, sizeof(text), "ENGINE SCREEN - %d of %d bars",
             count_bits(items), ENGINE_MAX_ITEMS);
    lv_label_set_text(s_values_caption, text);

    set_checked(s_values, 0, (flags & ENGINE_HIDE_MANIFOLD) == 0);

    for(int i = 0; i < ENGINE_ITEM_COUNT; i++)
        set_checked(s_values, (uint16_t)(1 + i), (items & (1u << i)) != 0);

    set_checked(s_bars, 0, (flags & ENGINE_HIDE_EGT_BARS) == 0);
    set_checked(s_bars, 1, (flags & ENGINE_HIDE_CHT_BARS) == 0 ||
                               (flags & ENGINE_HIDE_EGT_BARS) != 0);
}

static void toggle_flag(uint8_t flag)
{
    const uint8_t flags = app_settings().engine_flags ^ flag;
    const uint8_t both = ENGINE_HIDE_EGT_BARS | ENGINE_HIDE_CHT_BARS;

    /* One kind of cylinder bar always stays. */
    if((flags & both) != both)
        app_settings_set_engine_flags(flags);
}

/*
 * MAP, or one of the engine screen's bars: never more bars than it has
 * room for, never none. A refused toggle shows unchanged, as the menu
 * is set from the settings after every one.
 */
static void on_values(lv_event_t *e)
{
    const uint16_t button =
        lv_btnmatrix_get_selected_btn(lv_event_get_target(e));

    if(button == 0)
    {
        toggle_flag(ENGINE_HIDE_MANIFOLD);
    }
    else if(button <= ENGINE_ITEM_COUNT)
    {
        const uint16_t items =
            (uint16_t)(engine_items() ^ (1u << (button - 1)));

        if(items != 0 && count_bits(items) <= ENGINE_MAX_ITEMS)
            app_settings_set_engine_items(items);
    }

    menu_show_settings();
}

static void on_bars(lv_event_t *e)
{
    const uint16_t button =
        lv_btnmatrix_get_selected_btn(lv_event_get_target(e));

    if(button <= 1)
        toggle_flag(button == 0 ? ENGINE_HIDE_EGT_BARS : ENGINE_HIDE_CHT_BARS);

    menu_show_settings();
}

static void on_close(lv_event_t *e)
{
    (void)e;
    menu_close();
}

static lv_obj_t *make_caption_label(lv_obj_t *parent, const char *text,
                                    uint32_t colour)
{
    lv_obj_t *label = lv_label_create(parent);

    lv_label_set_text(label, text);
    lv_obj_set_style_text_font(label, FONT_SMALL, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(colour), 0);

    return label;
}

static lv_obj_t *make_toggles(lv_obj_t *parent, const char *const *map,
                              lv_coord_t height, lv_event_cb_t on_change)
{
    lv_obj_t *matrix = lv_btnmatrix_create(parent);

    lv_btnmatrix_set_map(matrix, (const char **)map);

    /*
     * Not checkable: LVGL would toggle a button itself on release - after
     * on_change, which already set it from the settings - and so show
     * the opposite of what is kept. And click triggered: otherwise
     * VALUE_CHANGED comes on the press, and again for every button a
     * finger slides onto. The checks are set from the settings alone,
     * once the finger is lifted.
     */
    lv_btnmatrix_set_btn_ctrl_all(matrix, LV_BTNMATRIX_CTRL_CLICK_TRIG);

    lv_obj_set_size(matrix, LV_PCT(100), height);
    lv_obj_set_style_bg_opa(matrix, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(matrix, 0, 0);
    lv_obj_set_style_pad_all(matrix, 0, 0);
    lv_obj_set_style_pad_gap(matrix, 6, 0);
    lv_obj_set_style_bg_color(matrix, lv_color_hex(BUTTON_COLOUR),
                              LV_PART_ITEMS);
    lv_obj_set_style_bg_color(matrix, lv_color_hex(CHECKED_COLOUR),
                              (uint32_t)LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_text_color(matrix, lv_color_white(), LV_PART_ITEMS);
    lv_obj_set_style_text_font(matrix, FONT_BODY, LV_PART_ITEMS);
    lv_obj_set_style_radius(matrix, 8, LV_PART_ITEMS);
    lv_obj_set_style_shadow_width(matrix, 0, LV_PART_ITEMS);
    lv_obj_add_event_cb(matrix, on_change, LV_EVENT_VALUE_CHANGED, NULL);

    return matrix;
}

static lv_obj_t *make_menu_button(lv_obj_t *parent, const char *text,
                                  lv_coord_t width, lv_event_cb_t on_click,
                                  intptr_t user_data)
{
    lv_obj_t *button = lv_btn_create(parent);

    lv_obj_set_size(button, width, 36);
    lv_obj_set_style_radius(button, 10, 0);
    lv_obj_set_style_shadow_width(button, 0, 0);
    lv_obj_set_style_bg_color(button, lv_color_hex(BUTTON_COLOUR), 0);
    lv_obj_add_event_cb(button, on_click, LV_EVENT_CLICKED, (void *)user_data);

    lv_obj_t *label = lv_label_create(button);

    lv_label_set_text(label, text);
    lv_obj_set_style_text_font(label, FONT_LARGE, 0);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_center(label);

    return button;
}

static void create_menu(void)
{
    /* Small enough for its corners to stay on the round panel. */
    s_menu = lv_obj_create(lv_layer_top());

    lv_obj_set_size(s_menu, 280, 346);
    lv_obj_center(s_menu);
    lv_obj_clear_flag(s_menu, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_set_style_bg_color(s_menu, lv_color_hex(MENU_BG_COLOUR), 0);
    lv_obj_set_style_bg_opa(s_menu, LV_OPA_90, 0);
    lv_obj_set_style_border_color(s_menu, lv_color_hex(MENU_BORDER_COLOUR), 0);
    lv_obj_set_style_border_width(s_menu, 2, 0);
    lv_obj_set_style_radius(s_menu, 20, 0);
    lv_obj_set_style_pad_all(s_menu, 10, 0);
    lv_obj_set_style_pad_row(s_menu, 6, 0);
    lv_obj_set_flex_flow(s_menu, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(s_menu, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t *title = lv_label_create(s_menu);

    lv_label_set_text(title, "ENGINE DISPLAY");
    lv_obj_set_style_text_color(title, lv_color_hex(CAPTION_COLOUR), 0);
    lv_obj_set_style_text_font(title, FONT_BODY, 0);

    make_caption_label(s_menu, "CYLINDERS SCREEN", SECTION_COLOUR);
    s_bars = make_toggles(s_menu, BARS_MAP, 34, on_bars);

    s_values_caption = make_caption_label(s_menu, "", SECTION_COLOUR);
    s_values = make_toggles(s_menu, VALUES_MAP, 4 * 34 + 3 * 6, on_values);

    make_menu_button(s_menu, "Close", 120, on_close, 0);
}

void engine_screen_open_menu(void)
{
    lv_obj_t *const active = lv_scr_act();

    if((active != s_engine_screen && active != s_cylinders_screen) ||
       menu_is_open())
        return;

    if(s_menu_backdrop == NULL)
    {
        s_menu_backdrop = lv_obj_create(lv_layer_top());

        lv_obj_remove_style_all(s_menu_backdrop);
        lv_obj_set_size(s_menu_backdrop, LV_PCT(100), LV_PCT(100));
        lv_obj_add_flag(s_menu_backdrop, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(s_menu_backdrop, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_event_cb(s_menu_backdrop, on_close, LV_EVENT_CLICKED,
                            NULL);
    }

    create_menu();
    menu_show_settings();

    lv_obj_clear_flag(s_menu_backdrop, LV_OBJ_FLAG_HIDDEN);
}
