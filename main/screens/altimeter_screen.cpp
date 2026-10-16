#include "altimeter_screen.h"
#include "fonts.h"
#include "navigation.h"
#include "app_settings.h"
#include "fast_draw.h"

#include "baro_bmp280.h"

#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const uint32_t UPDATE_PERIOD_MS = 40;

/* Smallest change worth a redraw, in display units. */
static const float REDRAW_ALTITUDE = 0.2f;
static const float REDRAW_SPEED_FRACTION = 0.002f;     /* of full scale */

static const float METRES_TO_FEET = 3.280840f;
static const float MS_TO_FEET_PER_MIN = 196.8504f;

/* QNH limits, hPa: every sea level pressure ever recorded. */
static const float QNH_MIN_HPA = 870.0f;
static const float QNH_MAX_HPA = 1085.0f;

/* ---- dial geometry, from the centre ---- */
static const float TICK_OUTER = 205.0f;
static const float TICK_LONG = 26.0f;
static const float TICK_SHORT = 12.0f;
static const float NUMBER_RADIUS = 166.0f;
static const float LONG_POINTER = 185.0f;
static const float SHORT_POINTER = 112.0f;
static const float POINTER_TAIL = 28.0f;

/* Vertical speed arc, on the left: zero at 9 o'clock, climb upwards. */
static const float VS_ZERO_DEG = 270.0f;
static const float VS_SWEEP_DEG = 70.0f;            /* full scale each way */
static const float VS_TRACK_RADIUS = 220.0f;
static const float VS_BAR_RADIUS = 228.0f;

#define NUMBER_FONT FONT_HUGE

static const uint32_t DIAL_COLOUR = 0xFFFFFF;
static const uint32_t TRACK_COLOUR = 0x505860;
static const uint32_t CLIMB_COLOUR = 0x30D040;
static const uint32_t SINK_COLOUR = 0xFF8C1A;
static const uint32_t CAPTION_COLOUR = 0x808890;
static const uint32_t WARNING_COLOUR = 0xFFD400;
static const uint32_t FAILED_COLOUR = 0xFF4040;

/* Units for the frame being drawn. Owned by the LVGL task. */
struct Scale
{
    float altitude_per_metre;
    float speed_per_ms;
    float speed_full_scale;
    float speed_tick;
    int speed_long_tick_every;
    const char *altitude_unit;
    const char *speed_unit;
};

static const Scale METRIC = {
    1.0f, 1.0f, 10.0f, 1.0f, 5, "m", "m/s"
};
static const Scale AVIATION = {
    METRES_TO_FEET, MS_TO_FEET_PER_MIN, 2000.0f, 500.0f, 2, "ft", "ft/min"
};

static const Scale *s_scale = &METRIC;

static lv_obj_t *s_screen = NULL;
static lv_obj_t *s_dial = NULL;
static lv_obj_t *s_altitude_label = NULL;
static lv_obj_t *s_speed_label = NULL;
static lv_obj_t *s_qnh_label = NULL;
static lv_obj_t *s_qnh_caption = NULL;
static lv_obj_t *s_flag_label = NULL;

/* What the frame shows, in display units. */
static float s_altitude = 0.0f;
static float s_speed = 0.0f;
static bool s_valid = false;

static float rad(float d)
{
    return d * (float)M_PI / 180.0f;
}

static float clampf(float value, float low, float high)
{
    return (value < low) ? low : (value > high) ? high : value;
}

/* A point at an angle clockwise from straight up. */
static void polar(float cx, float cy, float angle_deg, float radius,
                  float *x, float *y)
{
    *x = cx + radius * sinf(rad(angle_deg));
    *y = cy - radius * cosf(rad(angle_deg));
}

/* ---- drawing ---------------------------------------------------------------- */

static void draw_dial(lv_draw_ctx_t *draw_ctx, float cx, float cy)
{
    const lv_color_t white = lv_color_hex(DIAL_COLOUR);

    /* A tick every 20, a long one and a number every 100. */
    for(int tick = 0; tick < 50; tick++)
    {
        const bool long_tick = (tick % 5) == 0;
        const float angle = (float)tick * 7.2f;

        float x1, y1, x2, y2;
        polar(cx, cy, angle,
              TICK_OUTER - (long_tick ? TICK_LONG : TICK_SHORT), &x1, &y1);
        polar(cx, cy, angle, TICK_OUTER, &x2, &y2);

        fast_draw_line(draw_ctx, x1, y1, x2, y2, white, long_tick ? 4 : 2);
    }

    for(int digit = 0; digit < 10; digit++)
    {
        const char text[2] = { (char)('0' + digit), '\0' };

        float x, y;
        polar(cx, cy, (float)digit * 36.0f, NUMBER_RADIUS, &x, &y);

        fast_draw_text(draw_ctx, x, y, text, NUMBER_FONT, white);
    }
}

static void draw_speed_arc(lv_draw_ctx_t *draw_ctx, float cx, float cy)
{
    const Scale &scale = *s_scale;
    const float deg_per_unit = VS_SWEEP_DEG / scale.speed_full_scale;

    /* Track. */
    for(float a = -VS_SWEEP_DEG; a < VS_SWEEP_DEG; a += 4.0f)
    {
        float x1, y1, x2, y2;
        polar(cx, cy, VS_ZERO_DEG + a, VS_TRACK_RADIUS, &x1, &y1);
        polar(cx, cy, VS_ZERO_DEG + a + 4.0f, VS_TRACK_RADIUS, &x2, &y2);
        fast_draw_line(draw_ctx, x1, y1, x2, y2, lv_color_hex(TRACK_COLOUR), 2);
    }

    /* Ticks, longer at zero and every few steps. */
    const int steps = (int)(scale.speed_full_scale / scale.speed_tick + 0.5f);

    for(int i = -steps; i <= steps; i++)
    {
        const bool long_tick = (i % scale.speed_long_tick_every) == 0;
        const float angle =
            VS_ZERO_DEG + (float)i * scale.speed_tick * deg_per_unit;

        float x1, y1, x2, y2;
        polar(cx, cy, angle, VS_TRACK_RADIUS - (long_tick ? 10.0f : 5.0f),
              &x1, &y1);
        polar(cx, cy, angle, VS_TRACK_RADIUS, &x2, &y2);
        fast_draw_line(draw_ctx, x1, y1, x2, y2, lv_color_hex(DIAL_COLOUR),
                       long_tick ? 3 : 2);
    }

    if(!s_valid)
        return;

    /* The bar, from zero to the value, in short straight pieces. */
    const float end = clampf(s_speed * deg_per_unit, -VS_SWEEP_DEG,
                             VS_SWEEP_DEG);
    const lv_color_t colour =
        lv_color_hex(end >= 0.0f ? CLIMB_COLOUR : SINK_COLOUR);
    const float step = (end >= 0.0f) ? 2.0f : -2.0f;

    for(float a = 0.0f; fabsf(a) < fabsf(end); a += step)
    {
        const float b = (fabsf(a + step) > fabsf(end)) ? end : a + step;

        float x1, y1, x2, y2;
        polar(cx, cy, VS_ZERO_DEG + a, VS_BAR_RADIUS, &x1, &y1);
        polar(cx, cy, VS_ZERO_DEG + b, VS_BAR_RADIUS, &x2, &y2);
        fast_draw_line(draw_ctx, x1, y1, x2, y2, colour, 9);
    }
}

static void draw_pointer(lv_draw_ctx_t *draw_ctx, float cx, float cy,
                         float angle_deg, float length, uint16_t width)
{
    float x1, y1, x2, y2;

    polar(cx, cy, angle_deg + 180.0f, POINTER_TAIL, &x1, &y1);
    polar(cx, cy, angle_deg, length, &x2, &y2);

    fast_draw_line(draw_ctx, x1, y1, x2, y2, lv_color_hex(DIAL_COLOUR), width);
}

static void draw(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *draw_ctx = lv_event_get_draw_ctx(e);

    lv_area_t area;
    lv_obj_get_coords(obj, &area);

    const float cx = (float)(area.x1 + area.x2) * 0.5f;
    const float cy = (float)(area.y1 + area.y2) * 0.5f;

    draw_dial(draw_ctx, cx, cy);
    draw_speed_arc(draw_ctx, cx, cy);

    if(!s_valid)
        return;

    /* fmodf keeps the sign; below sea level the pointers simply turn
     * the other way, as they do on a real altimeter. */
    const float hundreds = fmodf(s_altitude, 1000.0f) * 0.36f;
    const float thousands = fmodf(s_altitude, 10000.0f) * 0.036f;

    draw_pointer(draw_ctx, cx, cy, thousands, SHORT_POINTER, 13);
    draw_pointer(draw_ctx, cx, cy, hundreds, LONG_POINTER, 6);

    fast_fill_circle(draw_ctx, cx, cy, 12.0f, lv_color_hex(DIAL_COLOUR));
    fast_fill_circle(draw_ctx, cx, cy, 5.0f, lv_color_black());
}

/* ---- readouts --------------------------------------------------------------- */

static void show_qnh(void)
{
    const float qnh = app_settings().qnh_pa;
    char text[16];

    snprintf(text, sizeof(text), "%.0f", qnh / 100.0f);
    lv_label_set_text(s_qnh_label, text);

    lv_label_set_text(s_qnh_caption,
                      (qnh == BARO_SEA_LEVEL_STANDARD_PA) ? "QNH hPa  STD"
                                                          : "QNH hPa");
}

static void set_qnh(float pascals)
{
    pascals = clampf(pascals, QNH_MIN_HPA * 100.0f, QNH_MAX_HPA * 100.0f);

    navigation_set_sea_level_pressure(pascals);
    app_settings_set_qnh(pascals);
    show_qnh();
}

/* Whole hectopascals: from STD's 1013.25 one step goes to 1014, not
 * 1014.25. */
static void step_qnh(int hpa)
{
    const float now = roundf(app_settings().qnh_pa / 100.0f);
    set_qnh((now + (float)hpa) * 100.0f);
}

static void on_minus(lv_event_t *e)
{
    (void)e;
    step_qnh(-1);
}

static void on_plus(lv_event_t *e)
{
    (void)e;
    step_qnh(+1);
}

static void on_standard(lv_event_t *e)
{
    (void)e;
    set_qnh(BARO_SEA_LEVEL_STANDARD_PA);
}

static void update(lv_timer_t *timer)
{
    (void)timer;

    if(lv_scr_act() != s_screen)
        return;

    if(!baro_present())
    {
        if(s_valid || lv_obj_has_flag(s_flag_label, LV_OBJ_FLAG_HIDDEN))
        {
            s_valid = false;
            lv_label_set_text(s_flag_label, "NO BAROMETER");
            lv_obj_set_style_text_color(s_flag_label,
                                        lv_color_hex(FAILED_COLOUR), 0);
            lv_obj_clear_flag(s_flag_label, LV_OBJ_FLAG_HIDDEN);
            lv_label_set_text(s_altitude_label, "----");
            lv_label_set_text(s_speed_label, "");
            lv_obj_invalidate(s_dial);
        }

        return;
    }

    const VerticalChannelSample air = navigation_sample().vertical;
    const Scale &scale = *s_scale;

    const float altitude = air.altitude_m * scale.altitude_per_metre;
    const float speed = air.vertical_speed_ms * scale.speed_per_ms;

    if(air.valid == s_valid &&
       fabsf(altitude - s_altitude) < REDRAW_ALTITUDE &&
       fabsf(speed - s_speed) <
           REDRAW_SPEED_FRACTION * scale.speed_full_scale)
        return;

    s_valid = air.valid;
    s_altitude = altitude;
    s_speed = speed;

    if(!s_valid)
    {
        lv_label_set_text(s_flag_label, "WAIT");
        lv_obj_set_style_text_color(s_flag_label,
                                    lv_color_hex(WARNING_COLOUR), 0);
        lv_obj_clear_flag(s_flag_label, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(s_altitude_label, "----");
        lv_label_set_text(s_speed_label, "");
    }
    else
    {
        lv_obj_add_flag(s_flag_label, LV_OBJ_FLAG_HIDDEN);

        char text[24];

        snprintf(text, sizeof(text), "%.0f %s", altitude, scale.altitude_unit);
        lv_label_set_text(s_altitude_label, text);

        /* Rounded to what is worth reading, and no "-0.0" at rest. */
        const float shown = (scale.speed_per_ms > 1.0f)
                                ? roundf(speed / 10.0f) * 10.0f
                                : roundf(speed * 10.0f) / 10.0f;

        if(shown == 0.0f)
            snprintf(text, sizeof(text), "0 %s", scale.speed_unit);
        else if(scale.speed_per_ms > 1.0f)
            snprintf(text, sizeof(text), "%+.0f %s", shown, scale.speed_unit);
        else
            snprintf(text, sizeof(text), "%+.1f %s", shown, scale.speed_unit);

        lv_label_set_text(s_speed_label, text);
    }

    lv_obj_invalidate(s_dial);
}

void altimeter_screen_units_changed(void)
{
    s_scale = (app_settings().units == UNITS_AVIATION) ? &AVIATION : &METRIC;

    /* Force the next update to redraw with the new numbers. */
    s_altitude = NAN;

    if(s_dial != NULL)
        lv_obj_invalidate(s_dial);
}

/* ---- construction ----------------------------------------------------------- */

static lv_obj_t *make_label(lv_obj_t *parent, const lv_font_t *font,
                            uint32_t colour, lv_coord_t y)
{
    lv_obj_t *label = lv_label_create(parent);

    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(colour), 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, y);

    return label;
}

static lv_obj_t *make_button(lv_obj_t *parent, const char *text,
                             lv_coord_t width, lv_coord_t height,
                             lv_coord_t x, lv_coord_t y)
{
    lv_obj_t *button = lv_btn_create(parent);

    lv_obj_set_size(button, width, height);
    lv_obj_align(button, LV_ALIGN_CENTER, x, y);
    lv_obj_set_style_radius(button, 10, 0);
    lv_obj_set_style_shadow_width(button, 0, 0);
    lv_obj_set_style_bg_color(button, lv_color_hex(0x2A3440), 0);

    lv_obj_t *label = lv_label_create(button);

    lv_label_set_text(label, text);
    lv_obj_set_style_text_font(label, FONT_HUGE, 0);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_center(label);

    return button;
}

/* Tap for one step; hold and it repeats. A tap is a short click, not a
 * click, since a click also arrives at the end of every hold. */
static void repeat_on_hold(lv_obj_t *button, lv_event_cb_t step)
{
    lv_obj_add_event_cb(button, step, LV_EVENT_SHORT_CLICKED, NULL);
    lv_obj_add_event_cb(button, step, LV_EVENT_LONG_PRESSED, NULL);
    lv_obj_add_event_cb(button, step, LV_EVENT_LONG_PRESSED_REPEAT, NULL);
}

void altimeter_screen_create(lv_obj_t *screen)
{
    s_screen = screen;

    s_dial = lv_obj_create(screen);

    lv_obj_remove_style_all(s_dial);
    lv_obj_set_size(s_dial, LV_PCT(100), LV_PCT(100));
    lv_obj_center(s_dial);
    lv_obj_clear_flag(s_dial, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(s_dial, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_dial, draw, LV_EVENT_DRAW_MAIN, NULL);

    /* Readouts above the hub: altitude, then vertical speed. */
    s_altitude_label = make_label(screen, FONT_HUGE,
                                  DIAL_COLOUR, -72);
    lv_obj_set_style_bg_color(s_altitude_label, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_altitude_label, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(s_altitude_label, lv_color_white(), 0);
    lv_obj_set_style_border_width(s_altitude_label, 2, 0);
    lv_obj_set_style_pad_hor(s_altitude_label, 10, 0);
    lv_obj_set_style_pad_ver(s_altitude_label, 3, 0);

    s_speed_label = make_label(screen, FONT_LARGE,
                               DIAL_COLOUR, -36);

    /* The QNH window below it, with its knob as two buttons. */
    s_qnh_label = make_label(screen, FONT_HUGE, DIAL_COLOUR, 66);
    lv_obj_set_style_bg_color(s_qnh_label, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_qnh_label, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(s_qnh_label, lv_color_hex(CAPTION_COLOUR), 0);
    lv_obj_set_style_border_width(s_qnh_label, 2, 0);
    lv_obj_set_style_pad_hor(s_qnh_label, 8, 0);
    lv_obj_set_style_pad_ver(s_qnh_label, 2, 0);

    s_qnh_caption = make_label(screen, FONT_SMALL,
                               CAPTION_COLOUR, 94);

    repeat_on_hold(make_button(screen, "-", 56, 48, -100, 66), on_minus);
    repeat_on_hold(make_button(screen, "+", 56, 48, 100, 66), on_plus);

    lv_obj_t *standard = make_button(screen, "STD", 72, 34, 0, 128);
    lv_obj_set_style_text_font(lv_obj_get_child(standard, 0),
                               FONT_BODY, 0);
    lv_obj_add_event_cb(standard, on_standard, LV_EVENT_CLICKED, NULL);

    s_flag_label = make_label(screen, FONT_HUGE,
                              WARNING_COLOUR, 0);
    lv_obj_add_flag(s_flag_label, LV_OBJ_FLAG_HIDDEN);

    altimeter_screen_units_changed();
    show_qnh();

    lv_timer_create(update, UPDATE_PERIOD_MS, NULL);
}
