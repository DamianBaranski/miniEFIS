#include "compass_screen.h"
#include "fonts.h"
#include "navigation.h"
#include "diag_stream.h"
#include "fast_draw.h"
#include "magnetometer.h"

#include "mag_qmc5883.h"

#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
 * Asked for every 20 ms, like the horizon, but only redrawn when the
 * heading has moved enough to show: a still compass costs nothing.
 */
static const uint32_t UPDATE_PERIOD_MS = 20;
static const float REDRAW_THRESHOLD_DEG = 0.1f;

/* Card geometry, from the centre of the screen. */
static const float CARD_RADIUS = 200.0f;            /* outer end of ticks */
static const float TICK_SHORT = 12.0f;              /* every 5 degrees */
static const float TICK_LONG = 22.0f;               /* every 10 degrees */
static const float LABEL_RADIUS = 158.0f;
static const float BEZEL_INNER = 205.0f;            /* fixed 45 degree marks */
static const float BEZEL_OUTER = 220.0f;

#define CARDINAL_FONT FONT_HUGE
#define NUMBER_FONT FONT_LARGE

static const uint32_t CARD_COLOUR = 0xFFFFFF;
static const uint32_t NORTH_COLOUR = 0xFF5030;
static const uint32_t LUBBER_COLOUR = 0xFFD400;
static const uint32_t AIRCRAFT_COLOUR = 0xFFD400;
static const uint32_t WARNING_COLOUR = 0xFFD400;
static const uint32_t FAILED_COLOUR = 0xFF4040;

static lv_obj_t *s_screen = NULL;
static lv_obj_t *s_card = NULL;
static lv_obj_t *s_readout = NULL;
static lv_obj_t *s_status = NULL;

/* Heading the card is drawn at. Owned by the LVGL task. */
static float s_heading = 0.0f;
static bool s_have_heading = false;
static int s_shown_state = -1;

static float rad(float d)
{
    return d * (float)M_PI / 180.0f;
}

/*
 * A point on the card at a bearing, for the current heading: the card
 * turns the opposite way to the aircraft, so whatever bearing is being
 * flown sits under the lubber line at the top.
 */
static void card_point(float cx, float cy, float bearing, float radius,
                       float *x, float *y)
{
    const float angle = rad(bearing - s_heading);

    *x = cx + radius * sinf(angle);
    *y = cy - radius * cosf(angle);
}

static void draw_card(lv_draw_ctx_t *draw_ctx, float cx, float cy)
{
    const lv_color_t white = lv_color_hex(CARD_COLOUR);

    for(int bearing = 0; bearing < 360; bearing += 5)
    {
        const bool long_tick = (bearing % 10) == 0;
        const float inner =
            CARD_RADIUS - (long_tick ? TICK_LONG : TICK_SHORT);

        float x1, y1, x2, y2;

        card_point(cx, cy, (float)bearing, inner, &x1, &y1);
        card_point(cx, cy, (float)bearing, CARD_RADIUS, &x2, &y2);

        fast_draw_line(draw_ctx, x1, y1, x2, y2, white, long_tick ? 3 : 2);
    }

    /* Tens of degrees every 30, the cardinal points as letters. */
    static const char *const labels[12] = {
        "N", "3", "6", "E", "12", "15", "S", "21", "24", "W", "30", "33"
    };

    for(int i = 0; i < 12; i++)
    {
        const bool cardinal = (i % 3) == 0;

        float x, y;
        card_point(cx, cy, (float)(i * 30), LABEL_RADIUS, &x, &y);

        fast_draw_text(draw_ctx, x, y, labels[i],
                       cardinal ? CARDINAL_FONT : NUMBER_FONT,
                       lv_color_hex(i == 0 ? NORTH_COLOUR : CARD_COLOUR));
    }
}

/* Fixed: the lubber line, the 45 degree bezel marks and the aircraft. */
static void draw_fixed(lv_draw_ctx_t *draw_ctx, float cx, float cy)
{
    const lv_color_t white = lv_color_white();
    const lv_color_t yellow = lv_color_hex(LUBBER_COLOUR);

    for(int bearing = 45; bearing < 360; bearing += 45)
    {
        const float angle = rad((float)bearing);

        fast_draw_line(draw_ctx,
                       cx + BEZEL_INNER * sinf(angle),
                       cy - BEZEL_INNER * cosf(angle),
                       cx + BEZEL_OUTER * sinf(angle),
                       cy - BEZEL_OUTER * cosf(angle),
                       white, 3);
    }

    /* Lubber line: a wedge pointing down at the card, built from
     * shortening horizontal strokes. */
    const float tip = cy - CARD_RADIUS - 2.0f;

    for(int row = 0; row < 16; row++)
    {
        const float half = (float)row * 0.7f;
        const float y = tip - (float)row;

        fast_draw_line(draw_ctx, cx - half - 0.5f, y, cx + half + 0.5f, y,
                       yellow, 1);
    }

    /* The aircraft, nose up. */
    const lv_color_t aircraft = lv_color_hex(AIRCRAFT_COLOUR);

    fast_draw_line(draw_ctx, cx, cy - 46, cx, cy + 34, aircraft, 5);
    fast_draw_line(draw_ctx, cx - 50, cy - 8, cx + 50, cy - 8, aircraft, 5);
    fast_draw_line(draw_ctx, cx - 20, cy + 30, cx + 20, cy + 30, aircraft, 4);
}

static void draw(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *draw_ctx = lv_event_get_draw_ctx(e);

    lv_area_t area;
    lv_obj_get_coords(obj, &area);

    const float cx = (float)(area.x1 + area.x2) * 0.5f;
    const float cy = (float)(area.y1 + area.y2) * 0.5f;

    draw_card(draw_ctx, cx, cy);
    draw_fixed(draw_ctx, cx, cy);
}

compass_state_t compass_state(bool heading_valid)
{
    if(!mag_present())
        return COMPASS_STATE_NO_MAGNETOMETER;

    if(magnetometer_calibrating())
        return COMPASS_STATE_CALIBRATING;

    if(!magnetometer_calibration().valid)
        return COMPASS_STATE_NOT_CALIBRATED;

    return heading_valid ? COMPASS_STATE_OK : COMPASS_STATE_NO_HEADING;
}

static void show_state(compass_state_t state)
{
    if((int)state == s_shown_state)
        return;

    s_shown_state = (int)state;

    const char *text = "";
    uint32_t colour = WARNING_COLOUR;

    switch(state)
    {
        case COMPASS_STATE_OK:
            text = "MAG";
            colour = 0x808890;
            break;

        case COMPASS_STATE_NO_MAGNETOMETER:
            text = "NO MAGNETOMETER";
            colour = FAILED_COLOUR;
            break;

        case COMPASS_STATE_NOT_CALIBRATED:
            text = "NOT CALIBRATED\nsee settings";
            break;

        case COMPASS_STATE_CALIBRATING:
            text = "CALIBRATING";
            break;

        case COMPASS_STATE_NO_HEADING:
            /* Calibrated, but the field is out of the plausible range:
             * something magnetic nearby, or the IMU still settling. */
            text = "NO HEADING";
            colour = FAILED_COLOUR;
            break;
    }

    lv_label_set_text(s_status, text);
    lv_obj_set_style_text_color(s_status, lv_color_hex(colour), 0);
}

static void update(lv_timer_t *timer)
{
    (void)timer;

    if(lv_scr_act() != s_screen)
        return;

    const AttitudeSample attitude = navigation_sample().attitude;

    show_state(compass_state(attitude.heading_valid));

    if(!attitude.heading_valid)
    {
        if(s_have_heading)
        {
            /* The card stays where it was, but the number goes: a stale
             * heading must not look like a live one. */
            s_have_heading = false;
            lv_label_set_text(s_readout, "---");
        }

        return;
    }

    float moved = fabsf(attitude.heading_deg - s_heading);

    if(moved > 180.0f)
        moved = 360.0f - moved;

    if(s_have_heading && moved < REDRAW_THRESHOLD_DEG)
        return;

    s_heading = attitude.heading_deg;
    s_have_heading = true;

    char text[8];
    snprintf(text, sizeof(text), "%03d",
             ((int)lroundf(s_heading)) % 360);
    lv_label_set_text(s_readout, text);

    lv_obj_invalidate(s_card);
}

/* Stream what the heading is made of to the PC while this is up. */
static void on_screen_loaded(lv_event_t *e)
{
    (void)e;
    diag_stream_begin(DIAG_STREAM_COMPASS);
}

static void on_screen_unloaded(lv_event_t *e)
{
    (void)e;
    diag_stream_end(DIAG_STREAM_COMPASS);
}

void compass_screen_create(lv_obj_t *screen)
{
    s_screen = screen;

    s_card = lv_obj_create(screen);

    lv_obj_remove_style_all(s_card);
    lv_obj_set_size(s_card, LV_PCT(100), LV_PCT(100));
    lv_obj_center(s_card);

    /* Taps go through to the screen, where the double tap is caught. */
    lv_obj_clear_flag(s_card, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(s_card, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_add_event_cb(s_card, draw, LV_EVENT_DRAW_MAIN, NULL);

    /* Below the aircraft, inside the ring of numbers. */
    s_readout = lv_label_create(screen);

    lv_obj_set_style_text_font(s_readout, FONT_HUGE, 0);
    lv_obj_set_style_text_color(s_readout, lv_color_white(), 0);
    lv_obj_set_style_bg_color(s_readout, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_readout, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(s_readout, lv_color_white(), 0);
    lv_obj_set_style_border_width(s_readout, 2, 0);
    lv_obj_set_style_pad_hor(s_readout, 10, 0);
    lv_obj_set_style_pad_ver(s_readout, 4, 0);
    lv_label_set_text(s_readout, "---");
    lv_obj_align(s_readout, LV_ALIGN_CENTER, 0, 72);

    s_status = lv_label_create(screen);

    lv_obj_set_style_text_font(s_status, FONT_SMALL, 0);
    lv_obj_set_style_text_align(s_status, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(s_status, "");
    lv_obj_align(s_status, LV_ALIGN_CENTER, 0, 112);

    lv_obj_add_event_cb(screen, on_screen_loaded, LV_EVENT_SCREEN_LOADED, NULL);
    lv_obj_add_event_cb(screen, on_screen_unloaded, LV_EVENT_SCREEN_UNLOADED,
                        NULL);

    lv_timer_create(update, UPDATE_PERIOD_MS, NULL);
}
