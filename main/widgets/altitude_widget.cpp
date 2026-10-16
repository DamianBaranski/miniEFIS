#include "altitude_widget.h"
#include "fonts.h"
#include "navigation.h"

#include "esp_timer.h"
#include <stdio.h>

/*
 * Updated at 5 Hz rather than with the horizon.
 *
 * Altitude and vertical speed are slow quantities, and a mechanical
 * altimeter's needle does not twitch either. There is nothing to gain
 * from rewriting these labels twenty times a second - the screen is
 * redrawn every frame regardless of when the text last changed.
 */
static const uint32_t UPDATE_PERIOD_MS = 200;

/* Sized for FONT_SMALL captions over FONT_LARGE values (fonts.h). */
static const lv_coord_t PANEL_WIDTH = 128;
static const lv_coord_t PANEL_HEIGHT = 106;
static const lv_coord_t BAR_WIDTH = 9;
static const lv_coord_t BAR_HEIGHT = 88;

/* Where each line starts, from the top of the panel's content. */
static const lv_coord_t ALT_CAPTION_Y = 0;
static const lv_coord_t ALT_VALUE_Y = 18;
static const lv_coord_t VS_CAPTION_Y = 47;
static const lv_coord_t VS_VALUE_Y = 65;

/* The bar saturates here; beyond it the number still reads true. */
static const float BAR_FULL_SCALE_MS = 5.0f;

static const float METRES_TO_FEET = 3.280840f;
static const float MS_TO_FEET_PER_MIN = 196.8504f;

static lv_obj_t *s_panel = NULL;
static lv_obj_t *s_altitude_label = NULL;
static lv_obj_t *s_vertical_speed_label = NULL;
static lv_obj_t *s_bar = NULL;
static lv_timer_t *s_timer = NULL;

static altitude_units_t s_units = ALTITUDE_UNITS_METRIC;

/* What this panel costs per frame, for the render budget. */
static volatile uint32_t s_cost_us = 0;
static int64_t s_draw_started = 0;

uint32_t altitude_widget_render_cost(void)
{
    return s_cost_us;
}

static void draw_begin(lv_event_t *e)
{
    (void)e;
    s_draw_started = esp_timer_get_time();
}

static void draw_end(lv_event_t *e)
{
    (void)e;
    s_cost_us = (uint32_t)(esp_timer_get_time() - s_draw_started);
}

static lv_obj_t *make_caption(lv_obj_t *parent, const char *text,
                              lv_coord_t y)
{
    lv_obj_t *label = lv_label_create(parent);

    lv_obj_set_style_text_color(label, lv_color_hex(0xB0C4D8), 0);
    lv_obj_set_style_text_font(label, FONT_SMALL, 0);
    lv_label_set_text(label, text);
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, 0, y);

    return label;
}

static lv_obj_t *make_value(lv_obj_t *parent, lv_coord_t y)
{
    lv_obj_t *label = lv_label_create(parent);

    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_set_style_text_font(label, FONT_LARGE, 0);
    lv_label_set_text(label, "---");
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, 0, y);

    return label;
}

static void update(lv_timer_t *timer)
{
    (void)timer;

    if(s_panel == NULL)
        return;

    const VerticalChannelSample air = navigation_sample().vertical;

    /* Nothing fitted, or nothing measured yet: stay out of the way
     * rather than show a zero that looks like a reading. */
    if(!air.valid)
    {
        lv_obj_add_flag(s_panel, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    lv_obj_clear_flag(s_panel, LV_OBJ_FLAG_HIDDEN);

    char buffer[16];

    if(s_units == ALTITUDE_UNITS_AVIATION)
    {
        snprintf(buffer, sizeof(buffer), "%d ft",
                 (int)(air.altitude_m * METRES_TO_FEET + 0.5f));
        lv_label_set_text(s_altitude_label, buffer);

        snprintf(buffer, sizeof(buffer), "%+d",
                 (int)(air.vertical_speed_ms * MS_TO_FEET_PER_MIN));
        lv_label_set_text(s_vertical_speed_label, buffer);
    }
    else
    {
        snprintf(buffer, sizeof(buffer), "%d m", (int)(air.altitude_m + 0.5f));
        lv_label_set_text(s_altitude_label, buffer);

        snprintf(buffer, sizeof(buffer), "%+.1f", air.vertical_speed_ms);
        lv_label_set_text(s_vertical_speed_label, buffer);
    }

    /* The bar is in tenths of a metre per second, symmetrical about the
     * middle, so climb grows upwards and descent downwards. */
    int32_t bar = (int32_t)(air.vertical_speed_ms * 10.0f);
    const int32_t limit = (int32_t)(BAR_FULL_SCALE_MS * 10.0f);

    if(bar > limit) bar = limit;
    if(bar < -limit) bar = -limit;

    lv_bar_set_value(s_bar, bar, LV_ANIM_OFF);
}

void altitude_widget_create(lv_obj_t *parent)
{
    altitude_widget_close();

    s_panel = lv_obj_create(parent);

    lv_obj_remove_style_all(s_panel);
    lv_obj_set_size(s_panel, PANEL_WIDTH, PANEL_HEIGHT);
    lv_obj_align(s_panel, LV_ALIGN_RIGHT_MID, -10, 0);
    lv_obj_clear_flag(s_panel, LV_OBJ_FLAG_SCROLLABLE);
    /* Taps go through to the screen, for the double tap menu. */
    lv_obj_clear_flag(s_panel, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_set_style_bg_color(s_panel, lv_color_black(), 0);
    /*
     * Opaque, not translucent. Blending a translucent panel means
     * reading back every pixel under it and mixing - on a framebuffer in
     * PSRAM that is far more expensive than simply writing it.
     */
    lv_obj_set_style_bg_opa(s_panel, LV_OPA_COVER, 0);
    /* Square corners: a radius makes LVGL mask the corner rows. */
    lv_obj_set_style_radius(s_panel, 0, 0);
    lv_obj_set_style_pad_all(s_panel, 8, 0);

    make_caption(s_panel, "ALT", ALT_CAPTION_Y);
    s_altitude_label = make_value(s_panel, ALT_VALUE_Y);

    make_caption(s_panel, "V/S", VS_CAPTION_Y);
    s_vertical_speed_label = make_value(s_panel, VS_VALUE_Y);

    /*
     * A stock bar rather than custom drawing: the redraw is already
     * bandwidth bound, and two rectangles cost almost nothing next to
     * the horizon behind them.
     */
    s_bar = lv_bar_create(s_panel);

    lv_obj_set_size(s_bar, BAR_WIDTH, BAR_HEIGHT);
    lv_obj_align(s_bar, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_bar_set_mode(s_bar, LV_BAR_MODE_SYMMETRICAL);
    lv_bar_set_range(s_bar, (int32_t)(-BAR_FULL_SCALE_MS * 10.0f),
                     (int32_t)(BAR_FULL_SCALE_MS * 10.0f));
    lv_bar_set_value(s_bar, 0, LV_ANIM_OFF);

    lv_obj_set_style_bg_color(s_bar, lv_color_hex(0x303840), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_bar, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_bar, lv_color_hex(0x30D040), LV_PART_INDICATOR);

    /* Hidden until the barometer reports, so it never shows a zero that
     * could be mistaken for a measurement. */
    lv_obj_add_flag(s_panel, LV_OBJ_FLAG_HIDDEN);

    lv_obj_add_event_cb(s_panel, draw_begin,
                        (lv_event_code_t)(LV_EVENT_DRAW_MAIN |
                                          LV_EVENT_PREPROCESS), NULL);
    lv_obj_add_event_cb(s_panel, draw_end, LV_EVENT_DRAW_POST, NULL);

    s_timer = lv_timer_create(update, UPDATE_PERIOD_MS, NULL);
}

void altitude_widget_close(void)
{
    if(s_timer != NULL)
    {
        lv_timer_del(s_timer);
        s_timer = NULL;
    }

    if(s_panel != NULL)
    {
        /* Deleting the panel takes its children with it. */
        lv_obj_del(s_panel);
        s_panel = NULL;
        s_altitude_label = NULL;
        s_vertical_speed_label = NULL;
        s_bar = NULL;
    }
}

void altitude_widget_set_units(altitude_units_t units)
{
    s_units = units;
}
