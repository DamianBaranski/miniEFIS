#include "hsi_screen.h"
#include "app_settings.h"
#include "fast_draw.h"
#include "fonts.h"
#include "route_store.h"

#include "esp_heap_caps.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* The GPS gives 5-10 fixes a second: 10 looks at that without lagging. */
static const uint32_t UPDATE_PERIOD_MS = 100;
static const float REDRAW_THRESHOLD_DEG = 0.2f;

/* Card geometry, from the centre of the screen. */
static const float CARD_RADIUS = 200.0f;            /* outer end of ticks */
static const float TICK_SHORT = 10.0f;              /* every 5 degrees */
static const float TICK_LONG = 18.0f;               /* every 10 degrees */
static const float LABEL_RADIUS = 166.0f;

/* Pointers on the card: a triangle, tip outwards, inside the ticks. */
static const float POINTER_TIP = 178.0f;
static const float POINTER_BASE = 160.0f;
static const float POINTER_HALF_WIDTH = 9.0f;

/* The deviation scales: three dots a side. */
static const float DOT_PX = 26.0f;
static const int DOTS = 3;
static const float BAR_HALF_LENGTH = 90.0f;
static const float AIRCRAFT_RADIUS = 12.0f;

/* Full scale, three dots: 1 NM across, 150 m (about 500 ft) up and down. */
static const float CROSS_TRACK_FULL_SCALE_M = 1852.0f;
static const float VERTICAL_FULL_SCALE_M = 150.0f;

/* A bar past full scale stops just beyond the last dot. */
static const float BAR_LIMIT_DOTS = 3.4f;

static const float METRES_PER_NM = 1852.0f;

#define CARDINAL_FONT FONT_HUGE
#define NUMBER_FONT FONT_LARGE

static const uint32_t CARD_COLOUR = 0xFFFFFF;
static const uint32_t SCALE_COLOUR = 0xC8C8C8;
static const uint32_t BEARING_COLOUR = 0xFF30D0;    /* magenta */
static const uint32_t TRACK_COLOUR = 0x30FF60;      /* green */
static const uint32_t VALUE_COLOUR = 0x30FF60;
static const uint32_t CAPTION_COLOUR = 0xC0C4C8;
static const uint32_t FLAG_COLOUR = 0xFF4040;
static const uint32_t WARNING_COLOUR = 0xFFD400;

static const uint32_t MENU_BG_COLOUR = 0x10161C;
static const uint32_t MENU_BORDER_COLOUR = 0x3A4A5A;
static const uint32_t BUTTON_COLOUR = 0x2A3440;
static const uint32_t BUTTON_ACTIVE_COLOUR = 0x1987D1;

/* What the card shows. Owned by the LVGL task. */
typedef struct {
    float track_deg;            /* card heading */
    bool show_bearing;
    float bearing_deg;
    bool show_desired_track;
    float desired_track_deg;
    bool show_lateral;
    float lateral_dots;         /* + bar to the right: course to the right */
    bool show_vertical;
    float vertical_dots;        /* + bar down: path below */
} view_t;

static lv_obj_t *s_screen = NULL;
static lv_obj_t *s_card = NULL;
static view_t s_view;
static bool s_drawn = false;

static lv_obj_t *s_waypoint = NULL;
static const lv_font_t *s_waypoint_font = NULL;
static lv_obj_t *s_bearing = NULL;
static lv_obj_t *s_distance = NULL;
static lv_obj_t *s_distance_caption = NULL;
static lv_obj_t *s_cross_track = NULL;
static lv_obj_t *s_flag = NULL;

static lv_obj_t *s_menu_backdrop = NULL;
static lv_obj_t *s_menu = NULL;
static lv_obj_t *s_menu_list = NULL;

static float rad(float d)
{
    return d * (float)M_PI / 180.0f;
}

/* A point on the card at a true bearing, for the track it is turned to. */
static void card_point(float cx, float cy, float bearing, float radius,
                       float *x, float *y)
{
    const float angle = rad(bearing - s_view.track_deg);

    *x = cx + radius * sinf(angle);
    *y = cy - radius * cosf(angle);
}

/* A filled triangle, as a fan of lines from the tip across the base. */
static void fill_triangle(lv_draw_ctx_t *draw_ctx, float tip_x, float tip_y,
                          float x1, float y1, float x2, float y2,
                          lv_color_t colour)
{
    const float steps = ceilf(hypotf(x2 - x1, y2 - y1));

    for(float i = 0.0f; i <= steps; i += 1.0f)
    {
        const float t = (steps > 0.0f) ? i / steps : 0.0f;

        fast_draw_line(draw_ctx, tip_x, tip_y, x1 + (x2 - x1) * t,
                       y1 + (y2 - y1) * t, colour, 2);
    }
}

/* A pointer on the card at a bearing, its tip towards the ticks. */
static void draw_pointer(lv_draw_ctx_t *draw_ctx, float cx, float cy,
                         float bearing, lv_color_t colour)
{
    const float angle = rad(bearing - s_view.track_deg);
    const float s = sinf(angle);
    const float c = cosf(angle);

    const float tip_x = cx + POINTER_TIP * s;
    const float tip_y = cy - POINTER_TIP * c;
    const float base_x = cx + POINTER_BASE * s;
    const float base_y = cy - POINTER_BASE * c;

    /* Across the radius: (c, s) is square to (s, -c). */
    fill_triangle(draw_ctx, tip_x, tip_y,
                  base_x - POINTER_HALF_WIDTH * c,
                  base_y - POINTER_HALF_WIDTH * s,
                  base_x + POINTER_HALF_WIDTH * c,
                  base_y + POINTER_HALF_WIDTH * s, colour);
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

    static const char *const labels[12] = {
        "N", "3", "6", "E", "12", "15", "S", "21", "24", "W", "30", "33"
    };

    for(int i = 0; i < 12; i++)
    {
        const bool cardinal = (i % 3) == 0;

        float x, y;
        card_point(cx, cy, (float)(i * 30), LABEL_RADIUS, &x, &y);

        fast_draw_text(draw_ctx, x, y, labels[i],
                       cardinal ? CARDINAL_FONT : NUMBER_FONT, white);
    }

    if(s_view.show_desired_track)
        draw_pointer(draw_ctx, cx, cy, s_view.desired_track_deg,
                     lv_color_hex(TRACK_COLOUR));

    if(s_view.show_bearing)
        draw_pointer(draw_ctx, cx, cy, s_view.bearing_deg,
                     lv_color_hex(BEARING_COLOUR));
}

/* Fixed: the lubber line, the dots and the aircraft. */
static void draw_fixed(lv_draw_ctx_t *draw_ctx, float cx, float cy)
{
    const lv_color_t white = lv_color_white();
    const lv_color_t scale = lv_color_hex(SCALE_COLOUR);

    /* Lubber line: a triangle above the card, pointing down at it. */
    const float tip = cy - CARD_RADIUS - 3.0f;

    fill_triangle(draw_ctx, cx, tip, cx - 10.0f, tip - 16.0f,
                  cx + 10.0f, tip - 16.0f, white);

    /* Across: short uprights. Up and down: short bars. */
    for(int i = 1; i <= DOTS; i++)
    {
        const float d = DOT_PX * (float)i;

        fast_draw_line(draw_ctx, cx - d, cy - 15.0f, cx - d, cy + 15.0f,
                       scale, 2);
        fast_draw_line(draw_ctx, cx + d, cy - 15.0f, cx + d, cy + 15.0f,
                       scale, 2);
        fast_draw_line(draw_ctx, cx - 14.0f, cy - d, cx + 14.0f, cy - d,
                       scale, 2);
        fast_draw_line(draw_ctx, cx - 14.0f, cy + d, cx + 14.0f, cy + d,
                       scale, 2);
    }

    /* The aircraft: a ring in the middle. */
    static const int SEGMENTS = 24;

    for(int i = 0; i < SEGMENTS; i++)
    {
        const float a1 = 2.0f * (float)M_PI * (float)i / SEGMENTS;
        const float a2 = 2.0f * (float)M_PI * (float)(i + 1) / SEGMENTS;

        fast_draw_line(draw_ctx,
                       cx + AIRCRAFT_RADIUS * cosf(a1),
                       cy + AIRCRAFT_RADIUS * sinf(a1),
                       cx + AIRCRAFT_RADIUS * cosf(a2),
                       cy + AIRCRAFT_RADIUS * sinf(a2), white, 3);
    }

    const lv_color_t magenta = lv_color_hex(BEARING_COLOUR);

    if(s_view.show_lateral)
    {
        const float x = cx + s_view.lateral_dots * DOT_PX;

        fast_draw_line(draw_ctx, x, cy - BAR_HALF_LENGTH, x,
                       cy + BAR_HALF_LENGTH, magenta, 4);
    }

    if(s_view.show_vertical)
    {
        const float y = cy + s_view.vertical_dots * DOT_PX;

        fast_draw_line(draw_ctx, cx - BAR_HALF_LENGTH, y,
                       cx + BAR_HALF_LENGTH, y, magenta, 4);
    }
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

/* ---- the numbers ------------------------------------------------------------ */

/* Only when it changed: setting a label redraws it, and the card under it. */
static void set_text(lv_obj_t *label, const char *text)
{
    if(strcmp(lv_label_get_text(label), text) != 0)
        lv_label_set_text(label, text);
}

static void set_flag(const char *text, uint32_t colour)
{
    set_text(s_flag, text);

    if(text[0] == '\0')
    {
        lv_obj_add_flag(s_flag, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    lv_obj_clear_flag(s_flag, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_text_color(s_flag, lv_color_hex(colour), 0);
    lv_obj_set_style_border_color(s_flag, lv_color_hex(colour), 0);
}

static float clamp_dots(float dots)
{
    if(dots > BAR_LIMIT_DOTS)
        return BAR_LIMIT_DOTS;
    if(dots < -BAR_LIMIT_DOTS)
        return -BAR_LIMIT_DOTS;
    return dots;
}

/* A distance in NM or km, as the units setting says. */
static void format_distance(char *text, size_t size, float metres,
                            bool nautical)
{
    const float value = metres / (nautical ? METRES_PER_NM : 1000.0f);

    snprintf(text, size, (value < 99.95f) ? "%.1f" : "%.0f", value);
}

static void show_numbers(const route_status_t &status, bool nautical)
{
    const RouteGuidance &g = status.guidance;

    set_text(s_waypoint, (status.count > 0) ? status.to_name : "----");

    /* Long names in the smaller font, to keep clear of the card. Only
     * on a change: a style set redraws the label even when the same. */
    const lv_font_t *font = (strlen(status.to_name) > 5) ? FONT_LARGE
                                                         : FONT_HUGE;

    if(font != s_waypoint_font)
    {
        s_waypoint_font = font;
        lv_obj_set_style_text_font(s_waypoint, font, 0);
    }

    set_text(s_distance_caption, nautical ? "DIST NM" : "DIST KM");

    char text[16];

    if(!g.valid)
    {
        set_text(s_bearing, "---");
        set_text(s_distance, "---");
        set_text(s_cross_track, "---");
        return;
    }

    snprintf(text, sizeof(text), "%03d", ((int)lroundf(g.bearing_deg)) % 360);
    set_text(s_bearing, text);

    format_distance(text, sizeof(text), (float)g.distance_m, nautical);
    set_text(s_distance, text);

    /* Which side of the leg the aircraft is: R right of it. */
    char side[2] = { 0, 0 };

    format_distance(text, sizeof(text), fabsf(g.cross_track_m), nautical);

    if(strcmp(text, "0.0") != 0)
        side[0] = (g.cross_track_m > 0.0f) ? 'R' : 'L';

    strncat(text, side, sizeof(text) - strlen(text) - 1);
    set_text(s_cross_track, text);
}

static bool view_differs(const view_t &a, const view_t &b)
{
    return fabsf(a.track_deg - b.track_deg) >= REDRAW_THRESHOLD_DEG ||
           a.show_bearing != b.show_bearing ||
           fabsf(a.bearing_deg - b.bearing_deg) >= REDRAW_THRESHOLD_DEG ||
           a.show_desired_track != b.show_desired_track ||
           fabsf(a.desired_track_deg - b.desired_track_deg) >=
               REDRAW_THRESHOLD_DEG ||
           a.show_lateral != b.show_lateral ||
           fabsf(a.lateral_dots - b.lateral_dots) * DOT_PX >= 0.5f ||
           a.show_vertical != b.show_vertical ||
           fabsf(a.vertical_dots - b.vertical_dots) * DOT_PX >= 0.5f;
}

static void menu_close(void);

static void update(lv_timer_t *timer)
{
    (void)timer;

    if(lv_scr_act() != s_screen)
    {
        menu_close();
        return;
    }

    route_status_t status;
    route_store_status(&status);

    const RouteGuidance &g = status.guidance;
    const bool nautical = app_settings().units == UNITS_AVIATION;

    show_numbers(status, nautical);

    if(status.count == 0)
        set_flag("NO ROUTE\nload a GPX file\nvia Wi-Fi", WARNING_COLOUR);
    else if(!status.have_fix)
        set_flag("NO GPS", FLAG_COLOUR);
    else
        set_flag("", FLAG_COLOUR);

    view_t view;
    view.track_deg = status.track_deg;
    view.show_bearing = g.valid;
    view.bearing_deg = g.bearing_deg;
    view.show_desired_track = g.valid;
    view.desired_track_deg = g.desired_track_deg;

    /* Right of the leg: the leg is to the left, and so is the bar. */
    view.show_lateral = g.valid;
    view.lateral_dots = clamp_dots(-g.cross_track_m /
                                   CROSS_TRACK_FULL_SCALE_M * DOTS);

    /* Above the path: the path is below, and so is the bar. */
    view.show_vertical = g.valid && g.vertical_valid;
    view.vertical_dots = clamp_dots(g.vertical_deviation_m /
                                    VERTICAL_FULL_SCALE_M * DOTS);

    if(s_drawn && !view_differs(view, s_view))
        return;

    s_view = view;
    s_drawn = true;
    lv_obj_invalidate(s_card);
}

/* ---- the route menu --------------------------------------------------------- */

/*
 * Opened by a swipe down on the HSI (screen_manager.cpp). The
 * waypoints, to pick where to go: a waypoint's name for direct to
 * it, from where the aircraft is now; LEG beside it for the leg into it
 * from the waypoint before, to rejoin the route as planned. The one
 * flown to now is lit, on whichever of the two it is.
 *
 * Over the screen on the top layer, like the screen menu, with a
 * backdrop of its own that closes it on a tap anywhere else - and keeps
 * that tap, and the double tap for the screen menu, off the HSI.
 */

static bool menu_is_open(void)
{
    return s_menu != NULL && !lv_obj_has_flag(s_menu, LV_OBJ_FLAG_HIDDEN);
}

static void menu_close(void)
{
    if(!menu_is_open())
        return;

    lv_obj_add_flag(s_menu, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_menu_backdrop, LV_OBJ_FLAG_HIDDEN);

    /* The rows go: they are made again, as the route is then. Async:
     * this is often a row's own button's click. */
    for(uint32_t i = 0; i < lv_obj_get_child_cnt(s_menu_list); i++)
        lv_obj_del_async(lv_obj_get_child(s_menu_list, (int32_t)i));
}

static void on_direct(lv_event_t *e)
{
    route_store_direct_to((int)(intptr_t)lv_event_get_user_data(e));
    menu_close();
}

static void on_leg(lv_event_t *e)
{
    route_store_activate_leg((int)(intptr_t)lv_event_get_user_data(e));
    menu_close();
}

static void on_close(lv_event_t *e)
{
    (void)e;
    menu_close();
}

static lv_obj_t *make_menu_button(lv_obj_t *parent, const char *text,
                                  bool lit, lv_event_cb_t on_click, int index)
{
    lv_obj_t *button = lv_btn_create(parent);

    lv_obj_set_height(button, 50);
    lv_obj_set_style_radius(button, 10, 0);
    lv_obj_set_style_shadow_width(button, 0, 0);
    lv_obj_set_style_bg_color(button, lv_color_hex(lit ? BUTTON_ACTIVE_COLOUR
                                                       : BUTTON_COLOUR), 0);
    lv_obj_add_event_cb(button, on_click, LV_EVENT_CLICKED,
                        (void *)(intptr_t)index);

    lv_obj_t *label = lv_label_create(button);

    lv_label_set_text(label, text);
    lv_label_set_long_mode(label, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_font(label, FONT_LARGE, 0);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_center(label);

    return button;
}

void hsi_screen_open_menu(void)
{
    if(lv_scr_act() != s_screen || menu_is_open())
        return;

    Route *route = (Route *)heap_caps_malloc(sizeof(Route), MALLOC_CAP_SPIRAM);

    if(route == NULL)
        return;

    route_status_t status;
    route_store_status(&status);
    route_store_get(route);

    /* The list is empty: the rows went when it last closed. */

    lv_obj_t *active_row = NULL;

    for(int i = 0; i < route->count; i++)
    {
        const bool active = i == status.guidance.to;

        lv_obj_t *row = lv_obj_create(s_menu_list);

        lv_obj_remove_style_all(row);
        lv_obj_set_size(row, LV_PCT(100), LV_SIZE_CONTENT);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_style_pad_column(row, 8, 0);

        char text[ROUTE_NAME_BYTES + 16];
        snprintf(text, sizeof(text), "%d  %s", i + 1, route->waypoint[i].name);

        lv_obj_t *direct = make_menu_button(row, text,
                                            active && status.guidance.direct,
                                            on_direct, i);
        lv_obj_set_flex_grow(direct, 1);

        /* Its label inside the button, dotted if the name is long. */
        lv_obj_t *label = lv_obj_get_child(direct, 0);
        lv_obj_set_width(label, LV_PCT(100));

        /* The first waypoint has no leg into it. */
        if(i > 0)
        {
            lv_obj_t *leg = make_menu_button(row, "LEG",
                                             active && !status.guidance.direct,
                                             on_leg, i);
            lv_obj_set_width(leg, 72);
        }

        if(active)
            active_row = row;
    }

    if(route->count == 0)
    {
        lv_obj_t *none = lv_label_create(s_menu_list);

        lv_obj_set_width(none, LV_PCT(100));
        lv_label_set_long_mode(none, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_align(none, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_font(none, FONT_LARGE, 0);
        lv_obj_set_style_text_color(none, lv_color_hex(WARNING_COLOUR), 0);
        lv_label_set_text(none, "No route.\nLoad a GPX file via Wi-Fi.");
    }

    free(route);

    lv_obj_clear_flag(s_menu_backdrop, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_menu, LV_OBJ_FLAG_HIDDEN);

    /* The waypoint flown to now in sight, for a long route. */
    if(active_row != NULL)
    {
        lv_obj_update_layout(s_menu);
        lv_obj_scroll_to_view(active_row, LV_ANIM_OFF);
    }
}

static void create_menu(void)
{
    s_menu_backdrop = lv_obj_create(lv_layer_top());

    lv_obj_remove_style_all(s_menu_backdrop);
    lv_obj_set_size(s_menu_backdrop, LV_PCT(100), LV_PCT(100));
    lv_obj_add_flag(s_menu_backdrop, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(s_menu_backdrop, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_menu_backdrop, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(s_menu_backdrop, on_close, LV_EVENT_CLICKED, NULL);

    /* Small enough for its corners to stay on the round panel. */
    s_menu = lv_obj_create(lv_layer_top());

    lv_obj_set_size(s_menu, 300, 340);
    lv_obj_center(s_menu);
    lv_obj_clear_flag(s_menu, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_set_style_bg_color(s_menu, lv_color_hex(MENU_BG_COLOUR), 0);
    lv_obj_set_style_bg_opa(s_menu, LV_OPA_90, 0);
    lv_obj_set_style_border_color(s_menu, lv_color_hex(MENU_BORDER_COLOUR), 0);
    lv_obj_set_style_border_width(s_menu, 2, 0);
    lv_obj_set_style_radius(s_menu, 20, 0);
    lv_obj_set_style_pad_all(s_menu, 12, 0);
    lv_obj_set_style_pad_row(s_menu, 8, 0);
    lv_obj_set_flex_flow(s_menu, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(s_menu, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t *title = lv_label_create(s_menu);

    lv_label_set_text(title, "DIRECT TO  /  LEG");
    lv_obj_set_style_text_color(title, lv_color_hex(0xA0A8B0), 0);
    lv_obj_set_style_text_font(title, FONT_BODY, 0);

    /* The waypoints, scrolled when there are more than fit. */
    s_menu_list = lv_obj_create(s_menu);

    lv_obj_remove_style_all(s_menu_list);
    lv_obj_set_width(s_menu_list, LV_PCT(100));
    lv_obj_set_flex_grow(s_menu_list, 1);
    lv_obj_set_flex_flow(s_menu_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(s_menu_list, 8, 0);
    lv_obj_set_scroll_dir(s_menu_list, LV_DIR_VER);

    lv_obj_t *close = lv_btn_create(s_menu);

    lv_obj_set_size(close, 120, 44);
    lv_obj_set_style_radius(close, 10, 0);
    lv_obj_set_style_shadow_width(close, 0, 0);
    lv_obj_set_style_bg_color(close, lv_color_hex(BUTTON_COLOUR), 0);
    lv_obj_add_event_cb(close, on_close, LV_EVENT_CLICKED, NULL);

    lv_obj_t *close_label = lv_label_create(close);

    lv_label_set_text(close_label, "Close");
    lv_obj_set_style_text_font(close_label, FONT_LARGE, 0);
    lv_obj_set_style_text_color(close_label, lv_color_white(), 0);
    lv_obj_center(close_label);

    lv_obj_add_flag(s_menu, LV_OBJ_FLAG_HIDDEN);
}

/* ---- building it ------------------------------------------------------------ */

static lv_obj_t *make_caption(lv_obj_t *screen, const char *text,
                              lv_coord_t x, lv_coord_t y)
{
    lv_obj_t *label = lv_label_create(screen);

    lv_obj_set_style_text_font(label, FONT_SMALL, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(CAPTION_COLOUR), 0);
    lv_label_set_text(label, text);
    lv_obj_align(label, LV_ALIGN_CENTER, x, y);

    return label;
}

static lv_obj_t *make_value(lv_obj_t *screen, lv_coord_t x, lv_coord_t y)
{
    lv_obj_t *label = lv_label_create(screen);

    /* A fixed width, centred: a changing number stays where it is. */
    lv_obj_set_width(label, 110);
    lv_label_set_long_mode(label, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(label, FONT_HUGE, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(VALUE_COLOUR), 0);
    lv_label_set_text(label, "---");
    lv_obj_align(label, LV_ALIGN_CENTER, x, y);

    return label;
}

void hsi_screen_create(lv_obj_t *screen)
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

    /* Four numbers in the corners inside the card, clear of its
     * letters wherever it turns: no corner more than 145 px out. */
    static const lv_coord_t X = 60;

    make_caption(screen, "WPT", -X, -104);
    make_caption(screen, "BRG", X, -104);
    s_distance_caption = make_caption(screen, "DIST NM", -X, 64);
    make_caption(screen, "XTE", X, 64);

    s_waypoint = make_value(screen, -X, -80);
    s_bearing = make_value(screen, X, -80);
    s_distance = make_value(screen, -X, 88);
    s_cross_track = make_value(screen, X, 88);

    /* What is missing, over the middle. */
    s_flag = lv_label_create(screen);

    lv_obj_set_style_text_font(s_flag, FONT_LARGE, 0);
    lv_obj_set_style_text_align(s_flag, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_bg_color(s_flag, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_flag, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_flag, 2, 0);
    lv_obj_set_style_pad_hor(s_flag, 10, 0);
    lv_obj_set_style_pad_ver(s_flag, 4, 0);
    lv_label_set_text(s_flag, "");
    lv_obj_align(s_flag, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(s_flag, LV_OBJ_FLAG_HIDDEN);

    create_menu();

    lv_timer_create(update, UPDATE_PERIOD_MS, NULL);
}
