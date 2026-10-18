#include "screen_manager.h"
#include "fonts.h"
#include "altimeter_screen.h"
#include "app_settings.h"
#include "compass_screen.h"
#include "engine_screen.h"
#include "hsi_screen.h"
#include "settings_screen.h"
#include "turn_coordinator_screen.h"

#include <stdint.h>

/*
 * Double tap detection.
 *
 * LVGL 8 has no double click event, so it is put together from two
 * short clicks close together in time and place. The window is generous
 * because the touch panel is only polled between redraws - every 40 ms
 * or so on the horizon - which makes the measured interval coarse.
 */
static const uint32_t DOUBLE_TAP_MS = 500;
static const lv_coord_t DOUBLE_TAP_SLOP_PX = 60;

static const uint32_t MENU_BG_COLOUR = 0x10161C;
static const uint32_t MENU_BORDER_COLOUR = 0x3A4A5A;
static const uint32_t BUTTON_COLOUR = 0x2A3440;
static const uint32_t BUTTON_CURRENT_COLOUR = 0x1987D1;

static const char *const SCREEN_NAMES[SCREEN_COUNT] = {
    "Artificial horizon",
    "Altimeter",
    "Turn coordinator",
    "Compass",
    "HSI",
    "Engine",
    "Cylinders",
    "Settings",
};

static lv_obj_t *s_screens[SCREEN_COUNT];
static screen_id_t s_current = SCREEN_HORIZON;

static lv_obj_t *s_backdrop = NULL;
static lv_obj_t *s_menu = NULL;
static lv_obj_t *s_menu_buttons[SCREEN_COUNT];

static bool s_have_tap = false;
static uint32_t s_last_tap_ms = 0;
static lv_point_t s_last_tap_point;

static bool menu_is_open(void)
{
    return s_menu != NULL && !lv_obj_has_flag(s_menu, LV_OBJ_FLAG_HIDDEN);
}

static void menu_open(void)
{
    /* Show where we are, and only the screens shown. */
    for(int i = 0; i < SCREEN_COUNT; i++)
    {
        if(screen_manager_visible((screen_id_t)i) || i == s_current)
            lv_obj_clear_flag(s_menu_buttons[i], LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(s_menu_buttons[i], LV_OBJ_FLAG_HIDDEN);

        lv_obj_set_style_bg_color(
            s_menu_buttons[i],
            lv_color_hex(i == s_current ? BUTTON_CURRENT_COLOUR
                                        : BUTTON_COLOUR),
            0);
    }

    lv_obj_clear_flag(s_backdrop, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_menu, LV_OBJ_FLAG_HIDDEN);
}

static void menu_close(void)
{
    lv_obj_add_flag(s_menu, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_backdrop, LV_OBJ_FLAG_HIDDEN);
    s_have_tap = false;
}

/*
 * Taps on a screen's background - anything that is not a control - end
 * up here. A control takes its own taps, so double tapping a button
 * presses it twice rather than opening the menu.
 */
static void on_tap(lv_event_t *e)
{
    (void)e;

    lv_point_t point;
    lv_indev_get_point(lv_indev_get_act(), &point);

    if(menu_is_open())
        return;

    if(s_have_tap &&
       lv_tick_elaps(s_last_tap_ms) <= DOUBLE_TAP_MS &&
       LV_ABS(point.x - s_last_tap_point.x) <= DOUBLE_TAP_SLOP_PX &&
       LV_ABS(point.y - s_last_tap_point.y) <= DOUBLE_TAP_SLOP_PX)
    {
        s_have_tap = false;
        menu_open();
        return;
    }

    s_have_tap = true;
    s_last_tap_ms = lv_tick_get();
    s_last_tap_point = point;
}

/*
 * A swipe left or right steps through the instruments, round and round:
 * left for the next, right for the previous. Settings is not in the
 * round - it is reached from the menu, and swiping on it does nothing.
 * A swipe down on the HSI opens its route menu, on the engine screens
 * what they show.
 */
static const screen_id_t SWIPE_ORDER[] = {
    SCREEN_HORIZON,
    SCREEN_ALTIMETER,
    SCREEN_TURN_COORDINATOR,
    SCREEN_COMPASS,
    SCREEN_HSI,
    SCREEN_ENGINE,
    SCREEN_CYLINDERS,
};

static const int SWIPE_COUNT =
    (int)(sizeof(SWIPE_ORDER) / sizeof(SWIPE_ORDER[0]));

static bool is_instrument(screen_id_t screen)
{
    for(int i = 0; i < SWIPE_COUNT; i++)
        if(SWIPE_ORDER[i] == screen)
            return true;

    return false;
}

bool screen_manager_visible(screen_id_t screen)
{
    return screen < SCREEN_COUNT &&
           (app_settings().hidden_screens & (1u << screen)) == 0;
}

static void on_gesture(lv_event_t *e)
{
    (void)e;

    if(menu_is_open())
        return;

    lv_indev_t *indev = lv_indev_get_act();
    const lv_dir_t dir = lv_indev_get_gesture_dir(indev);

    /* Down on the HSI, from the top: its route menu. */
    if(dir == LV_DIR_BOTTOM && s_current == SCREEN_HSI)
    {
        lv_indev_wait_release(indev);
        s_have_tap = false;
        hsi_screen_open_menu();
        return;
    }

    /* And on the engine screens: what they show. */
    if(dir == LV_DIR_BOTTOM &&
       (s_current == SCREEN_ENGINE || s_current == SCREEN_CYLINDERS))
    {
        lv_indev_wait_release(indev);
        s_have_tap = false;
        engine_screen_open_menu();
        return;
    }

    if(dir != LV_DIR_LEFT && dir != LV_DIR_RIGHT)
        return;

    int position = -1;

    for(int i = 0; i < SWIPE_COUNT; i++)
        if(SWIPE_ORDER[i] == s_current)
            position = i;

    if(position < 0)
        return;

    const int step = (dir == LV_DIR_LEFT) ? 1 : SWIPE_COUNT - 1;
    int next = position;

    /* On round to the next one shown; none other shown, nowhere to go. */
    do
        next = (next + step) % SWIPE_COUNT;
    while(next != position && !screen_manager_visible(SWIPE_ORDER[next]));

    if(next == position)
        return;

    /*
     * The finger is still down. Without this its release would still
     * count as a tap - half a double tap - or, if the swipe began on a
     * control, as a press of it: the altimeter's setting would move.
     */
    lv_indev_wait_release(indev);
    s_have_tap = false;

    screen_manager_show(SWIPE_ORDER[next]);
}

static void on_menu_button(lv_event_t *e)
{
    const screen_id_t screen =
        (screen_id_t)(intptr_t)lv_event_get_user_data(e);

    menu_close();
    screen_manager_show(screen);
}

static void on_backdrop(lv_event_t *e)
{
    (void)e;
    menu_close();
}

static void create_menu(void)
{
    /*
     * While the menu is up, an invisible full screen backdrop under it
     * takes every tap that misses the menu and closes it - and keeps
     * those taps off the screen beneath. With no style it costs nothing
     * to draw, and it is hidden the rest of the time so the screens get
     * their own input.
     */
    s_backdrop = lv_obj_create(lv_layer_top());

    lv_obj_remove_style_all(s_backdrop);
    lv_obj_set_size(s_backdrop, LV_PCT(100), LV_PCT(100));
    lv_obj_add_flag(s_backdrop, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(s_backdrop, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_backdrop, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(s_backdrop, on_backdrop, LV_EVENT_CLICKED, NULL);

    s_menu = lv_obj_create(lv_layer_top());

    /* Two columns: eight screens in one would run off the round panel. */
    lv_obj_set_size(s_menu, 320, LV_SIZE_CONTENT);
    lv_obj_center(s_menu);
    lv_obj_clear_flag(s_menu, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_set_style_bg_color(s_menu, lv_color_hex(MENU_BG_COLOUR), 0);
    lv_obj_set_style_bg_opa(s_menu, LV_OPA_90, 0);
    lv_obj_set_style_border_color(s_menu, lv_color_hex(MENU_BORDER_COLOUR), 0);
    lv_obj_set_style_border_width(s_menu, 2, 0);
    lv_obj_set_style_radius(s_menu, 20, 0);
    lv_obj_set_style_pad_all(s_menu, 14, 0);
    lv_obj_set_style_pad_row(s_menu, 8, 0);
    lv_obj_set_style_pad_column(s_menu, 8, 0);

    lv_obj_set_flex_flow(s_menu, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(s_menu, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t *title = lv_label_create(s_menu);

    /* A full row of its own, so the buttons start below it. */
    lv_obj_set_width(title, LV_PCT(100));
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(title, "MENU");
    lv_obj_set_style_text_color(title, lv_color_hex(0xA0A8B0), 0);
    lv_obj_set_style_text_font(title, FONT_BODY, 0);

    for(int i = 0; i < SCREEN_COUNT; i++)
    {
        lv_obj_t *button = lv_btn_create(s_menu);

        lv_obj_set_size(button, 140, 48);
        lv_obj_set_style_radius(button, 12, 0);
        lv_obj_set_style_shadow_width(button, 0, 0);

        lv_obj_add_event_cb(button, on_menu_button, LV_EVENT_CLICKED,
                            (void *)(intptr_t)i);

        lv_obj_t *label = lv_label_create(button);

        lv_label_set_text(label, SCREEN_NAMES[i]);
        lv_obj_set_width(label, 130);
        lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(label, lv_color_white(), 0);
        lv_obj_set_style_text_font(label, FONT_LARGE, 0);
        lv_obj_center(label);

        s_menu_buttons[i] = button;
    }

    lv_obj_add_flag(s_menu, LV_OBJ_FLAG_HIDDEN);
}

static lv_obj_t *create_screen(void)
{
    lv_obj_t *screen = lv_obj_create(NULL);

    lv_obj_set_style_bg_color(screen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

    return screen;
}

static void take_request(lv_timer_t *timer);

void screen_manager_create(lv_obj_t *horizon_screen)
{
    lv_timer_create(take_request, 100, NULL);

    s_screens[SCREEN_HORIZON] = horizon_screen;

    s_screens[SCREEN_ALTIMETER] = create_screen();
    altimeter_screen_create(s_screens[SCREEN_ALTIMETER]);

    s_screens[SCREEN_TURN_COORDINATOR] = create_screen();
    turn_coordinator_screen_create(s_screens[SCREEN_TURN_COORDINATOR]);

    s_screens[SCREEN_COMPASS] = create_screen();
    compass_screen_create(s_screens[SCREEN_COMPASS]);

    s_screens[SCREEN_HSI] = create_screen();
    hsi_screen_create(s_screens[SCREEN_HSI]);

    s_screens[SCREEN_ENGINE] = create_screen();
    engine_screen_create(s_screens[SCREEN_ENGINE]);

    s_screens[SCREEN_CYLINDERS] = create_screen();
    cylinders_screen_create(s_screens[SCREEN_CYLINDERS]);

    s_screens[SCREEN_SETTINGS] = create_screen();
    settings_screen_create(s_screens[SCREEN_SETTINGS]);

    for(int i = 0; i < SCREEN_COUNT; i++)
    {
        lv_obj_add_flag(s_screens[i], LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(s_screens[i], on_tap, LV_EVENT_SHORT_CLICKED,
                            NULL);
        lv_obj_add_event_cb(s_screens[i], on_gesture, LV_EVENT_GESTURE,
                            NULL);
    }

    create_menu();

    s_current = SCREEN_HORIZON;

    /* Back where it was left - unless it has been hidden since, then
     * the first instrument shown. */
    screen_id_t start = (screen_id_t)app_settings().screen;

    if(start >= SCREEN_COUNT || !screen_manager_visible(start))
    {
        start = SCREEN_SETTINGS;

        for(int i = SWIPE_COUNT - 1; i >= 0; i--)
            if(screen_manager_visible(SWIPE_ORDER[i]))
                start = SWIPE_ORDER[i];
    }

    screen_manager_show(start);
}

void screen_manager_show(screen_id_t screen)
{
    if(screen >= SCREEN_COUNT || screen == s_current)
        return;

    s_current = screen;
    lv_scr_load(s_screens[screen]);

    app_settings_set_screen((uint8_t)screen);
}

/* Set from any task, acted on by the LVGL task; -1 for nothing asked.
 * The screen, with REQUEST_MENU for its menu too. */
static volatile int s_requested = -1;
static const int REQUEST_MENU = 0x100;

void screen_manager_request(screen_id_t screen, bool open_menu)
{
    if(screen < SCREEN_COUNT)
        s_requested = (int)screen | (open_menu ? REQUEST_MENU : 0);
}

static void take_request(lv_timer_t *timer)
{
    (void)timer;

    const int requested = s_requested;

    if(requested < 0)
        return;

    s_requested = -1;
    menu_close();

    const screen_id_t screen = (screen_id_t)(requested & ~REQUEST_MENU);

    screen_manager_show(screen);

    if((requested & REQUEST_MENU) == 0)
        return;

    if(screen == SCREEN_HSI)
        hsi_screen_open_menu();
    else if(screen == SCREEN_ENGINE || screen == SCREEN_CYLINDERS)
        engine_screen_open_menu();
}

int screen_manager_instrument_count(void)
{
    return SWIPE_COUNT;
}

screen_id_t screen_manager_instrument(int position)
{
    return SWIPE_ORDER[position];
}

const char *screen_manager_name(screen_id_t screen)
{
    return (screen < SCREEN_COUNT) ? SCREEN_NAMES[screen] : "";
}

bool screen_manager_set_visible(screen_id_t screen, bool visible)
{
    if(!is_instrument(screen))
        return false;

    const uint8_t bit = (uint8_t)(1u << screen);
    const uint8_t hidden = app_settings().hidden_screens;
    const uint8_t mask = visible ? (uint8_t)(hidden & ~bit)
                                 : (uint8_t)(hidden | bit);

    /* At least one instrument stays. */
    bool any = false;

    for(int i = 0; i < SWIPE_COUNT; i++)
        any = any || (mask & (1u << SWIPE_ORDER[i])) == 0;

    if(!any)
        return false;

    app_settings_set_hidden_screens(mask);
    return true;
}
