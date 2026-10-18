#include "settings_screen.h"
#include "fonts.h"
#include "alignment.h"
#include "altimeter_screen.h"
#include "altitude_widget.h"
#include "app_settings.h"
#include "diag_stream.h"
#include "navigation.h"
#include "display_st7701.h"
#include "engine_limits.h"
#include "magnetometer.h"
#include "screen_manager.h"
#include "sensor_readings.h"
#include "wifi_debug.h"

#include "mag_qmc5883.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static const uint32_t UPDATE_PERIOD_MS = 200;
static const uint32_t CALIBRATION_UPDATE_MS = 100;

/* Inside the round panel at every height the content scrolls through. */
static const lv_coord_t CONTENT_WIDTH = 320;

static const uint32_t SECTION_COLOUR = 0xFFD400;
static const uint32_t TEXT_COLOUR = 0xFFFFFF;
static const uint32_t HINT_COLOUR = 0x808890;
static const uint32_t GOOD_COLOUR = 0x30D040;
static const uint32_t WARNING_COLOUR = 0xFFD400;
static const uint32_t FAILED_COLOUR = 0xFF4040;
static const uint32_t BUTTON_COLOUR = 0x2A3440;
static const uint32_t SELECTED_COLOUR = 0x1987D1;

/*
 * The pages. The main one lists the others; each has a Back button to
 * where it was opened from. Only the page showing exists: it is built
 * when opened and deleted when left, as LVGL's heap has little room.
 */
typedef enum {
    PAGE_MAIN = 0,
    PAGE_DISPLAY,
    PAGE_SCREENS,
    PAGE_ENGINE,
    PAGE_MOUNTING,
    PAGE_WIFI,
    PAGE_COMPASS,
    PAGE_CUSTOM,                /* the custom engine's values */
    PAGE_CUSTOM_GENERAL,        /* its cylinders and cooling */
    PAGE_PARAM,                 /* + engine_param_t: one value's limits */
} page_t;

static lv_obj_t *s_screen = NULL;
static int s_page = PAGE_MAIN;
static bool s_calibrating = false;

/* On the compass page; NULL elsewhere. */
static lv_obj_t *s_live = NULL;
static lv_obj_t *s_calibration_status = NULL;
static lv_obj_t *s_clear_button = NULL;

/* Calibration, a screen of its own. */
static lv_obj_t *s_calibration_screen = NULL;
static lv_obj_t *s_coverage_bar = NULL;
static lv_obj_t *s_calibration_progress = NULL;
static lv_obj_t *s_save_button = NULL;
static lv_timer_t *s_calibration_timer = NULL;

static void show_page(int page);

/* ---- helpers ------------------------------------------------------------- */

static lv_obj_t *make_text(lv_obj_t *parent, const lv_font_t *font,
                           uint32_t colour, const char *text)
{
    lv_obj_t *label = lv_label_create(parent);

    lv_obj_set_width(label, CONTENT_WIDTH);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(colour), 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_label_set_text(label, text);

    return label;
}

static lv_obj_t *make_button(lv_obj_t *parent, const char *text,
                             lv_coord_t width, lv_event_cb_t on_click)
{
    lv_obj_t *button = lv_btn_create(parent);

    lv_obj_set_size(button, width, 48);
    lv_obj_set_style_radius(button, 12, 0);
    lv_obj_set_style_shadow_width(button, 0, 0);
    lv_obj_set_style_bg_color(button, lv_color_hex(BUTTON_COLOUR), 0);
    lv_obj_set_style_bg_color(button, lv_color_hex(0x1A2028),
                              LV_STATE_DISABLED);
    lv_obj_add_event_cb(button, on_click, LV_EVENT_CLICKED, NULL);

    lv_obj_t *label = lv_label_create(button);

    lv_label_set_text(label, text);
    lv_obj_set_style_text_font(label, FONT_LARGE, 0);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_set_style_text_color(label, lv_color_hex(0x606870),
                                LV_STATE_DISABLED);
    lv_obj_center(label);

    return button;
}

/* A row: a name on the left, a switch on the right. Returns the switch. */
static lv_obj_t *make_switch_row(lv_obj_t *parent, const char *text,
                                 const lv_font_t *font, bool checked,
                                 lv_event_cb_t on_change, void *user_data)
{
    lv_obj_t *row = lv_obj_create(parent);

    lv_obj_remove_style_all(row);
    lv_obj_set_size(row, 260, 40);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *label = lv_label_create(row);

    lv_label_set_text(label, text);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(TEXT_COLOUR), 0);
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 0, 0);

    lv_obj_t *toggle = lv_switch_create(row);

    lv_obj_set_size(toggle, 70, 36);
    lv_obj_align(toggle, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_set_style_bg_color(toggle, lv_color_hex(0x303840), LV_PART_MAIN);
    lv_obj_set_style_bg_color(toggle, lv_color_hex(SELECTED_COLOUR),
                              (uint32_t)LV_PART_INDICATOR | LV_STATE_CHECKED);

    if(checked)
        lv_obj_add_state(toggle, LV_STATE_CHECKED);

    lv_obj_add_event_cb(toggle, on_change, LV_EVENT_VALUE_CHANGED, user_data);

    return toggle;
}

/*
 * A grid of buttons, styled as the rest. They report on release only
 * (click triggered): by default a button matrix reports on the press,
 * and again for every button a finger slides onto. Checked ones are set
 * by the caller, from the settings.
 */
static lv_obj_t *make_matrix(lv_obj_t *parent, const char *const *map,
                             lv_coord_t width, lv_coord_t height,
                             lv_event_cb_t on_click)
{
    lv_obj_t *matrix = lv_btnmatrix_create(parent);

    lv_btnmatrix_set_map(matrix, (const char **)map);
    lv_btnmatrix_set_btn_ctrl_all(matrix, LV_BTNMATRIX_CTRL_CLICK_TRIG);

    lv_obj_set_size(matrix, width, height);
    lv_obj_set_style_bg_opa(matrix, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(matrix, 0, 0);
    lv_obj_set_style_pad_all(matrix, 0, 0);
    lv_obj_set_style_pad_gap(matrix, 8, 0);
    lv_obj_set_style_bg_color(matrix, lv_color_hex(BUTTON_COLOUR),
                              LV_PART_ITEMS);
    lv_obj_set_style_bg_color(matrix, lv_color_hex(SELECTED_COLOUR),
                              (uint32_t)LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_text_color(matrix, lv_color_white(), LV_PART_ITEMS);
    lv_obj_set_style_text_font(matrix, FONT_LARGE, LV_PART_ITEMS);
    lv_obj_set_style_radius(matrix, 10, LV_PART_ITEMS);
    lv_obj_set_style_shadow_width(matrix, 0, LV_PART_ITEMS);
    lv_obj_add_event_cb(matrix, on_click, LV_EVENT_VALUE_CHANGED, NULL);

    return matrix;
}

/* The one checked button of a matrix used as a choice. */
static void check_only(lv_obj_t *matrix, uint16_t button)
{
    lv_btnmatrix_clear_btn_ctrl_all(matrix, LV_BTNMATRIX_CTRL_CHECKED);
    lv_btnmatrix_set_btn_ctrl(matrix, button, LV_BTNMATRIX_CTRL_CHECKED);
}

static uint16_t selected(lv_event_t *e)
{
    return lv_btnmatrix_get_selected_btn(lv_event_get_target(e));
}

/* On a page that may have gone: NULL is left alone. */
static void set_status(lv_obj_t *label, uint32_t colour, const char *text)
{
    if(label == NULL)
        return;

    lv_label_set_text(label, text);
    lv_obj_set_style_text_color(label, lv_color_hex(colour), 0);
}

static void set_enabled(lv_obj_t *obj, bool enabled)
{
    if(obj == NULL)
        return;

    if(enabled)
        lv_obj_clear_state(obj, LV_STATE_DISABLED);
    else
        lv_obj_add_state(obj, LV_STATE_DISABLED);
}

/* ---- calibration screen -------------------------------------------------- */

static void close_calibration(void)
{
    if(s_calibration_timer != NULL)
    {
        lv_timer_del(s_calibration_timer);
        s_calibration_timer = NULL;
    }

    s_calibrating = false;
    lv_scr_load(s_screen);
}

static void update_calibration(lv_timer_t *timer)
{
    (void)timer;

    const MagCalibrator::Progress progress =
        magnetometer_calibration_progress();

    lv_bar_set_value(s_coverage_bar, (int32_t)(progress.coverage * 100.0f),
                     LV_ANIM_OFF);

    /* What is holding it up, worst first. */
    const MagCalibratorConfig limits;
    char text[96];
    uint32_t colour = HINT_COLOUR;

    if(progress.ready)
    {
        snprintf(text, sizeof(text), "READY\nSave, or turn round again "
                 "to refine it");
        colour = GOOD_COLOUR;
    }
    else if(progress.tilt_spread_deg > limits.max_tilt_spread_deg)
    {
        snprintf(text, sizeof(text), "Keep it level - tilted %.0f deg\n"
                 "Cancel and start again", progress.tilt_spread_deg);
        colour = WARNING_COLOUR;
    }
    else if(progress.coverage > 0.9f &&
            progress.fit_error > limits.max_fit_error)
    {
        snprintf(text, sizeof(text), "Field disturbed while turning:\n"
                 "move away from steel, start again");
        colour = WARNING_COLOUR;
    }
    else
    {
        snprintf(text, sizeof(text), "%.0f%% of the circle\nkeep turning",
                 progress.coverage * 100.0f);
    }

    set_status(s_calibration_progress, colour, text);

    if(progress.ready)
        lv_obj_clear_state(s_save_button, LV_STATE_DISABLED);
    else
        lv_obj_add_state(s_save_button, LV_STATE_DISABLED);
}

static void on_calibrate(lv_event_t *e)
{
    (void)e;

    magnetometer_calibration_start();

    /* The settings screen unloads, but stays on this page. */
    s_calibrating = true;
    lv_scr_load(s_calibration_screen);
    update_calibration(NULL);

    s_calibration_timer =
        lv_timer_create(update_calibration, CALIBRATION_UPDATE_MS, NULL);
}

static void on_save(lv_event_t *e)
{
    (void)e;

    if(!magnetometer_calibration_finish())
        return;

    close_calibration();
}

static void on_cancel(lv_event_t *e)
{
    (void)e;

    magnetometer_calibration_cancel();
    close_calibration();
}

static void on_clear(lv_event_t *e)
{
    (void)e;

    magnetometer_clear_calibration();
}

/*
 * A separate screen rather than an overlay on this one. With full
 * screen redraws, anything left underneath an overlay is still painted
 * every frame and then covered; as a screen of its own, only this is
 * drawn. It is not one of screen_manager.cpp's screens, so it has no double tap
 * handler: the menu stays shut until calibration is saved or cancelled.
 */
static void create_calibration_screen(void)
{
    lv_obj_t *const screen = lv_obj_create(NULL);

    s_calibration_screen = screen;

    lv_obj_set_style_bg_color(screen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_top(screen, 56, 0);
    lv_obj_set_style_pad_row(screen, 10, 0);
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_set_flex_flow(screen, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(screen, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    make_text(screen, FONT_LARGE, TEXT_COLOUR,
              "CALIBRATING");

    lv_obj_t *instructions = make_text(
        screen, FONT_BODY, HINT_COLOUR,
        "Keep it level, as installed, and turn it slowly once round - "
        "a full circle. Away from steel and other aircraft.");
    lv_obj_set_width(instructions, 300);

    lv_obj_t *bar = lv_bar_create(screen);

    lv_obj_set_size(bar, 260, 16);
    lv_bar_set_range(bar, 0, 100);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0x303840), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(bar, lv_color_hex(SELECTED_COLOUR),
                              LV_PART_INDICATOR);

    s_coverage_bar = bar;

    s_calibration_progress =
        make_text(screen, FONT_BODY, HINT_COLOUR, "");

    lv_obj_t *buttons = lv_obj_create(screen);

    lv_obj_remove_style_all(buttons);
    lv_obj_set_size(buttons, 280, 48);

    lv_obj_t *cancel = make_button(buttons, "Cancel", 130, on_cancel);
    lv_obj_align(cancel, LV_ALIGN_LEFT_MID, 0, 0);

    s_save_button = make_button(buttons, "Save", 130, on_save);
    lv_obj_align(s_save_button, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_state(s_save_button, LV_STATE_DISABLED);
}

/* ---- display --------------------------------------------------------------- */

static const char *const UNITS_MAP[] = { "m, m/s", "ft, ft/min", "" };

static lv_obj_t *s_brightness_label = NULL;

static void on_units(lv_event_t *e)
{
    const uint16_t index = selected(e);

    if(index > 1)
        return;

    const units_t units = (index == 1) ? UNITS_AVIATION : UNITS_METRIC;

    app_settings_set_units(units);
    altitude_widget_set_units(units == UNITS_AVIATION ? ALTITUDE_UNITS_AVIATION
                                                      : ALTITUDE_UNITS_METRIC);
    altimeter_screen_units_changed();

    check_only(lv_event_get_target(e), index);
}

static void show_brightness(uint8_t percent)
{
    char text[24];
    snprintf(text, sizeof(text), "Backlight %u%%", (unsigned)percent);
    set_status(s_brightness_label, TEXT_COLOUR, text);
}

static void on_brightness(lv_event_t *e)
{
    const uint8_t percent =
        (uint8_t)lv_slider_get_value(lv_event_get_target(e));

    display_set_backlight(percent);
    app_settings_set_brightness(percent);
    show_brightness(percent);
}

static void build_display(lv_obj_t *screen)
{
    lv_obj_t *units = make_matrix(screen, UNITS_MAP, 260, 44, on_units);
    lv_obj_set_style_text_font(units, FONT_BODY, LV_PART_ITEMS);
    check_only(units, app_settings().units == UNITS_AVIATION ? 1 : 0);

    s_brightness_label = make_text(screen, FONT_BODY, TEXT_COLOUR, "");

    lv_obj_t *slider = lv_slider_create(screen);

    lv_obj_set_width(slider, 240);
    lv_slider_set_range(slider, 5, 100);
    lv_slider_set_value(slider, app_settings().brightness, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(slider, lv_color_hex(0x303840), LV_PART_MAIN);
    lv_obj_set_style_bg_color(slider, lv_color_hex(SELECTED_COLOUR),
                              LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(slider, lv_color_white(), LV_PART_KNOB);
    lv_obj_add_event_cb(slider, on_brightness, LV_EVENT_VALUE_CHANGED, NULL);

    show_brightness(app_settings().brightness);
}

/* ---- screens ---------------------------------------------------------------- */

static void on_screen_shown(lv_event_t *e)
{
    lv_obj_t *toggle = lv_event_get_target(e);
    const screen_id_t screen =
        (screen_id_t)(intptr_t)lv_event_get_user_data(e);
    const bool shown = lv_obj_has_state(toggle, LV_STATE_CHECKED);

    /* Refused for the last one shown: back on. */
    if(!screen_manager_set_visible(screen, shown))
        lv_obj_add_state(toggle, LV_STATE_CHECKED);
}

static void build_screens(lv_obj_t *screen)
{
    for(int i = 0; i < screen_manager_instrument_count(); i++)
    {
        const screen_id_t id = screen_manager_instrument(i);

        make_switch_row(screen, screen_manager_name(id), FONT_BODY,
                        screen_manager_visible(id), on_screen_shown,
                        (void *)(intptr_t)id);
    }

    make_text(screen, FONT_SMALL, HINT_COLOUR,
              "Off: left out of the swipes and the menu - the HSI "
              "with no GPS fitted, say. One stays on.");
}

/* ---- engine ----------------------------------------------------------------- */

static const char *const ENGINE_MAP[] = {
    "Rotax 912", "O-320", "\n", "IO-540", "Custom", ""
};

static const char *const COPY_MAP[] = { "912", "O-320", "IO-540", "" };

static void on_engine_type(lv_event_t *e)
{
    const uint16_t index = selected(e);

    if(index >= ENGINE_TYPE_COUNT || index == app_settings().engine_type)
        return;

    app_settings_set_engine_type((uint8_t)index);

    /* A new engine: its own values, until set otherwise in the engine
     * screens' menu. */
    app_settings_set_engine_items(0);

    /* Again, for the custom engine's buttons to come or go. */
    show_page(PAGE_ENGINE);
}

static void on_edit_custom(lv_event_t *e)
{
    (void)e;
    show_page(PAGE_CUSTOM);
}

static lv_obj_t *s_copy_status = NULL;

static void on_copy(lv_event_t *e)
{
    const uint16_t index = selected(e);

    if(index >= ENGINE_PRESET_COUNT)
        return;

    engine_custom_copy((engine_type_t)index);
    engine_custom_changed();

    /* Its values, as for a new engine. */
    app_settings_set_engine_items(0);

    char text[64];
    snprintf(text, sizeof(text), "Copied from %s",
             engine_type_name((engine_type_t)index));
    set_status(s_copy_status, GOOD_COLOUR, text);
}

static void build_engine(lv_obj_t *screen)
{
    lv_obj_t *types = make_matrix(screen, ENGINE_MAP, 300, 2 * 44 + 8,
                                  on_engine_type);
    check_only(types, app_settings().engine_type % ENGINE_TYPE_COUNT);

    if(app_settings().engine_type == ENGINE_TYPE_CUSTOM)
    {
        make_button(screen, "Edit custom engine", 260, on_edit_custom);

        make_text(screen, FONT_SMALL, SECTION_COLOUR,
                  "START THE CUSTOM ENGINE FROM");
        make_matrix(screen, COPY_MAP, 300, 44, on_copy);

        s_copy_status = make_text(screen, FONT_SMALL, HINT_COLOUR,
                                  "Replaces all its limits");
    }

    make_text(screen, FONT_SMALL, HINT_COLOUR,
              "The green, amber and red ranges on the engine screens. "
              "The types' are their usual limits, the custom engine's "
              "are set here - check them against the engine's manual.");
}

/* ---- rows of numbers --------------------------------------------------------- */

/*
 * A page of numbers, each set with - and +, as a row: its name, then
 * "-", the value and "+" in one button matrix for the whole page - a
 * row of objects each would not fit LVGL's heap. Tapping a value does
 * what the page says (switches a limit off and on). The values are
 * drawn from s_number_text, so a change shows on redrawing.
 */
#define MAX_ROWS 6

static const lv_coord_t ROW_PITCH = 50;
static const lv_coord_t ROW_HEIGHT = 42;

typedef struct {
    int rows;
    const char *names[MAX_ROWS];
    void (*format)(int row, char *text, size_t size);
    void (*step)(int row, int direction);   /* -1 or +1 */
    void (*tap)(int row);                   /* the value tapped */
} number_page_t;

static number_page_t s_numbers;
static lv_obj_t *s_number_matrix = NULL;
static char s_number_text[MAX_ROWS][16];
static const char *s_number_map[MAX_ROWS * 4 + 1];

static void show_numbers(void)
{
    for(int i = 0; i < s_numbers.rows; i++)
        s_numbers.format(i, s_number_text[i], sizeof(s_number_text[i]));

    if(s_number_matrix != NULL)
        lv_obj_invalidate(s_number_matrix);
}

static void on_number(lv_event_t *e)
{
    const uint16_t button = selected(e);
    const int row = button / 3;

    if(row >= s_numbers.rows)
        return;

    switch(button % 3)
    {
    case 0:  s_numbers.step(row, -1); break;
    case 2:  s_numbers.step(row, +1); break;
    default:
        if(s_numbers.tap != NULL)
            s_numbers.tap(row);
        break;
    }

    show_numbers();
}

static void build_numbers(lv_obj_t *screen)
{
    const int rows = s_numbers.rows;
    lv_obj_t *box = lv_obj_create(screen);

    lv_obj_remove_style_all(box);
    /* Narrow enough for the last row, low on the round panel. */
    lv_obj_set_size(box, 296, ROW_PITCH * rows - (ROW_PITCH - ROW_HEIGHT));
    lv_obj_clear_flag(box, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);

    int n = 0;

    for(int i = 0; i < rows; i++)
    {
        lv_obj_t *name = lv_label_create(box);

        lv_label_set_text(name, s_numbers.names[i]);
        lv_obj_set_style_text_font(name, FONT_SMALL, 0);
        lv_obj_set_style_text_color(name, lv_color_hex(TEXT_COLOUR), 0);
        lv_obj_align(name, LV_ALIGN_TOP_LEFT, 0,
                     ROW_PITCH * i + ROW_HEIGHT / 2 - 9);

        s_number_map[n++] = "-";
        s_number_map[n++] = s_number_text[i];
        s_number_map[n++] = "+";
        s_number_map[n++] = (i + 1 < rows) ? "\n" : "";
    }

    s_number_map[n] = "";

    s_number_matrix = make_matrix(box, s_number_map, 184,
                                  ROW_PITCH * rows - (ROW_PITCH - ROW_HEIGHT),
                                  on_number);
    lv_obj_set_style_pad_row(s_number_matrix, ROW_PITCH - ROW_HEIGHT, 0);
    lv_obj_set_style_pad_column(s_number_matrix, 6, 0);
    lv_obj_align(s_number_matrix, LV_ALIGN_TOP_RIGHT, 0, 0);

    /* The value twice as wide as - and +, and held down it doesn't
     * repeat: a tap is a switch. */
    for(int i = 0; i < rows; i++)
    {
        lv_btnmatrix_set_btn_width(s_number_matrix, (uint16_t)(i * 3 + 1), 2);
        lv_btnmatrix_set_btn_ctrl(s_number_matrix, (uint16_t)(i * 3 + 1),
                                  LV_BTNMATRIX_CTRL_NO_REPEAT);
    }

    show_numbers();
}

/* ---- the custom engine ------------------------------------------------------- */

static const char *s_custom_map[(1 + ENGINE_PARAM_COUNT) * 2];

static void on_custom_value(lv_event_t *e)
{
    const uint16_t button = selected(e);

    if(button == 0)
        show_page(PAGE_CUSTOM_GENERAL);
    else if(button <= ENGINE_PARAM_COUNT)
        show_page(PAGE_PARAM + button - 1);
}

static void build_custom(lv_obj_t *screen)
{
    int n = 0;

    s_custom_map[n++] = "Cylinders, cooling";

    for(int i = 0; i < ENGINE_PARAM_COUNT; i++)
    {
        s_custom_map[n++] = "\n";
        s_custom_map[n++] = engine_param_info((engine_param_t)i)->name;
    }

    s_custom_map[n] = "";

    const int rows = 1 + ENGINE_PARAM_COUNT;

    make_text(screen, FONT_SMALL, HINT_COLOUR,
              "Each value's scale, and where its amber and red begin.");
    make_matrix(screen, s_custom_map, 280, rows * 44 + (rows - 1) * 8,
                on_custom_value);
}

/* Cylinders and cooling. */

static void general_format(int row, char *text, size_t size)
{
    const engine_limits_t *l = engine_custom();

    if(row == 0)
        snprintf(text, size, "%d", l->cylinders);
    else
        snprintf(text, size, "%s", l->coolant ? "yes" : "no");
}

static void general_step(int row, int direction)
{
    engine_limits_t *l = engine_custom();

    if(row == 0)
    {
        const int cylinders = l->cylinders + direction;

        if(cylinders < 1 || cylinders > ENGINE_MAX_CYLINDERS)
            return;

        l->cylinders = cylinders;
    }
    else
    {
        l->coolant = !l->coolant;

        /* Its default bars change: coolant or EGT. */
        app_settings_set_engine_items(0);
    }

    engine_custom_changed();
}

static void general_tap(int row)
{
    if(row == 1)
        general_step(row, 1);
}

static void build_custom_general(lv_obj_t *screen)
{
    s_numbers.rows = 2;
    s_numbers.names[0] = "Cylinders";
    s_numbers.names[1] = "Liquid cooled";
    s_numbers.format = general_format;
    s_numbers.step = general_step;
    s_numbers.tap = general_tap;

    build_numbers(screen);

    make_text(screen, FONT_SMALL, HINT_COLOUR,
              "Liquid cooled: the engine screen shows the coolant "
              "temperature, else the hottest EGT.");
}

/* One value's scale and limits. */

static engine_param_t s_param = ENGINE_PARAM_RPM;

static float *range_field(engine_range_t *r, int row)
{
    float *const fields[6] = { &r->min, &r->max, &r->red_low,
                               &r->amber_low, &r->amber_high, &r->red_high };
    return fields[row];
}

static void param_format(int row, char *text, size_t size)
{
    const engine_param_info_t *info = engine_param_info(s_param);
    const float value = *range_field(&engine_custom()->range[s_param], row);

    if(isnan(value))
        snprintf(text, size, "off");
    else
        snprintf(text, size, "%.*f", info->decimals, value * info->scale);
}

/* A value as shown, back on the step grid: no creeping 0.30000001. */
static float on_grid(float shown, float step)
{
    return roundf(shown / step) * step;
}

static void param_step(int row, int direction)
{
    const engine_param_info_t *info = engine_param_info(s_param);
    engine_range_t *r = &engine_custom()->range[s_param];
    float *field = range_field(r, row);

    /* A limit that is off comes on in the middle of the scale. */
    float shown = isnan(*field) ? (r->min + r->max) * 0.5f * info->scale
                                : *field * info->scale +
                                      info->step * (float)direction;

    shown = on_grid(shown, info->step);

    const float value = shown / info->scale;

    /* The scale keeps at least a step from bottom to top. */
    if(row == 0 && value > r->max - info->step / info->scale)
        return;
    if(row == 1 && value < r->min + info->step / info->scale)
        return;

    *field = value;
    engine_custom_changed();
}

static void param_tap(int row)
{
    /* The scale's ends can't be off. */
    if(row < 2)
        return;

    float *field = range_field(&engine_custom()->range[s_param], row);

    if(isnan(*field))
    {
        param_step(row, 0);
        return;
    }

    *field = NAN;
    engine_custom_changed();
}

static void build_param(lv_obj_t *screen)
{
    char text[48];
    snprintf(text, sizeof(text), "in %s", engine_param_info(s_param)->unit);
    make_text(screen, FONT_BODY, HINT_COLOUR, text);

    s_numbers.rows = 6;
    s_numbers.names[0] = "Scale from";
    s_numbers.names[1] = "Scale to";
    s_numbers.names[2] = "Red below";
    s_numbers.names[3] = "Amber below";
    s_numbers.names[4] = "Amber above";
    s_numbers.names[5] = "Red above";
    s_numbers.format = param_format;
    s_numbers.step = param_step;
    s_numbers.tap = param_tap;

    build_numbers(screen);

    make_text(screen, FONT_SMALL, HINT_COLOUR,
              "Tap a limit to switch it off or on. Green is between "
              "them; with none, the bar has no colours.");
}

/* ---- mounting --------------------------------------------------------------- */

static lv_obj_t *s_level_status = NULL;
static lv_obj_t *s_level_button = NULL;
static lv_obj_t *s_level_clear_button = NULL;
static lv_timer_t *s_level_timer = NULL;

static void show_alignment(void)
{
    char text[96];

    if(alignment_set())
    {
        float pitch, roll;
        mountingTilt(alignment_get(), &pitch, &roll);

        snprintf(text, sizeof(text),
                 "Levelled - unit pitched %+.1f, rolled %+.1f deg",
                 pitch, roll);
        set_status(s_level_status, GOOD_COLOUR, text);
        set_enabled(s_level_clear_button, true);
    }
    else
    {
        set_status(s_level_status, HINT_COLOUR,
                   "Not levelled - unit taken as square in the panel");
        set_enabled(s_level_clear_button, false);
    }
}

static void stop_levelling(void)
{
    if(s_level_timer != NULL)
    {
        lv_timer_del(s_level_timer);
        s_level_timer = NULL;
    }

    set_enabled(s_level_button, true);
}

/* Runs on if the page is left part way: the measurement still has to
 * be taken, only not shown. */
static void update_levelling(lv_timer_t *timer)
{
    (void)timer;

    char text[96];

    switch(alignment_level_poll())
    {
        case ALIGNMENT_MEASURING:
            return;

        case ALIGNMENT_DONE:
            stop_levelling();
            show_alignment();
            return;

        case ALIGNMENT_MOVED:
            stop_levelling();
            set_status(s_level_status, WARNING_COLOUR,
                       "Moved while measuring - hold still, try again");
            return;

        case ALIGNMENT_TOO_STEEP:
            stop_levelling();
            snprintf(text, sizeof(text),
                     "More than %.0f deg out - is the aircraft level?",
                     ALIGNMENT_MAX_TILT_DEG);
            set_status(s_level_status, FAILED_COLOUR, text);
            return;

        default:
            stop_levelling();
            show_alignment();
            return;
    }
}

static void on_level(lv_event_t *e)
{
    (void)e;

    if(s_level_timer != NULL)
        return;

    alignment_level_start();

    set_enabled(s_level_button, false);
    set_status(s_level_status, TEXT_COLOUR, "Measuring - hold still...");

    s_level_timer = lv_timer_create(update_levelling, CALIBRATION_UPDATE_MS,
                                    NULL);
}

static void on_level_clear(lv_event_t *e)
{
    (void)e;

    if(s_level_timer != NULL)
        return;

    alignment_clear();
    show_alignment();
}

static void build_mounting(lv_obj_t *screen)
{
    s_level_status = make_text(screen, FONT_SMALL, HINT_COLOUR, "");

    s_level_button = make_button(screen, "Set level", 220, on_level);
    s_level_clear_button = make_button(screen, "Clear level", 220,
                                       on_level_clear);

    make_text(screen, FONT_SMALL, HINT_COLOUR,
              "For a unit not square in the panel. With the aircraft "
              "level - in flying attitude, on level ground - and still, "
              "press Set level: the horizon then reads level. Takes a "
              "few seconds; the gyro is measured again after it.");

    show_alignment();

    if(s_level_timer != NULL)
    {
        set_enabled(s_level_button, false);
        set_status(s_level_status, TEXT_COLOUR, "Measuring - hold still...");
    }
}

/* ---- Wi-Fi debug ---------------------------------------------------------- */

static lv_obj_t *s_wifi_status = NULL;

static void show_wifi_status(void)
{
    wifi_debug_status_t status;
    wifi_debug_get_status(&status);

    char text[160];
    uint32_t colour = HINT_COLOUR;

    if(!status.enabled)
    {
        snprintf(text, sizeof(text), "Off");
    }
    else if(status.joined)
    {
        snprintf(text, sizeof(text), "http://%s\non %s", status.ip,
                 status.ssid);
        colour = GOOD_COLOUR;
    }
    else if(status.access_point)
    {
        snprintf(text, sizeof(text),
                 "Join Wi-Fi \"%s\"%s,\nthen http://192.168.4.1",
                 status.ap_ssid, status.ap_open ? " (open)" : "");
        colour = WARNING_COLOUR;
    }
    else if(status.joining)
    {
        snprintf(text, sizeof(text), "Joining %s...", status.ssid);
    }
    else
    {
        snprintf(text, sizeof(text), "Starting...");
    }

    set_status(s_wifi_status, colour, text);
}

static void update_wifi(lv_timer_t *timer)
{
    (void)timer;

    if(lv_scr_act() == s_screen && s_wifi_status != NULL)
        show_wifi_status();
}

static void on_wifi_debug(lv_event_t *e)
{
    const bool enabled =
        lv_obj_has_state(lv_event_get_target(e), LV_STATE_CHECKED);

    app_settings_set_wifi_debug(enabled);
    wifi_debug_set_enabled(enabled);
    show_wifi_status();
}

static void build_wifi(lv_obj_t *screen)
{
    make_switch_row(screen, "Wi-Fi debug", FONT_LARGE,
                    app_settings().wifi_debug, on_wifi_debug, NULL);

    s_wifi_status = make_text(screen, FONT_BODY, HINT_COLOUR, "");

    make_text(screen, FONT_SMALL, HINT_COLOUR,
              "The log, and firmware updates, in a web browser. "
              "Switching off forgets the network set on the page. "
              "No password on the page - leave it off when not needed.");

    show_wifi_status();
}

/* ---- compass --------------------------------------------------------------- */

static void update_compass(lv_timer_t *timer)
{
    (void)timer;

    if(lv_scr_act() != s_screen || s_live == NULL)
        return;

    const MagCalibration calibration = magnetometer_calibration();

    if(calibration.valid)
    {
        char text[64];

        snprintf(text, sizeof(text), "Calibrated - field measured %.1f uT",
                 calibration.field);
        set_status(s_calibration_status, GOOD_COLOUR, text);
        set_enabled(s_clear_button, true);
    }
    else
    {
        set_status(s_calibration_status, WARNING_COLOUR,
                   "Not calibrated - no heading until it is");
        set_enabled(s_clear_button, false);
    }

    sensor_readings_t readings;
    sensor_readings_get(&readings);

    if(!readings.mag_valid)
    {
        lv_label_set_text(s_live, "no reading");
        return;
    }

    const AttitudeVec3 body = magnetometer_to_body(readings.mag);

    const AttitudeSample attitude = navigation_sample().attitude;

    char heading[8];

    if(attitude.heading_valid)
        snprintf(heading, sizeof(heading), "%03d",
                 ((int)(attitude.heading_deg + 0.5f)) % 360);
    else
        snprintf(heading, sizeof(heading), "---");

    char text[96];

    snprintf(text, sizeof(text), "F %+.1f   R %+.1f   D %+.1f\nHDG %s",
             body.x, body.y, body.z, heading);
    lv_label_set_text(s_live, text);
}

static void build_compass(lv_obj_t *screen)
{
    if(!mag_present())
    {
        make_text(screen, FONT_BODY, HINT_COLOUR, "No magnetometer fitted");
        return;
    }

    char title[48];
    snprintf(title, sizeof(title), "MAGNETOMETER (%s)", mag_chip_name());
    make_text(screen, FONT_BODY, SECTION_COLOUR, title);

    s_live = make_text(screen, FONT_LARGE, TEXT_COLOUR, "");

    make_text(screen, FONT_SMALL, HINT_COLOUR,
              "To check, hold it level: D is the largest and positive "
              "(northern hemisphere). Point the nose north: F positive, "
              "R near zero. Turn right: HDG goes up.");

    make_text(screen, FONT_BODY, SECTION_COLOUR, "CALIBRATION");

    s_calibration_status = make_text(screen, FONT_SMALL,
                                     WARNING_COLOUR, "");

    make_button(screen, "Calibrate", 220, on_calibrate);
    s_clear_button = make_button(screen, "Clear calibration", 220, on_clear);

    /* Every sensor, raw, on the serial console and /stream for
     * tools/sensor_plot.py, while this page is open. */
    diag_stream_begin(DIAG_STREAM_DIAGNOSTICS);

    update_compass(NULL);
}

/* ---- the pages ---------------------------------------------------------------- */

static const char *const MAIN_MAP[] = {
    "Display", "\n", "Screens", "\n", "Engine", "\n",
    "Mounting", "\n", "Wi-Fi", "\n", "Compass", ""
};

static const page_t MAIN_PAGES[] = {
    PAGE_DISPLAY, PAGE_SCREENS, PAGE_ENGINE, PAGE_MOUNTING, PAGE_WIFI,
    PAGE_COMPASS
};

static void on_main(lv_event_t *e)
{
    const uint16_t button = selected(e);

    if(button < sizeof(MAIN_PAGES) / sizeof(MAIN_PAGES[0]))
        show_page(MAIN_PAGES[button]);
}

static void build_main(lv_obj_t *screen)
{
    const int rows = (int)(sizeof(MAIN_PAGES) / sizeof(MAIN_PAGES[0]));

    make_matrix(screen, MAIN_MAP, 240, rows * 46 + (rows - 1) * 8, on_main);

    make_text(screen, FONT_SMALL, 0x606870, "double tap for menu");
}

/* Where Back goes from a page. */
static int parent_of(int page)
{
    if(page >= PAGE_PARAM || page == PAGE_CUSTOM_GENERAL)
        return PAGE_CUSTOM;
    if(page == PAGE_CUSTOM)
        return PAGE_ENGINE;
    return PAGE_MAIN;
}

static const char *title_of(int page)
{
    switch(page)
    {
    case PAGE_MAIN:           return "SETTINGS";
    case PAGE_DISPLAY:        return "DISPLAY";
    case PAGE_SCREENS:        return "SCREENS";
    case PAGE_ENGINE:         return "ENGINE";
    case PAGE_MOUNTING:       return "MOUNTING";
    case PAGE_WIFI:           return "WI-FI DEBUG";
    case PAGE_COMPASS:        return "COMPASS";
    case PAGE_CUSTOM:         return "CUSTOM ENGINE";
    case PAGE_CUSTOM_GENERAL: return "CYLINDERS, COOLING";
    default:
        return engine_param_info((engine_param_t)(page - PAGE_PARAM))->name;
    }
}

static void on_back(lv_event_t *e)
{
    (void)e;
    show_page(parent_of(s_page));
}

static void build_page(void *argument)
{
    const int page = (int)(intptr_t)argument;

    /* Leaving the compass page: no more raw stream. */
    if(s_page == PAGE_COMPASS && page != PAGE_COMPASS)
        diag_stream_end(DIAG_STREAM_DIAGNOSTICS);

    /* What the old page's timers would have written to goes with it. */
    s_live = NULL;
    s_calibration_status = NULL;
    s_clear_button = NULL;
    s_brightness_label = NULL;
    s_level_status = NULL;
    s_level_button = NULL;
    s_level_clear_button = NULL;
    s_wifi_status = NULL;
    s_copy_status = NULL;
    s_number_matrix = NULL;

    lv_obj_clean(s_screen);
    lv_obj_scroll_to_y(s_screen, 0, LV_ANIM_OFF);

    s_page = page;

    if(page != PAGE_MAIN)
    {
        lv_obj_t *back = make_button(s_screen, LV_SYMBOL_LEFT "  Back", 140,
                                     on_back);
        lv_obj_set_height(back, 40);
    }

    make_text(s_screen, FONT_LARGE, TEXT_COLOUR, title_of(page));

    if(page >= PAGE_PARAM && page < (int)PAGE_PARAM + (int)ENGINE_PARAM_COUNT)
    {
        s_param = (engine_param_t)(page - PAGE_PARAM);
        build_param(s_screen);
        return;
    }

    switch(page)
    {
    case PAGE_DISPLAY:        build_display(s_screen);        break;
    case PAGE_SCREENS:        build_screens(s_screen);        break;
    case PAGE_ENGINE:         build_engine(s_screen);         break;
    case PAGE_MOUNTING:       build_mounting(s_screen);       break;
    case PAGE_WIFI:           build_wifi(s_screen);           break;
    case PAGE_COMPASS:        build_compass(s_screen);        break;
    case PAGE_CUSTOM:         build_custom(s_screen);         break;
    case PAGE_CUSTOM_GENERAL: build_custom_general(s_screen); break;
    default:                  build_main(s_screen);           break;
    }
}

/*
 * Later, not now: this is usually a button's click, and the button is
 * on the page about to be deleted - LVGL must not delete an object in
 * the middle of its own event.
 */
static void show_page(int page)
{
    lv_async_call(build_page, (void *)(intptr_t)page);
}

/* Left - for another screen, not the calibration: back to the top for
 * the next time. */
static void on_unloaded(lv_event_t *e)
{
    (void)e;

    if(!s_calibrating)
        show_page(PAGE_MAIN);
}

/* ---- screen -------------------------------------------------------------- */

void settings_screen_create(lv_obj_t *screen)
{
    s_screen = screen;

    lv_obj_set_flex_flow(screen, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(screen, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_top(screen, 44, 0);
    lv_obj_set_style_pad_bottom(screen, 80, 0);
    lv_obj_set_style_pad_row(screen, 10, 0);
    lv_obj_add_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(screen, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(screen, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_event_cb(screen, on_unloaded, LV_EVENT_SCREEN_UNLOADED, NULL);

    build_page((void *)(intptr_t)PAGE_MAIN);

    create_calibration_screen();

    lv_timer_create(update_compass, UPDATE_PERIOD_MS, NULL);
    lv_timer_create(update_wifi, 500, NULL);
}
