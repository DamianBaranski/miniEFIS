#include "settings_screen.h"
#include "fonts.h"
#include "alignment.h"
#include "altimeter_screen.h"
#include "altitude_widget.h"
#include "app_settings.h"
#include "navigation.h"
#include "display_st7701.h"
#include "magnetometer.h"
#include "screen_manager.h"
#include "sensor_readings.h"
#include "wifi_debug.h"

#include "mag_qmc5883.h"

#include <stdio.h>

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

static lv_obj_t *s_screen = NULL;

static lv_obj_t *s_live = NULL;
static lv_obj_t *s_calibration_status = NULL;
static lv_obj_t *s_clear_button = NULL;

/* Calibration, a screen of its own. */
static lv_obj_t *s_calibration_screen = NULL;
static lv_obj_t *s_coverage_bar = NULL;
static lv_obj_t *s_calibration_progress = NULL;
static lv_obj_t *s_save_button = NULL;
static lv_timer_t *s_calibration_timer = NULL;

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

static void set_status(lv_obj_t *label, uint32_t colour, const char *text)
{
    lv_label_set_text(label, text);
    lv_obj_set_style_text_color(label, lv_color_hex(colour), 0);
}

/* ---- calibration screen -------------------------------------------------- */

static void close_calibration(void)
{
    if(s_calibration_timer != NULL)
    {
        lv_timer_del(s_calibration_timer);
        s_calibration_timer = NULL;
    }

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
    lv_obj_t *matrix = lv_event_get_target(e);
    const uint16_t index = lv_btnmatrix_get_selected_btn(matrix);

    if(index > 1)
        return;

    const units_t units = (index == 1) ? UNITS_AVIATION : UNITS_METRIC;

    app_settings_set_units(units);
    altitude_widget_set_units(units == UNITS_AVIATION ? ALTITUDE_UNITS_AVIATION
                                                      : ALTITUDE_UNITS_METRIC);
    altimeter_screen_units_changed();

    /* Keep exactly one checked, even if the checked one was tapped. */
    lv_btnmatrix_clear_btn_ctrl_all(matrix, LV_BTNMATRIX_CTRL_CHECKED);
    lv_btnmatrix_set_btn_ctrl(matrix, index, LV_BTNMATRIX_CTRL_CHECKED);
}

static void show_brightness(uint8_t percent)
{
    char text[24];
    snprintf(text, sizeof(text), "Backlight %u%%", (unsigned)percent);
    lv_label_set_text(s_brightness_label, text);
}

static void on_brightness(lv_event_t *e)
{
    const uint8_t percent =
        (uint8_t)lv_slider_get_value(lv_event_get_target(e));

    display_set_backlight(percent);
    app_settings_set_brightness(percent);
    show_brightness(percent);
}

static void make_display_section(lv_obj_t *screen)
{
    make_text(screen, FONT_BODY, SECTION_COLOUR, "DISPLAY");

    lv_obj_t *units = lv_btnmatrix_create(screen);

    lv_btnmatrix_set_map(units, (const char **)UNITS_MAP);
    lv_btnmatrix_set_btn_ctrl_all(units, LV_BTNMATRIX_CTRL_CHECKABLE);
    lv_btnmatrix_set_one_checked(units, true);
    lv_btnmatrix_set_btn_ctrl(units,
                              app_settings().units == UNITS_AVIATION ? 1 : 0,
                              LV_BTNMATRIX_CTRL_CHECKED);

    lv_obj_set_size(units, 260, 44);
    lv_obj_set_style_bg_opa(units, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(units, 0, 0);
    lv_obj_set_style_pad_all(units, 0, 0);
    lv_obj_set_style_pad_gap(units, 6, 0);
    lv_obj_set_style_bg_color(units, lv_color_hex(BUTTON_COLOUR), LV_PART_ITEMS);
    lv_obj_set_style_bg_color(units, lv_color_hex(SELECTED_COLOUR),
                              (uint32_t)LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_text_color(units, lv_color_white(), LV_PART_ITEMS);
    lv_obj_set_style_text_font(units, FONT_BODY, LV_PART_ITEMS);
    lv_obj_set_style_radius(units, 8, LV_PART_ITEMS);
    lv_obj_set_style_shadow_width(units, 0, LV_PART_ITEMS);
    lv_obj_add_event_cb(units, on_units, LV_EVENT_VALUE_CHANGED, NULL);

    s_brightness_label = make_text(screen, FONT_BODY,
                                   TEXT_COLOUR, "");

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

static void make_screens_section(lv_obj_t *screen)
{
    make_text(screen, FONT_BODY, SECTION_COLOUR, "SCREENS");

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
        lv_obj_clear_state(s_level_clear_button, LV_STATE_DISABLED);
    }
    else
    {
        set_status(s_level_status, HINT_COLOUR,
                   "Not levelled - unit taken as square in the panel");
        lv_obj_add_state(s_level_clear_button, LV_STATE_DISABLED);
    }
}

static void stop_levelling(void)
{
    if(s_level_timer != NULL)
    {
        lv_timer_del(s_level_timer);
        s_level_timer = NULL;
    }

    lv_obj_clear_state(s_level_button, LV_STATE_DISABLED);
}

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

    lv_obj_add_state(s_level_button, LV_STATE_DISABLED);
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

static void make_mounting_section(lv_obj_t *screen)
{
    make_text(screen, FONT_BODY, SECTION_COLOUR, "MOUNTING");

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

    if(lv_scr_act() == s_screen)
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

static void make_wifi_section(lv_obj_t *screen)
{
    make_text(screen, FONT_BODY, SECTION_COLOUR, "WI-FI DEBUG");

    make_switch_row(screen, "Wi-Fi debug", FONT_LARGE,
                    app_settings().wifi_debug, on_wifi_debug, NULL);

    s_wifi_status = make_text(screen, FONT_BODY, HINT_COLOUR, "");

    make_text(screen, FONT_SMALL, HINT_COLOUR,
              "The log, and firmware updates, in a web browser. "
              "Switching off forgets the network set on the page. "
              "No password on the page - leave it off when not needed.");

    show_wifi_status();
    lv_timer_create(update_wifi, 500, NULL);
}

/* ---- live values ---------------------------------------------------------- */

static void update(lv_timer_t *timer)
{
    (void)timer;

    if(lv_scr_act() != s_screen || !mag_present())
        return;

    const MagCalibration calibration = magnetometer_calibration();

    if(calibration.valid)
    {
        char text[64];

        snprintf(text, sizeof(text), "Calibrated - field measured %.1f uT",
                 calibration.field);
        set_status(s_calibration_status, GOOD_COLOUR, text);
        lv_obj_clear_state(s_clear_button, LV_STATE_DISABLED);
    }
    else
    {
        set_status(s_calibration_status, WARNING_COLOUR,
                   "Not calibrated - no heading until it is");
        lv_obj_add_state(s_clear_button, LV_STATE_DISABLED);
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

/* ---- screen -------------------------------------------------------------- */

void settings_screen_create(lv_obj_t *screen)
{
    s_screen = screen;

    lv_obj_set_flex_flow(screen, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(screen, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_top(screen, 48, 0);
    lv_obj_set_style_pad_bottom(screen, 80, 0);
    lv_obj_set_style_pad_row(screen, 10, 0);
    lv_obj_add_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(screen, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(screen, LV_SCROLLBAR_MODE_OFF);

    make_text(screen, FONT_LARGE, TEXT_COLOUR, "SETTINGS");

    make_display_section(screen);
    make_screens_section(screen);
    make_mounting_section(screen);
    make_wifi_section(screen);

    if(!mag_present())
    {
        make_text(screen, FONT_BODY, HINT_COLOUR,
                  "No magnetometer fitted");
        make_text(screen, FONT_SMALL, 0x606870,
                  "double tap for menu");
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

    make_text(screen, FONT_SMALL, 0x606870,
              "double tap for menu");

    create_calibration_screen();

    lv_timer_create(update, UPDATE_PERIOD_MS, NULL);
}
