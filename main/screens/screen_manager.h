#pragma once

#include "lvgl.h"

/*
 * Screens, and the menu and swipes that switch between them.
 *
 * A double tap on any part of a screen that is not a control opens the
 * menu; a single tap outside it closes it again. A swipe left or right
 * steps through the instruments - not settings.
 * Instruments can be hidden in settings - the HSI with no GPS fitted,
 * say: they are then left out of both, though still there to be shown.
 *
 * Widgets on a screen that should let taps through - instrument
 * drawings, panels - must not be clickable, so the tap falls through to
 * the screen.
 *
 * Every screen is built once at start-up and kept, so switching is just
 * a matter of loading another one: in particular the
 * horizon is never torn down, because closing it would also reset the
 * attitude filter and force a new gyro calibration.
 *
 * Runs in the LVGL task, like every other widget.
 */

typedef enum {
    SCREEN_HORIZON = 0,
    SCREEN_ALTIMETER,
    SCREEN_TURN_COORDINATOR,
    SCREEN_COMPASS,
    SCREEN_HSI,
    SCREEN_ENGINE,
    SCREEN_CYLINDERS,
    SCREEN_SETTINGS,
    SCREEN_COUNT
} screen_id_t;

/*
 * horizon_screen is the screen the instrument has already been built on;
 * the others are created here. Opens on the screen last shown, which is
 * kept across restarts (app_settings.h).
 */
void screen_manager_create(lv_obj_t *horizon_screen);

/* From the LVGL task. */
void screen_manager_show(screen_id_t screen);

/* The instruments that can be hidden, in swipe order, and their names. */
int screen_manager_instrument_count(void);
screen_id_t screen_manager_instrument(int position);
const char *screen_manager_name(screen_id_t screen);

/* From the LVGL task. Kept across restarts. False, and nothing done, for
 * hiding the last instrument shown - or a screen that is not one. */
bool screen_manager_visible(screen_id_t screen);
bool screen_manager_set_visible(screen_id_t screen, bool visible);

/* From any task: shown at the LVGL task's next look, within 100 ms. For
 * the Wi-Fi debug page, which runs in the web server's task. With
 * open_menu, the screen's own swipe-down menu opens too, if it has one -
 * to see it in a screenshot. */
void screen_manager_request(screen_id_t screen, bool open_menu = false);
