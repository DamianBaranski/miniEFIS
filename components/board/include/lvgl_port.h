#pragma once

#include "esp_err.h"
#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * LVGL binding: display flush, touch input and the tick source.
 *
 * LVGL is not thread safe, so every LVGL call has to come from one task.
 * lvgl_port_start() creates that task, calls build_ui inside it and then
 * runs LVGL's handler loop forever - so widgets must be created in the
 * build_ui callback, not from app_main.
 */

typedef void (*lvgl_port_ui_cb_t)(void);

esp_err_t lvgl_port_start(lvgl_port_ui_cb_t build_ui);

/*
 * Rendering statistics, for diagnosing an unsteady display: if the
 * attitude is smooth but frames are slow or uneven, the problem is here
 * and not in the sensor. handler_max_us covers LVGL's redraw, so it
 * excludes the wait for the panel's vertical blanking.
 */
typedef struct {
    uint32_t frames;            /* completed flushes since the last call */
    uint32_t handler_max_us;    /* slowest lv_timer_handler() call */
    uint32_t vsyncs;            /* panel frames since the last call */
    uint32_t flush_wait_max_us; /* longest wait for the panel to switch
                                   to the new frame */
    uint32_t draw_max_us;       /* handler entry to flush: all drawing */
    uint32_t frames_cut;        /* panel frames cut short since start-up:
                                   each one a flicker (lvgl_port.cpp) */
} lvgl_port_stats_t;

/*
 * Stop LVGL for good: the task finishes the frame it is on and parks.
 * For handing the display to something else (display_generate()); the
 * way back is a restart. From any task but LVGL's own.
 */
esp_err_t lvgl_port_stop(uint32_t timeout_ms);

/*
 * The framebuffer on the panel now, BOARD_LCD_WIDTH x BOARD_LCD_HEIGHT
 * RGB565, or NULL before the first frame. LVGL draws into the other one,
 * so this stays put until the next flush - long enough to copy out a
 * screenshot, give or take a frame. Any task.
 */
const void *lvgl_port_shown_frame(void);

/* Reads the counters and resets them. Safe from any task. */
void lvgl_port_take_stats(lvgl_port_stats_t *stats);

#ifdef __cplusplus
}
#endif
