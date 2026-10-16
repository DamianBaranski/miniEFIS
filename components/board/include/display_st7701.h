#pragma once

#include "board_pins.h"
#include "esp_err.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ST7701 480x480 RGB panel.
 *
 * Pixels go out over a parallel 16-bit RGB bus straight from PSRAM
 * framebuffers; the panel's configuration registers are a separate,
 * write-only SPI link used once at start-up. Chip select and reset hang
 * off the IO expander, so tca9554_init() has to have run first.
 */

esp_err_t display_init(void);

/* Panel handle, for esp_lcd calls the wrapper below does not cover
 * (the LVGL port uses it to fetch the framebuffers). */
esp_lcd_panel_handle_t display_panel(void);

/* Inclusive coordinates, as LVGL supplies them. */
esp_err_t display_draw_bitmap(int x_start, int y_start,
                              int x_end, int y_end,
                              const void *pixels);

/*
 * Guard words beside the framebuffers (see display_st7701.cpp). Returns
 * how many were found overwritten since the last call, with the first
 * such value and where it was, and re-arms them. Safe from any task.
 */
int display_guard_check(uint32_t *value, const void **address);

/* How many guard blocks were placed. */
int display_guard_count(void);

/*
 * Drop the framebuffers and generate every pixel instead: fill is
 * called from the panel's interrupt for each bounce buffer, with the
 * pixel index it starts at. It must be IRAM_ATTR and touch only
 * internal RAM - which is the point, since the picture then survives
 * flash writes untouched. LVGL must be stopped first
 * (lvgl_port_stop()), and there is no way back but a restart.
 */
esp_err_t display_generate(esp_lcd_rgb_panel_bounce_buf_fill_cb_t fill,
                           void *context);

/* 0..100 percent. */
esp_err_t display_set_backlight(uint8_t percent);
uint8_t display_get_backlight(void);

#ifdef __cplusplus
}
#endif
