#pragma once

#include "lvgl.h"

/*
 * Digital Artificial Horizon for LVGL 8.
 *
 * The instrument only: the attitude it shows comes from the navigation
 * snapshot (navigation.h). _create() / _update() / _close() run in the
 * LVGL task.
 */

void horizon_screen_create(lv_obj_t *parent);
void horizon_screen_update(void);
void horizon_screen_close(void);

/* Cost of the last redraw, split by phase, in microseconds. */
void horizon_screen_render_cost(uint32_t *custom_us, uint32_t *ground_us,
                                    uint32_t *symbols_us,
                                    uint32_t *background_us,
                                    uint32_t *screen_us);
