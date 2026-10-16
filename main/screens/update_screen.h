#pragma once

#include <stdint.h>

/*
 * "Updating firmware" on the display, while an update is received and
 * written over Wi-Fi (wifi_debug.h).
 *
 * Not an LVGL screen. Writing flash disturbs anything the panel reads
 * from PSRAM, and the framebuffers are in PSRAM, so the picture jumped
 * all through an update. Instead update_screen_begin() stops LVGL and
 * switches the panel to generating its pixels (display_generate()):
 * each bounce buffer is filled, in the panel's interrupt, from a few
 * values and a small font held in internal RAM. Nothing the picture
 * depends on is in PSRAM or flash, so it holds steady.
 *
 * There is no way back to the instruments but a restart - which every
 * update ends with, whether it worked or not.
 */

typedef enum {
    UPDATE_SCREEN_RECEIVING = 0,    /* the upload, into memory */
    UPDATE_SCREEN_WRITING,          /* into flash */
    UPDATE_SCREEN_RESTARTING,
} update_screen_phase_t;

/* Stop LVGL and take over the panel. From the update's task. False if
 * LVGL would not stop; the instruments then stay, frozen. */
bool update_screen_begin(void);

/* From any task. `percent` is how far through the phase. */
void update_screen_progress(update_screen_phase_t phase, uint8_t percent);

/* The update failed; the caller restarts the unit shortly after. */
void update_screen_failed(void);
