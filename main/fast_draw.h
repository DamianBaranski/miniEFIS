#pragma once

#include "lvgl.h"
#include <stdint.h>

/*
 * Drawing straight into the framebuffer, for the instruments.
 *
 * LVGL's own primitives are general and pay for it: mask setup for
 * every line, text layout for every label. Redrawn in full every frame,
 * that overhead dominates - see the notes on each function. These cover
 * the few shapes the instruments draw a lot of, and nothing else.
 *
 * All of them draw into the draw context of an LVGL draw event and
 * respect its clip area. 16-bit colour only.
 */

/*
 * One run of pixels in a row.
 *
 * Written two at a time through a 32-bit store where alignment allows.
 * The framebuffer is in PSRAM and every store can stall, so halving the
 * number of them is worth the fiddly ends.
 */
static inline void fast_fill_span(lv_color_t *row, lv_coord_t from, lv_coord_t to,
                             lv_color_t colour)
{
    if(from > to)
        return;

    const uint16_t value = colour.full;

    uint16_t *pixel = (uint16_t *)(row + from);
    uint16_t *const end = (uint16_t *)(row + to) + 1;

    /* Reach a 4-byte boundary first. */
    if((((uintptr_t)pixel) & 0x2u) != 0u)
        *pixel++ = value;

    const uint32_t pair = ((uint32_t)value << 16) | (uint32_t)value;

    uint32_t *wide = (uint32_t *)pixel;
    uint32_t *const wide_end =
        (uint32_t *)(((uintptr_t)end) & ~(uintptr_t)0x3u);

    while(wide < wide_end)
        *wide++ = pair;

    pixel = (uint16_t *)wide;

    while(pixel < end)
        *pixel++ = value;
}

/* A line of the given width, square ended, not anti-aliased. */
void fast_draw_line(
    lv_draw_ctx_t *draw_ctx,
    float x1,
    float y1,
    float x2,
    float y2,
    lv_color_t color,
    uint16_t width);

/* A filled circle, not anti-aliased. */
void fast_fill_circle(
    lv_draw_ctx_t *draw_ctx,
    float cx,
    float cy,
    float radius,
    lv_color_t color);

/* A short single line of text, upright, centred on (x, y). */
void fast_draw_text(
    lv_draw_ctx_t *draw_ctx,
    float x,
    float y,
    const char *text,
    const lv_font_t *font,
    lv_color_t color);
