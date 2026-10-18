#include "fast_draw.h"

#include <math.h>

/*
 * A line, as a filled rotated rectangle, written straight into the
 * framebuffer.
 *
 * LVGL's line drawing measured about 300 microseconds per line here,
 * against some 13 microseconds' worth of pixels: nearly all of it is
 * building and applying edge masks, and this instrument draws a good
 * thirty short lines every frame - the pitch ladder, the roll scale, the
 * aircraft symbol. Filling the rectangle directly costs the pixels and
 * little else.
 *
 * The trade is anti-aliasing: LVGL feathers its edges and this does not,
 * so the lines are a touch harder edged. At these widths, on a moving
 * instrument, smooth motion is worth more than smooth edges.
 *
 * The rectangle is the intersection of four half-planes - two parallel
 * to the line, two capping its ends - so each row reduces to one run.
 */
void fast_draw_line(
    lv_draw_ctx_t *draw_ctx,
    float x1,
    float y1,
    float x2,
    float y2,
    lv_color_t color,
    uint16_t width)
{
    const float dx = x2 - x1;
    const float dy = y2 - y1;
    const float length = sqrtf(dx * dx + dy * dy);

    if(length < 1.0e-3f)
        return;

    /* Along the line, and across it. */
    const float ux = dx / length;
    const float uy = dy / length;
    const float px = -uy;
    const float py = ux;

    const float half = (float)width * 0.5f;

    lv_area_t clip;

    if(!_lv_area_intersect(&clip, draw_ctx->clip_area, draw_ctx->buf_area))
        return;

    /* Only the rows the line can reach. */
    const float span = half + 1.0f;
    float top = ((y1 < y2) ? y1 : y2) - span;
    float bottom = ((y1 > y2) ? y1 : y2) + span;

    lv_coord_t y_from = (lv_coord_t)floorf(top);
    lv_coord_t y_to = (lv_coord_t)ceilf(bottom);

    if(y_from < clip.y1) y_from = clip.y1;
    if(y_to > clip.y2) y_to = clip.y2;

    lv_color_t *const buffer = (lv_color_t *)draw_ctx->buf;
    const lv_coord_t stride = lv_area_get_width(draw_ctx->buf_area);
    const lv_coord_t origin_x = draw_ctx->buf_area->x1;
    const lv_coord_t origin_y = draw_ctx->buf_area->y1;

    /* Half-planes as a*x + b*y <= c, all four inward. */
    const float a[4] = {  px, -px,  ux, -ux };
    const float b[4] = {  py, -py,  uy, -uy };
    const float c[4] = {
        px * x1 + py * y1 + half,
        -(px * x1 + py * y1) + half,
        ux * x2 + uy * y2,
        -(ux * x1 + uy * y1)
    };

    for(lv_coord_t y = y_from; y <= y_to; y++)
    {
        float low = (float)clip.x1;
        float high = (float)clip.x2;
        bool empty = false;

        for(int edge = 0; edge < 4; edge++)
        {
            const float limit = c[edge] - b[edge] * (float)y;

            if(fabsf(a[edge]) < 1.0e-6f)
            {
                /* The edge is horizontal: this row is either wholly in
                 * or wholly out of it. */
                if(limit < 0.0f)
                {
                    empty = true;
                    break;
                }

                continue;
            }

            const float bound = limit / a[edge];

            if(a[edge] > 0.0f)
            {
                if(bound < high) high = bound;
            }
            else
            {
                if(bound > low) low = bound;
            }
        }

        /*
         * Empty rows are found here, in float, before anything is
         * converted. A line within a hair of horizontal or vertical
         * divides by almost nothing above, and its bounds can be
         * hundreds of thousands of pixels out; converted to lv_coord_t
         * first, they wrapped - 100000 became -31072 - and the span
         * began 62 KB before the row, writing over whatever PSRAM held
         * there. At a roll within a tenth of a degree of level the bank
         * pointer is such a line: that crashed the unit, through a heap
         * it had overwritten. Here low and high are both inside the
         * clip area when they convert.
         */
        if(empty || low > high)
            continue;

        const lv_coord_t from = (lv_coord_t)ceilf(low);
        const lv_coord_t to = (lv_coord_t)floorf(high);

        if(from > to)
            continue;

        lv_color_t *const row =
            buffer + (y - origin_y) * (int32_t)stride - origin_x;

        fast_fill_span(row, from, to, color);
    }
}

/* One run per row, like the lines: no masks. */
void fast_fill_circle(
    lv_draw_ctx_t *draw_ctx,
    float cx,
    float cy,
    float radius,
    lv_color_t color)
{
    lv_area_t clip;

    if(!_lv_area_intersect(&clip, draw_ctx->clip_area, draw_ctx->buf_area))
        return;

    lv_coord_t y_from = (lv_coord_t)ceilf(cy - radius);
    lv_coord_t y_to = (lv_coord_t)floorf(cy + radius);

    if(y_from < clip.y1) y_from = clip.y1;
    if(y_to > clip.y2) y_to = clip.y2;

    lv_color_t *const buffer = (lv_color_t *)draw_ctx->buf;
    const lv_coord_t stride = lv_area_get_width(draw_ctx->buf_area);
    const lv_coord_t origin_x = draw_ctx->buf_area->x1;
    const lv_coord_t origin_y = draw_ctx->buf_area->y1;

    for(lv_coord_t y = y_from; y <= y_to; y++)
    {
        const float dy = (float)y - cy;
        const float half = sqrtf(fmaxf(radius * radius - dy * dy, 0.0f));

        lv_coord_t from = (lv_coord_t)ceilf(cx - half);
        lv_coord_t to = (lv_coord_t)floorf(cx + half);

        if(from < clip.x1) from = clip.x1;
        if(to > clip.x2) to = clip.x2;

        if(from > to)
            continue;

        lv_color_t *const row =
            buffer + (y - origin_y) * (int32_t)stride - origin_x;

        fast_fill_span(row, from, to, color);
    }
}

/*
 * Drawn glyph by glyph rather than through lv_draw_label.
 *
 * The label path costs around 330 microseconds a call here - text
 * layout, alignment, line breaking - which for a one or two digit number
 * is all overhead, and twelve of them a frame came to 4 ms. Placing the
 * glyphs directly needs the advance widths and nothing else.
 *
 * Skipped when the point is well outside the clip area: the glyphs would
 * be clipped away anyway, but only after being looked up.
 */
void fast_draw_text(
    lv_draw_ctx_t *draw_ctx,
    float x,
    float y,
    const char *text,
    const lv_font_t *font,
    lv_color_t color)
{
    const lv_coord_t px = (lv_coord_t)lroundf(x);
    const lv_coord_t py = (lv_coord_t)lroundf(y);
    const lv_area_t *clip = draw_ctx->clip_area;

    if(px < clip->x1 - 48 || px > clip->x2 + 48 ||
       py < clip->y1 - font->line_height || py > clip->y2 + font->line_height)
    {
        return;
    }

    lv_draw_label_dsc_t dsc;
    lv_draw_label_dsc_init(&dsc);

    dsc.color = color;
    dsc.font = font;

    /* UTF-8: the degree sign is two bytes. */
    lv_coord_t total_width = 0;
    uint32_t i = 0;
    uint32_t glyph;

    while((glyph = _lv_txt_encoded_next(text, &i)) != 0)
        total_width += (lv_coord_t)lv_font_get_glyph_width(font, glyph, 0);

    lv_point_t pen;
    pen.x = (lv_coord_t)(px - total_width / 2);
    pen.y = (lv_coord_t)(py - font->line_height / 2);

    i = 0;

    while((glyph = _lv_txt_encoded_next(text, &i)) != 0)
    {
        lv_draw_letter(draw_ctx, &dsc, &pen, glyph);

        pen.x += (lv_coord_t)lv_font_get_glyph_width(font, glyph, 0);
    }
}
