#include "horizon_screen.h"
#include "fonts.h"
#include "fast_draw.h"
#include "navigation.h"

#include "esp_timer.h"
#include <math.h>
#include <stdio.h>

/*
 * Digital Artificial Horizon for LVGL 8.
 *
 * This file is only the instrument: it owns the LVGL objects and the
 * drawing. The attitude comes from the navigation snapshot
 * (navigation.h), the Kalman filter's output that every screen shares.
 */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static lv_obj_t *s_horizon = NULL;
static lv_obj_t *s_roll_label = NULL;
static lv_obj_t *s_pitch_label = NULL;
static lv_obj_t *s_status_label = NULL;
static lv_timer_t *s_timer = NULL;

/*
 * Attitude for the frame being drawn. Owned by the LVGL task:
 * horizon_screen_update() takes one consistent snapshot of the filter
 * output per frame, and the draw callback reads only these.
 */
static float s_roll = 0.0f;
static float s_pitch = 0.0f;

static constexpr float PITCH_PIXELS_PER_DEGREE = 4.0f;

/* Pitch ladder: marks this far either side of the current pitch, at
 * this spacing. 35 degrees is 140 px. */
static constexpr float LADDER_REACH_DEG = 35.0f;
static constexpr int LADDER_STEP_DEG = 10;

/*
 * And only drawn within this many pixels above and below the centre:
 * below the roll scale, whose pointer reaches down to about 176 px, and
 * above the status line at the bottom. Room for the whole ladder when
 * level; banked, the rungs and numbers that would swing into those are
 * cut off at the band's edge.
 */
static constexpr float LADDER_MASK_PX = 150.0f;

/* Pitch ladder numbering, and how far clear of the mark it sits. */
#define PITCH_LABEL_FONT FONT_LARGE
static constexpr float PITCH_LABEL_GAP = 17.0f;

/* Roll arc radius, as a fraction of the display's own radius. */
static constexpr float ROLL_SCALE_RADIUS_FRACTION = 0.86f;

static constexpr uint32_t SKY_COLOUR = 0x1987D1;
static constexpr uint32_t GROUND_COLOUR = 0x8B542E;

/*
 * Where the redraw time goes, in microseconds. Written by the LVGL task
 * and read by the diagnostics; single words, so no lock.
 *
 * s_cost_custom_us covers this file's whole draw callback, so the
 * difference between it and the redraw as a whole is LVGL's own work:
 * the container's sky background fill, the labels, and the wait for the
 * blanking interval.
 */
static volatile uint32_t s_cost_custom_us = 0;
static volatile uint32_t s_cost_ground_us = 0;
static volatile uint32_t s_cost_symbols_us = 0;
static volatile uint32_t s_cost_background_us = 0;
static volatile uint32_t s_cost_screen_us = 0;
static int64_t s_draw_event_started = 0;
static int64_t s_screen_event_started = 0;

void horizon_screen_render_cost(uint32_t *custom_us, uint32_t *ground_us,
                                    uint32_t *symbols_us,
                                    uint32_t *background_us,
                                    uint32_t *screen_us)
{
    if(custom_us != NULL) *custom_us = s_cost_custom_us;
    if(ground_us != NULL) *ground_us = s_cost_ground_us;
    if(symbols_us != NULL) *symbols_us = s_cost_symbols_us;
    if(background_us != NULL) *background_us = s_cost_background_us;
    if(screen_us != NULL) *screen_us = s_cost_screen_us;
}

/*
 * Every pixel inside this object is written by the draw callback, so it
 * hides whatever is under it. Saying so lets LVGL skip drawing the
 * screen. The class handler runs before this callback and will have said
 * "not covered", judging by the transparent background style; this
 * overrides it.
 */
static void horizon_screen_cover_check(lv_event_t *e)
{
    lv_cover_check_info_t *info =
        (lv_cover_check_info_t *)lv_event_get_param(e);

    info->res = LV_COVER_RES_COVER;
}

/*
 * Is the screen underneath being drawn at all?
 *
 * It should not be: the container reports that it covers everything, so
 * LVGL ought to skip straight to it. If these callbacks ever fire, a
 * whole extra screen fill is being paid for and thrown away.
 */
static void horizon_screen_screen_begin(lv_event_t *e)
{
    (void)e;
    s_screen_event_started = esp_timer_get_time();
}

static void horizon_screen_screen_end(lv_event_t *e)
{
    (void)e;
    s_cost_screen_us =
        (uint32_t)(esp_timer_get_time() - s_screen_event_started);
}

/*
 * Runs before the widget class draws the container background, where the
 * normal callback below runs after it. The gap between the two is
 * therefore the cost of that background fill, which is otherwise inside
 * LVGL and invisible.
 */
static void horizon_screen_draw_begin(lv_event_t *e)
{
    (void)e;
    s_draw_event_started = esp_timer_get_time();
}

/* -------------------------------------------------------------------------- */

static float rad(float d)
{
    return d * (float)M_PI / 180.0f;
}

/* -------------------------------------------------------------------------- */
/* Drawing helpers                                                            */
/* -------------------------------------------------------------------------- */

/*
 * Sky and ground, one write per pixel.
 *
 * The obvious way - fill the screen with sky, then draw the ground as a
 * polygon over it - paints about 345k pixels on a 230k pixel screen, and
 * the polygon half of that goes through LVGL's mask machinery. On this
 * board the framebuffer is in PSRAM and the panel's DMA is reading a
 * whole frame every 20 ms, so the redraw is bandwidth bound: pixels
 * written is the thing that costs, and writing a third of them twice is
 * the single largest waste in the frame.
 *
 * The horizon is a straight line, so every row of the screen is split
 * by it at exactly one point. Walking the rows and filling two runs per
 * row writes each pixel once, with no masks at all.
 */
static void draw_background(
    lv_draw_ctx_t *draw_ctx,
    float hx,
    float hy,
    float nx,
    float ny,
    float cx,
    float cy,
    float radius,
    lv_color_t sky,
    lv_color_t ground)
{
    /*
     * The panel is round, so the four corners of the 480x480 frame are
     * not wired to anything. Skipping them is a fifth of the pixels -
     * the single cheapest saving left, because the fill is limited by
     * how fast PSRAM can be written and by nothing else.
     *
     * They are painted black for the first few frames all the same:
     * there are two framebuffers and neither is cleared at start-up, so
     * without this they would hold whatever PSRAM powered up with. It
     * should never be visible; it costs two frames to be certain.
     */
    static int clear_corners = 4;
    const bool clearing = (clear_corners > 0);

    if(clearing)
        clear_corners--;

    const lv_color_t black = lv_color_black();

    lv_area_t clip;

    if(!_lv_area_intersect(&clip, draw_ctx->clip_area, draw_ctx->buf_area))
        return;

    lv_color_t *const buffer = (lv_color_t *)draw_ctx->buf;
    const lv_coord_t stride = lv_area_get_width(draw_ctx->buf_area);
    const lv_coord_t origin_x = draw_ctx->buf_area->x1;
    const lv_coord_t origin_y = draw_ctx->buf_area->y1;

    for(lv_coord_t y = clip.y1; y <= clip.y2; y++)
    {
        lv_color_t *const row =
            buffer + (y - origin_y) * (int32_t)stride - origin_x;

        /* How far the visible circle reaches across this row. */
        const float height = (float)y - cy;
        const float squared = radius * radius - height * height;

        lv_coord_t from = clip.x1;
        lv_coord_t to = clip.x2;

        if(squared <= 0.0f)
        {
            /* Entirely outside the glass. */
            if(clearing)
                fast_fill_span(row, clip.x1, clip.x2, black);

            continue;
        }

        const float half_width = sqrtf(squared);
        const lv_coord_t circle_from = (lv_coord_t)ceilf(cx - half_width);
        const lv_coord_t circle_to = (lv_coord_t)floorf(cx + half_width);

        if(circle_from > from) from = circle_from;
        if(circle_to < to) to = circle_to;

        if(clearing)
        {
            fast_fill_span(row, clip.x1, (lv_coord_t)(from - 1), black);
            fast_fill_span(row, (lv_coord_t)(to + 1), clip.x2, black);
        }

        if(from > to)
            continue;

        /*
         * Where this row crosses the horizon. Ground is the side the
         * normal points to: nx*(x - hx) + ny*(y - hy) > 0.
         */
        const float offset = ny * ((float)y - hy);

        lv_coord_t boundary;

        if(fabsf(nx) < 1.0e-4f)
        {
            /* The horizon is level on this axis: the whole row is one
             * colour, so push the split off the end. */
            boundary = (offset > 0.0f) ? from : (lv_coord_t)(to + 1);
        }
        else
        {
            const float crossing = hx - offset / nx;

            if(crossing < (float)from)
                boundary = from;
            else if(crossing > (float)(to + 1))
                boundary = (lv_coord_t)(to + 1);
            else
                boundary = (lv_coord_t)crossing;

            /* nx < 0 puts the ground on the low side of the crossing. */
            if(nx < 0.0f)
            {
                fast_fill_span(row, from, (lv_coord_t)(boundary - 1), ground);
                fast_fill_span(row, boundary, to, sky);
                continue;
            }
        }

        fast_fill_span(row, from, (lv_coord_t)(boundary - 1), sky);
        fast_fill_span(row, boundary, to, ground);
    }
}

/*
 * A small number, centred on a point.
 *
 * The glyphs stay upright rather than turning with the ladder. LVGL 8
 * cannot rotate text without going through a canvas and a transform,
 * which would cost more than the rest of the instrument put together;
 * upright numbers on a rotating ladder are also what most glass panels
 * do, and they stay readable at any bank.
 *
 * Skipped when the point is off the screen: the draw would be clipped
 * away, but the text layout happens first and is the expensive part.
 */
static void draw_number(
    lv_draw_ctx_t *draw_ctx,
    const lv_area_t *bounds,
    float x,
    float y,
    int value,
    const lv_font_t *font)
{
    const lv_coord_t px = (lv_coord_t)lroundf(x);
    const lv_coord_t py = (lv_coord_t)lroundf(y);

    if(px < bounds->x1 - 24 || px > bounds->x2 + 24 ||
       py < bounds->y1 - 16 || py > bounds->y2 + 16)
    {
        return;
    }

    char text[8];
    snprintf(text, sizeof(text), "%d", value);

    fast_draw_text(draw_ctx, x, y, text, font, lv_color_white());
}

/* -------------------------------------------------------------------------- */
/* Main horizon drawing                                                       */
/* -------------------------------------------------------------------------- */

static void horizon_screen_draw(lv_event_t *e)
{
    const int64_t draw_started = esp_timer_get_time();

    s_cost_background_us = (uint32_t)(draw_started - s_draw_event_started);

    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *draw_ctx = lv_event_get_draw_ctx(e);

    lv_area_t area;
    lv_obj_get_coords(obj, &area);

    const float x1 = (float)area.x1;
    const float y1 = (float)area.y1;
    const float x2 = (float)area.x2;
    const float y2 = (float)area.y2;

    const float width = x2 - x1 + 1.0f;
    const float height = y2 - y1 + 1.0f;

    const float cx = (x1 + x2) * 0.5f;
    const float cy = (y1 + y2) * 0.5f;

    /*
     * Keep the horizon well outside the clipping rectangle.
     * LVGL clips drawing to the object automatically.
     */
    const float diagonal =
        sqrtf(width * width + height * height) * 2.0f;

    /* The glass is a circle inscribed in the panel. */
    const float display_radius = ((width < height) ? width : height) * 0.5f;

    /*
     * The horizon turns against the bank: rolling right, the world
     * rotates anticlockwise in the pilot's view, its right side rising.
     * Screen y runs down, so that is a negative angle here.
     */
    const float a = rad(-s_roll);

    /*
     * Unit vector along the horizon.
     *
     * roll = 0 -> horizontal.
     */
    const float ux = cosf(a);
    const float uy = sinf(a);

    /*
     * Unit vector pointing toward the ground.
     *
     * roll = 0 -> down.
     */
    const float nx = -sinf(a);
    const float ny = cosf(a);

    /*
     * Pitch moves the horizon in the normal direction.
     *
     * Not clamped: the ladder is positioned against this, so limiting
     * one and not the other would slide them apart past the limit. A
     * horizon that leaves the screen at steep pitch is correct - that
     * is the point at which the ladder is what you read.
     */
    const float pitch_offset = s_pitch * PITCH_PIXELS_PER_DEGREE;

    const float hx = cx + nx * pitch_offset;
    const float hy = cy + ny * pitch_offset;

    /*
     * Sky and ground in one pass. This also stands in for the widget's
     * background, which is why the container has none: see
     * horizon_screen_create().
     */
    const int64_t ground_started = esp_timer_get_time();

    draw_background(
        draw_ctx,
        hx,
        hy,
        nx,
        ny,
        cx,
        cy,
        display_radius,
        lv_color_hex(SKY_COLOUR),
        lv_color_hex(GROUND_COLOUR));

    s_cost_ground_us = (uint32_t)(esp_timer_get_time() - ground_started);

    const int64_t symbols_started = esp_timer_get_time();

    /*
     * Main horizon line.
     */
    fast_draw_line(
        draw_ctx,
        hx - ux * diagonal,
        hy - uy * diagonal,
        hx + ux * diagonal,
        hy + uy * diagonal,
        lv_color_white(),
        3);

    /* ---------------------------------------------------------------------- */
    /* Pitch ladder                                                           */
    /* ---------------------------------------------------------------------- */

    /*
     * Marks every 10 degrees within LADDER_REACH_DEG of the current
     * pitch, wherever that is - climbing at 20, the ladder runs from 50
     * down to -10 - and never past the vertical. The zero mark is the
     * horizon line itself.
     */
    const int first_mark =
        (int)ceilf((s_pitch - LADDER_REACH_DEG) / LADDER_STEP_DEG) *
        LADDER_STEP_DEG;
    const int last_mark =
        (int)floorf((s_pitch + LADDER_REACH_DEG) / LADDER_STEP_DEG) *
        LADDER_STEP_DEG;

    /* The mask: everything in the ladder is clipped to the band. */
    const lv_area_t *const full_clip = draw_ctx->clip_area;

    lv_area_t band = *full_clip;
    band.y1 = (lv_coord_t)LV_MAX(band.y1, (lv_coord_t)(cy - LADDER_MASK_PX));
    band.y2 = (lv_coord_t)LV_MIN(band.y2, (lv_coord_t)(cy + LADDER_MASK_PX));

    draw_ctx->clip_area = &band;

    for(int mark = first_mark;
        band.y1 <= band.y2 && mark <= last_mark;
        mark += LADDER_STEP_DEG)
    {
        if(mark == 0 || mark < -90 || mark > 90)
            continue;

        const float mark_pitch = (float)mark;

        /*
         * The ladder is painted on the horizon, not on the screen.
         *
         * A mark sits its own angle away from the horizon line, on the
         * sky side for a climb and the ground side for a dive, so it
         * has to carry the same sign as the horizon's own offset.
         * Reversed, the marks slide up the screen as the horizon slides
         * down and the +10 mark lands below centre when level.
         */
        const float offset =
            (s_pitch - mark_pitch) *
            PITCH_PIXELS_PER_DEGREE;

        const float mx = cx + nx * offset;
        const float my = cy + ny * offset;

        /* Short, medium, long, repeating: 10, 20, 30, then 40, 50, 60. */
        const int rung = (abs(mark) / LADDER_STEP_DEG) % 3;
        const float mark_length =
            (rung == 1) ? 25.0f :
            (rung == 2) ? 35.0f :
                          45.0f;

        const float lx1 =
            mx - ux * mark_length;

        const float ly1 =
            my - uy * mark_length;

        const float lx2 =
            mx + ux * mark_length;

        const float ly2 =
            my + uy * mark_length;

        fast_draw_line(
            draw_ctx,
            lx1,
            ly1,
            lx2,
            ly2,
            lv_color_white(),
            3);

        /*
         * Small extension at each end, pointing back towards the
         * horizon - downwards from a climb mark, upwards from a dive
         * one. It is what tells you at a glance which side of the
         * horizon a mark belongs to when the horizon itself is off the
         * screen.
         */
        const float tick = (mark_pitch > 0.0f) ? 6.0f : -6.0f;

        fast_draw_line(
            draw_ctx,
            lx1,
            ly1,
            lx1 + nx * tick,
            ly1 + ny * tick,
            lv_color_white(),
            2);

        fast_draw_line(
            draw_ctx,
            lx2,
            ly2,
            lx2 + nx * tick,
            ly2 + ny * tick,
            lv_color_white(),
            2);

        /*
         * The angle, at both ends, just clear of the mark. Unsigned:
         * which side of the horizon the mark is on already says whether
         * it is a climb or a dive.
         */
        const float label_reach = mark_length + PITCH_LABEL_GAP;
        const int label_value = abs(mark);

        draw_number(
            draw_ctx,
            &area,
            mx - ux * label_reach,
            my - uy * label_reach,
            label_value,
            PITCH_LABEL_FONT);

        draw_number(
            draw_ctx,
            &area,
            mx + ux * label_reach,
            my + uy * label_reach,
            label_value,
            PITCH_LABEL_FONT);
    }

    draw_ctx->clip_area = full_clip;

    /* ---------------------------------------------------------------------- */
    /* Roll scale at the top                                                 */
    /* ---------------------------------------------------------------------- */

    /*
     * Centred on the display and set close to its rim, so the arc is as
     * long as the glass allows and a degree of bank is a visible
     * distance along it. The marks run inwards from the arc.
     */
    const float roll_center_x = cx;
    const float roll_center_y = cy;
    const float roll_radius = display_radius * ROLL_SCALE_RADIUS_FRACTION;

    const int roll_angles[] = {
        -60, -45, -30, -20, -10,
          0,
          10,  20,  30,  45,  60
    };

    for(unsigned int i = 0;
        i < sizeof(roll_angles) / sizeof(roll_angles[0]);
        ++i)
    {
        const int bank = roll_angles[i];
        const float angle = rad((float)bank - 90.0f);

        /* Long marks at the cardinal banks, short ones in between, and
         * the longest straight up as the zero reference. */
        const float length =
            (bank == 0) ? 30.0f :
            (abs(bank) == 30 || abs(bank) == 60) ? 24.0f :
            (abs(bank) == 45) ? 18.0f :
                                13.0f;

        const float width = (bank == 0) ? 4.0f : ((length > 16.0f) ? 3.0f : 2.0f);

        const float outer_x = roll_center_x + cosf(angle) * roll_radius;
        const float outer_y = roll_center_y + sinf(angle) * roll_radius;

        const float inner_x =
            roll_center_x + cosf(angle) * (roll_radius - length);

        const float inner_y =
            roll_center_y + sinf(angle) * (roll_radius - length);

        fast_draw_line(
            draw_ctx,
            inner_x,
            inner_y,
            outer_x,
            outer_y,
            lv_color_white(),
            (uint16_t)width);
    }

    /*
     * Bank pointer: rides the arc at the current roll angle, just inside
     * the marks so it reads against them.
     */
    /* It turns with the horizon, against the bank. */
    const float roll_pointer_angle = rad(-s_roll - 90.0f);

    const float pointer_outer = roll_radius - 3.0f;
    const float pointer_inner = roll_radius - 28.0f;

    fast_draw_line(
        draw_ctx,
        roll_center_x + cosf(roll_pointer_angle) * pointer_outer,
        roll_center_y + sinf(roll_pointer_angle) * pointer_outer,
        roll_center_x + cosf(roll_pointer_angle) * pointer_inner,
        roll_center_y + sinf(roll_pointer_angle) * pointer_inner,
        lv_color_hex(0xFFD400),
        7);

    /* ---------------------------------------------------------------------- */
    /* Fixed aircraft symbol                                                  */
    /* ---------------------------------------------------------------------- */

    const float aircraft_y = cy;

    /*
     * Wings.
     */
    fast_draw_line(
        draw_ctx,
        cx - 75.0f,
        aircraft_y,
        cx - 18.0f,
        aircraft_y,
        lv_color_white(),
        6);

    fast_draw_line(
        draw_ctx,
        cx + 18.0f,
        aircraft_y,
        cx + 75.0f,
        aircraft_y,
        lv_color_white(),
        6);

    /*
     * Fuselage.
     */
    fast_draw_line(
        draw_ctx,
        cx,
        aircraft_y - 18.0f,
        cx,
        aircraft_y + 18.0f,
        lv_color_white(),
        6);

    /*
     * Small center mark.
     */
    fast_draw_line(
        draw_ctx,
        cx - 7.0f,
        aircraft_y,
        cx + 7.0f,
        aircraft_y,
        lv_color_hex(0xFFD400),
        4);

    s_cost_symbols_us = (uint32_t)(esp_timer_get_time() - symbols_started);
    s_cost_custom_us = (uint32_t)(esp_timer_get_time() - draw_started);
}

/* -------------------------------------------------------------------------- */
/* LVGL object creation                                                       */
/* -------------------------------------------------------------------------- */

/*
 * The label backgrounds are opaque rather than translucent: alpha
 * blending reads every pixel back out of the PSRAM framebuffer before
 * writing it, which costs several times a plain fill.
 */
static void horizon_screen_create_labels(lv_obj_t *parent)
{
    /*
     * Top information.
     */
    s_roll_label = lv_label_create(parent);

    lv_obj_set_style_text_color(
        s_roll_label,
        lv_color_white(),
        0);

    lv_obj_set_style_bg_color(
        s_roll_label,
        lv_color_black(),
        0);

    lv_obj_set_style_bg_opa(
        s_roll_label,
        LV_OPA_COVER,
        0);

    lv_obj_set_style_pad_left(
        s_roll_label,
        8,
        0);

    lv_obj_set_style_pad_right(
        s_roll_label,
        8,
        0);

    lv_label_set_text(
        s_roll_label,
        "ROLL +0.0");

    lv_obj_align(
        s_roll_label,
        LV_ALIGN_TOP_LEFT,
        8,
        8);

    /*
     * Pitch information.
     */
    s_pitch_label = lv_label_create(parent);

    lv_obj_set_style_text_color(
        s_pitch_label,
        lv_color_white(),
        0);

    lv_obj_set_style_bg_color(
        s_pitch_label,
        lv_color_black(),
        0);

    lv_obj_set_style_bg_opa(
        s_pitch_label,
        LV_OPA_COVER,
        0);

    lv_obj_set_style_pad_left(
        s_pitch_label,
        8,
        0);

    lv_obj_set_style_pad_right(
        s_pitch_label,
        8,
        0);

    lv_label_set_text(
        s_pitch_label,
        "PITCH +0.0");

    lv_obj_align(
        s_pitch_label,
        LV_ALIGN_TOP_RIGHT,
        -8,
        8);

    /*
     * Status.
     */
    s_status_label = lv_label_create(parent);

    lv_obj_set_style_text_color(
        s_status_label,
        lv_color_white(),
        0);

    /* Replaced every frame by horizon_screen_update(). */
    lv_label_set_text(
        s_status_label,
        "NO IMU DATA");

    lv_obj_align(
        s_status_label,
        LV_ALIGN_BOTTOM_MID,
        0,
        -8);
}

/* -------------------------------------------------------------------------- */

void horizon_screen_create(lv_obj_t *parent)
{
    horizon_screen_close();

    /*
     * Container.
     */
    s_horizon = lv_obj_create(parent);

    lv_obj_remove_style_all(s_horizon);

    /*
     * No background style: the draw callback paints every pixel itself,
     * in one pass. But LVGL decides whether to fill the screen
     * underneath by asking objects whether they cover it, and it judges
     * that from the background style - which is now transparent. So the
     * cover check is answered explicitly below, otherwise LVGL fills the
     * whole screen first and every one of those pixels is then
     * overwritten: 20 ms a frame for something never seen.
     */

    lv_obj_set_size(
        s_horizon,
        LV_PCT(100),
        LV_PCT(100));

    lv_obj_center(s_horizon);

    lv_obj_clear_flag(
        s_horizon,
        LV_OBJ_FLAG_SCROLLABLE);

    /*
     * Not clickable, so taps fall through to the screen, where the
     * double tap for the menu is caught (screen_manager.cpp).
     */
    lv_obj_clear_flag(
        s_horizon,
        LV_OBJ_FLAG_CLICKABLE);

    /*
     * Draw event.
     */
    lv_obj_add_event_cb(
        s_horizon,
        horizon_screen_cover_check,
        LV_EVENT_COVER_CHECK,
        NULL);

    lv_obj_add_event_cb(
        parent,
        horizon_screen_screen_begin,
        (lv_event_code_t)(LV_EVENT_DRAW_MAIN | LV_EVENT_PREPROCESS),
        NULL);

    lv_obj_add_event_cb(
        parent,
        horizon_screen_screen_end,
        LV_EVENT_DRAW_MAIN,
        NULL);

    lv_obj_add_event_cb(
        s_horizon,
        horizon_screen_draw_begin,
        (lv_event_code_t)(LV_EVENT_DRAW_MAIN | LV_EVENT_PREPROCESS),
        NULL);

    lv_obj_add_event_cb(
        s_horizon,
        horizon_screen_draw,
        LV_EVENT_DRAW_MAIN,
        NULL);

    /*
     * Labels are created on parent so they remain above the drawing.
     */
    horizon_screen_create_labels(parent);

    /*
     * Ask for a redraw every 20 ms.
     *
     * This is a request, not a rate: LVGL redraws when something has
     * been invalidated and the previous frame has been flushed, so what
     * actually comes out is whatever the render pipeline can sustain -
     * on this board a frame takes two of the panel's periods, so 24 Hz.
     *
     * The value matters because it used to be 50 ms, which silently
     * became the limit once the drawing got fast enough: the renderer
     * was idle a third of the time waiting to be asked.
     */
    s_timer = lv_timer_create(
        [](lv_timer_t *)
        {
            horizon_screen_update();
        },
        20,
        NULL);
}
/* -------------------------------------------------------------------------- */

static const char *horizon_screen_status_text(AttitudeStatus status)
{
    switch(status)
    {
        /*
         * Kept short on purpose. Every glyph is re-rendered every frame,
         * and the colour already carries the state: white is healthy,
         * amber is degraded, red is dead. The long forms cost several
         * milliseconds a frame to say what the colour says for nothing.
         */
        case AttitudeStatus::Calibrating:
            return "CAL - HOLD STILL";

        case AttitudeStatus::Fused:
            return "AHRS";

        case AttitudeStatus::GyroOnly:
            return "GYRO ONLY";

        default:
            return "NO IMU";
    }
}

static lv_color_t horizon_screen_status_color(AttitudeStatus status)
{
    switch(status)
    {
        case AttitudeStatus::Fused:
            return lv_color_white();

        case AttitudeStatus::Calibrating:
        case AttitudeStatus::GyroOnly:
            return lv_color_hex(0xFFD400);

        default:
            return lv_color_hex(0xFF4040);
    }
}

void horizon_screen_update()
{
    if(s_horizon == NULL)
        return;

    /*
     * The filter runs in the IMU task; take one consistent snapshot of
     * its output for this frame. No attitude maths happens here.
     */
    const AttitudeSample attitude = navigation_sample().attitude;

    s_roll = attitude.roll_deg;
    s_pitch = attitude.pitch_deg;

    char buf[48];

    /* The heading line only appears when a magnetometer is feeding the
     * filter; without one the status text is shown on its own. */
    if(attitude.heading_valid)
    {
        const int heading = ((int)(attitude.heading_deg + 0.5f)) % 360;

        snprintf(
            buf,
            sizeof(buf),
            "%s  HDG %03d",
            horizon_screen_status_text(attitude.status),
            heading);
    }
    else
    {
        snprintf(
            buf,
            sizeof(buf),
            "%s",
            horizon_screen_status_text(attitude.status));
    }

    lv_label_set_text(
        s_status_label,
        buf);

    lv_obj_set_style_text_color(
        s_status_label,
        horizon_screen_status_color(attitude.status),
        0);

    snprintf(
        buf,
        sizeof(buf),
        "ROLL %+.1f",
        s_roll);

    lv_label_set_text(
        s_roll_label,
        buf);

    snprintf(
        buf,
        sizeof(buf),
        "PITCH %+.1f",
        s_pitch);

    lv_label_set_text(
        s_pitch_label,
        buf);

    lv_obj_invalidate(s_horizon);
}

/* -------------------------------------------------------------------------- */

void horizon_screen_close()
{
    if(s_timer != NULL)
    {
        lv_timer_del(s_timer);
        s_timer = NULL;
    }

    if(s_roll_label != NULL)
    {
        lv_obj_del(s_roll_label);
        s_roll_label = NULL;
    }

    if(s_pitch_label != NULL)
    {
        lv_obj_del(s_pitch_label);
        s_pitch_label = NULL;
    }

    if(s_status_label != NULL)
    {
        lv_obj_del(s_status_label);
        s_status_label = NULL;
    }

    if(s_horizon != NULL)
    {
        lv_obj_del(s_horizon);
        s_horizon = NULL;
    }

    s_roll = 0.0f;
    s_pitch = 0.0f;

    /* The filter is not this instrument's to reset: every screen shares
     * it (navigation.h), and it goes on running without a display. */
}
