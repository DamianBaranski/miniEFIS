#include "turn_coordinator_screen.h"
#include "fonts.h"
#include "fast_draw.h"
#include "navigation.h"

#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const uint32_t UPDATE_PERIOD_MS = 20;

/* Smallest change worth a redraw. */
static const float REDRAW_RATE_DPS = 0.02f;
static const float REDRAW_SLIP_DEG = 0.05f;

/*
 * The symbol banks 20 degrees for a standard rate turn, which is where
 * the lower marks sit - the same proportion as most real instruments.
 * It stops a little past double standard rate.
 */
static const float SYMBOL_DEG_AT_STANDARD_RATE = 20.0f;
static const float SYMBOL_MAX_DEG = 45.0f;

/* Marks around the dial, from the centre. */
static const float MARK_INNER = 160.0f;
static const float MARK_OUTER = 200.0f;

/* Aircraft symbol, rear view. */
static const float WING_SPAN_HALF = 140.0f;
static const float FUSELAGE_RADIUS = 16.0f;
static const float FIN_HEIGHT = 34.0f;

/*
 * Inclinometer: a tube curved up at the ends, an arc of a circle whose
 * centre is above the screen centre, so the ball rests at the bottom and
 * climbs as it moves out.
 */
static const float TUBE_CENTRE_Y = -170.0f;      /* from screen centre */
static const float TUBE_RADIUS = 300.0f;
static const float TUBE_HALF_WIDTH = 15.0f;
static const float TUBE_HALF_ANGLE_DEG = 17.0f;
static const float BALL_RADIUS = 12.0f;

static const uint32_t MARK_COLOUR = 0xFFFFFF;
static const uint32_t SYMBOL_COLOUR = 0xFFFFFF;
static const uint32_t TUBE_COLOUR = 0x808890;
static const uint32_t BALL_COLOUR = 0xFFFFFF;
static const uint32_t TEXT_COLOUR = 0xA0A8B0;
static const uint32_t FLAG_COLOUR = 0xFF4040;
static const uint32_t CAUTION_COLOUR = 0xFFD400;

static lv_obj_t *s_screen = NULL;
static lv_obj_t *s_dial = NULL;

/* What the frame being drawn shows. Owned by the LVGL task. */
static TurnSample s_shown;
static AttitudeStatus s_status = AttitudeStatus::NoData;

static float rad(float d)
{
    return d * (float)M_PI / 180.0f;
}

static float clampf(float value, float low, float high)
{
    return (value < low) ? low : (value > high) ? high : value;
}

/* ---- drawing -------------------------------------------------------------- */

/* A mark on the dial, at an angle clockwise from straight up. */
static void draw_mark(lv_draw_ctx_t *draw_ctx, float cx, float cy,
                      float angle_deg, uint16_t width)
{
    const float a = rad(angle_deg);

    fast_draw_line(draw_ctx,
                   cx + MARK_INNER * sinf(a), cy - MARK_INNER * cosf(a),
                   cx + MARK_OUTER * sinf(a), cy - MARK_OUTER * cosf(a),
                   lv_color_hex(MARK_COLOUR), width);
}

static void draw_dial(lv_draw_ctx_t *draw_ctx, float cx, float cy)
{
    /* Wings level, and standard rate either way. */
    draw_mark(draw_ctx, cx, cy, 90.0f, 7);
    draw_mark(draw_ctx, cx, cy, 270.0f, 7);
    draw_mark(draw_ctx, cx, cy, 90.0f + SYMBOL_DEG_AT_STANDARD_RATE, 7);
    draw_mark(draw_ctx, cx, cy, 270.0f - SYMBOL_DEG_AT_STANDARD_RATE, 7);

    const lv_color_t text = lv_color_hex(MARK_COLOUR);
    const float label_radius = MARK_INNER - 22.0f;
    const float label_angle = rad(90.0f + SYMBOL_DEG_AT_STANDARD_RATE + 8.0f);

    fast_draw_text(draw_ctx, cx - label_radius * sinf(label_angle),
                   cy - label_radius * cosf(label_angle), "L",
                   FONT_HUGE, text);
    fast_draw_text(draw_ctx, cx + label_radius * sinf(label_angle),
                   cy - label_radius * cosf(label_angle), "R",
                   FONT_HUGE, text);

    fast_draw_text(draw_ctx, cx, cy + 74.0f, "2 MIN",
                   FONT_LARGE, text);
    fast_draw_text(draw_ctx, cx, cy - 168.0f, "TURN COORDINATOR",
                   FONT_SMALL, lv_color_hex(TEXT_COLOUR));
    fast_draw_text(draw_ctx, cx, cy + 190.0f, "NO PITCH INFORMATION",
                   FONT_SMALL, lv_color_hex(TEXT_COLOUR));
}

static void draw_aircraft(lv_draw_ctx_t *draw_ctx, float cx, float cy,
                          float bank_deg)
{
    const lv_color_t white = lv_color_hex(SYMBOL_COLOUR);

    /* Positive banks clockwise: right wing down, for a right turn. */
    const float a = rad(bank_deg);
    const float ux = cosf(a);
    const float uy = sinf(a);

    fast_draw_line(draw_ctx,
                   cx - WING_SPAN_HALF * ux, cy - WING_SPAN_HALF * uy,
                   cx + WING_SPAN_HALF * ux, cy + WING_SPAN_HALF * uy,
                   white, 7);

    /* Fin, up from the fuselage. */
    fast_draw_line(draw_ctx, cx, cy,
                   cx + FIN_HEIGHT * uy, cy - FIN_HEIGHT * ux,
                   white, 5);

    fast_fill_circle(draw_ctx, cx, cy, FUSELAGE_RADIUS, white);
    fast_fill_circle(draw_ctx, cx, cy, FUSELAGE_RADIUS - 6.0f, lv_color_black());
}

/* A point along the tube, at an angle from its lowest point. */
static void tube_point(float cx, float cy, float angle_deg, float radius,
                       float *x, float *y)
{
    const float a = rad(angle_deg);

    *x = cx + radius * sinf(a);
    *y = cy + TUBE_CENTRE_Y + radius * cosf(a);
}

static void draw_inclinometer(lv_draw_ctx_t *draw_ctx, float cx, float cy,
                              float slip_deg)
{
    const lv_color_t tube = lv_color_hex(TUBE_COLOUR);
    const int segments = 12;

    /* Both walls of the tube as polylines, and the ends closed. */
    for(int wall = -1; wall <= 1; wall += 2)
    {
        const float radius = TUBE_RADIUS + (float)wall * TUBE_HALF_WIDTH;

        for(int i = 0; i < segments; i++)
        {
            const float a0 = -TUBE_HALF_ANGLE_DEG +
                             2.0f * TUBE_HALF_ANGLE_DEG * (float)i / segments;
            const float a1 = -TUBE_HALF_ANGLE_DEG +
                             2.0f * TUBE_HALF_ANGLE_DEG * (float)(i + 1) / segments;

            float x0, y0, x1, y1;
            tube_point(cx, cy, a0, radius, &x0, &y0);
            tube_point(cx, cy, a1, radius, &x1, &y1);

            fast_draw_line(draw_ctx, x0, y0, x1, y1, tube, 2);
        }
    }

    for(int end = -1; end <= 1; end += 2)
    {
        float x0, y0, x1, y1;
        const float a = (float)end * TUBE_HALF_ANGLE_DEG;

        tube_point(cx, cy, a, TUBE_RADIUS - TUBE_HALF_WIDTH, &x0, &y0);
        tube_point(cx, cy, a, TUBE_RADIUS + TUBE_HALF_WIDTH, &x1, &y1);

        fast_draw_line(draw_ctx, x0, y0, x1, y1, tube, 2);
    }

    /*
     * The ball sits where the tube's wall is square to the apparent
     * gravity, which on a circular tube is the slip angle itself. It
     * stops at the ends.
     */
    const float limit = TUBE_HALF_ANGLE_DEG -
                        (BALL_RADIUS / TUBE_RADIUS) * 180.0f / (float)M_PI;
    const float ball_angle = clampf(slip_deg, -limit, limit);

    float bx, by;
    tube_point(cx, cy, ball_angle, TUBE_RADIUS, &bx, &by);

    fast_fill_circle(draw_ctx, bx, by, BALL_RADIUS, lv_color_hex(BALL_COLOUR));

    /* The two wires the ball should sit between. */
    const float gap = BALL_RADIUS + 3.0f;
    const float top = cy + TUBE_CENTRE_Y + TUBE_RADIUS - TUBE_HALF_WIDTH - 4.0f;
    const float bottom = cy + TUBE_CENTRE_Y + TUBE_RADIUS + TUBE_HALF_WIDTH + 4.0f;

    fast_draw_line(draw_ctx, cx - gap, top, cx - gap, bottom,
                   lv_color_hex(MARK_COLOUR), 2);
    fast_draw_line(draw_ctx, cx + gap, top, cx + gap, bottom,
                   lv_color_hex(MARK_COLOUR), 2);
}

static void draw_readout(lv_draw_ctx_t *draw_ctx, float cx, float cy)
{
    char text[24];
    lv_color_t colour = lv_color_white();

    switch(s_status)
    {
        case AttitudeStatus::Fused:
        case AttitudeStatus::GyroOnly:
        {
            /* Rounded to what the eye can use; a turn below 0.05 deg/s
             * reads as straight rather than flickering L and R. */
            const float rate = s_shown.turn_rate_dps;
            const float shown = fabsf(rate) < 0.05f ? 0.0f : fabsf(rate);
            const char side =
                (shown == 0.0f) ? ' ' : (rate > 0.0f ? 'R' : 'L');

            snprintf(text, sizeof(text), "%.1f deg/s %c", shown, side);
            break;
        }

        case AttitudeStatus::Calibrating:
            snprintf(text, sizeof(text), "CAL - HOLD STILL");
            colour = lv_color_hex(CAUTION_COLOUR);
            break;

        default:
            snprintf(text, sizeof(text), "OFF");
            colour = lv_color_hex(FLAG_COLOUR);
            break;
    }

    fast_draw_text(draw_ctx, cx, cy - 120.0f, text, FONT_LARGE,
                   colour);
}

static void draw(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *draw_ctx = lv_event_get_draw_ctx(e);

    lv_area_t area;
    lv_obj_get_coords(obj, &area);

    const float cx = (float)(area.x1 + area.x2) * 0.5f;
    const float cy = (float)(area.y1 + area.y2) * 0.5f;

    const float bank = clampf(
        s_shown.turn_rate_dps *
            (SYMBOL_DEG_AT_STANDARD_RATE / TURN_STANDARD_RATE_DPS),
        -SYMBOL_MAX_DEG, SYMBOL_MAX_DEG);

    draw_dial(draw_ctx, cx, cy);
    draw_inclinometer(draw_ctx, cx, cy, s_shown.slip_deg);
    draw_aircraft(draw_ctx, cx, cy, bank);
    draw_readout(draw_ctx, cx, cy);
}

static void update(lv_timer_t *timer)
{
    (void)timer;

    if(lv_scr_act() != s_screen)
        return;

    const NavigationSample nav = navigation_sample();
    const TurnSample &sample = nav.turn;
    const AttitudeStatus status = nav.attitude.status;

    if(status == s_status &&
       fabsf(sample.turn_rate_dps - s_shown.turn_rate_dps) < REDRAW_RATE_DPS &&
       fabsf(sample.slip_deg - s_shown.slip_deg) < REDRAW_SLIP_DEG)
        return;

    s_shown = sample;
    s_status = status;

    lv_obj_invalidate(s_dial);
}

void turn_coordinator_screen_create(lv_obj_t *screen)
{
    s_screen = screen;

    s_dial = lv_obj_create(screen);

    lv_obj_remove_style_all(s_dial);
    lv_obj_set_size(s_dial, LV_PCT(100), LV_PCT(100));
    lv_obj_center(s_dial);

    /* Taps go through to the screen, for the double tap menu. */
    lv_obj_clear_flag(s_dial, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(s_dial, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_add_event_cb(s_dial, draw, LV_EVENT_DRAW_MAIN, NULL);

    lv_timer_create(update, UPDATE_PERIOD_MS, NULL);
}
