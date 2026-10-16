/*
 * Host tests for magnetometer calibration: the compass swing.
 *
 * A simulated sensor with a known hard iron offset and per-axis gain
 * error is turned once round about the vertical - level, or sitting
 * nose up as an aircraft on its wheels does - and the calibration
 * recovered from it is checked by what matters: the heading it gives,
 * at every heading, level and banked.
 */

#include "mag_calibration.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>

static const float DEG = 3.14159265f / 180.0f;

static int g_failures = 0;
static int g_checks = 0;

static void check_near(float actual, float expected, float tolerance,
                       const char *what)
{
    g_checks++;

    if(fabsf(actual - expected) > tolerance)
    {
        g_failures++;
        printf("  FAIL %-50s %+8.3f, expected %+.3f +- %.3f\n",
               what, actual, expected, tolerance);
    }
    else
    {
        printf("  ok   %-50s %+8.3f\n", what, actual);
    }
}

static void check_true(bool condition, const char *what)
{
    g_checks++;

    if(!condition)
    {
        g_failures++;
        printf("  FAIL %s\n", what);
    }
    else
    {
        printf("  ok   %s\n", what);
    }
}

/* ---- simulated sensor ------------------------------------------------ */

/* The earth's field: 48 uT, dipping 67 degrees, as in Poland. */
static const float FIELD = 48.0f;
static const float TRUE_DIP = 67.0f;

/* What the installation does, in sensor axes. */
static const AttitudeVec3 HARD_IRON = { 12.0f, -7.5f, 20.0f };
static const AttitudeVec3 SOFT_IRON = { 1.06f, 0.95f, 1.00f };

static uint32_t s_rng = 99u;

static float noise(float amplitude)
{
    s_rng = s_rng * 1664525u + 1013904223u;
    return ((float)((s_rng >> 8) & 0xFFFFu) / 32767.5f - 1.0f) * amplitude;
}

/*
 * Body axes x forward, y right, z down; for these tests the sensor's
 * axes are the body's. An earth vector (north, east, down) seen from a
 * body at heading, pitch and roll.
 */
static AttitudeVec3 to_body(float n, float e, float d, float heading,
                            float pitch, float roll)
{
    const float sy = sinf(heading * DEG), cy = cosf(heading * DEG);
    const float sp = sinf(pitch * DEG), cp = cosf(pitch * DEG);
    const float sr = sinf(roll * DEG), cr = cosf(roll * DEG);

    const float x1 = n * cy + e * sy;
    const float y1 = -n * sy + e * cy;
    const float z1 = d;

    const float x2 = x1 * cp - z1 * sp;
    const float z2 = x1 * sp + z1 * cp;

    AttitudeVec3 out = { x2, y1 * cr + z2 * sr, -y1 * sr + z2 * cr };
    return out;
}

static AttitudeVec3 earth_field(void)
{
    AttitudeVec3 f = { FIELD * cosf(TRUE_DIP * DEG), 0.0f,
                       FIELD * sinf(TRUE_DIP * DEG) };
    return f;
}

/* What the installed sensor reads at this attitude. */
static AttitudeVec3 reading(float heading, float pitch, float roll)
{
    const AttitudeVec3 f = earth_field();
    const AttitudeVec3 b = to_body(f.x, f.y, f.z, heading, pitch, roll);

    AttitudeVec3 out = {
        b.x * SOFT_IRON.x + HARD_IRON.x + noise(0.2f),
        b.y * SOFT_IRON.y + HARD_IRON.y + noise(0.2f),
        b.z * SOFT_IRON.z + HARD_IRON.z + noise(0.2f)
    };
    return out;
}

/* Gravity's direction, as the accelerometer gives it, in body axes. */
static AttitudeVec3 down_at(float heading, float pitch, float roll)
{
    return to_body(0.0f, 0.0f, 1.0f, heading, pitch, roll);
}

/* Heading from a corrected field, levelled with the true attitude. */
static float heading_of(const AttitudeVec3 &m, float pitch, float roll)
{
    const float sr = sinf(roll * DEG), cr = cosf(roll * DEG);
    const float sp = sinf(pitch * DEG), cp = cosf(pitch * DEG);

    const float x = m.x * cp + (m.y * sr + m.z * cr) * sp;
    const float y = m.y * cr - m.z * sr;

    float h = atan2f(-y, x) / DEG;
    return (h < 0.0f) ? h + 360.0f : h;
}

static float wrap180(float a)
{
    while(a > 180.0f) a -= 360.0f;
    while(a < -180.0f) a += 360.0f;
    return a;
}

/* A swing: from one heading through `degrees`, at a steady pitch, at
 * about 10 deg/s, sampled at 50 Hz - with a small wobble in roll. */
static void swing(MagCalibrator &c, float degrees, float pitch,
                  float wobble = 0.5f)
{
    const int samples = (int)(fabsf(degrees) / 10.0f * 50.0f);

    for(int i = 0; i < samples; i++)
    {
        const float h = 30.0f + degrees * (float)i / (float)samples;
        const float roll = wobble * sinf((float)i * 0.05f);

        c.add(reading(h, pitch, roll), down_at(h, pitch, roll));
    }
}

/* Worst heading error of a calibration, all round, at this attitude. */
static float worst_heading_error(const MagCalibration &cal, float pitch,
                                 float roll)
{
    float worst = 0.0f;

    for(int h = 0; h < 360; h += 5)
    {
        const AttitudeVec3 m = magCorrect(cal, reading((float)h, pitch, roll));
        const float error =
            fabsf(wrap180(heading_of(m, pitch, roll) - (float)h));

        if(error > worst)
            worst = error;
    }

    return worst;
}

/* ---- tests ----------------------------------------------------------- */

static void test_level_swing(void)
{
    printf("\nlevel swing\n");

    MagCalibrator c;

    swing(c, 180.0f, 0.0f);
    check_true(!c.progress().ready, "half a turn is not enough");

    MagCalibration cal;
    check_true(!c.result(&cal), "and gives no result");

    swing(c, 360.0f, 0.0f);

    const MagCalibrator::Progress p = c.progress();
    printf("  info coverage %.0f%%, tilt %.1f deg, fit error %.3f\n",
           p.coverage * 100.0f, p.tilt_spread_deg, p.fit_error);
    check_true(p.ready, "a full turn is");
    check_true(c.result(&cal), "and gives a result");

    check_near(cal.offset.x, HARD_IRON.x, 0.3f, "hard iron, forward axis");
    check_near(cal.offset.y, HARD_IRON.y, 0.3f, "hard iron, right axis");
    check_near(cal.offset.z, HARD_IRON.z, 1.0f,
               "hard iron, vertical axis, from the dip");
    check_near(cal.field, FIELD, 2.0f, "field strength");
    check_near(cal.scale.x / cal.scale.y, SOFT_IRON.y / SOFT_IRON.x, 0.01f,
               "soft iron: forward against right gain");

    check_near(worst_heading_error(cal, 0.0f, 0.0f), 0.0f, 1.0f,
               "heading error level, worst all round");
    check_near(worst_heading_error(cal, 5.0f, 20.0f), 0.0f, 1.5f,
               "heading error at 20 bank, 5 pitch");

    /* Uncalibrated, for comparison. */
    MagCalibration none;
    printf("  info uncalibrated, the worst level heading error is %.0f deg\n",
           worst_heading_error(none, 0.0f, 0.0f));
}

/* An aircraft on its wheels sits nose up; the swing is still about the
 * true vertical, and the calibration has to hold in level flight. */
static void test_nose_up_swing(void)
{
    printf("\nswing sitting 10 deg nose up\n");

    MagCalibrator c;
    swing(c, 400.0f, 10.0f);

    /*
     * Looser than level: the plane of the swing is tilted against the
     * sensor's axes, so the gains the ellipse gives are only nearly
     * the per-axis ones the correction applies. Two degrees is well
     * inside the five a compass deviation card allows.
     */
    MagCalibration cal;
    check_true(c.result(&cal), "calibrated");
    check_near(worst_heading_error(cal, 0.0f, 0.0f), 0.0f, 2.5f,
               "heading error in level flight, worst all round");
    check_near(worst_heading_error(cal, 10.0f, 0.0f), 0.0f, 2.5f,
               "heading error at the swing's own attitude");
}

/*
 * The dip is set, not measured, and it matters in a bank: the vertical
 * offset it gives is off by the horizontal field times the error in
 * tan(dip), which near 67 degrees is steep. A world magnetic model
 * gives the dip to a degree or so anywhere; a degree out has to cost
 * only a couple of degrees of heading at 20 degrees of bank.
 */
static void test_wrong_dip(void)
{
    printf("\ndip set wrong\n");

    for(int error = 1; error <= 4; error += 3)
    {
        MagCalibratorConfig config;
        config.dip_deg = TRUE_DIP - (float)error;

        MagCalibrator c(config);
        swing(c, 400.0f, 0.0f);

        MagCalibration cal;
        c.result(&cal);

        const float level = worst_heading_error(cal, 0.0f, 0.0f);
        const float banked = worst_heading_error(cal, 0.0f, 20.0f);

        printf("  info dip %d deg out: worst heading error %.1f level, "
               "%.1f at 20 deg of bank\n", error, level, banked);

        if(error == 1)
        {
            check_near(level, 0.0f, 1.0f, "1 deg out, level: no worse");
            check_near(banked, 0.0f, 3.0f,
                       "1 deg out, 20 deg bank: a couple of degrees");
        }
    }
}

static void test_wobble_rejected(void)
{
    printf("\ntilted about during the swing\n");

    MagCalibrator c;
    swing(c, 400.0f, 0.0f, 25.0f);

    const MagCalibrator::Progress p = c.progress();
    printf("  info tilt spread %.1f deg\n", p.tilt_spread_deg);
    check_true(!p.ready, "rolled 25 deg back and forth: not accepted");
}

static void test_disturbed_rejected(void)
{
    printf("\nfield disturbed during the swing\n");

    MagCalibrator c;

    /* A steel structure alongside for part of the turn. */
    const int samples = 2000;

    for(int i = 0; i < samples; i++)
    {
        const float h = 30.0f + 400.0f * (float)i / (float)samples;
        AttitudeVec3 m = reading(h, 0.0f, 0.0f);

        if(i > 600 && i < 1100)
        {
            m.x += 15.0f;
            m.y -= 10.0f;
        }

        c.add(m, down_at(h, 0.0f, 0.0f));
    }

    const MagCalibrator::Progress p = c.progress();
    printf("  info fit error %.3f\n", p.fit_error);
    check_true(!p.ready, "not accepted");
}

static void test_reset(void)
{
    printf("\nreset\n");

    MagCalibrator c;
    swing(c, 400.0f, 0.0f);
    c.reset();
    check_true(c.progress().samples == 0, "reset clears it");
    check_true(!c.progress().ready, "and it is not ready");
}

int main(void)
{
    test_level_swing();
    test_nose_up_swing();
    test_wrong_dip();
    test_wobble_rejected();
    test_disturbed_rejected();
    test_reset();

    printf("\n%d checks, %d failed\n", g_checks, g_failures);

    return g_failures == 0 ? 0 : 1;
}
