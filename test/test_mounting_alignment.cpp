/*
 * Host tests for mounting alignment: levelling a unit installed at an
 * angle in the panel.
 *
 * Gravity is simulated as a unit tilted in pitch and roll would see it
 * with the aircraft level, and the alignment levelled from it is
 * checked by what matters: gravity then reads straight down, a turn
 * about the aircraft's vertical stays a turn about the vertical, and
 * the tilt reported is the one put in.
 */

#include "mounting_alignment.h"

#include <math.h>
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
        printf("  FAIL %-50s %+8.4f, expected %+.4f +- %.4f\n",
               what, actual, expected, tolerance);
    }
    else
    {
        printf("  ok   %-50s %+8.4f\n", what, actual);
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

/*
 * A vector in aircraft axes as a unit pitched and rolled against the
 * aircraft sees it - the unit's attitude applied the way the filter's
 * is: roll about x after pitch about y.
 */
static AttitudeVec3 in_unit(const AttitudeVec3 &v, float pitch_deg,
                            float roll_deg)
{
    const float sp = sinf(pitch_deg * DEG), cp = cosf(pitch_deg * DEG);
    const float sr = sinf(roll_deg * DEG), cr = cosf(roll_deg * DEG);

    /* Pitch: about y. */
    const AttitudeVec3 p = {
        cp * v.x - sp * v.z,
        v.y,
        sp * v.x + cp * v.z
    };

    /* Roll: about x. */
    const AttitudeVec3 out = {
        p.x,
        cr * p.y + sr * p.z,
        -sr * p.y + cr * p.z
    };

    return out;
}

static const AttitudeVec3 DOWN = { 0.0f, 0.0f, 1.0f };

static void test_identity(void)
{
    printf("default: no alignment\n");

    const MountingAlignment alignment;
    const AttitudeVec3 v = { 0.3f, -0.4f, 0.8f };
    const AttitudeVec3 out = mountingApply(alignment, v);

    check_true(mountingIsIdentity(alignment), "identity");
    check_near(out.x, v.x, 1e-6f, "x unchanged");
    check_near(out.y, v.y, 1e-6f, "y unchanged");
    check_near(out.z, v.z, 1e-6f, "z unchanged");
}

static void test_level(float pitch, float roll)
{
    printf("unit pitched %+.0f, rolled %+.0f\n", pitch, roll);

    const AttitudeVec3 gravity = in_unit(DOWN, pitch, roll);

    /* As the accelerometer has it: not exactly 1 g. */
    const AttitudeVec3 measured = {
        gravity.x * 0.98f, gravity.y * 0.98f, gravity.z * 0.98f
    };

    MountingAlignment alignment;
    check_true(mountingLevel(MountingAlignment(), measured, 20.0f,
                             &alignment), "levelled");

    const AttitudeVec3 down = mountingApply(alignment, gravity);
    check_near(down.x, 0.0f, 1e-5f, "gravity forward");
    check_near(down.y, 0.0f, 1e-5f, "gravity right");
    check_near(down.z, 1.0f, 1e-5f, "gravity down");

    float tilt_pitch, tilt_roll;
    mountingTilt(alignment, &tilt_pitch, &tilt_roll);
    check_near(tilt_pitch,
               atan2f(-gravity.x, sqrtf(gravity.y * gravity.y +
                                        gravity.z * gravity.z)) / DEG,
               1e-3f, "tilt pitch as the unit reads it");
    check_near(tilt_roll, atan2f(gravity.y, gravity.z) / DEG, 1e-3f,
               "tilt roll as the unit reads it");

    /* Back and forth is the identity. */
    const AttitudeVec3 v = { 0.3f, -0.4f, 0.8f };
    const AttitudeVec3 round = mountingUnapply(alignment,
                                               mountingApply(alignment, v));
    check_near(round.x, v.x, 1e-5f, "unapply undoes apply, x");
    check_near(round.y, v.y, 1e-5f, "unapply undoes apply, y");
    check_near(round.z, v.z, 1e-5f, "unapply undoes apply, z");

    /* Lengths kept: a rotation. */
    const AttitudeVec3 a = mountingApply(alignment, v);
    check_near(sqrtf(a.x * a.x + a.y * a.y + a.z * a.z),
               sqrtf(v.x * v.x + v.y * v.y + v.z * v.z), 1e-5f,
               "length kept");
}

/*
 * A yaw rate about the aircraft's vertical, seen by the tilted unit as
 * a mix of its axes, comes back as yaw alone - the turn coordinator
 * and the heading must not see a turn as a roll.
 */
static void test_turn_stays_a_turn(void)
{
    printf("a turn stays a turn\n");

    const float pitch = -8.0f, roll = 3.0f;
    MountingAlignment alignment;

    mountingLevel(MountingAlignment(), in_unit(DOWN, pitch, roll), 20.0f,
                  &alignment);

    const AttitudeVec3 yaw = { 0.0f, 0.0f, 15.0f };
    const AttitudeVec3 rate = mountingApply(alignment,
                                            in_unit(yaw, pitch, roll));

    check_near(rate.x, 0.0f, 1e-4f, "roll rate");
    check_near(rate.y, 0.0f, 1e-4f, "pitch rate");
    check_near(rate.z, 15.0f, 1e-4f, "yaw rate");
}

static void test_refine(void)
{
    printf("levelling again refines\n");

    MountingAlignment first;
    mountingLevel(MountingAlignment(), in_unit(DOWN, 6.0f, -4.0f), 20.0f,
                  &first);

    /* The second time, gravity arrives with the first alignment applied:
     * already level, so nothing changes. */
    const AttitudeVec3 seen = mountingApply(first, in_unit(DOWN, 6.0f, -4.0f));
    MountingAlignment second;
    check_true(mountingLevel(first, seen, 20.0f, &second), "levelled again");

    float p1, r1, p2, r2;
    mountingTilt(first, &p1, &r1);
    mountingTilt(second, &p2, &r2);
    check_near(p2, p1, 1e-3f, "pitch unchanged");
    check_near(r2, r1, 1e-3f, "roll unchanged");
}

static void test_rejected(void)
{
    printf("rejected\n");

    MountingAlignment out;
    out.m[0][0] = 7.0f;             /* to see it left alone */

    check_true(!mountingLevel(MountingAlignment(),
                              in_unit(DOWN, 30.0f, 0.0f), 20.0f, &out),
               "too steep");

    const AttitudeVec3 none = { 0.0f, 0.0f, 0.1f };
    check_true(!mountingLevel(MountingAlignment(), none, 20.0f, &out),
               "no gravity");

    check_true(out.m[0][0] == 7.0f, "output untouched");

    /* Two steps of 15 each way: each allowed, together not. */
    MountingAlignment first;
    mountingLevel(MountingAlignment(), in_unit(DOWN, 15.0f, 0.0f), 20.0f,
                  &first);
    check_true(!mountingLevel(first, in_unit(DOWN, 15.0f, 0.0f), 20.0f, &out),
               "too steep altogether");
}

int main(void)
{
    test_identity();
    test_level(5.0f, 0.0f);
    test_level(0.0f, -3.0f);
    test_level(-12.0f, 7.0f);
    test_turn_stays_a_turn();
    test_refine();
    test_rejected();

    printf("\n%d checks, %d failed\n", g_checks, g_failures);

    return g_failures == 0 ? 0 : 1;
}
