/*
 * Host tests for TurnCoordinator.
 *
 * Body rates and specific force are worked out for a known manoeuvre -
 * a coordinated turn, a bank held without turning, a turn while pitched
 * - and the instrument is checked against the turn rate and ball
 * position that manoeuvre should show.
 */

#include "turn_coordinator.h"

#include <math.h>
#include <stdio.h>

static const float DT = 0.01f;              /* 100 Hz, as the firmware runs */
static const float RAD = 3.14159265f / 180.0f;

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

/*
 * Body rates for a turn at earth rate omega (deg/s), at the given bank
 * and pitch: the earth's vertical, expressed in body axes, times omega.
 */
static AttitudeVec3 turn_rates(float omega, float roll_deg, float pitch_deg)
{
    const float r = roll_deg * RAD;
    const float p = pitch_deg * RAD;

    AttitudeVec3 w;

    w.x = -omega * sinf(p);
    w.y = omega * sinf(r) * cosf(p);
    w.z = omega * cosf(r) * cosf(p);

    return w;
}

/* Gravity alone, as the accelerometer reads it, at a bank and pitch. */
static AttitudeVec3 gravity(float roll_deg, float pitch_deg)
{
    const float r = roll_deg * RAD;
    const float p = pitch_deg * RAD;

    AttitudeVec3 a;

    a.x = -sinf(p);
    a.y = sinf(r) * cosf(p);
    a.z = cosf(r) * cosf(p);

    return a;
}

static void run(TurnCoordinator &tc, const AttitudeVec3 &accel,
                const AttitudeVec3 &gyro, float roll, float pitch,
                float seconds)
{
    for(int i = 0; i < (int)(seconds / DT); i++)
        tc.update(accel, gyro, roll, pitch, DT);
}

static void test_turn_rate(void)
{
    printf("\nturn rate\n");

    TurnCoordinator tc;

    check_true(!tc.sample().valid, "not valid before the first update");

    const AttitudeVec3 still = { 0.0f, 0.0f, 0.0f };

    run(tc, gravity(0, 0), still, 0, 0, 2.0f);

    check_true(tc.sample().valid, "valid once updated");
    check_near(tc.sample().turn_rate_dps, 0.0f, 1.0e-4f, "level and still");
    check_near(tc.sample().slip_deg, 0.0f, 1.0e-4f, "ball centred");

    /* Standard rate right at 25 degrees of bank, coordinated: the
     * specific force is along body z, 1/cos(bank) g. */
    const float bank = 25.0f;
    const AttitudeVec3 coordinated = { 0.0f, 0.0f, 1.0f / cosf(bank * RAD) };

    run(tc, coordinated, turn_rates(3.0f, bank, 0), bank, 0, 5.0f);

    check_near(tc.sample().turn_rate_dps, 3.0f, 0.01f,
               "coordinated standard rate turn, right");
    check_near(tc.sample().slip_deg, 0.0f, 0.01f,
               "ball centred in a coordinated turn");

    run(tc, coordinated, turn_rates(-3.0f, -bank, 0), -bank, 0, 5.0f);
    check_near(tc.sample().turn_rate_dps, -3.0f, 0.01f,
               "coordinated standard rate turn, left");

    /* Climbing turn: pitch must not change the reading. */
    run(tc, gravity(0, 10), turn_rates(4.0f, 0, 10), 0, 10, 5.0f);
    check_near(tc.sample().turn_rate_dps, 4.0f, 0.01f,
               "4 deg/s with 10 degrees of pitch");

    /* A roll alone is not a turn. */
    const AttitudeVec3 rolling = { 20.0f, 0.0f, 0.0f };

    run(tc, gravity(0, 0), rolling, 0, 0, 5.0f);
    check_near(tc.sample().turn_rate_dps, 0.0f, 0.01f, "rolling, not turning");

    /* Steep pitch: limited rather than dividing by zero. */
    run(tc, gravity(0, 0), turn_rates(3.0f, 0, 89), 0, 89, 2.0f);
    check_true(isfinite(tc.sample().turn_rate_dps) &&
               fabsf(tc.sample().turn_rate_dps) < 30.0f,
               "finite at 89 degrees of pitch");
}

static void test_ball(void)
{
    printf("\nslip ball\n");

    TurnCoordinator tc;
    const AttitudeVec3 still = { 0.0f, 0.0f, 0.0f };

    /* Banked right but not turning: slipping, ball falls to the right. */
    run(tc, gravity(15, 0), still, 15, 0, 3.0f);

    check_near(tc.sample().slip_deg, 15.0f, 0.05f, "static bank right, ball right");
    check_near(tc.sample().turn_rate_dps, 0.0f, 0.01f, "and no turn shown");

    run(tc, gravity(-15, 0), still, -15, 0, 3.0f);
    check_near(tc.sample().slip_deg, -15.0f, 0.05f, "static bank left, ball left");

    /* Skidding right turn, wings level: the side force pushes the ball
     * out, to the left. */
    const AttitudeVec3 skid = { 0.0f, -0.1f, 1.0f };

    run(tc, skid, turn_rates(3.0f, 0, 0), 0, 0, 3.0f);
    check_near(tc.sample().slip_deg, atan2f(-0.1f, 1.0f) / RAD, 0.05f,
               "flat skid right, ball left");
}

static void test_damping(void)
{
    printf("\ndamping\n");

    TurnCoordinatorConfig config;
    TurnCoordinator tc(config);

    const AttitudeVec3 still = { 0.0f, 0.0f, 0.0f };
    run(tc, gravity(0, 0), still, 0, 0, 1.0f);

    /* A step in rate: one time constant later it should be about 63% of
     * the way there. */
    const AttitudeVec3 yaw = { 0.0f, 0.0f, 10.0f };

    run(tc, gravity(0, 0), yaw, 0, 0, config.rate_time_constant_s);
    check_near(tc.sample().turn_rate_dps, 6.32f, 0.2f,
               "63% of a step after one time constant");

    /* Vibration: a 20 Hz side to side shake of 0.3 g barely moves the
     * ball. */
    float worst = 0.0f;

    for(int i = 0; i < 300; i++)
    {
        const float shake = 0.3f * sinf(2.0f * 3.14159265f * 20.0f * i * DT);
        const AttitudeVec3 accel = { 0.0f, shake, 1.0f };

        tc.update(accel, still, 0, 0, DT);

        if(i > 100 && fabsf(tc.sample().slip_deg) > worst)
            worst = fabsf(tc.sample().slip_deg);
    }

    check_near(worst, 0.0f, 2.0f, "20 Hz shake of 0.3 g: ball within 2 degrees");

    tc.reset();
    check_true(!tc.sample().valid, "reset clears it");
}

int main(void)
{
    test_turn_rate();
    test_ball();
    test_damping();

    printf("\n%d checks, %d failed\n", g_checks, g_failures);

    return g_failures == 0 ? 0 : 1;
}
