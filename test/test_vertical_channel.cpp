/*
 * Host tests for VerticalChannel.
 *
 * Driven with a simulated barometer and accelerometer: a true vertical
 * motion is chosen, the accelerometer is fed its acceleration and the
 * barometer its altitude with realistic noise, and the estimate is
 * checked against the truth that produced them.
 *
 * The barometer is delivered every fourth step, as the real one is:
 * 25 Hz against the IMU's 100 Hz.
 */

#include "vertical_channel.h"

#include <math.h>
#include <stdio.h>

static const float DT = 0.01f;              /* 100 Hz, as the firmware runs */
static const int BARO_EVERY = 4;            /* 25 Hz */

/* Measured on the board: 0.106 m standard deviation. */
static const float BARO_NOISE_M = 0.106f;

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

/* Deterministic noise, so a failure is reproducible. */
static uint32_t s_rng = 1u;

static float noise(float amplitude)
{
    s_rng = s_rng * 1664525u + 1013904223u;
    const float unit = (float)((s_rng >> 8) & 0xFFFFu) / 32767.5f - 1.0f;
    return unit * amplitude;
}

/* Simulated truth. */
static float s_altitude = 0.0f;
static float s_speed = 0.0f;
static int s_step = 0;

/* An error in the accelerometer channel, as an attitude error produces. */
static float s_accel_error = 0.0f;

static void run(VerticalChannel &channel, float seconds, float acceleration,
                bool with_baro = true, float baro_noise = BARO_NOISE_M)
{
    const int steps = (int)(seconds / DT + 0.5f);

    for(int i = 0; i < steps; i++)
    {
        s_speed += acceleration * DT;
        s_altitude += s_speed * DT;
        s_step++;

        const float measured_acceleration =
            acceleration + s_accel_error + noise(0.02f);

        const float baro = s_altitude + noise(baro_noise);
        const bool deliver = with_baro && (s_step % BARO_EVERY == 0);

        channel.update(measured_acceleration, deliver ? &baro : 0, DT);
    }
}

static void test_startup(void)
{
    printf("\nstart-up\n");

    s_altitude = 100.0f; s_speed = 0.0f; s_step = 0; s_accel_error = 0.0f;

    VerticalChannel channel;

    check_true(!channel.valid(), "invalid before any barometer reading");

    channel.update(0.0f, 0, DT);
    check_true(!channel.valid(), "still invalid on acceleration alone");

    run(channel, 1.0f, 0.0f);
    check_true(channel.valid(), "valid once the barometer has reported");
    check_near(channel.altitude(), 100.0f, 0.5f, "altitude picked up");
}

static void test_steady_and_quiet(void)
{
    printf("\nsitting still\n");

    s_altitude = 100.0f; s_speed = 0.0f; s_step = 0; s_accel_error = 0.0f;
    s_rng = 1u;

    VerticalChannel channel;
    run(channel, 10.0f, 0.0f);

    check_near(channel.altitude(), 100.0f, 0.2f, "altitude holds");
    check_near(channel.verticalSpeed(), 0.0f, 0.15f, "vertical speed near zero");

    /*
     * The point of the whole exercise: differentiating the barometer
     * directly would give +-2.5 m/s of noise at this sample rate, which
     * is unreadable. Measure what the filter actually delivers.
     */
    float worst = 0.0f;

    for(int i = 0; i < 1000; i++)
    {
        run(channel, DT, 0.0f);

        const float speed = channel.verticalSpeed();

        if(fabsf(speed) > worst)
            worst = fabsf(speed);
    }

    printf("  info worst vertical speed excursion over 10 s: %.3f m/s\n", worst);
    check_true(worst < 0.4f, "vertical speed noise under 0.4 m/s");
}

static void test_climb(void)
{
    printf("\nsteady climb\n");

    s_altitude = 0.0f; s_speed = 0.0f; s_step = 0; s_accel_error = 0.0f;

    VerticalChannel channel;
    run(channel, 1.0f, 0.0f);

    /* Accelerate to 5 m/s over 2 s, then hold it for 20 s. */
    run(channel, 2.0f, 2.5f);
    run(channel, 20.0f, 0.0f);

    check_near(channel.verticalSpeed(), 5.0f, 0.3f, "tracks 5 m/s climb");
    check_near(channel.altitude(), s_altitude, 1.0f, "altitude tracks truth");

    /* Level off, and it must come back to zero rather than hang. */
    run(channel, 2.0f, -2.5f);
    run(channel, 8.0f, 0.0f);

    check_near(channel.verticalSpeed(), 0.0f, 0.2f, "settles after levelling");
    check_near(channel.altitude(), s_altitude, 1.0f, "altitude still tracks");
}

/*
 * The reason the loop is third order.
 *
 * Vertical acceleration comes from the attitude estimate, so an attitude
 * error tilts gravity into it - and in a sustained turn that error is
 * systematic. Integrated without correction, 0.5 m/s^2 would read as a
 * 30 m/s climb after a minute. The bias state has to absorb it.
 */
static void test_accelerometer_error(void)
{
    printf("\nsystematic accelerometer error\n");

    s_altitude = 500.0f; s_speed = 0.0f; s_step = 0; s_accel_error = 0.0f;

    VerticalChannel channel;
    run(channel, 2.0f, 0.0f);

    s_accel_error = 0.5f;               /* as a 3 degree attitude error gives */
    run(channel, 30.0f, 0.0f);

    check_near(channel.verticalSpeed(), 0.0f, 0.3f,
               "level flight still reads level");
    check_near(channel.altitude(), s_altitude, 1.0f, "altitude still right");
    check_near(channel.sample().accel_bias_ms2, 0.5f, 0.15f,
               "the error was identified as a bias");

    /* And a real climb on top of the error is still seen. */
    run(channel, 2.0f, 2.0f);
    run(channel, 15.0f, 0.0f);
    check_near(channel.verticalSpeed(), 4.0f, 0.4f,
               "a real climb is still tracked through it");

    s_accel_error = 0.0f;
}

static void test_barometer_gap(void)
{
    printf("\nbarometer interruptions\n");

    s_altitude = 200.0f; s_speed = 0.0f; s_step = 0; s_accel_error = 0.0f;

    VerticalChannel channel;
    run(channel, 2.0f, 0.0f);
    run(channel, 2.0f, 1.0f);           /* climbing at 2 m/s */

    const float speed_before = channel.verticalSpeed();

    /* Two seconds with no barometer at all: the accelerometer carries it. */
    run(channel, 2.0f, 0.0f, false);

    check_near(channel.verticalSpeed(), speed_before, 0.5f,
               "coasts through a barometer dropout");

    run(channel, 4.0f, 0.0f);
    check_near(channel.verticalSpeed(), 2.0f, 0.3f, "recovers when it returns");
}

static void test_altitude_jump(void)
{
    printf("\nreference pressure changed\n");

    s_altitude = 300.0f; s_speed = 0.0f; s_step = 0; s_accel_error = 0.0f;

    VerticalChannel channel;
    run(channel, 3.0f, 0.0f);

    /* A new QNH setting, or a glitch: 200 m in one step. */
    s_altitude += 200.0f;
    run(channel, 0.5f, 0.0f);

    check_near(channel.altitude(), s_altitude, 2.0f,
               "adopts the new reference at once");
    check_near(channel.verticalSpeed(), 0.0f, 0.3f,
               "without inventing a vertical speed");
}

/*
 * A QNH change of a couple of hPa moves the barometric altitude by
 * 16 m - under the jump threshold, so on its own it would be worked off
 * slowly, as a measurement error, with a false climb while it was.
 * shiftAltitude() moves the estimate with it instead.
 */
static void test_reference_shift(void)
{
    printf("\nsmall QNH change, shifted\n");

    s_altitude = 300.0f; s_speed = 0.0f; s_step = 0; s_accel_error = 0.0f;

    VerticalChannel unshifted;
    VerticalChannel shifted;
    run(unshifted, 5.0f, 0.0f);
    s_altitude = 300.0f; s_speed = 0.0f; s_step = 0;
    run(shifted, 5.0f, 0.0f);

    const float qnh_step_m = 16.0f;

    s_altitude += qnh_step_m;
    const float target = s_altitude;
    shifted.shiftAltitude(qnh_step_m);

    const float saved_altitude = s_altitude;
    run(shifted, 0.3f, 0.0f);
    s_altitude = saved_altitude; s_speed = 0.0f;
    run(unshifted, 0.3f, 0.0f);

    check_near(shifted.altitude(), target, 0.5f, "shifted: there at once");
    check_near(shifted.verticalSpeed(), 0.0f, 0.2f,
               "shifted: no false vertical speed");
    check_true(fabsf(unshifted.altitude() - target) > 3.0f,
               "unshifted: still catching up (why the shift exists)");

    VerticalChannel fresh;
    fresh.shiftAltitude(10.0f);
    check_true(!fresh.valid(), "a shift before the first reading is ignored");
}

int main(void)
{
    test_startup();
    test_steady_and_quiet();
    test_climb();
    test_accelerometer_error();
    test_barometer_gap();
    test_altitude_jump();
    test_reference_shift();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return (g_failures == 0) ? 0 : 1;
}
