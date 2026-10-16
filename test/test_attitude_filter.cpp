/*
 * Host tests for AttitudeFilter.
 *
 * The filter is driven with a simulated IMU: a true attitude is chosen,
 * and the accelerometer, gyroscope and magnetometer readings a sensor
 * would produce in that attitude are computed and fed in. Every attitude
 * change is driven by the Euler rates that actually produce it, so the
 * gyro and the accelerometer always agree about the motion - feeding an
 * attitude jump with no matching rate would be physically impossible and
 * would make the numbers below meaningless.
 */

#include "attitude_filter.h"

#include <math.h>
#include <stdio.h>

static const float DEG = (float)M_PI / 180.0f;
static const float DT = 0.01f;              /* 100 Hz, as the firmware runs */

/* Earth field, NED, roughly central Europe: 20 uT north, 44 uT down. */
static const float BH = 20.0f;
static const float BZ = 44.0f;

static int g_failures = 0;
static int g_checks = 0;

static void check_near(float actual, float expected, float tolerance,
                       const char *what)
{
    g_checks++;

    const bool ok = fabsf(actual - expected) <= tolerance;

    if(!ok)
    {
        g_failures++;
        printf("  FAIL %-52s %+8.2f, expected %+.2f +- %.2f\n",
               what, actual, expected, tolerance);
    }
    else
    {
        printf("  ok   %-52s %+8.2f\n", what, actual);
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

/* ---- simulated sensors ---------------------------------------------- */

/* Accelerometer scale error, as a real sensor has. 1.0 means perfect. */
static float s_accel_scale = 1.0f;

/* Gyro bias actually present in the simulated sensor, deg/s. */
static float s_bias_x = 0.0f;
static float s_bias_y = 0.0f;
static float s_bias_z = 0.0f;

/* Current simulated truth. */
static float s_roll = 0.0f;
static float s_pitch = 0.0f;
static float s_yaw = 0.0f;

static AttitudeVec3 vec(float x, float y, float z)
{
    AttitudeVec3 v;
    v.x = x; v.y = y; v.z = z;
    return v;
}

/* Rotate an NED vector into body axes: R_x(roll) R_y(pitch) R_z(yaw). */
static AttitudeVec3 to_body(float north, float east, float down,
                            float roll, float pitch, float yaw)
{
    const float sr = sinf(roll * DEG),  cr = cosf(roll * DEG);
    const float sp = sinf(pitch * DEG), cp = cosf(pitch * DEG);
    const float sy = sinf(yaw * DEG),   cy = cosf(yaw * DEG);

    const float ux = north * cy + east * sy;
    const float uy = -north * sy + east * cy;
    const float uz = down;

    const float vx = ux * cp - uz * sp;
    const float vy = uy;
    const float vz = ux * sp + uz * cp;

    return vec(vx, vy * cr + vz * sr, -vy * sr + vz * cr);
}

/* Feed one sample: Euler rates, plus optional extra non-gravity specific
 * force and an optional forced magnetometer reading. */
static void step(AttitudeFilter &filter,
                 float roll_rate, float pitch_rate, float yaw_rate,
                 bool use_mag,
                 float extra_x = 0.0f, float extra_y = 0.0f,
                 float extra_z = 0.0f,
                 const AttitudeVec3 *mag_override = 0)
{
    const float sr = sinf(s_roll * DEG), cr = cosf(s_roll * DEG);
    const float sp = sinf(s_pitch * DEG), cp = cosf(s_pitch * DEG);

    AttitudeVec3 accel = to_body(0.0f, 0.0f, 1.0f, s_roll, s_pitch, s_yaw);
    accel.x *= s_accel_scale;
    accel.y *= s_accel_scale;
    accel.z *= s_accel_scale;
    accel.x += extra_x;
    accel.y += extra_y;
    accel.z += extra_z;

    /* Euler rates -> body rates, plus the sensor's own bias. */
    const AttitudeVec3 gyro = vec(
        roll_rate - yaw_rate * sp + s_bias_x,
        pitch_rate * cr + yaw_rate * cp * sr + s_bias_y,
        -pitch_rate * sr + yaw_rate * cp * cr + s_bias_z);

    AttitudeVec3 mag = to_body(BH, 0.0f, BZ, s_roll, s_pitch, s_yaw);

    if(mag_override != 0)
        mag = *mag_override;

    const AttitudeVec3 *mag_arg =
        (use_mag || mag_override != 0) ? &mag : 0;

    filter.updateWithDt(accel, gyro, mag_arg, DT);
}

static void hold(AttitudeFilter &filter, float seconds, bool use_mag,
                 float extra_x = 0.0f, float extra_y = 0.0f,
                 float extra_z = 0.0f,
                 const AttitudeVec3 *mag_override = 0)
{
    const int steps = (int)(seconds / DT + 0.5f);

    for(int i = 0; i < steps; i++)
        step(filter, 0.0f, 0.0f, 0.0f, use_mag, extra_x, extra_y, extra_z,
             mag_override);
}

static void ramp(AttitudeFilter &filter, float roll, float pitch, float yaw,
                 float seconds, bool use_mag)
{
    const int steps = (int)(seconds / DT + 0.5f);

    const float roll_step = (roll - s_roll) / steps;
    const float pitch_step = (pitch - s_pitch) / steps;
    const float yaw_step = (yaw - s_yaw) / steps;

    for(int i = 0; i < steps; i++)
    {
        s_roll += roll_step;
        s_pitch += pitch_step;
        s_yaw += yaw_step;

        step(filter, roll_step / DT, pitch_step / DT, yaw_step / DT, use_mag);
    }
}

/* Deterministic pseudo-random noise, so a failure is reproducible. */
static uint32_t s_rng = 12345u;

static float noise(float amplitude)
{
    s_rng = s_rng * 1664525u + 1013904223u;
    const float unit = (float)((s_rng >> 8) & 0xFFFFu) / 32767.5f - 1.0f;
    return unit * amplitude;
}

/* ---- tests ---------------------------------------------------------- */

static void test_static_helpers(void)
{
    printf("\nstatic helpers\n");

    float roll = 0.0f;
    float pitch = 0.0f;

    AttitudeFilter::accelAttitude(vec(0.0f, 0.0f, 1.0f), &roll, &pitch);
    check_near(roll, 0.0f, 0.01f, "accelAttitude, level: roll");
    check_near(pitch, 0.0f, 0.01f, "accelAttitude, level: pitch");

    AttitudeFilter::accelAttitude(to_body(0, 0, 1.0f, 30.0f, -20.0f, 0.0f),
                                  &roll, &pitch);
    check_near(roll, 30.0f, 0.01f, "accelAttitude, 30/-20: roll");
    check_near(pitch, -20.0f, 0.01f, "accelAttitude, 30/-20: pitch");

    check_near(AttitudeFilter::magHeading(to_body(BH, 0, BZ, 0, 0, 137.0f),
                                          0.0f, 0.0f),
               137.0f, 0.1f, "magHeading, level");

    check_near(AttitudeFilter::magHeading(to_body(BH, 0, BZ, 35, -15, 137.0f),
                                          35.0f, -15.0f),
               137.0f, 0.1f, "magHeading, tilt compensated");

    /* The level frame components themselves: horizontal field along the
     * nose and the right wing, vertical field straight down. */
    const AttitudeVec3 level =
        AttitudeFilter::magLevel(to_body(BH, 0, BZ, 35, -15, 137.0f),
                                 35.0f, -15.0f);
    const float h137 = 137.0f * 3.14159265f / 180.0f;

    check_near(level.x, BH * cosf(h137), 0.05f, "magLevel, forward component");
    check_near(level.y, -BH * sinf(h137), 0.05f, "magLevel, right component");
    check_near(level.z, BZ, 0.05f, "magLevel, vertical component");
}

static void test_gyro_accel(void)
{
    printf("\ngyro + accelerometer, no magnetometer\n");

    s_roll = s_pitch = s_yaw = 0.0f;
    s_bias_x = s_bias_y = s_bias_z = 0.0f;

    AttitudeFilter filter;

    check_true(filter.status() == AttitudeStatus::NoData,
               "status is NoData before any sample");

    hold(filter, 0.5f, false);
    check_true(filter.status() == AttitudeStatus::Calibrating,
               "status is Calibrating during the bias measurement");

    hold(filter, 2.0f, false);
    check_true(filter.status() == AttitudeStatus::Fused,
               "status is Fused once calibrated");
    check_near(filter.roll(), 0.0f, 0.2f, "level: roll");
    check_near(filter.pitch(), 0.0f, 0.2f, "level: pitch");
    check_true(!filter.headingValid(), "no heading without a magnetometer");

    ramp(filter, 30.0f, 0.0f, 0.0f, 1.0f, false);
    check_near(filter.roll(), 30.0f, 0.5f, "after a 30 deg/s roll: roll");
    check_near(filter.pitch(), 0.0f, 0.5f, "after a 30 deg/s roll: pitch");

    ramp(filter, 30.0f, 10.0f, 0.0f, 1.0f, false);
    hold(filter, 1.0f, false);
    check_near(filter.roll(), 30.0f, 0.5f, "pitched up while banked: roll");
    check_near(filter.pitch(), 10.0f, 0.5f, "pitched up while banked: pitch");

    /*
     * A sustained 0.5 g forward acceleration. An accelerometer alone
     * would read 26.6 deg of false pitch; the gyro has to hold most of
     * that off.
     */
    ramp(filter, 0.0f, 0.0f, 0.0f, 1.0f, false);
    hold(filter, 2.0f, false);
    hold(filter, 2.0f, false, 0.5f);
    check_true(fabsf(filter.pitch()) < 8.0f,
               "2 s of 0.5 g forward accel: under 8 deg of false pitch");

    /* Whatever the acceleration left behind has to wash out once it
     * stops - the gate rejects a 0.5 g reading outright, so here there
     * is next to nothing to wash out, but a filter that let some in
     * would have to recover within these limits. */
    hold(filter, 4.0f, false);
    check_true(fabsf(filter.pitch()) < 2.0f, "4 s after the accel stops");
    hold(filter, 6.0f, false);
    check_near(filter.pitch(), 0.0f, 0.5f, "10 s after the accel stops");

    /* Accelerometer swamped: the filter must reject it and coast. */
    const float before = filter.roll();
    hold(filter, 1.0f, false, 0.0f, 2.0f, 0.0f);
    check_true(filter.status() == AttitudeStatus::GyroOnly,
               "2 g lateral: status is GyroOnly");
    check_near(filter.roll(), before, 0.5f,
               "2 g lateral: attitude held on the gyro");
}

static void test_gyro_bias(void)
{
    printf("\ngyro bias\n");

    s_roll = s_pitch = s_yaw = 0.0f;
    s_bias_x = 2.5f;
    s_bias_y = -1.5f;
    s_bias_z = 1.0f;

    AttitudeFilter filter;

    hold(filter, 2.0f, false);
    check_true(filter.status() == AttitudeStatus::Fused,
               "calibrated with a bias present");

    const AttitudeVec3 bias = filter.gyroBias();
    check_near(bias.x, 2.5f, 0.1f, "measured bias x");
    check_near(bias.y, -1.5f, 0.1f, "measured bias y");
    check_near(bias.z, 1.0f, 0.1f, "measured bias z");

    hold(filter, 30.0f, false);
    check_near(filter.roll(), 0.0f, 0.2f, "30 s level: roll drift");
    check_near(filter.pitch(), 0.0f, 0.2f, "30 s level: pitch drift");

    ramp(filter, 30.0f, 0.0f, 0.0f, 1.0f, false);
    check_near(filter.roll(), 30.0f, 0.5f, "still tracks a roll");
}

static void test_magnetometer(void)
{
    printf("\nwith a magnetometer\n");

    s_roll = s_pitch = 0.0f;
    s_yaw = 90.0f;
    s_bias_x = s_bias_y = s_bias_z = 0.0f;

    AttitudeFilter filter;

    hold(filter, 2.0f, true);
    check_true(filter.headingValid(), "heading becomes valid");
    check_near(filter.heading(), 90.0f, 1.0f, "level on 090");

    /* The heading must not swing when the unit is tilted. */
    ramp(filter, 30.0f, 10.0f, 90.0f, 1.5f, true);
    hold(filter, 1.0f, true);
    check_near(filter.heading(), 90.0f, 2.0f,
               "same heading at 30 bank / 10 pitch");

    {
        /* An uncompensated heading would be wildly wrong here. */
        const AttitudeVec3 m = to_body(BH, 0.0f, BZ, 30.0f, 10.0f, 90.0f);
        float naive = atan2f(-m.y, m.x) / DEG;

        if(naive < 0.0f)
            naive += 360.0f;

        check_true(fabsf(naive - 90.0f) > 50.0f,
                   "  (an uncompensated heading is off by over 50 deg)");
    }

    ramp(filter, 0.0f, 0.0f, 90.0f, 1.5f, true);
    hold(filter, 1.0f, true);
    ramp(filter, 0.0f, 0.0f, 150.0f, 3.0f, true);
    hold(filter, 1.0f, true);
    check_near(filter.heading(), 150.0f, 2.0f, "after a 20 deg/s turn to 150");

    /*
     * Gravity says nothing about the z gyro offset, so a drift that
     * appears after calibration can only be learned out with a
     * magnetometer fitted.
     */
    s_bias_z = 2.0f;
    hold(filter, 30.0f, true);
    check_near(filter.heading(), 150.0f, 2.0f, "30 s on 150 with the mag");
    check_near(filter.gyroBias().z, 2.0f, 0.5f, "z bias learned from the mag");

    const float heading_before = filter.heading();
    hold(filter, 30.0f, false);
    check_true(fabsf(filter.heading() - heading_before) < 10.0f,
               "30 s with the mag off: under 10 deg of drift (60 uncorrected)");

    /* A disturbed sample must be discarded, not believed. */
    const float heading_steady = filter.heading();
    AttitudeVec3 junk = vec(200.0f, -150.0f, 50.0f);
    hold(filter, 2.0f, false, 0.0f, 0.0f, 0.0f, &junk);
    check_near(filter.heading(), heading_steady, 1.0f,
               "out-of-range mag samples ignored");
}

/*
 * Output jitter with a noisy sensor.
 *
 * This is the "is the horizon steady" test: a display that twitches is
 * unusable even when the average attitude is right. The amplitudes
 * below are far above the QMI8658's own noise - roughly what vibration
 * adds - and the filter still has to hold the horizon to a fraction of
 * a degree.
 */
static void test_noise_rejection(void)
{
    printf("\nsteadiness with a noisy sensor\n");

    s_roll = s_pitch = s_yaw = 0.0f;
    s_bias_x = s_bias_y = s_bias_z = 0.0f;
    s_rng = 12345u;

    AttitudeFilter filter;
    hold(filter, 2.0f, false);

    const float accel_noise = 0.005f;       /* 5 mg */
    const float gyro_noise = 0.5f;          /* 0.5 deg/s */

    float roll_min = 1.0e9f, roll_max = -1.0e9f;
    float pitch_min = 1.0e9f, pitch_max = -1.0e9f;

    /* Settle first, then measure over 10 s. */
    for(int i = 0; i < 1000 + 1000; i++)
    {
        const AttitudeVec3 accel =
            vec(noise(accel_noise), noise(accel_noise),
                1.0f + noise(accel_noise));

        const AttitudeVec3 gyro =
            vec(noise(gyro_noise), noise(gyro_noise), noise(gyro_noise));

        filter.updateWithDt(accel, gyro, 0, DT);

        if(i < 1000)
            continue;

        const AttitudeSample sample = filter.sample();

        if(sample.roll_deg < roll_min) roll_min = sample.roll_deg;
        if(sample.roll_deg > roll_max) roll_max = sample.roll_deg;
        if(sample.pitch_deg < pitch_min) pitch_min = sample.pitch_deg;
        if(sample.pitch_deg > pitch_max) pitch_max = sample.pitch_deg;
    }

    const float roll_span = roll_max - roll_min;
    const float pitch_span = pitch_max - pitch_min;

    printf("  info roll spans %.3f deg, pitch spans %.3f deg over 10 s\n",
           roll_span, pitch_span);

    check_true(roll_span < 0.5f, "roll jitter under 0.5 deg peak to peak");
    check_true(pitch_span < 0.5f, "pitch jitter under 0.5 deg peak to peak");
}

/*
 * Steep pitch. A filter holding Euler angles has to clamp tan(pitch)
 * somewhere below the vertical and degrades as it approaches it; the
 * quaternion form has no such limit, which this checks.
 */
static void test_steep_pitch(void)
{
    printf("\nsteep pitch\n");

    s_roll = s_pitch = s_yaw = 0.0f;
    s_bias_x = s_bias_y = s_bias_z = 0.0f;

    AttitudeFilter filter;
    hold(filter, 2.0f, false);

    ramp(filter, 0.0f, 80.0f, 0.0f, 4.0f, false);
    hold(filter, 1.0f, false);
    check_near(filter.pitch(), 80.0f, 1.0f, "pitched up to 80 deg");
    check_near(filter.roll(), 0.0f, 1.0f, "roll stays level at 80 deg pitch");

    /* And rolling while steeply pitched must not throw it. */
    ramp(filter, 40.0f, 80.0f, 0.0f, 2.0f, false);
    hold(filter, 1.0f, false);
    check_near(filter.pitch(), 80.0f, 1.5f, "40 deg bank at 80 deg pitch: pitch");
    check_near(filter.roll(), 40.0f, 1.5f, "40 deg bank at 80 deg pitch: roll");

    ramp(filter, 0.0f, 0.0f, 0.0f, 4.0f, false);
    hold(filter, 2.0f, false);
    check_near(filter.pitch(), 0.0f, 0.5f, "back to level: pitch");
    check_near(filter.roll(), 0.0f, 0.5f, "back to level: roll");
}

/*
 * A real accelerometer is not exactly 1 g.
 *
 * The board this was written for reads 0.95 g sitting still. Judged
 * against a nominal 1 g that spends a quarter of the trust window before
 * anything has even moved, and - because the calibration gate used the
 * same test - calibration could never complete, so the gyro was never
 * integrated and its 3.6 deg/s of bias was never removed. The reference
 * is measured at rest instead, which this pins down.
 */
static void test_miscalibrated_accelerometer(void)
{
    printf("\naccelerometer reading 5%% low, as the real one does\n");

    s_roll = s_pitch = s_yaw = 0.0f;
    s_bias_x = 3.6f;
    s_bias_y = 1.1f;
    s_bias_z = -0.3f;
    s_accel_scale = 0.95f;

    AttitudeFilter filter;

    hold(filter, 3.0f, false);

    check_true(filter.status() == AttitudeStatus::Fused,
               "calibration completes despite the scale error");
    check_true(filter.sample().accel_trust > 0.95f,
               "accelerometer is fully trusted at rest");

    const AttitudeVec3 bias = filter.gyroBias();
    check_near(bias.x, 3.6f, 0.1f, "measured bias x");
    check_near(bias.y, 1.1f, 0.1f, "measured bias y");

    check_near(filter.roll(), 0.0f, 0.3f, "level: roll");
    check_near(filter.pitch(), 0.0f, 0.3f, "level: pitch");

    /* And the bias must stay removed, not reappear as drift. */
    hold(filter, 30.0f, false);
    check_near(filter.roll(), 0.0f, 0.5f, "30 s later: roll");
    check_near(filter.pitch(), 0.0f, 0.5f, "30 s later: pitch");

    /* A real disturbance must still be rejected with the shifted
     * reference - the window has moved, not widened. */
    hold(filter, 1.0f, false, 0.0f, 2.0f, 0.0f);
    check_true(filter.status() == AttitudeStatus::GyroOnly,
               "2 g lateral still rejected");

    s_accel_scale = 1.0f;
    s_bias_x = s_bias_y = s_bias_z = 0.0f;
}

static void test_reset(void)
{
    printf("\nreset\n");

    s_roll = s_pitch = s_yaw = 0.0f;
    s_bias_x = s_bias_y = s_bias_z = 0.0f;

    AttitudeFilter filter;
    hold(filter, 2.0f, false);
    check_true(filter.status() == AttitudeStatus::Fused, "fused before reset");

    filter.reset();
    check_true(filter.status() == AttitudeStatus::NoData,
               "status is NoData after reset");

    hold(filter, 0.5f, false);
    check_true(filter.status() == AttitudeStatus::Calibrating,
               "calibration restarts after reset");
}

/*
 * A coordinated turn: the case a gravity-referenced filter gets wrong.
 * Banked at 30 degrees in a level turn, the specific force stays along
 * body z at 1/cos(30) = 1.155 g, so the accelerometer says "level"
 * while the aircraft is banked. A filter that believes it slowly rolls
 * the horizon level in the turn; this one has to recognise the reading
 * as not gravity and hold the bank on the gyro.
 */
static void test_coordinated_turn(void)
{
    printf("\nsustained coordinated turn, at an assumed 30 m/s\n");

    s_roll = s_pitch = s_yaw = 0.0f;
    s_bias_x = s_bias_y = s_bias_z = 0.0f;
    s_rng = 777u;

    /* An aircraft: the turn handling is off by default. */
    AttitudeConfig flying;
    flying.accel_turn_speed_ms = 30.0f;

    AttitudeFilter filter(flying);
    hold(filter, 2.0f, false);

    /* Roll in over 3 s, then hold 30 degrees of bank and turn at the
     * rate that keeps it coordinated: tan(30) g / v at 50 m/s. */
    const float bank = 30.0f;
    const float yaw_rate = tanf(bank * DEG) * 9.80665f / 50.0f / DEG;
    const int roll_steps = 300;

    for(int i = 0; i < roll_steps; i++)
    {
        s_roll += bank / roll_steps;
        step(filter, bank / roll_steps / DT, 0.0f, 0.0f, false);
    }

    float worst = 0.0f;

    for(int i = 0; i < 6000; i++)          /* one minute */
    {
        s_yaw += yaw_rate * DT;

        /* What the accelerometer reads in a coordinated turn: 1/cos(bank)
         * g straight down the body z axis, nothing across it. */
        const float sr = sinf(s_roll * DEG), cr = cosf(s_roll * DEG);
        const AttitudeVec3 accel = vec(noise(0.005f), noise(0.005f),
                                       1.0f / cr + noise(0.005f));
        const AttitudeVec3 gyro = vec(noise(0.3f),
                                      yaw_rate * sr + noise(0.3f),
                                      yaw_rate * cr + noise(0.3f));

        filter.updateWithDt(accel, gyro, 0, DT);

        const float error = fabsf(filter.roll() - bank);
        if(error > worst)
            worst = error;
    }

    printf("  info worst bank error over the minute %.2f deg\n", worst);
    check_true(worst < 3.0f, "a minute at 30 deg of bank: within 3 deg");
    check_true(filter.status() == AttitudeStatus::GyroOnly,
               "the turn's specific force is recognised, not believed");

    /* Roll out: back to level, and the accelerometer is believed again. */
    for(int i = 0; i < roll_steps; i++)
    {
        s_roll -= bank / roll_steps;
        step(filter, -bank / roll_steps / DT, 0.0f, 0.0f, false);
    }

    hold(filter, 3.0f, false);
    check_near(filter.roll(), 0.0f, 0.5f, "rolled out: level again");
    check_true(filter.status() == AttitudeStatus::Fused, "and fused again");
}

/*
 * Re-acquisition. A gyro that saturates, or a burst of lost samples,
 * leaves the estimate far from the truth with a covariance that still
 * claims it is right - so every accelerometer sample afterwards fails
 * the gate. The filter has to notice that the accelerometer looks like
 * gravity and keeps disagreeing, and let itself be corrected.
 */
static void test_reacquire(void)
{
    printf("\nre-acquisition after a gyro upset\n");

    s_roll = s_pitch = s_yaw = 0.0f;
    s_bias_x = s_bias_y = s_bias_z = 0.0f;

    AttitudeFilter filter;
    hold(filter, 2.0f, false);

    /* Half a second of a gyro reading 80 deg/s of roll that is not
     * happening: the estimate rolls 40 degrees while the unit sits
     * still. */
    for(int i = 0; i < 50; i++)
        filter.updateWithDt(vec(0.0f, 0.0f, 1.0f), vec(80.0f, 0.0f, 0.0f),
                            0, DT);

    check_true(fabsf(filter.roll()) > 30.0f, "  (the upset took hold)");

    hold(filter, 5.0f, false);
    check_near(filter.roll(), 0.0f, 1.0f, "5 s later: back on the horizon");
    check_true(filter.status() == AttitudeStatus::Fused, "and fused");
}

/*
 * A field that is plausible in strength but points the wrong way - a
 * steel structure, a speaker magnet - must neither turn the heading
 * nor, above all, tilt the horizon.
 */
static void test_mag_disturbance(void)
{
    printf("\nmagnetic disturbance\n");

    s_roll = 20.0f;
    s_pitch = 5.0f;
    s_yaw = 40.0f;
    s_bias_x = s_bias_y = s_bias_z = 0.0f;

    AttitudeFilter filter;
    hold(filter, 3.0f, true);
    check_near(filter.heading(), 40.0f, 1.0f, "on 040 to begin with");

    const float roll_before = filter.roll();
    const float pitch_before = filter.pitch();

    /* The same field strength, rotated 70 degrees. */
    const AttitudeVec3 wrong = to_body(BH, 0.0f, BZ, s_roll, s_pitch,
                                       s_yaw - 70.0f);
    hold(filter, 1.0f, false, 0.0f, 0.0f, 0.0f, &wrong);

    check_near(filter.heading(), 40.0f, 2.0f,
               "a second of a 70 deg wrong field: heading held");
    check_near(filter.roll(), roll_before, 0.1f, "and the roll untouched");
    check_near(filter.pitch(), pitch_before, 0.1f, "and the pitch untouched");

    /* A lasting change - the unit moved to where the field really is
     * different - is accepted after a few seconds rather than fought. */
    hold(filter, 6.0f, false, 0.0f, 0.0f, 0.0f, &wrong);
    check_near(filter.heading(), 330.0f, 2.0f,
               "a field that stays different is adopted");
    check_near(filter.roll(), roll_before, 0.3f, "still without tilting");
}

/*
 * The covariance has to mean something: small where the estimate is
 * well observed, growing where it is not.
 */
static void test_uncertainty(void)
{
    printf("\nuncertainty\n");

    s_roll = s_pitch = s_yaw = 0.0f;
    s_bias_x = s_bias_y = s_bias_z = 0.0f;

    AttitudeFilter filter;
    hold(filter, 5.0f, true);

    const AttitudeSample settled = filter.sample();
    check_true(settled.tilt_sigma_deg < 1.0f, "tilt sigma under 1 deg at rest");
    check_true(settled.heading_sigma_deg < 3.0f,
               "heading sigma under 3 deg with the magnetometer");

    /* Ten minutes without the magnetometer: heading is now a guess. */
    hold(filter, 600.0f, false);
    const AttitudeSample coasted = filter.sample();
    printf("  info heading sigma %.1f deg after 10 min without the mag\n",
           coasted.heading_sigma_deg);
    check_true(coasted.heading_sigma_deg > settled.heading_sigma_deg * 3.0f,
               "heading sigma grows without the magnetometer");
    check_true(coasted.tilt_sigma_deg < 1.0f,
               "tilt sigma stays small: gravity still observes it");
}

/*
 * The first heading measurement turns the estimate as far as it needs
 * to - up to half way round - in one step, and the covariance has to
 * agree with what was done. A first order correction falls short of a
 * large turn (asked for 180, it does 115) while the covariance claims
 * the full turn, after which every measurement fails the gate.
 */
static void test_heading_acquisition(void)
{
    printf("\nheading acquisition, half way round, upside down\n");

    s_roll = 157.0f;
    s_pitch = -38.0f;
    s_yaw = 180.0f;
    s_bias_x = s_bias_y = s_bias_z = 0.0f;

    AttitudeFilter filter;

    /* Calibration takes 1.5 s; then one magnetometer sample. */
    hold(filter, 1.5f, false);
    hold(filter, 0.02f, true);

    check_true(filter.headingValid(), "heading valid after the first sample");
    check_near(filter.heading(), 180.0f, 1.0f, "and right at once");

    hold(filter, 5.0f, true);
    check_near(filter.heading(), 180.0f, 1.0f, "and stays right");
    check_near(filter.roll(), 157.0f, 0.5f, "roll, upside down");
    check_near(filter.pitch(), -38.0f, 0.5f, "pitch, upside down");
}

/*
 * After a turn, gyro and magnetometer disagree about how far it went:
 * the gyro's scale is a percent or two out, and a real installation's
 * calibration leaves a heading error that changes with heading. The
 * heading must settle on the magnetometer promptly when the turn ends -
 * not hold the gyro's answer for seconds and then jump, which is what
 * a filter does that thinks rotation costs it no accuracy.
 */
static void test_heading_after_turn(void)
{
    printf("\nheading after a turn, gyro and magnetometer disagreeing\n");

    s_roll = s_pitch = 0.0f;
    s_bias_x = s_bias_y = s_bias_z = 0.0f;

    AttitudeFilter filter;

    float yaw = 0.0f;
    const float gyro_scale = 1.02f;

    /* The heading the magnetometer reports: 15 deg out at 090. */
    auto measured_mag = [](float true_yaw) {
        const float wrong = true_yaw + 15.0f * sinf(true_yaw * DEG);
        return to_body(BH, 0.0f, BZ, 0.0f, 0.0f, wrong);
    };

    auto run = [&](float seconds, float rate) {
        const int steps = (int)(seconds / DT + 0.5f);

        for(int i = 0; i < steps; i++)
        {
            yaw += rate * DT;

            const AttitudeVec3 mag = measured_mag(yaw);
            filter.updateWithDt(vec(0.0f, 0.0f, 1.0f),
                                vec(0.0f, 0.0f, rate * gyro_scale),
                                (i % 2 == 0) ? &mag : 0, DT);
        }
    };

    run(4.0f, 0.0f);
    check_near(filter.heading(), 0.0f, 1.0f, "on 000");

    run(1.0f, 90.0f);                   /* a brisk quarter turn */

    run(0.5f, 0.0f);
    const float half_second = filter.heading();
    run(0.5f, 0.0f);
    const float one_second = filter.heading();
    run(3.0f, 0.0f);
    const float four_seconds = filter.heading();

    printf("  info after the turn: %.1f at 0.5 s, %.1f at 1 s, %.1f at 4 s "
           "(magnetometer says 105, gyro 91.8)\n",
           half_second, one_second, four_seconds);

    check_near(half_second, 105.0f, 2.0f,
               "on the magnetometer's heading within half a second");
    check_near(four_seconds - one_second, 0.0f, 1.0f,
               "and no change after that");
}

/*
 * The default: speed 0, a unit in the hand. Turned about, its
 * accelerometer still reads gravity alone, and the filter has to keep
 * using it - not fall back on the gyro whenever the unit moves.
 */
static void test_moving_by_hand(void)
{
    printf("\nturned about by hand, speed 0\n");

    s_roll = s_pitch = s_yaw = 0.0f;
    s_bias_x = s_bias_y = s_bias_z = 0.0f;

    AttitudeFilter filter;
    hold(filter, 2.0f, false);

    int gyro_only = 0;
    int samples = 0;

    /* Twenty seconds of rocking in pitch and roll and turning, never
     * still: up to 20 deg/s about every axis. */
    for(int i = 0; i < 2000; i++)
    {
        const float t = i * DT;
        const float roll_rate = 20.0f * cosf(t * 0.9f);
        const float pitch_rate = 15.0f * cosf(t * 1.3f + 1.0f);
        const float yaw_rate = 20.0f * cosf(t * 0.5f + 2.0f);

        s_roll += roll_rate * DT;
        s_pitch += pitch_rate * DT;
        s_yaw += yaw_rate * DT;

        step(filter, roll_rate, pitch_rate, yaw_rate, false);

        samples++;
        if(filter.status() == AttitudeStatus::GyroOnly)
            gyro_only++;
    }

    printf("  info gyro only on %d of %d samples\n", gyro_only, samples);
    check_true(gyro_only == 0, "the accelerometer is used throughout");
    check_near(filter.roll(), s_roll, 1.0f, "roll follows");
    check_near(filter.pitch(), s_pitch, 1.0f, "pitch follows");
}

int main(void)
{
    test_static_helpers();
    test_gyro_accel();
    test_gyro_bias();
    test_magnetometer();
    test_miscalibrated_accelerometer();
    test_noise_rejection();
    test_steep_pitch();
    test_reset();
    test_coordinated_turn();
    test_reacquire();
    test_mag_disturbance();
    test_uncertainty();
    test_heading_acquisition();
    test_heading_after_turn();
    test_moving_by_hand();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return (g_failures == 0) ? 0 : 1;
}
