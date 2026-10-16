/*
 * Navigator: the whole chain, from simulated sensors to every estimate
 * the screens show.
 *
 * Build and run on the host - see CMakeLists.txt in this directory.
 */

#include "navigator.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const float DEG = (float)M_PI / 180.0f;
static const float DT = 0.01f;
static const float G = 9.80665f;

/* Earth field, NED, roughly central Europe. */
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
        g_failures++;

    printf("  %s %-52s %+8.3f", ok ? "ok  " : "FAIL", what, actual);

    if(!ok)
        printf(", expected %+.3f +- %.3f", expected, tolerance);

    printf("\n");
}

static void check_true(bool condition, const char *what)
{
    g_checks++;

    if(!condition)
        g_failures++;

    printf("  %s %s\n", condition ? "ok  " : "FAIL", what);
}

static uint32_t s_rng = 4242u;

static float noise(float amplitude)
{
    s_rng = s_rng * 1664525u + 1013904223u;
    const float unit = (float)((s_rng >> 8) & 0xFFFFu) / 32767.5f - 1.0f;
    return unit * amplitude;
}

static AttitudeVec3 vec(float x, float y, float z)
{
    AttitudeVec3 v;
    v.x = x; v.y = y; v.z = z;
    return v;
}

/* NED vector into body axes for roll, pitch and yaw in degrees. */
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

/* Simulated aircraft, level flight only: bank, heading, altitude. */
static float s_roll = 0.0f;
static float s_yaw = 0.0f;
static float s_altitude = 250.0f;
static int s_step = 0;

/* True airspeed of the simulated aircraft. */
static const float SPEED_MS = 50.0f;

/*
 * One step of coordinated flight at a given roll rate: the turn rate is
 * whatever the bank makes it, g tan(bank) / V, and the specific force
 * stays straight down body z at 1/cos(bank) g, as in any coordinated
 * turn. Magnetometer every other step and barometer every fourth, as
 * the firmware reads them.
 */
static void fly(Navigator &navigator, float roll_rate, float climb_ms)
{
    s_roll += roll_rate * DT;

    const float yaw_rate = G * tanf(s_roll * DEG) / SPEED_MS / DEG;

    s_yaw += yaw_rate * DT;
    s_altitude += climb_ms * DT;
    s_step++;

    const float sr = sinf(s_roll * DEG), cr = cosf(s_roll * DEG);

    const AttitudeVec3 accel = vec(noise(0.005f), noise(0.005f),
                                   1.0f / cr + noise(0.005f));
    const AttitudeVec3 gyro = vec(roll_rate + noise(0.3f),
                                  yaw_rate * sr + noise(0.3f),
                                  yaw_rate * cr + noise(0.3f));

    const AttitudeVec3 mag = to_body(BH, 0.0f, BZ, s_roll, 0.0f, s_yaw);
    const float baro = s_altitude + noise(0.18f);

    navigator.update(accel, gyro, (s_step % 2 == 0) ? &mag : 0,
                     (s_step % 4 == 0) ? &baro : 0, DT);
}

static void fly_for(Navigator &navigator, float seconds, float roll_rate,
                    float climb_ms = 0.0f)
{
    const int steps = (int)(seconds / DT + 0.5f);

    for(int i = 0; i < steps; i++)
        fly(navigator, roll_rate, climb_ms);
}

static float wrap180(float angle)
{
    while(angle > 180.0f) angle -= 360.0f;
    while(angle < -180.0f) angle += 360.0f;
    return angle;
}

static void test_turn(void)
{
    printf("\nstandard rate turn, level, at an assumed 30 m/s\n");

    Navigator navigator;

    /* An aircraft: the turn handling is off by default. */
    AttitudeConfig flying;
    flying.accel_turn_speed_ms = 30.0f;
    navigator.attitudeFilter().setConfig(flying);

    fly_for(navigator, 5.0f, 0.0f);

    NavigationSample start = navigator.sample();
    check_true(start.attitude.status == AttitudeStatus::Fused, "fused");
    check_true(start.attitude.heading_valid, "heading valid");
    check_true(start.vertical.valid, "altitude valid");
    check_near(wrap180(start.attitude.heading_deg), 0.0f, 1.0f, "heading 000");
    check_near(start.vertical.altitude_m, s_altitude, 0.5f, "altitude");

    /* Standard rate, 3 deg/s, at 50 m/s needs 15 degrees of bank. */
    const float bank = atanf(3.0f * DEG * SPEED_MS / G) / DEG;

    fly_for(navigator, 2.0f, bank / 2.0f);

    float worst_climb = 0.0f;
    float worst_bank = 0.0f;

    for(int i = 0; i < 6000; i++)           /* a minute: 180 degrees */
    {
        fly(navigator, 0.0f, 0.0f);

        const NavigationSample now = navigator.sample();

        worst_climb = fmaxf(worst_climb,
                            fabsf(now.vertical.vertical_speed_ms));
        worst_bank = fmaxf(worst_bank,
                           fabsf(now.attitude.roll_deg - bank));
    }

    const NavigationSample turning = navigator.sample();

    printf("  info bank %.1f deg; worst bank error %.2f deg, worst vertical "
           "speed %.2f m/s\n", bank, worst_bank, worst_climb);

    /*
     * The limits are physical. Rolling in, the first two degrees of bank
     * come before the turn is fast enough to be recognised, and the
     * accelerometer - reading "level", as it does in any coordinated
     * turn - is believed for them: about a degree of bank error, which
     * nothing can correct without airspeed until the turn ends. Tilt
     * compensation then turns that into tan(dip) times as much heading
     * error, 2.2 degrees here.
     */
    check_near(turning.turn.turn_rate_dps, 3.0f, 0.15f, "turn rate");
    check_near(turning.turn.slip_deg, 0.0f, 0.5f, "ball centred");
    check_true(worst_bank < 1.5f, "bank held to within 1.5 deg");
    check_near(wrap180(turning.attitude.heading_deg - s_yaw), 0.0f, 3.0f,
               "heading follows the turn");
    check_true(worst_climb < 0.3f,
               "level turn: vertical speed under 0.3 m/s throughout");
    check_near(turning.vertical.altitude_m, s_altitude, 1.0f,
               "altitude held");

    /* Roll out and climb. */
    fly_for(navigator, 2.0f, -bank / 2.0f);
    fly_for(navigator, 20.0f, 0.0f, 3.0f);

    const NavigationSample climbing = navigator.sample();
    check_near(climbing.vertical.vertical_speed_ms, 3.0f, 0.3f,
               "climbing at 3 m/s");
    check_near(climbing.turn.turn_rate_dps, 0.0f, 0.15f, "wings level");
    check_near(climbing.attitude.roll_deg, 0.0f, 0.5f, "roll level");
}

static void test_startup(void)
{
    printf("\nstart-up\n");

    Navigator navigator;

    check_true(navigator.sample().attitude.status == AttitudeStatus::NoData,
               "no data before the first sample");

    s_roll = 0.0f; s_yaw = 0.0f; s_altitude = 250.0f; s_step = 0;
    fly_for(navigator, 0.5f, 0.0f);

    const NavigationSample early = navigator.sample();
    check_true(early.attitude.status == AttitudeStatus::Calibrating,
               "calibrating at first");
    check_true(!early.turn.valid,
               "no turn rate from an uncalibrated gyro");
}

int main(void)
{
    test_startup();

    s_roll = 0.0f; s_yaw = 0.0f; s_altitude = 250.0f; s_step = 0;
    test_turn();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return (g_failures == 0) ? 0 : 1;
}
