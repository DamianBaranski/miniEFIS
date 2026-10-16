#pragma once

#include "attitude_filter.h"
#include "published_snapshot.h"

/*
 * TurnCoordinator - what a turn coordinator shows: rate of turn, and the
 * slip ball.
 *
 * Rate of turn is how fast the heading is changing, in deg/s, positive
 * to the right - the earth frame yaw rate, taken from the body rates and
 * the current attitude:
 *
 *   turn rate = (q sin(roll) + r cos(roll)) / cos(pitch)
 *
 * So it reads the true turn whatever the bank, and nothing for a roll
 * on its own. A mechanical turn coordinator's gyro is canted to show
 * roll rate as well, as an early cue; this one does not, since the
 * attitude is right there on the horizon.
 *
 * The ball is the inclinometer: it settles where the apparent gravity
 * points across the aircraft. In a coordinated turn the specific force
 * stays in the aircraft's plane of symmetry and the ball is centred;
 * slipping or skidding moves it to one side. Its position is that
 * apparent gravity's angle from body down, as a ball in a circular tube
 * would sit - positive to the right.
 *
 * Both are smoothed by a first order lag, like the damped needle and
 * fluid filled tube of the real instrument. Without that the needle
 * shows every bump and the ball shows engine vibration.
 *
 * Inputs are in the body axes AttitudeFilter uses (x forward, y right,
 * z down), with the accelerometer reading +1 g down at rest and the
 * gyro already corrected for its bias. Portable: no ESP-IDF dependency.
 *
 * update() from one task; sample() from any.
 */

struct TurnCoordinatorConfig
{
    /* Time constants of the display lag, seconds. */
    float rate_time_constant_s = 0.5f;
    float ball_time_constant_s = 0.3f;

    /* Beyond this pitch the turn rate formula divides by nearly zero,
     * and the instrument has nothing useful to say anyway. */
    float max_pitch_deg = 80.0f;
};

struct TurnSample
{
    float turn_rate_dps = 0.0f;     /* positive turning right */
    float slip_deg = 0.0f;          /* ball, positive to the right */
    bool valid = false;             /* false until the first update */
};

/* Standard rate: 360 degrees in two minutes. */
static const float TURN_STANDARD_RATE_DPS = 3.0f;

class TurnCoordinator
{
public:
    TurnCoordinator();
    explicit TurnCoordinator(const TurnCoordinatorConfig &config);

    void reset();

    void update(const AttitudeVec3 &accel_g,
                const AttitudeVec3 &gyro_dps,
                float roll_deg,
                float pitch_deg,
                float dt_s);

    TurnSample sample() const { return m_published.read(); }

    /* Unfiltered values for one sample - for testing, and for anyone
     * who wants their own smoothing. */
    static float turnRate(const AttitudeVec3 &gyro_dps,
                          float roll_deg,
                          float pitch_deg);
    static float slipAngle(const AttitudeVec3 &accel_g);

private:
    TurnCoordinatorConfig m_config;
    TurnSample m_state;
    PublishedSnapshot<TurnSample> m_published;
};
