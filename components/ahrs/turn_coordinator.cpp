#include "turn_coordinator.h"

#include <math.h>

static const float DEG_PER_RAD = 57.29577951f;

static float clampf(float value, float low, float high)
{
    return (value < low) ? low : (value > high) ? high : value;
}

TurnCoordinator::TurnCoordinator()
{
    reset();
}

TurnCoordinator::TurnCoordinator(const TurnCoordinatorConfig &config)
    : m_config(config)
{
    reset();
}

void TurnCoordinator::reset()
{
    m_state = TurnSample();
    m_published.publish(m_state);
}

float TurnCoordinator::turnRate(const AttitudeVec3 &gyro_dps,
                                float roll_deg,
                                float pitch_deg)
{
    const float roll = roll_deg / DEG_PER_RAD;
    const float pitch = pitch_deg / DEG_PER_RAD;

    return (gyro_dps.y * sinf(roll) + gyro_dps.z * cosf(roll)) / cosf(pitch);
}

float TurnCoordinator::slipAngle(const AttitudeVec3 &accel_g)
{
    /* Measured from body down, so it is the ball's place in a tube that
     * is level with the wings. Upside down the ball would fall out of
     * the tube; it is pinned to the end instead, by the caller's limit. */
    return atan2f(accel_g.y, accel_g.z) * DEG_PER_RAD;
}

void TurnCoordinator::update(const AttitudeVec3 &accel_g,
                             const AttitudeVec3 &gyro_dps,
                             float roll_deg,
                             float pitch_deg,
                             float dt_s)
{
    if(dt_s <= 0.0f)
        return;

    const float pitch = clampf(pitch_deg, -m_config.max_pitch_deg,
                               m_config.max_pitch_deg);

    const float rate = turnRate(gyro_dps, roll_deg, pitch);
    const float slip = slipAngle(accel_g);

    if(!m_state.valid)
    {
        /* Start where the instrument is, not swinging in from zero. */
        m_state.turn_rate_dps = rate;
        m_state.slip_deg = slip;
        m_state.valid = true;
    }
    else
    {
        const float rate_alpha =
            dt_s / (m_config.rate_time_constant_s + dt_s);
        const float ball_alpha =
            dt_s / (m_config.ball_time_constant_s + dt_s);

        m_state.turn_rate_dps += rate_alpha * (rate - m_state.turn_rate_dps);
        m_state.slip_deg += ball_alpha * (slip - m_state.slip_deg);
    }

    m_published.publish(m_state);
}
