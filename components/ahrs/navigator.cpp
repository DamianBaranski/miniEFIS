#include "navigator.h"

void Navigator::update(const AttitudeVec3 &accel_g,
                       const AttitudeVec3 &gyro_dps,
                       const AttitudeVec3 *mag_uT,
                       const float *altitude_m,
                       float dt_s)
{
    m_attitude.updateWithDt(accel_g, gyro_dps, mag_uT, dt_s);

    NavigationSample out;

    out.attitude = m_attitude.sample();
    out.gyro_bias_dps = m_attitude.gyroBias();
    out.rate_dps.x = gyro_dps.x - out.gyro_bias_dps.x;
    out.rate_dps.y = gyro_dps.y - out.gyro_bias_dps.y;
    out.rate_dps.z = gyro_dps.z - out.gyro_bias_dps.z;

    /* The turn coordinator only means anything once the bias is known:
     * before that the rates carry several deg/s of offset. */
    if(out.attitude.status != AttitudeStatus::Calibrating &&
       out.attitude.status != AttitudeStatus::NoData)
    {
        m_turn.update(accel_g, out.rate_dps, out.attitude.roll_deg,
                      out.attitude.pitch_deg, dt_s);
    }

    m_vertical.update(m_attitude.verticalAcceleration(accel_g), altitude_m,
                      dt_s);

    out.turn = m_turn.sample();
    out.vertical = m_vertical.sample();

    m_published.publish(out);
}

NavigationSample Navigator::sample() const
{
    NavigationSample out = m_published.read();

    /* The attitude's own snapshot knows its age, and whether it has gone
     * stale - the IMU stopping must show on every screen. */
    const AttitudeSample live = m_attitude.sample();

    out.attitude.age_ms = live.age_ms;

    if(live.status == AttitudeStatus::NoData)
        out.attitude.status = AttitudeStatus::NoData;

    return out;
}
