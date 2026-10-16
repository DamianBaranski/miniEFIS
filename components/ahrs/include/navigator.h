#pragma once

#include "attitude_filter.h"
#include "published_snapshot.h"
#include "turn_coordinator.h"
#include "vertical_channel.h"

/*
 * Navigator - every estimate the instrument shows, from one set of
 * sensor samples, in one snapshot.
 *
 * Each IMU sample goes through, in order:
 *
 *   AttitudeFilter    the Kalman filter: attitude, heading, gyro bias
 *   TurnCoordinator   rate of turn and slip, from the bias corrected
 *                     gyro and that attitude
 *   VerticalChannel   the Kalman filter for altitude and vertical speed,
 *                     driven by the vertical acceleration the attitude
 *                     gives, corrected by the barometer
 *
 * and the results are published together. A screen that shows the
 * heading and the turn rate, or the attitude and the altitude, gets
 * them from the same instant - no screen keeps estimates of its own.
 *
 * Portable, like the rest of this component: no ESP-IDF dependency
 * beyond the clock. update() from one task; sample() from any.
 */

struct NavigationSample
{
    AttitudeSample attitude;

    /* Body rates with the estimated gyro bias removed, deg/s. */
    AttitudeVec3 rate_dps = { 0.0f, 0.0f, 0.0f };
    AttitudeVec3 gyro_bias_dps = { 0.0f, 0.0f, 0.0f };

    TurnSample turn;

    /* Valid once a barometer has reported. */
    VerticalChannelSample vertical;
};

class Navigator
{
public:
    Navigator() {}

    /*
     * One IMU sample, in body axes (attitude_filter.h): accelerometer in
     * g, gyro in deg/s. mag_uT is a fresh, calibrated magnetometer
     * sample in body axes, or NULL; altitude_m a fresh barometric
     * altitude, or NULL. Each of those must be passed once only - the
     * filters count every one as a new measurement.
     */
    void update(const AttitudeVec3 &accel_g,
                const AttitudeVec3 &gyro_dps,
                const AttitudeVec3 *mag_uT,
                const float *altitude_m,
                float dt_s);

    /* A changed altimeter setting: move the altitude, not the rest.
     * From the updating task. */
    void shiftAltitude(float delta_m) { m_vertical.shiftAltitude(delta_m); }

    /* Re-measure the gyro bias; the unit must be held still. Any task. */
    void requestGyroCalibration() { m_attitude.requestGyroCalibration(); }

    /* One consistent snapshot of everything. Safe from any task. */
    NavigationSample sample() const;

    /* For tuning (setConfig); not for reading - use sample(). */
    AttitudeFilter &attitudeFilter() { return m_attitude; }
    VerticalChannel &verticalChannel() { return m_vertical; }

private:
    AttitudeFilter m_attitude;
    TurnCoordinator m_turn;
    VerticalChannel m_vertical;

    PublishedSnapshot<NavigationSample> m_published;
};
