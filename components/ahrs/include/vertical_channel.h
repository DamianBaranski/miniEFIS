#pragma once

#include "published_snapshot.h"
#include <stdint.h>

/*
 * VerticalChannel - altitude and vertical speed from a barometer and the
 * accelerometer: a three state Kalman filter.
 *
 * A barometer alone cannot give vertical speed. Differentiating it means
 * differentiating its noise: a BMP280 settles to about 0.1 m of noise,
 * so a naive difference over a 40 ms sample is +-2.5 m/s of nonsense.
 * Smoothing enough to hide that costs seconds of lag.
 *
 * So the accelerometer provides the rate and the barometer corrects it.
 * The state is altitude, vertical speed and the vertical accelerometer's
 * bias; the vertical acceleration drives the prediction, every
 * barometer sample is a measurement of altitude, and the covariance
 * decides how far each one moves each state - so the bias is learned
 * from how the altitude keeps disagreeing, without a hand-set gain.
 *
 * The bias state is what makes it work in practice. Vertical
 * acceleration is derived from the attitude estimate, so any attitude
 * error tilts gravity into it and the integration would run away - and
 * in a sustained turn the attitude error is systematic, not random.
 *
 * Barometer readings are gated like the attitude filter's measurements:
 * one the covariance says is impossible is rejected. A step too large to
 * be flight - a changed reference pressure, a glitch that persists - is
 * taken as a new starting point rather than integrated, so it does not
 * throw a false vertical speed while it is worked off.
 *
 * Units are SI throughout: metres, m/s, m/s^2, upward positive.
 *
 * Threading: update() from one task; every accessor is a lock-free
 * snapshot safe from any task.
 */

struct VerticalChannelSample
{
    float altitude_m = 0.0f;
    float vertical_speed_ms = 0.0f;

    /* Estimated vertical accelerometer error, m/s^2. Large values mean
     * the attitude estimate is off, or the sensor needs calibrating. */
    float accel_bias_ms2 = 0.0f;

    /* One standard deviation, from the covariance. */
    float altitude_sigma_m = 0.0f;
    float vertical_speed_sigma_ms = 0.0f;

    /* False until the first barometer reading arrives. */
    bool valid = false;
};

/* The filter's model of the sensors. */
struct VerticalChannelConfig
{
    /*
     * Vertical acceleration noise, m/s^2 per root hertz: the sensor, the
     * vibration, and what the attitude error adds. It sets how much the
     * prediction is trusted against the barometer, and so the trade
     * between vertical speed lag and vertical speed noise.
     */
    float accel_noise_ms2 = 0.05f;

    /* How fast the accelerometer bias may wander, m/s^2 per root s. */
    float accel_bias_walk_ms2 = 0.01f;

    /* Barometric altitude noise, m: the sensor and the air moving. */
    float baro_noise_m = 0.15f;

    /* Squared Mahalanobis distance beyond which a barometer reading is
     * rejected, and how long rejections may go on before the barometer
     * is taken as right and the filter restarted from it. */
    float baro_gate = 25.0f;
    float baro_reacquire_s = 1.0f;

    /* Clamp on the bias state, so a spell of bad data cannot park it
     * somewhere absurd. */
    float accel_bias_limit_ms2 = 4.0f;

    /*
     * A barometer step larger than this is treated as a new starting
     * point at once - a changed reference pressure, or a sensor glitch.
     */
    float altitude_jump_m = 50.0f;

    float dt_min_s = 0.0005f;
    float dt_max_s = 0.1f;
};

class VerticalChannel
{
public:
    VerticalChannel();
    explicit VerticalChannel(const VerticalChannelConfig &config);

    void setConfig(const VerticalChannelConfig &config);
    VerticalChannelConfig config() const { return m_config; }

    void reset();

    /*
     * One step.
     *
     * acceleration_up_ms2 is the vertical acceleration in earth axes
     * with gravity already removed - AttitudeFilter::verticalAcceleration()
     * produces it. Each barometer reading must be passed once only. altitude_m is a fresh barometric altitude, or NULL on
     * the steps between barometer samples, which are slower than the
     * IMU.
     */
    void update(float acceleration_up_ms2,
                const float *altitude_m,
                float dt_s);

    /*
     * Move the altitude by this much at once, leaving vertical speed and
     * bias alone - for a changed reference pressure, which moves every
     * barometric altitude by the same amount. Without this the change
     * would be treated as a measurement error and worked off over the
     * filter's response time, so the altimeter would drift to its new reading
     * instead of jumping to it as a real one does. From the updating
     * task.
     */
    void shiftAltitude(float delta_m);

    VerticalChannelSample sample() const { return m_published.read(); }

    float altitude() const { return sample().altitude_m; }
    float verticalSpeed() const { return sample().vertical_speed_ms; }
    bool valid() const { return sample().valid; }

private:
    void publish();
    void restart(float altitude_m);

    VerticalChannelConfig m_config;

    /* State: altitude, vertical speed, accelerometer bias. */
    float m_x[3];
    float m_P[3][3];
    bool m_initialised;
    float m_reject_s;

    PublishedSnapshot<VerticalChannelSample> m_published;
};
