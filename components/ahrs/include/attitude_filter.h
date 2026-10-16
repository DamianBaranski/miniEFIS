#pragma once

#include "published_snapshot.h"
#include <stdint.h>

/*
 * AttitudeFilter - attitude and heading from a gyroscope, accelerometer
 * and magnetometer: a multiplicative extended Kalman filter (MEKF).
 *
 * State
 * -----
 * The attitude is a unit quaternion, body to earth (north, east, down),
 * and the gyro's zero offset is three more states. The quaternion has
 * four numbers but only three degrees of freedom, so the filter does
 * not estimate it directly: it estimates a small rotation error, three
 * angles in body axes, and folds each correction back into the
 * quaternion (the "multiplicative" part). With the bias errors that is
 * six error states and a 6x6 covariance - which is what makes this a
 * Kalman filter rather than a fixed-gain blend: every correction is
 * weighed by how uncertain the estimate is against how noisy the
 * measurement is, and the bias is learned from the correlations the
 * covariance carries, not from a hand-tuned integrator.
 *
 * Steps
 * -----
 *   predict      every sample: the bias corrected gyro rates are
 *                integrated into the quaternion, and the covariance
 *                grows by the gyro's noise and the bias's random walk.
 *
 *   gravity      every sample: the accelerometer's direction against
 *                the direction the estimate predicts for gravity. It
 *                measures roll and pitch, and through them the gyro
 *                bias about any axis that is not vertical.
 *
 *                An accelerometer measures gravity plus every
 *                acceleration, so its noise is raised as its magnitude
 *                departs from 1 g and as the body rotates fast, and a
 *                reading the covariance says is implausible - the
 *                innovation's Mahalanobis distance beyond a chi-square
 *                gate - is rejected outright. In a turn, a pull-up or
 *                a long acceleration the filter therefore coasts on the
 *                gyro instead of being dragged towards a false horizon.
 *
 *   heading      with each magnetometer sample: the tilt compensated
 *                magnetic heading against the estimate's. Its
 *                correction is confined to a rotation about the earth
 *                vertical, so however distorted the field, it cannot
 *                tilt the horizon. It measures the heading and, through
 *                it, the bias about the vertical - which gravity cannot
 *                see. Gated like gravity.
 *
 * Without a magnetometer the heading is simply unobserved: its variance
 * grows, and it is reported as invalid.
 *
 * If the accelerometer looks static - 1 g, no rotation - yet keeps
 * failing the gate for a few seconds, it is the estimate that is wrong
 * (after a long coast, a saturated gyro, missed samples), so its
 * covariance is opened up and the next measurement pulls it back.
 * The magnetometer is treated the same way.
 *
 * Start-up
 * --------
 * The gyro's zero offset is first measured by averaging while the unit
 * is held still, which gives the filter a bias good to a few hundredths
 * of a deg/s and the magnitude of 1 g as this accelerometer reads it.
 * Until then the attitude is the accelerometer's alone, lightly
 * smoothed, and the status is Calibrating.
 *
 * Axis convention
 * ---------------
 * Body axes are x forward, y right, z down, with the gyro positive by
 * the right hand rule about each axis and the accelerometer reading
 * +1 g on z when level. Remap your sensor into this convention before
 * calling update(); the filter does not know how the board is mounted.
 *
 * Units: accelerometer in g, gyroscope in deg/s, magnetometer in uT
 * (only its direction is used for heading; the magnitude is checked
 * against the plausibility window).
 *
 * Threading
 * ---------
 * update() must be called from one task only. Every accessor takes a
 * consistent lock-free snapshot and may be called from any task.
 *
 * Limits: with no air data or GPS a sustained acceleration that the
 * gate cannot recognise - small, and long - still leaks into the
 * attitude, at the rate the gravity update is allowed to correct it.
 */

struct AttitudeVec3
{
    float x;
    float y;
    float z;
};

enum class AttitudeStatus
{
    NoData,         /* no sample within config.stale_timeout_ms        */
    Calibrating,    /* measuring gyro bias, the unit must be held still */
    Fused,          /* gyro corrected by the accelerometer              */
    GyroOnly        /* accelerometer rejected, coasting on the gyro     */
};

struct AttitudeSample
{
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;

    /* Magnetic heading, 0..360 deg. Only meaningful if heading_valid. */
    float heading_deg = 0.0f;
    bool heading_valid = false;

    /*
     * Weight given to the accelerometer on the latest sample, 0..1: the
     * filter's base accelerometer noise over the noise it used, or 0 if
     * the sample was rejected by the gate.
     */
    float accel_trust = 0.0f;

    /* One standard deviation of the estimate, from the covariance. */
    float tilt_sigma_deg = 0.0f;        /* roll and pitch */
    float heading_sigma_deg = 0.0f;

    AttitudeStatus status = AttitudeStatus::NoData;

    /* Age of this estimate, milliseconds since the sample that made it. */
    uint32_t age_ms = 0;
};

/*
 * Tuning: the filter's model of the sensors. The defaults suit the
 * QMI8658 and QMC5883 on this board in a hand or a small aircraft;
 * assign individual fields to change them.
 */
struct AttitudeConfig
{
    /*
     * Gyro rate noise, deg/s per root hertz - how fast the attitude's
     * uncertainty grows between corrections. Higher than the part's
     * datasheet figure (0.015) on purpose: it also has to cover
     * vibration, scale factor error and the integration's own error.
     */
    float gyro_noise_dps = 0.15f;

    /*
     * Rotation's own error, as a fraction of the rate, per root second.
     * A gyro's scale is a percent or two out, and a real installation's
     * magnetometer calibration leaves a heading error that changes with
     * heading, so after a turn the gyro and the magnetometer disagree
     * about how far it went - by more the further it went. Growing the
     * uncertainty with the rotation says so: after a turn the filter
     * expects to be corrected, and takes the magnetometer's answer
     * within a fraction of a second. Without it the filter believes its
     * gyro to the end, rejects the magnetometer, and only gives way
     * seconds later - a heading that holds after a turn and then moves.
     * At rest it adds nothing, so a field that changes while the unit
     * is still is still rejected as a disturbance.
     */
    float gyro_scale_noise = 0.05f;

    /* Gyro bias random walk, deg/s per root second: how fast the zero
     * offset is believed to wander, so how fast it can be re-learned. */
    float gyro_bias_walk_dps = 0.005f;

    /*
     * Accelerometer direction noise, in g, when it reads exactly 1 g and
     * the body is still. Covers the sensor's noise and the small
     * accelerations of being held or flown smoothly.
     */
    float accel_noise_g = 0.03f;

    /*
     * How long the accelerometer's errors stay correlated, seconds. Its
     * error is mostly not white noise but acceleration - the hand, the
     * air, the engine - lasting a good fraction of a second, so a
     * hundred samples in that time carry nowhere near a hundred
     * samples' worth of information. Weighed as independent, a gentle
     * acceleration held for a few seconds is overwhelming evidence, and
     * the filter tilts the horizon and learns a false gyro bias to
     * explain it.
     *
     * So the gain uses the noise this correlation implies - the white
     * figure times sqrt(correlation / interval) - while the gate keeps
     * the white figure, so that a reading which really is implausible
     * is still recognised as one.
     */
    float accel_correlation_s = 0.5f;

    /*
     * How fast that noise is raised: it doubles in variance once the
     * magnitude is accel_norm_window_g away from 1 g, or the body rotates
     * at accel_rate_window_dps - where centripetal terms appear.
     */
    float accel_norm_window_g = 0.05f;
    float accel_rate_window_dps = 60.0f;

    /*
     * Turning flight. An aircraft moving at speed V and turning at rate w
     * about its pitch or yaw axis accelerates at V w towards the centre
     * of the turn - and in a coordinated turn that acceleration adds to
     * gravity exactly along the body's own down axis, so the
     * accelerometer reads "level" while the aircraft is banked, and
     * barely more than 1 g at a gentle bank. Neither the magnitude test
     * nor the gate can see it.
     *
     * Without an airspeed sensor the filter cannot subtract it, but it
     * can bound it: V w, with V this assumed speed and w smoothed over
     * accel_turn_smoothing_s so that vibration does not count as
     * turning. Below accel_turn_limit_g that bound is added to the
     * accelerometer's noise; beyond it the accelerometer is not used at
     * all and the turn is flown on the gyro - at 30 m/s, any turn faster
     * than 0.4 deg/s, two degrees of bank at a light aircraft's speed.
     *
     * Merely raising the noise in a turn does not work: the gate widens
     * with it, and a reading that would have been rejected is accepted
     * at low weight and bleeds the bank away over the turn. And the
     * limit has to be low: the few degrees believed while rolling into a
     * turn become a tilt error, which the magnetometer's tilt
     * compensation turns into a heading error, which teaches the filter
     * a wrong vertical gyro bias, which in a bank tilts the pitch.
     *
     * 0 by default: a unit held in the hand or carried turns at walking
     * pace at most, so turning never makes its accelerometer wrong, and
     * it is used all the time - excluding it whenever the unit rotated
     * left the attitude coasting on the gyro for as long as the unit
     * kept moving. In an aircraft without airspeed that same choice lets
     * a sustained turn lean the horizon towards level - by the bank
     * angle, at the rate the gravity update corrects - so set the
     * aircraft's speed here, or better, feed it a measured one.
     */
    float accel_turn_speed_ms = 0.0f;
    float accel_turn_limit_g = 0.03f;
    float accel_turn_smoothing_s = 0.3f;

    /*
     * A gyro bias error looks exactly like a turn: a residual 1 deg/s
     * about the vertical is a 1 deg/s "turn" to the test above, and
     * excluding the accelerometer for it would stop the very updates
     * that learn the bias out - for ever. A real turn banked more than a
     * few degrees reads measurably over 1 g, though, and a gentle one
     * does not last long. So a "turn" that goes on for
     * accel_turn_timeout_s with the accelerometer within
     * accel_turn_norm_g of 1 g is taken for bias, and the accelerometer
     * is used again (still through the gate).
     */
    float accel_turn_norm_g = 0.005f;
    float accel_turn_timeout_s = 20.0f;

    /*
     * The gyro bias is learned - from either sensor - only while the
     * rotation, smoothed, is slower than this. Rolling into a turn the
     * accelerometer disagrees with the gyro for reasons that have
     * nothing to do with bias, and what would be learned there is
     * wrong. Above any plausible residual bias, so a bias can always be
     * learned out.
     */
    float bias_learn_rate_dps = 2.5f;

    /* Below this weight the status is GyroOnly. */
    float accel_trust_min = 0.05f;

    /*
     * Chi-square gates on the squared Mahalanobis distance of each
     * innovation. 16 is four standard deviations: a sample further out
     * than that is not noise but a different signal, and is rejected.
     */
    float accel_gate = 16.0f;
    float mag_gate = 16.0f;

    /*
     * Heading measurement noise, deg. Tilt compensation is only as good
     * as the tilt, and the calibration is never perfect.
     */
    float mag_noise_deg = 3.0f;

    /*
     * Magnetometer plausibility window in uT. Earth's field is 25..65 uT;
     * a sample outside this range is being distorted by something nearby
     * and is discarded.
     */
    float mag_min_norm = 15.0f;
    float mag_max_norm = 90.0f;
    uint32_t mag_timeout_ms = 1000;     /* no accepted sample -> no heading */

    /* Heading is unobservable this close to the vertical. */
    float mag_max_pitch_deg = 70.0f;

    /*
     * The magnetometer teaches the gyro bias only while the accelerometer
     * is in use - not in a turn, when the tilt the heading is levelled
     * with is unobserved and a tilt error would be learned as bias.
     */
    float mag_bias_learning_gap_s = 0.5f;

    /* Heading is reported valid only while its sigma is below this. */
    float heading_valid_sigma_deg = 10.0f;

    /*
     * Re-acquisition: after this long of an accelerometer that looks
     * static (within reacquire_norm_g of 1 g, turning slower than
     * reacquire_rate_dps) but is still being rejected, the attitude's
     * covariance is opened to reacquire_sigma_deg so it can be pulled
     * back. The magnetometer likewise, for heading.
     */
    float reacquire_s = 2.0f;
    float reacquire_norm_g = 0.03f;
    float reacquire_rate_dps = 10.0f;
    float reacquire_sigma_deg = 30.0f;

    /* Covariance at the end of the start-up calibration. */
    float initial_tilt_sigma_deg = 2.0f;
    float initial_bias_sigma_dps = 0.1f;

    /*
     * Gyro bias. Zero-rate offsets of 3-4 deg/s are normal for this
     * part, so the clamp has to leave room above that.
     */
    float bias_limit_dps = 10.0f;

    /*
     * Start-up bias measurement: average this many samples while the
     * unit is still, restarting if it moves.
     */
    uint32_t gyro_cal_samples = 150;
    float gyro_cal_move_limit_dps = 6.0f;
    float gyro_cal_move_limit_g = 0.08f;

    /* Smoothing of the accelerometer-only output shown while calibrating. */
    float accel_fallback_alpha = 0.18f;

    /*
     * Sample interval limits, seconds. A starved caller must not
     * integrate one huge step, and a repeated timestamp must not be
     * treated as an interval.
     */
    float dt_min_s = 0.0005f;
    float dt_max_s = 0.05f;

    /* Age at which the estimate is reported as AttitudeStatus::NoData. */
    uint32_t stale_timeout_ms = 500;
};

class AttitudeFilter
{
public:
    AttitudeFilter();
    explicit AttitudeFilter(const AttitudeConfig &config);

    void setConfig(const AttitudeConfig &config);
    AttitudeConfig config() const { return m_config; }

    /* Discard all state and re-measure the gyro bias. */
    void reset();

    /* Re-measure the gyro bias only; the unit must be held still. */
    void requestGyroCalibration();

    /*
     * Feed one IMU sample. Pass mag = NULL when there is no
     * magnetometer, or when this particular sample has no fresh
     * magnetometer reading to go with it. The reading must already be
     * calibrated (mag_calibration.h) and in body axes, uT. Each one
     * must be passed once only: the filter counts every one as an
     * independent measurement.
     *
     * The interval is measured from the platform clock. Call at a steady
     * rate of 50 Hz or more.
     */
    void update(const AttitudeVec3 &accel_g,
                const AttitudeVec3 &gyro_dps,
                const AttitudeVec3 *mag = 0);

    /*
     * As update(), but with the interval supplied by the caller - for
     * replaying logged data, for tests, or when the sample timestamps
     * come from the sensor itself.
     */
    void updateWithDt(const AttitudeVec3 &accel_g,
                      const AttitudeVec3 &gyro_dps,
                      const AttitudeVec3 *mag,
                      float dt_s);

    /* One consistent snapshot. Safe from any task. */
    AttitudeSample sample() const;

    float roll() const { return sample().roll_deg; }
    float pitch() const { return sample().pitch_deg; }
    float heading() const { return sample().heading_deg; }
    bool headingValid() const { return sample().heading_valid; }
    AttitudeStatus status() const { return sample().status; }

    /* Current gyro zero offset estimate, deg/s. Safe from any task. */
    AttitudeVec3 gyroBias() const { return m_published_bias.read(); }

    /*
     * Vertical acceleration in earth axes, upward positive, m/s^2, with
     * gravity removed - what the vertical channel integrates.
     *
     * Derived by projecting the measured acceleration onto the estimated
     * direction of gravity, so it inherits any attitude error. That
     * error is systematic rather than random, which is why the vertical
     * channel carries a bias state to absorb it.
     *
     * Call from the updating task, alongside update().
     */
    float verticalAcceleration(const AttitudeVec3 &accel_g) const;

    /* Roll and pitch straight from gravity, with no filtering. */
    static void accelAttitude(const AttitudeVec3 &accel_g,
                              float *roll_deg,
                              float *pitch_deg);

    /*
     * A magnetometer sample rotated into the level frame: x forward and
     * y right along the horizon, z straight down. x and y are the two
     * components the heading is taken from - atan2(-y, x) - and z is
     * what tilt compensation moves out of the way.
     */
    static AttitudeVec3 magLevel(const AttitudeVec3 &mag,
                                 float roll_deg,
                                 float pitch_deg);

    /*
     * Magnetic heading from a magnetometer sample levelled with the
     * given roll and pitch. Returns 0..360 deg.
     */
    static float magHeading(const AttitudeVec3 &mag,
                            float roll_deg,
                            float pitch_deg);

private:
    static const int N = 6;             /* error states */

    void publish(const AttitudeSample &sample);
    void eulerFromQuaternion(float *roll_deg, float *pitch_deg,
                             float *heading_deg) const;
    void setQuaternionFromEuler(float roll_deg, float pitch_deg,
                                float heading_deg);
    void gravityInBody(float g[3]) const;
    void clearCalibration();
    void calibrateStep(const AttitudeVec3 &gyro_dps, float accel_magnitude);
    void startFilter();
    bool magPlausible(const AttitudeVec3 &mag) const;

    void predict(const float rate_rad[3], float dt);
    bool gravityUpdate(const AttitudeVec3 &accel_g, const float rate[3],
                       float dt, float *trust);
    bool headingUpdate(const AttitudeVec3 &mag, float dt);
    void inject(const float dx[N]);
    void openTiltCovariance(float sigma_rad);
    void openHeadingCovariance(float sigma_rad);
    void tidyCovariance();
    float tiltVariance() const;
    bool biasLearnable() const;
    float headingVariance() const;

    AttitudeConfig m_config;

    /* Filter state. Owned by the updating task. */
    float m_q[4];                   /* body to earth rotation, w x y z */
    float m_bias[3];                /* gyro zero offset, rad/s */
    float m_P[N][N];                /* error covariance: angles, biases */
    float m_gravity_reference;      /* |a| measured at rest, g */
    bool m_bias_valid;
    bool m_heading_seen;            /* a magnetometer sample was used */
    float m_accel_reject_s;         /* static-looking yet rejected */
    float m_turn_rate;              /* smoothed pitch and yaw rate, rad/s */
    float m_rate;                   /* smoothed total rate, rad/s */
    float m_turn_quiet_s;           /* "turning" at 1 g: perhaps bias */
    float m_since_gravity_s;        /* since the accelerometer was used */
    float m_mag_reject_s;
    float m_time_s;                 /* sum of the intervals given */
    float m_last_mag_s;             /* when a heading was last used */
    uint32_t m_last_us;
    bool m_have_sample;

    /* Start-up bias measurement. */
    uint32_t m_cal_count;
    AttitudeVec3 m_cal_sum;
    AttitudeVec3 m_cal_ref;
    float m_cal_magnitude_sum;
    float m_cal_magnitude_ref;

    /* Set from any task, consumed by the updating task. */
    volatile bool m_cal_request;

    PublishedSnapshot<AttitudeSample> m_published;
    PublishedSnapshot<AttitudeVec3> m_published_bias;
    volatile uint32_t m_published_us;
};
