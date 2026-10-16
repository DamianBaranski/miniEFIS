#include "attitude_filter.h"

#include <math.h>
#include <string.h>

#if defined(ESP_PLATFORM)
#include "esp_timer.h"
#else
#include <time.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* -------------------------------------------------------------------------- */
/* Helpers                                                                    */
/* -------------------------------------------------------------------------- */

static uint32_t platformMicros(void)
{
#if defined(ESP_PLATFORM)
    return (uint32_t)esp_timer_get_time();
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000000ULL +
                      (uint64_t)ts.tv_nsec / 1000ULL);
#endif
}

static float deg(float radians)
{
    return radians * 180.0f / (float)M_PI;
}

static float rad(float degrees)
{
    return degrees * (float)M_PI / 180.0f;
}

static float clampf(float value, float min_value, float max_value)
{
    if(value < min_value) return min_value;
    if(value > max_value) return max_value;
    return value;
}

/* Fold an angle into -180..+180 deg so errors take the short way round. */
static float wrap180(float angle)
{
    while(angle > 180.0f) angle -= 360.0f;
    while(angle < -180.0f) angle += 360.0f;
    return angle;
}

static float norm3(const AttitudeVec3 &v)
{
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

static void quaternionNormalise(float *q)
{
    const float norm =
        sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);

    if(norm < 1.0e-9f)
    {
        /* Cannot happen from a normalised start, but a NaN here would be
         * unrecoverable, so fall back to level rather than divide. */
        q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
        return;
    }

    const float inverse = 1.0f / norm;

    q[0] *= inverse;
    q[1] *= inverse;
    q[2] *= inverse;
    q[3] *= inverse;
}

/* q = q * r, Hamilton product: r is a rotation in body axes. */
static void quaternionMultiplyRight(float *q, const float *r)
{
    const float w = q[0] * r[0] - q[1] * r[1] - q[2] * r[2] - q[3] * r[3];
    const float x = q[0] * r[1] + q[1] * r[0] + q[2] * r[3] - q[3] * r[2];
    const float y = q[0] * r[2] - q[1] * r[3] + q[2] * r[0] + q[3] * r[1];
    const float z = q[0] * r[3] + q[1] * r[2] - q[2] * r[1] + q[3] * r[0];

    q[0] = w; q[1] = x; q[2] = y; q[3] = z;
}

/* Inverse of a symmetric 3x3 matrix. False if it is singular. */
static bool invertSymmetric3(const float m[3][3], float out[3][3])
{
    const float c00 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
    const float c01 = m[1][2] * m[2][0] - m[1][0] * m[2][2];
    const float c02 = m[1][0] * m[2][1] - m[1][1] * m[2][0];

    const float determinant = m[0][0] * c00 + m[0][1] * c01 + m[0][2] * c02;

    if(fabsf(determinant) < 1.0e-20f)
        return false;

    const float inverse = 1.0f / determinant;

    out[0][0] = c00 * inverse;
    out[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inverse;
    out[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inverse;
    out[1][0] = c01 * inverse;
    out[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inverse;
    out[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inverse;
    out[2][0] = c02 * inverse;
    out[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inverse;
    out[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inverse;

    return true;
}

/* -------------------------------------------------------------------------- */
/* Construction                                                               */
/* -------------------------------------------------------------------------- */

AttitudeFilter::AttitudeFilter()
{
    reset();
}

AttitudeFilter::AttitudeFilter(const AttitudeConfig &config)
    : m_config(config)
{
    reset();
}

void AttitudeFilter::setConfig(const AttitudeConfig &config)
{
    m_config = config;
}

void AttitudeFilter::clearCalibration()
{
    m_cal_count = 0;
    m_cal_sum.x = 0.0f; m_cal_sum.y = 0.0f; m_cal_sum.z = 0.0f;
    m_cal_ref.x = 0.0f; m_cal_ref.y = 0.0f; m_cal_ref.z = 0.0f;
    m_cal_magnitude_sum = 0.0f;
    m_cal_magnitude_ref = 0.0f;
}

void AttitudeFilter::reset()
{
    m_q[0] = 1.0f; m_q[1] = 0.0f; m_q[2] = 0.0f; m_q[3] = 0.0f;
    m_bias[0] = 0.0f; m_bias[1] = 0.0f; m_bias[2] = 0.0f;
    memset(m_P, 0, sizeof(m_P));
    m_gravity_reference = 1.0f;
    m_bias_valid = false;
    m_heading_seen = false;
    m_accel_reject_s = 0.0f;
    m_mag_reject_s = 0.0f;
    m_turn_rate = 0.0f;
    m_rate = 0.0f;
    m_turn_quiet_s = 0.0f;
    m_since_gravity_s = 0.0f;
    m_time_s = 0.0f;
    m_last_mag_s = 0.0f;
    m_last_us = 0;
    m_have_sample = false;
    m_cal_request = true;
    m_published_us = 0;
    clearCalibration();

    AttitudeSample empty;
    publish(empty);

    AttitudeVec3 zero = { 0.0f, 0.0f, 0.0f };
    m_published_bias.publish(zero);
}

void AttitudeFilter::requestGyroCalibration()
{
    m_cal_request = true;
}

/* -------------------------------------------------------------------------- */
/* Published snapshot                                                         */
/* -------------------------------------------------------------------------- */

void AttitudeFilter::publish(const AttitudeSample &sample)
{
    /* Stamp first: a reader that gets the value must not be able to pair
     * it with an older time. */
    m_published_us = platformMicros();
    m_published.publish(sample);
}

AttitudeSample AttitudeFilter::sample() const
{
    AttitudeSample out = m_published.read();

    if(out.status == AttitudeStatus::NoData)
        return out;

    /* Unsigned arithmetic, so this survives the clock rollover. */
    out.age_ms = (uint32_t)(platformMicros() - m_published_us) / 1000u;

    if(out.age_ms > m_config.stale_timeout_ms)
        out.status = AttitudeStatus::NoData;

    return out;
}

/* Unit vector along gravity - earth down - in body axes, as this
 * estimate has it. It is also the third row of the body to earth
 * rotation, which is what makes it the heading measurement's Jacobian. */
void AttitudeFilter::gravityInBody(float g[3]) const
{
    const float w = m_q[0];
    const float x = m_q[1];
    const float y = m_q[2];
    const float z = m_q[3];

    g[0] = 2.0f * (x * z - w * y);
    g[1] = 2.0f * (y * z + w * x);
    g[2] = 1.0f - 2.0f * (x * x + y * y);
}

float AttitudeFilter::verticalAcceleration(const AttitudeVec3 &accel_g) const
{
    static const float GRAVITY_MS2 = 9.80665f;

    float g[3];
    gravityInBody(g);

    /*
     * The component of the measured acceleration along that direction,
     * in g. Sitting still it is exactly 1, since the accelerometer then
     * reads gravity and nothing else; the excess over 1 is what is left
     * once gravity is accounted for.
     */
    const float along_gravity =
        accel_g.x * g[0] + accel_g.y * g[1] + accel_g.z * g[2];

    return (along_gravity - 1.0f) * GRAVITY_MS2;
}

/* -------------------------------------------------------------------------- */
/* Individual measurements                                                    */
/* -------------------------------------------------------------------------- */

void AttitudeFilter::accelAttitude(const AttitudeVec3 &accel_g,
                                   float *roll_deg,
                                   float *pitch_deg)
{
    const float roll_rad =
        atan2f(accel_g.y, accel_g.z);

    const float pitch_rad =
        atan2f(
            -accel_g.x,
            sqrtf(accel_g.y * accel_g.y + accel_g.z * accel_g.z));

    if(roll_deg != 0) *roll_deg = deg(roll_rad);
    if(pitch_deg != 0) *pitch_deg = deg(pitch_rad);
}

AttitudeVec3 AttitudeFilter::magLevel(const AttitudeVec3 &mag,
                                      float roll_deg,
                                      float pitch_deg)
{
    /*
     * Undo roll and pitch to get the field in the local level frame,
     * where its horizontal components give the heading directly. Using
     * the raw body frame components instead is the classic mistake: the
     * heading would then swing with every bank and pitch change.
     */
    const float sr = sinf(rad(roll_deg));
    const float cr = cosf(rad(roll_deg));
    const float sp = sinf(rad(pitch_deg));
    const float cp = cosf(rad(pitch_deg));

    AttitudeVec3 level;

    level.x = mag.x * cp + (mag.y * sr + mag.z * cr) * sp;
    level.y = mag.y * cr - mag.z * sr;
    level.z = -mag.x * sp + (mag.y * sr + mag.z * cr) * cp;

    return level;
}

float AttitudeFilter::magHeading(const AttitudeVec3 &mag,
                                 float roll_deg,
                                 float pitch_deg)
{
    const AttitudeVec3 level = magLevel(mag, roll_deg, pitch_deg);

    float heading = deg(atan2f(-level.y, level.x));

    if(heading < 0.0f)
        heading += 360.0f;

    return heading;
}

/* A field too weak or too strong to be the earth's alone. */
bool AttitudeFilter::magPlausible(const AttitudeVec3 &mag) const
{
    const float magnitude = norm3(mag);

    return magnitude >= m_config.mag_min_norm &&
           magnitude <= m_config.mag_max_norm;
}

/*
 * Average the gyro while the unit is still to get its zero offset, and
 * take the gravity magnitude at the same time.
 *
 * An uncalibrated offset of a few deg/s would run the attitude away
 * within seconds, so the rates are not integrated until this completes.
 * Any movement during the window restarts it.
 *
 * Movement is judged by how much the readings change, never by how far
 * the accelerometer is from a nominal 1 g: the reference that test
 * needs is what is being measured here. Gating on it instead deadlocks
 * - an accelerometer reading a few percent off never looks still
 * enough to start, so calibration restarts on every sample.
 */
void AttitudeFilter::calibrateStep(const AttitudeVec3 &gyro_dps,
                                   float accel_magnitude)
{
    const bool first = (m_cal_count == 0);

    if(first)
    {
        m_cal_ref = gyro_dps;
        m_cal_magnitude_ref = accel_magnitude;
    }

    const float limit = m_config.gyro_cal_move_limit_dps;

    const bool moved =
        fabsf(gyro_dps.x - m_cal_ref.x) > limit ||
        fabsf(gyro_dps.y - m_cal_ref.y) > limit ||
        fabsf(gyro_dps.z - m_cal_ref.z) > limit ||
        fabsf(accel_magnitude - m_cal_magnitude_ref) >
            m_config.gyro_cal_move_limit_g;

    if(!first && moved)
    {
        clearCalibration();
        return;
    }

    m_cal_sum.x += gyro_dps.x;
    m_cal_sum.y += gyro_dps.y;
    m_cal_sum.z += gyro_dps.z;
    m_cal_magnitude_sum += accel_magnitude;
    m_cal_count++;

    if(m_cal_count < m_config.gyro_cal_samples)
        return;

    const float n = (float)m_cal_count;
    const float bias_limit = m_config.bias_limit_dps;

    m_bias[0] = rad(clampf(m_cal_sum.x / n, -bias_limit, bias_limit));
    m_bias[1] = rad(clampf(m_cal_sum.y / n, -bias_limit, bias_limit));
    m_bias[2] = rad(clampf(m_cal_sum.z / n, -bias_limit, bias_limit));

    /*
     * The measured magnitude becomes the "one g" the accelerometer's
     * noise model compares against. Sanity bounded: if the unit was
     * moved throughout, or an axis is dead, fall back to nominal rather
     * than enshrining nonsense.
     */
    const float measured = m_cal_magnitude_sum / n;

    m_gravity_reference =
        (measured > 0.7f && measured < 1.3f) ? measured : 1.0f;

    m_bias_valid = true;
    clearCalibration();
    startFilter();
}

/*
 * The covariance the filter starts from, once the bias is known: roll
 * and pitch close, from the smoothed accelerometer; heading unknown
 * until a magnetometer says otherwise; bias as good as the average.
 */
void AttitudeFilter::startFilter()
{
    memset(m_P, 0, sizeof(m_P));

    openTiltCovariance(rad(m_config.initial_tilt_sigma_deg));
    openHeadingCovariance((float)M_PI);

    const float bias_sigma = rad(m_config.initial_bias_sigma_dps);

    for(int i = 3; i < N; i++)
        m_P[i][i] = bias_sigma * bias_sigma;

    m_heading_seen = false;
    m_accel_reject_s = 0.0f;
    m_mag_reject_s = 0.0f;
}

/* -------------------------------------------------------------------------- */
/* Quaternion and Euler angles                                                */
/* -------------------------------------------------------------------------- */

/*
 * m_q rotates a vector from body axes into earth axes (north, east,
 * down). The Euler angles below are the aerospace sequence, yaw then
 * pitch then roll, so they match the accelerometer formulas in
 * accelAttitude() exactly.
 */

void AttitudeFilter::eulerFromQuaternion(float *roll_deg, float *pitch_deg,
                                         float *heading_deg) const
{
    const float w = m_q[0];
    const float x = m_q[1];
    const float y = m_q[2];
    const float z = m_q[3];

    if(roll_deg != 0)
    {
        *roll_deg = deg(atan2f(2.0f * (y * z + w * x),
                               1.0f - 2.0f * (x * x + y * y)));
    }

    if(pitch_deg != 0)
    {
        /* asin's argument can drift a hair outside [-1,1] through
         * rounding, which would produce a NaN that then poisons the
         * whole state. */
        const float sin_pitch = clampf(2.0f * (w * y - x * z), -1.0f, 1.0f);
        *pitch_deg = deg(asinf(sin_pitch));
    }

    if(heading_deg != 0)
    {
        float heading = deg(atan2f(2.0f * (x * y + w * z),
                                   1.0f - 2.0f * (y * y + z * z)));

        if(heading < 0.0f)
            heading += 360.0f;

        *heading_deg = heading;
    }
}

void AttitudeFilter::setQuaternionFromEuler(float roll_deg, float pitch_deg,
                                            float heading_deg)
{
    const float half_roll = rad(roll_deg) * 0.5f;
    const float half_pitch = rad(pitch_deg) * 0.5f;
    const float half_heading = rad(heading_deg) * 0.5f;

    const float cr = cosf(half_roll),  sr = sinf(half_roll);
    const float cp = cosf(half_pitch), sp = sinf(half_pitch);
    const float cy = cosf(half_heading), sy = sinf(half_heading);

    m_q[0] = cr * cp * cy + sr * sp * sy;
    m_q[1] = sr * cp * cy - cr * sp * sy;
    m_q[2] = cr * sp * cy + sr * cp * sy;
    m_q[3] = cr * cp * sy - sr * sp * cy;
}

/* -------------------------------------------------------------------------- */
/* Covariance                                                                 */
/* -------------------------------------------------------------------------- */

/*
 * Roll and pitch uncertainty: the attitude covariance across the
 * vertical, averaged over the two horizontal directions.
 */
float AttitudeFilter::tiltVariance() const
{
    const float trace = m_P[0][0] + m_P[1][1] + m_P[2][2];
    return 0.5f * (trace - headingVariance());
}

bool AttitudeFilter::biasLearnable() const
{
    return deg(m_rate) < m_config.bias_learn_rate_dps;
}

/* Heading uncertainty: the attitude covariance about the vertical. */
float AttitudeFilter::headingVariance() const
{
    float g[3];
    gravityInBody(g);

    float variance = 0.0f;

    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            variance += g[i] * m_P[i][j] * g[j];

    return variance;
}

/* Widen roll and pitch - the attitude error across the vertical - by
 * sigma. Adding a positive semidefinite term keeps the covariance one. */
void AttitudeFilter::openTiltCovariance(float sigma_rad)
{
    float g[3];
    gravityInBody(g);

    const float variance = sigma_rad * sigma_rad;

    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            m_P[i][j] += variance * ((i == j ? 1.0f : 0.0f) - g[i] * g[j]);
}

/* Widen heading - the attitude error about the vertical - by sigma. */
void AttitudeFilter::openHeadingCovariance(float sigma_rad)
{
    float g[3];
    gravityInBody(g);

    const float variance = sigma_rad * sigma_rad;

    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            m_P[i][j] += variance * g[i] * g[j];
}

/*
 * Keep the covariance symmetric, and bounded where nothing observes
 * it: without a magnetometer the heading and the vertical bias only
 * ever grow. A bound is applied by scaling that state's row and column
 * together, which keeps the matrix positive semidefinite.
 */
void AttitudeFilter::tidyCovariance()
{
    for(int i = 0; i < N; i++)
        for(int j = i + 1; j < N; j++)
        {
            const float mean = 0.5f * (m_P[i][j] + m_P[j][i]);
            m_P[i][j] = mean;
            m_P[j][i] = mean;
        }

    const float angle_cap = (float)(M_PI * M_PI);
    const float bias_limit = rad(m_config.bias_limit_dps);
    const float bias_cap = bias_limit * bias_limit;

    for(int i = 0; i < N; i++)
    {
        const float cap = (i < 3) ? angle_cap : bias_cap;

        if(!(m_P[i][i] > 1.0e-14f))
        {
            /* Collapsed, or NaN: start this state over at a small value
             * rather than let it poison the rest. */
            for(int j = 0; j < N; j++)
            {
                m_P[i][j] = 0.0f;
                m_P[j][i] = 0.0f;
            }

            m_P[i][i] = 1.0e-10f;
            continue;
        }

        if(m_P[i][i] <= cap)
            continue;

        const float scale = sqrtf(cap / m_P[i][i]);

        for(int j = 0; j < N; j++)
        {
            m_P[i][j] *= scale;
            m_P[j][i] *= scale;
        }
    }
}

/* -------------------------------------------------------------------------- */
/* The filter                                                                 */
/* -------------------------------------------------------------------------- */

/*
 * Prediction: integrate the rates, grow the covariance.
 *
 * The error dynamics in body axes are
 *
 *   d(angle)/dt = -[rate x] angle - bias error
 *   d(bias)/dt  =  random walk
 *
 * discretised to first order, which at 100 Hz and the rates a hand or a
 * light aircraft reach is far below everything else in the error budget.
 */
void AttitudeFilter::predict(const float rate[3], float dt)
{
    /* The quaternion: an exact rotation by rate * dt, not a first order
     * step, so fast rolls do not shrink or skew it. */
    const float angle =
        sqrtf(rate[0] * rate[0] + rate[1] * rate[1] + rate[2] * rate[2]) * dt;

    if(angle > 1.0e-9f)
    {
        const float half = 0.5f * angle;
        const float scale = sinf(half) / (angle / dt);

        const float rotation[4] = {
            cosf(half), rate[0] * scale, rate[1] * scale, rate[2] * scale
        };

        quaternionMultiplyRight(m_q, rotation);
        quaternionNormalise(m_q);
    }

    /* Transition matrix. */
    float F[N][N];
    memset(F, 0, sizeof(F));

    for(int i = 0; i < N; i++)
        F[i][i] = 1.0f;

    F[0][1] =  rate[2] * dt;  F[0][2] = -rate[1] * dt;
    F[1][0] = -rate[2] * dt;  F[1][2] =  rate[0] * dt;
    F[2][0] =  rate[1] * dt;  F[2][1] = -rate[0] * dt;

    for(int i = 0; i < 3; i++)
        F[i][i + 3] = -dt;

    /* P = F P F' + Q */
    float FP[N][N];

    for(int i = 0; i < N; i++)
        for(int j = 0; j < N; j++)
        {
            float sum = 0.0f;

            for(int k = 0; k < N; k++)
                sum += F[i][k] * m_P[k][j];

            FP[i][j] = sum;
        }

    for(int i = 0; i < N; i++)
        for(int j = 0; j < N; j++)
        {
            float sum = 0.0f;

            for(int k = 0; k < N; k++)
                sum += FP[i][k] * F[j][k];

            m_P[i][j] = sum;
        }

    const float gyro_noise = rad(m_config.gyro_noise_dps);
    const float bias_walk = rad(m_config.gyro_bias_walk_dps);
    const float scale_noise = m_config.gyro_scale_noise;

    for(int i = 0; i < 3; i++)
    {
        /* The rate's own error grows with the rate: gyro_scale_noise. */
        const float rotation = scale_noise * rate[i];

        m_P[i][i] += (gyro_noise * gyro_noise + rotation * rotation) * dt;
        m_P[i + 3][i + 3] += bias_walk * bias_walk * dt;
    }
}

/*
 * Fold an error state estimate into the nominal state: the angles as a
 * small rotation of the quaternion in body axes, the bias added.
 */
void AttitudeFilter::inject(const float dx[N])
{
    /*
     * The exact rotation, not the small-angle (1, angle/2): corrections
     * are normally tiny, but the first heading measurement can turn the
     * estimate half way round, and the small-angle form would fall well
     * short of that - 180 degrees asked for, 115 done - while the
     * covariance claimed it had all been done.
     */
    const float angle =
        sqrtf(dx[0] * dx[0] + dx[1] * dx[1] + dx[2] * dx[2]);

    if(angle > 1.0e-9f)
    {
        const float scale = sinf(0.5f * angle) / angle;

        const float rotation[4] = {
            cosf(0.5f * angle), dx[0] * scale, dx[1] * scale, dx[2] * scale
        };

        quaternionMultiplyRight(m_q, rotation);
        quaternionNormalise(m_q);
    }

    const float limit = rad(m_config.bias_limit_dps);

    for(int i = 0; i < 3; i++)
        m_bias[i] = clampf(m_bias[i] + dx[i + 3], -limit, limit);
}

/*
 * Gravity: the accelerometer's direction against the one predicted.
 *
 * Measurement model, with h the predicted direction of gravity in body
 * axes and e the attitude error:
 *
 *   a / |a| = h + [h x] e + noise
 *
 * so H = [ [h x]  0 ]. Rotation about gravity itself is in the null
 * space of [h x] - an accelerometer cannot see heading.
 */
bool AttitudeFilter::gravityUpdate(const AttitudeVec3 &accel_g,
                                   const float rate[3], float dt,
                                   float *trust)
{
    *trust = 0.0f;

    const float magnitude = norm3(accel_g);

    /* No usable gravity vector at all - free fall, or a dead sensor. */
    if(magnitude < 0.1f)
        return false;

    /*
     * The noise model. Anything but gravity in the reading shows first
     * as a magnitude other than 1 g, and rotation brings centripetal
     * terms; both raise the noise, so the reading is believed less.
     */
    const float rate_dps =
        deg(sqrtf(rate[0] * rate[0] + rate[1] * rate[1] + rate[2] * rate[2]));

    const float norm_error =
        (magnitude - m_gravity_reference) / m_config.accel_norm_window_g;
    const float rate_error = rate_dps / m_config.accel_rate_window_dps;

    /*
     * Turning: the centripetal acceleration could be as much as V w, in
     * g - rotation about pitch and yaw only, since a roll about the
     * direction of flight turns nothing. Smoothed, so that vibration
     * does not read as turning. Past the limit the reading says nothing
     * reliable about tilt, and is not used.
     */
    const float turning =
        m_config.accel_turn_speed_ms * m_turn_rate / 9.80665f;

    if(turning > m_config.accel_turn_limit_g)
    {
        /* At 1 g for long, it is a bias and not a turn. */
        const bool at_one_g = fabsf(magnitude - m_gravity_reference) <
                              m_config.accel_turn_norm_g;

        m_turn_quiet_s = at_one_g ? m_turn_quiet_s + dt : 0.0f;

        if(m_turn_quiet_s < m_config.accel_turn_timeout_s)
        {
            m_accel_reject_s = 0.0f;
            return false;
        }
    }
    else
    {
        m_turn_quiet_s = 0.0f;
    }

    const float inflation = 1.0f + norm_error * norm_error +
                            rate_error * rate_error;

    const float base_noise = m_config.accel_noise_g / m_gravity_reference;

    /* R_gate judges plausibility, R the weight: see accel_correlation_s. */
    const float R_gate = base_noise * base_noise * inflation +
                         turning * turning;
    const float R = R_gate * fmaxf(1.0f, m_config.accel_correlation_s / dt);

    float h[3];
    gravityInBody(h);

    const float measured[3] = {
        accel_g.x / magnitude, accel_g.y / magnitude, accel_g.z / magnitude
    };

    /*
     * The innovation, as the model can explain it. a/|a| - h is a chord
     * between two unit vectors, and for a large error it has a part
     * along h that no attitude change produces - enough on its own to
     * fail the gate whatever the covariance. So only its part across h
     * is used, lengthened from the chord to the arc: exactly the
     * rotation that takes h onto the measurement.
     */
    const float along = measured[0] * h[0] + measured[1] * h[1] +
                        measured[2] * h[2];

    float innovation[3] = {
        measured[0] - along * h[0],
        measured[1] - along * h[1],
        measured[2] - along * h[2]
    };

    const float across = sqrtf(innovation[0] * innovation[0] +
                               innovation[1] * innovation[1] +
                               innovation[2] * innovation[2]);

    if(across > 1.0e-6f)
    {
        const float arc = atan2f(across, along) / across;

        for(int i = 0; i < 3; i++)
            innovation[i] *= arc;
    }

    /* H's attitude block, [h x]; its bias block is zero. */
    const float Hs[3][3] = {
        {  0.0f, -h[2],  h[1] },
        {  h[2],  0.0f, -h[0] },
        { -h[1],  h[0],  0.0f }
    };

    /* P H' (6x3): only P's first three columns meet the nonzero block. */
    float PHt[N][3];

    for(int i = 0; i < N; i++)
        for(int j = 0; j < 3; j++)
        {
            float sum = 0.0f;

            for(int k = 0; k < 3; k++)
                sum += m_P[i][k] * Hs[j][k];

            PHt[i][j] = sum;
        }

    /* H P H', then S = H P H' + R for the gate and for the gain. */
    float HPHt[3][3];

    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
        {
            float sum = 0.0f;

            for(int k = 0; k < 3; k++)
                sum += Hs[i][k] * PHt[k][j];

            HPHt[i][j] = sum;
        }

    float S[3][3];

    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            S[i][j] = HPHt[i][j] + ((i == j) ? R_gate : 0.0f);

    float Sinv[3][3];

    if(!invertSymmetric3(S, Sinv))
        return false;

    float distance = 0.0f;

    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            distance += innovation[i] * Sinv[i][j] * innovation[j];

    if(distance > m_config.accel_gate)
    {
        /*
         * Rejected. If the accelerometer nonetheless looks like gravity
         * alone - 1 g, no rotation - and goes on being rejected, the
         * estimate is what is wrong: open its covariance so the next
         * sample can pull it back.
         */
        const bool looks_static =
            fabsf(magnitude - m_gravity_reference) <
                m_config.reacquire_norm_g &&
            rate_dps < m_config.reacquire_rate_dps;

        m_accel_reject_s = looks_static ? m_accel_reject_s + dt : 0.0f;

        if(m_accel_reject_s > m_config.reacquire_s)
        {
            openTiltCovariance(rad(m_config.reacquire_sigma_deg));
            m_accel_reject_s = 0.0f;
        }

        return false;
    }

    m_accel_reject_s = 0.0f;
    *trust = 1.0f / inflation;

    /* Passed the gate: now the gain, with the noise as it really is. */
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            S[i][j] = HPHt[i][j] + ((i == j) ? R : 0.0f);

    if(!invertSymmetric3(S, Sinv))
        return false;

    /* K = P H' S^-1 */
    float K[N][3];

    for(int i = 0; i < N; i++)
        for(int j = 0; j < 3; j++)
        {
            float sum = 0.0f;

            for(int k = 0; k < 3; k++)
                sum += PHt[i][k] * Sinv[k][j];

            K[i][j] = sum;
        }

    /* No bias learning while rotating: see bias_learn_rate_dps. The
     * Joseph form below is right for this gain as for any other. */
    if(!biasLearnable())
        for(int i = 3; i < N; i++)
            for(int j = 0; j < 3; j++)
                K[i][j] = 0.0f;

    float dx[N];

    for(int i = 0; i < N; i++)
        dx[i] = K[i][0] * innovation[0] + K[i][1] * innovation[1] +
                K[i][2] * innovation[2];

    /*
     * Joseph form, P = (I - KH) P (I - KH)' + K R K': longer than
     * P - KHP, but it stays symmetric and positive definite in single
     * precision, which the short form does not reliably do.
     */
    float A[N][N];

    for(int i = 0; i < N; i++)
        for(int j = 0; j < N; j++)
        {
            float kh = 0.0f;

            if(j < 3)
                for(int k = 0; k < 3; k++)
                    kh += K[i][k] * Hs[k][j];

            A[i][j] = ((i == j) ? 1.0f : 0.0f) - kh;
        }

    float AP[N][N];

    for(int i = 0; i < N; i++)
        for(int j = 0; j < N; j++)
        {
            float sum = 0.0f;

            for(int k = 0; k < N; k++)
                sum += A[i][k] * m_P[k][j];

            AP[i][j] = sum;
        }

    for(int i = 0; i < N; i++)
        for(int j = 0; j < N; j++)
        {
            float sum = 0.0f;

            for(int k = 0; k < N; k++)
                sum += AP[i][k] * A[j][k];

            sum += R * (K[i][0] * K[j][0] + K[i][1] * K[j][1] +
                        K[i][2] * K[j][2]);

            m_P[i][j] = sum;
        }

    inject(dx);

    return true;
}

/*
 * Heading: the tilt compensated magnetic heading against the estimate's.
 *
 * A small attitude error e in body axes turns the heading by its
 * component about the earth vertical, g . e, with g gravity in body
 * axes - so H = [ g'  0 ].
 *
 * The gain's attitude part is then confined to that same axis. The
 * covariance carries correlations between heading and tilt, and an
 * unrestricted gain would let a disturbed field move the horizon; this
 * way the magnetometer can only turn the estimate about the vertical,
 * and learn the bias. The Joseph form below is correct for any gain,
 * so the covariance stays right with the restricted one.
 */
bool AttitudeFilter::headingUpdate(const AttitudeVec3 &mag, float dt)
{
    float roll, pitch, heading;
    eulerFromQuaternion(&roll, &pitch, &heading);

    if(fabsf(pitch) > m_config.mag_max_pitch_deg)
        return false;

    const float innovation =
        rad(wrap180(magHeading(mag, roll, pitch) - heading));

    float g[3];
    gravityInBody(g);

    /*
     * The noise. The horizontal field shrinks as the pitch steepens, and
     * with it what tilt compensation has to work with. And tilt
     * compensation is only as good as the tilt: an error of e in it
     * moves the heading by up to tan(dip) e - two and a half times over
     * in central Europe. So the tilt's own uncertainty is added, and
     * when the tilt is in doubt - coasting through a turn - the
     * magnetometer is believed correspondingly less, rather than
     * teaching the filter a heading error as a gyro bias.
     */
    const AttitudeVec3 level = magLevel(mag, roll, pitch);
    const float horizontal = sqrtf(level.x * level.x + level.y * level.y);

    if(horizontal < 1.0e-3f)
        return false;

    const float tan_dip = fabsf(level.z) / horizontal;
    const float cos_pitch = cosf(rad(pitch));
    const float noise = rad(m_config.mag_noise_deg);
    const float R = noise * noise / (cos_pitch * cos_pitch) +
                    tan_dip * tan_dip * fmaxf(tiltVariance(), 0.0f);

    float PHt[N];

    for(int i = 0; i < N; i++)
        PHt[i] = m_P[i][0] * g[0] + m_P[i][1] * g[1] + m_P[i][2] * g[2];

    const float S = g[0] * PHt[0] + g[1] * PHt[1] + g[2] * PHt[2] + R;

    if(!(S > 0.0f))
        return false;

    if(innovation * innovation / S > m_config.mag_gate)
    {
        /* A field that passed the plausibility window and still keeps
         * disagreeing: the heading is what is wrong. */
        m_mag_reject_s += dt;

        if(m_mag_reject_s > m_config.reacquire_s)
        {
            openHeadingCovariance((float)M_PI);
            m_mag_reject_s = 0.0f;
        }

        return false;
    }

    m_mag_reject_s = 0.0f;

    float K[N];

    for(int i = 0; i < N; i++)
        K[i] = PHt[i] / S;

    /* Attitude correction about the vertical only. */
    const float about_vertical = g[0] * K[0] + g[1] * K[1] + g[2] * K[2];

    for(int i = 0; i < 3; i++)
        K[i] = g[i] * about_vertical;

    /*
     * And no bias learning while the tilt is unobserved. In a turn the
     * accelerometer is not used, so any tilt error goes uncorrected;
     * levelled with it, the field reads as a heading error that the
     * covariance has no way to attribute to the tilt, and learned as a
     * vertical gyro bias it would, in a bank, tilt the pitch in turn.
     */
    if(m_since_gravity_s > m_config.mag_bias_learning_gap_s ||
       !biasLearnable())
        for(int i = 3; i < N; i++)
            K[i] = 0.0f;

    float dx[N];

    for(int i = 0; i < N; i++)
        dx[i] = K[i] * innovation;

    /* Joseph form with H = [g 0]. */
    float A[N][N];

    for(int i = 0; i < N; i++)
        for(int j = 0; j < N; j++)
            A[i][j] = ((i == j) ? 1.0f : 0.0f) - ((j < 3) ? K[i] * g[j] : 0.0f);

    float AP[N][N];

    for(int i = 0; i < N; i++)
        for(int j = 0; j < N; j++)
        {
            float sum = 0.0f;

            for(int k = 0; k < N; k++)
                sum += A[i][k] * m_P[k][j];

            AP[i][j] = sum;
        }

    for(int i = 0; i < N; i++)
        for(int j = 0; j < N; j++)
        {
            float sum = 0.0f;

            for(int k = 0; k < N; k++)
                sum += AP[i][k] * A[j][k];

            m_P[i][j] = sum + R * K[i] * K[j];
        }

    inject(dx);

    return true;
}

void AttitudeFilter::update(const AttitudeVec3 &accel_g,
                            const AttitudeVec3 &gyro_dps,
                            const AttitudeVec3 *mag)
{
    /* Unsigned arithmetic, so this survives the clock rollover. */
    const uint32_t now_us = platformMicros();
    const uint32_t elapsed_us = now_us - m_last_us;
    const bool have_interval = m_have_sample;

    m_last_us = now_us;
    m_have_sample = true;

    updateWithDt(
        accel_g,
        gyro_dps,
        mag,
        have_interval ? (float)elapsed_us * 1.0e-6f : 0.0f);
}

void AttitudeFilter::updateWithDt(const AttitudeVec3 &accel_g,
                                  const AttitudeVec3 &gyro_dps,
                                  const AttitudeVec3 *mag,
                                  float dt_s)
{
    if(m_cal_request)
    {
        m_cal_request = false;
        m_bias_valid = false;
        clearCalibration();
    }

    /* The filter's own clock, from the intervals it is given, so that
     * replayed data behaves exactly like live data. */
    if(dt_s > 0.0f)
        m_time_s += clampf(dt_s, 0.0f, m_config.dt_max_s);

    const bool mag_ok = (mag != 0) && magPlausible(*mag);

    AttitudeSample out;

    /* ---- gyro bias not known yet: accelerometer only ------------------ */

    if(!m_bias_valid)
    {
        float accel_roll;
        float accel_pitch;
        accelAttitude(accel_g, &accel_roll, &accel_pitch);

        /*
         * Track the accelerometer attitude while calibrating, lightly
         * filtered because it is raw. This doubles as the initial
         * condition: by the time the bias is known the quaternion is
         * already close.
         */
        float roll, pitch, heading;
        eulerFromQuaternion(&roll, &pitch, &heading);

        const float alpha = m_config.accel_fallback_alpha;

        roll = wrap180(roll + alpha * wrap180(accel_roll - roll));
        pitch += alpha * (accel_pitch - pitch);

        setQuaternionFromEuler(roll, pitch, heading);

        /* May finish the calibration, and start the filter. */
        calibrateStep(gyro_dps, norm3(accel_g));

        out.status = m_bias_valid ? AttitudeStatus::Fused
                                  : AttitudeStatus::Calibrating;
        out.accel_trust = 1.0f;
    }
    else if(dt_s <= 0.0f)
    {
        /* First sample after a restart: no interval to integrate over. */
        out.status = AttitudeStatus::Fused;
        out.accel_trust = 1.0f;
    }
    else
    {
        const float dt = clampf(dt_s, m_config.dt_min_s, m_config.dt_max_s);

        const float rate[3] = {
            rad(gyro_dps.x) - m_bias[0],
            rad(gyro_dps.y) - m_bias[1],
            rad(gyro_dps.z) - m_bias[2]
        };

        predict(rate, dt);

        /* Smoothed rates, so that vibration counts neither as turning
         * nor as rotating. */
        const float smoothing = dt / (m_config.accel_turn_smoothing_s + dt);

        m_turn_rate += smoothing *
            (sqrtf(rate[1] * rate[1] + rate[2] * rate[2]) - m_turn_rate);
        m_rate += smoothing *
            (sqrtf(rate[0] * rate[0] + rate[1] * rate[1] +
                   rate[2] * rate[2]) - m_rate);

        float trust = 0.0f;
        const bool gravity_used = gravityUpdate(accel_g, rate, dt, &trust);

        m_since_gravity_s = gravity_used ? 0.0f : m_since_gravity_s + dt;

        if(mag_ok && headingUpdate(*mag, dt))
        {
            m_heading_seen = true;
            m_last_mag_s = m_time_s;
        }

        tidyCovariance();

        out.accel_trust = trust;
        out.status = (gravity_used && trust > m_config.accel_trust_min)
            ? AttitudeStatus::Fused
            : AttitudeStatus::GyroOnly;
    }

    /* ---- outputs ------------------------------------------------------ */

    eulerFromQuaternion(&out.roll_deg, &out.pitch_deg, &out.heading_deg);

    if(m_bias_valid)
    {
        out.tilt_sigma_deg = deg(sqrtf(fmaxf(tiltVariance(), 0.0f)));
        out.heading_sigma_deg = deg(sqrtf(fmaxf(headingVariance(), 0.0f)));
    }
    else
    {
        out.tilt_sigma_deg = m_config.reacquire_sigma_deg;
        out.heading_sigma_deg = 180.0f;
    }

    out.heading_valid =
        m_heading_seen &&
        (m_time_s - m_last_mag_s) * 1000.0f <= (float)m_config.mag_timeout_ms &&
        out.heading_sigma_deg < m_config.heading_valid_sigma_deg;

    publish(out);

    const AttitudeVec3 bias_dps = {
        deg(m_bias[0]), deg(m_bias[1]), deg(m_bias[2])
    };

    m_published_bias.publish(bias_dps);
}
