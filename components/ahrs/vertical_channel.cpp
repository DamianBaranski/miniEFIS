#include "vertical_channel.h"

#include <math.h>
#include <string.h>

static float clampf(float value, float min_value, float max_value)
{
    if(value < min_value) return min_value;
    if(value > max_value) return max_value;
    return value;
}

VerticalChannel::VerticalChannel()
{
    reset();
}

VerticalChannel::VerticalChannel(const VerticalChannelConfig &config)
    : m_config(config)
{
    reset();
}

void VerticalChannel::setConfig(const VerticalChannelConfig &config)
{
    m_config = config;
}

void VerticalChannel::reset()
{
    memset(m_x, 0, sizeof(m_x));
    memset(m_P, 0, sizeof(m_P));
    m_initialised = false;
    m_reject_s = 0.0f;

    publish();
}

/*
 * Start over at a barometric altitude: level, no bias, the altitude as
 * good as one reading, the speed known only roughly.
 */
void VerticalChannel::restart(float altitude_m)
{
    m_x[0] = altitude_m;
    m_x[1] = 0.0f;
    m_x[2] = 0.0f;

    memset(m_P, 0, sizeof(m_P));

    const float baro = m_config.baro_noise_m;

    m_P[0][0] = baro * baro;
    m_P[1][1] = 1.0f;                   /* (1 m/s)^2 */
    m_P[2][2] = 0.25f;                  /* (0.5 m/s^2)^2 */

    m_initialised = true;
    m_reject_s = 0.0f;
}

void VerticalChannel::shiftAltitude(float delta_m)
{
    if(!m_initialised)
        return;

    m_x[0] += delta_m;
    publish();
}

void VerticalChannel::publish()
{
    VerticalChannelSample out;

    out.altitude_m = m_x[0];
    out.vertical_speed_ms = m_x[1];
    out.accel_bias_ms2 = m_x[2];
    out.altitude_sigma_m = sqrtf(fmaxf(m_P[0][0], 0.0f));
    out.vertical_speed_sigma_ms = sqrtf(fmaxf(m_P[1][1], 0.0f));
    out.valid = m_initialised;

    m_published.publish(out);
}

void VerticalChannel::update(float acceleration_up_ms2,
                             const float *altitude_m,
                             float dt_s)
{
    /* Nothing to report until the barometer has said where we are. */
    if(!m_initialised)
    {
        if(altitude_m == 0)
            return;

        restart(*altitude_m);
        publish();
        return;
    }

    if(dt_s <= 0.0f)
        return;

    const float dt = clampf(dt_s, m_config.dt_min_s, m_config.dt_max_s);

    /* ---- predict --------------------------------------------------------

     * x' = F x + B a, with the bias subtracted from the measured
     * acceleration:
     *
     *       | 1  dt  -dt^2/2 |
     *   F = | 0   1  -dt     |
     *       | 0   0   1      |
     */
    const float acceleration = acceleration_up_ms2 - m_x[2];

    m_x[0] += m_x[1] * dt + 0.5f * acceleration * dt * dt;
    m_x[1] += acceleration * dt;

    const float F[3][3] = {
        { 1.0f, dt,   -0.5f * dt * dt },
        { 0.0f, 1.0f, -dt },
        { 0.0f, 0.0f, 1.0f }
    };

    float FP[3][3];

    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            FP[i][j] = F[i][0] * m_P[0][j] + F[i][1] * m_P[1][j] +
                       F[i][2] * m_P[2][j];

    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            m_P[i][j] = FP[i][0] * F[j][0] + FP[i][1] * F[j][1] +
                        FP[i][2] * F[j][2];

    /* Acceleration noise as white noise into speed, integrated into
     * altitude; the bias as a random walk. */
    const float qa = m_config.accel_noise_ms2 * m_config.accel_noise_ms2;
    const float qb =
        m_config.accel_bias_walk_ms2 * m_config.accel_bias_walk_ms2;

    m_P[0][0] += qa * dt * dt * dt / 3.0f;
    m_P[0][1] += qa * dt * dt / 2.0f;
    m_P[1][0] += qa * dt * dt / 2.0f;
    m_P[1][1] += qa * dt;
    m_P[2][2] += qb * dt;

    /* ---- barometer ---------------------------------------------------- */

    if(altitude_m != 0)
    {
        const float innovation = *altitude_m - m_x[0];

        /* A step too large to be flight is a new reference, not an
         * error: integrating it would throw a false vertical speed. */
        if(fabsf(innovation) > m_config.altitude_jump_m)
        {
            restart(*altitude_m);
            publish();
            return;
        }

        const float R = m_config.baro_noise_m * m_config.baro_noise_m;
        const float S = m_P[0][0] + R;

        if(innovation * innovation / S > m_config.baro_gate)
        {
            /* Rejected - a glitch. If the barometer goes on disagreeing
             * it is right and the estimate is not: start again from it. */
            m_reject_s += dt;

            if(m_reject_s > m_config.baro_reacquire_s)
                restart(*altitude_m);
        }
        else
        {
            m_reject_s = 0.0f;

            /* H = [1 0 0]: K is P's first column over S. */
            const float K[3] = {
                m_P[0][0] / S, m_P[1][0] / S, m_P[2][0] / S
            };

            for(int i = 0; i < 3; i++)
                m_x[i] += K[i] * innovation;

            /* Joseph form: (I - KH) P (I - KH)' + K R K'. */
            float A[3][3];

            for(int i = 0; i < 3; i++)
                for(int j = 0; j < 3; j++)
                    A[i][j] = ((i == j) ? 1.0f : 0.0f) -
                              ((j == 0) ? K[i] : 0.0f);

            float AP[3][3];

            for(int i = 0; i < 3; i++)
                for(int j = 0; j < 3; j++)
                    AP[i][j] = A[i][0] * m_P[0][j] + A[i][1] * m_P[1][j] +
                               A[i][2] * m_P[2][j];

            for(int i = 0; i < 3; i++)
                for(int j = 0; j < 3; j++)
                    m_P[i][j] = AP[i][0] * A[j][0] + AP[i][1] * A[j][1] +
                                AP[i][2] * A[j][2] + K[i] * R * K[j];
        }
    }

    m_x[2] = clampf(m_x[2], -m_config.accel_bias_limit_ms2,
                    m_config.accel_bias_limit_ms2);

    /* Symmetric, whatever rounding did. */
    for(int i = 0; i < 3; i++)
        for(int j = i + 1; j < 3; j++)
        {
            const float mean = 0.5f * (m_P[i][j] + m_P[j][i]);
            m_P[i][j] = mean;
            m_P[j][i] = mean;
        }

    publish();
}
