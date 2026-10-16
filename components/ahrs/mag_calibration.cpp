#include "mag_calibration.h"

#include <math.h>

/* ---- calibration --------------------------------------------------------- */

AttitudeVec3 magCorrect(const MagCalibration &calibration,
                        const AttitudeVec3 &sensor)
{
    if(!calibration.valid)
        return sensor;

    AttitudeVec3 out;

    out.x = (sensor.x - calibration.offset.x) * calibration.scale.x;
    out.y = (sensor.y - calibration.offset.y) * calibration.scale.y;
    out.z = (sensor.z - calibration.offset.z) * calibration.scale.z;

    return out;
}

MagCalibrator::MagCalibrator()
{
    reset();
}

MagCalibrator::MagCalibrator(const MagCalibratorConfig &config)
    : m_config(config)
{
    reset();
}

void MagCalibrator::reset()
{
    m_samples = 0;
    m_stride = 1;
    m_count = 0;
    m_down_sum = { 0.0f, 0.0f, 0.0f };
    m_down_count = 0;
}

static const float DEG_PER_RAD = 57.29577951f;

static float dot(const AttitudeVec3 &a, const AttitudeVec3 &b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static AttitudeVec3 scaled(const AttitudeVec3 &v, float k)
{
    AttitudeVec3 out = { v.x * k, v.y * k, v.z * k };
    return out;
}

static float component(const AttitudeVec3 &v, int axis)
{
    return (axis == 0) ? v.x : (axis == 1) ? v.y : v.z;
}

static void set_component(AttitudeVec3 *v, int axis, float value)
{
    if(axis == 0) v->x = value;
    else if(axis == 1) v->y = value;
    else v->z = value;
}

void MagCalibrator::add(const AttitudeVec3 &sensor, const AttitudeVec3 &down)
{
    const float length = sqrtf(dot(down, down));

    if(length < 0.5f)
        return;

    m_down_sum.x += down.x / length;
    m_down_sum.y += down.y / length;
    m_down_sum.z += down.z / length;
    m_down_count++;

    /*
     * Every m_stride-th sample is kept. When the store fills, every
     * other one is dropped and the stride doubles, so however long the
     * swing it is all represented, evenly.
     */
    if((m_samples++ % m_stride) != 0)
        return;

    if(m_count == MAX_POINTS)
    {
        for(int i = 0; i < MAX_POINTS / 2; i++)
            m_points[i] = m_points[2 * i];

        m_count = MAX_POINTS / 2;
        m_stride *= 2;
    }

    m_points[m_count++] = sensor;
}

/* Solves the 4x4 system m x = b in place; false if singular. */
static bool solve4(float m[4][4], float b[4], float x[4])
{
    for(int col = 0; col < 4; col++)
    {
        int pivot = col;

        for(int row = col + 1; row < 4; row++)
            if(fabsf(m[row][col]) > fabsf(m[pivot][col]))
                pivot = row;

        if(fabsf(m[pivot][col]) < 1.0e-12f)
            return false;

        if(pivot != col)
        {
            for(int k = 0; k < 4; k++)
            {
                const float t = m[col][k];
                m[col][k] = m[pivot][k];
                m[pivot][k] = t;
            }

            const float t = b[col];
            b[col] = b[pivot];
            b[pivot] = t;
        }

        for(int row = col + 1; row < 4; row++)
        {
            const float f = m[row][col] / m[col][col];

            for(int k = col; k < 4; k++)
                m[row][k] -= f * m[col][k];

            b[row] -= f * b[col];
        }
    }

    for(int row = 3; row >= 0; row--)
    {
        float sum = b[row];

        for(int k = row + 1; k < 4; k++)
            sum -= m[row][k] * x[k];

        x[row] = sum / m[row][row];
    }

    return true;
}

/*
 * The fit, over the points kept so far.
 *
 * Each point is taken into the horizontal plane - two axes square to
 * the mean of gravity - and an axis-aligned ellipse
 *
 *   A u^2 + B v^2 + C u + D v = 1
 *
 * fitted by least squares: its centre is the hard iron in that plane,
 * its two semi-axes the gain on each of the two horizontal sensor axes.
 * The plane's axes are taken from the sensor's own, so that they are
 * the sensor's axes when the unit is level and the gains belong to
 * them.
 */
MagCalibrator::Fit MagCalibrator::fit() const
{
    Fit out;

    if(m_count < 8 || m_down_count == 0)
        return out;

    const float down_length = sqrtf(dot(m_down_sum, m_down_sum));

    if(down_length < 1.0e-6f)
        return out;

    out.down = scaled(m_down_sum, 1.0f / down_length);

    /* Spread of gravity over the swing: the mean of unit vectors
     * shortens as they fan out. */
    const float mean_length = down_length / (float)m_down_count;
    out.tilt_spread_deg =
        acosf(fminf(mean_length, 1.0f)) * DEG_PER_RAD;

    /* The vertical sensor axis, and the first of the other two. */
    out.vertical_axis = 0;

    for(int axis = 1; axis < 3; axis++)
        if(fabsf(component(out.down, axis)) >
           fabsf(component(out.down, out.vertical_axis)))
            out.vertical_axis = axis;

    const int first = (out.vertical_axis + 1) % 3;

    AttitudeVec3 e = { 0.0f, 0.0f, 0.0f };
    set_component(&e, first, 1.0f);

    /* u: that axis, made square to down; v completes the plane. */
    const AttitudeVec3 u_raw = {
        e.x - out.down.x * dot(e, out.down),
        e.y - out.down.y * dot(e, out.down),
        e.z - out.down.z * dot(e, out.down)
    };

    out.u = scaled(u_raw, 1.0f / sqrtf(dot(u_raw, u_raw)));
    out.v = {
        out.down.y * out.u.z - out.down.z * out.u.y,
        out.down.z * out.u.x - out.down.x * out.u.z,
        out.down.x * out.u.y - out.down.y * out.u.x
    };

    /*
     * Least squares. Coordinates are taken from the mean point, which
     * keeps the sums well conditioned in single precision.
     */
    float mean_u = 0.0f, mean_v = 0.0f, mean_down = 0.0f;

    for(int i = 0; i < m_count; i++)
    {
        mean_u += dot(m_points[i], out.u);
        mean_v += dot(m_points[i], out.v);
        mean_down += dot(m_points[i], out.down);
    }

    mean_u /= (float)m_count;
    mean_v /= (float)m_count;
    out.along_down = mean_down / (float)m_count;

    float N[4][4] = {};
    float r[4] = {};

    for(int i = 0; i < m_count; i++)
    {
        const float pu = dot(m_points[i], out.u) - mean_u;
        const float pv = dot(m_points[i], out.v) - mean_v;
        const float row[4] = { pu * pu, pv * pv, pu, pv };

        for(int j = 0; j < 4; j++)
        {
            for(int k = 0; k < 4; k++)
                N[j][k] += row[j] * row[k];

            r[j] += row[j];
        }
    }

    float q[4];

    if(!solve4(N, r, q) || q[0] <= 0.0f || q[1] <= 0.0f)
        return out;

    const float cu = -q[2] / (2.0f * q[0]);
    const float cv = -q[3] / (2.0f * q[1]);
    const float g = 1.0f + q[0] * cu * cu + q[1] * cv * cv;

    if(g <= 0.0f)
        return out;

    out.centre_u = cu + mean_u;
    out.centre_v = cv + mean_v;
    out.axis_u = sqrtf(g / q[0]);
    out.axis_v = sqrtf(g / q[1]);

    /* Coverage, in 10 degree steps round the ellipse, and how far the
     * points lie off it. */
    bool bins[36] = {};
    float error_sum = 0.0f;

    for(int i = 0; i < m_count; i++)
    {
        const float nu =
            (dot(m_points[i], out.u) - out.centre_u) / out.axis_u;
        const float nv =
            (dot(m_points[i], out.v) - out.centre_v) / out.axis_v;

        const float radius = sqrtf(nu * nu + nv * nv);
        error_sum += (radius - 1.0f) * (radius - 1.0f);

        float angle = atan2f(nv, nu) * DEG_PER_RAD;

        if(angle < 0.0f)
            angle += 360.0f;

        int bin = (int)(angle / 10.0f);

        if(bin > 35)
            bin = 35;

        bins[bin] = true;
    }

    int covered = 0;

    for(int i = 0; i < 36; i++)
        if(bins[i])
            covered++;

    out.coverage = (float)covered / 36.0f;
    out.fit_error = sqrtf(error_sum / (float)m_count);
    out.ok = true;

    return out;
}

MagCalibrator::Progress MagCalibrator::progress() const
{
    Progress progress;

    progress.samples = m_samples;

    const Fit f = fit();

    if(!f.ok)
        return progress;

    progress.coverage = f.coverage;
    progress.tilt_spread_deg = f.tilt_spread_deg;
    progress.radius = sqrtf(f.axis_u * f.axis_v);
    progress.fit_error = f.fit_error;

    progress.ready =
        m_samples >= m_config.min_samples &&
        f.coverage * 360.0f >= m_config.min_coverage_deg &&
        f.tilt_spread_deg <= m_config.max_tilt_spread_deg &&
        progress.radius >= m_config.min_radius &&
        f.fit_error <= m_config.max_fit_error;

    return progress;
}

bool MagCalibrator::result(MagCalibration *out) const
{
    if(out == 0 || !progress().ready)
        return false;

    const Fit f = fit();

    if(!f.ok)
        return false;

    /*
     * The horizontal field is the geometric mean of the semi-axes: the
     * gains of the two axes taken to average one. The vertical axis'
     * gain cannot be seen and is taken as that average too.
     */
    const float dip = m_config.dip_deg / DEG_PER_RAD;
    const float horizontal = sqrtf(f.axis_u * f.axis_v);
    const float field = horizontal / cosf(dip);

    /* The unseen offset along down: what was measured there, less the
     * field the dip says should be there. */
    const float offset_down = f.along_down - horizontal * tanf(dip);

    const AttitudeVec3 offset = {
        f.u.x * f.centre_u + f.v.x * f.centre_v + f.down.x * offset_down,
        f.u.y * f.centre_u + f.v.y * f.centre_v + f.down.y * offset_down,
        f.u.z * f.centre_u + f.v.z * f.centre_v + f.down.z * offset_down
    };

    const float normalise = m_config.nominal_field / field;
    const int first = (f.vertical_axis + 1) % 3;
    const int second = (f.vertical_axis + 2) % 3;

    out->valid = true;
    out->offset = offset;
    set_component(&out->scale, f.vertical_axis, normalise);
    set_component(&out->scale, first, normalise * horizontal / f.axis_u);
    set_component(&out->scale, second, normalise * horizontal / f.axis_v);
    out->field = field;

    return true;
}

void MagCalibrator::snapshot(Snapshot *out) const
{
    out->progress = progress();
    out->count = 0;
    out->fitted = false;

    const Fit f = fit();

    if(!f.ok)
        return;

    out->fitted = true;
    out->centre_u = f.centre_u;
    out->centre_v = f.centre_v;
    out->axis_u = f.axis_u;
    out->axis_v = f.axis_v;

    for(int i = 0; i < m_count; i++)
    {
        out->u[i] = dot(m_points[i], f.u);
        out->v[i] = dot(m_points[i], f.v);
    }

    out->count = m_count;
}
