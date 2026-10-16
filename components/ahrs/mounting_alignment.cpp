#include "mounting_alignment.h"

#include <math.h>

static const float DEG_PER_RAD = 57.29577951f;

AttitudeVec3 mountingApply(const MountingAlignment &alignment,
                           const AttitudeVec3 &unit)
{
    const float (*m)[3] = alignment.m;
    AttitudeVec3 out;

    out.x = m[0][0] * unit.x + m[0][1] * unit.y + m[0][2] * unit.z;
    out.y = m[1][0] * unit.x + m[1][1] * unit.y + m[1][2] * unit.z;
    out.z = m[2][0] * unit.x + m[2][1] * unit.y + m[2][2] * unit.z;

    return out;
}

/* A rotation: the inverse is the transpose. */
AttitudeVec3 mountingUnapply(const MountingAlignment &alignment,
                             const AttitudeVec3 &aircraft)
{
    const float (*m)[3] = alignment.m;
    AttitudeVec3 out;

    out.x = m[0][0] * aircraft.x + m[1][0] * aircraft.y + m[2][0] * aircraft.z;
    out.y = m[0][1] * aircraft.x + m[1][1] * aircraft.y + m[2][1] * aircraft.z;
    out.z = m[0][2] * aircraft.x + m[1][2] * aircraft.y + m[2][2] * aircraft.z;

    return out;
}

bool mountingLevel(const MountingAlignment &current,
                   const AttitudeVec3 &down,
                   float max_tilt_deg,
                   MountingAlignment *out)
{
    const float length =
        sqrtf(down.x * down.x + down.y * down.y + down.z * down.z);

    if(out == 0 || length < 0.5f)
        return false;

    const float ax = down.x / length;
    const float ay = down.y / length;
    const float az = down.z / length;

    /* The cosine of the largest tilt allowed: well short of 90 degrees,
     * which also keeps 1 + c below clear of zero. */
    const float min_cos = cosf(fminf(max_tilt_deg, 80.0f) / DEG_PER_RAD);

    if(az < min_cos)
        return false;

    /*
     * The smallest rotation taking a onto z (Rodrigues): about a x z,
     * v = (ay, -ax, 0), by the angle whose cosine is c = az,
     *
     *   R = I + [v]x + [v]x^2 / (1 + c)
     */
    const float vx = ay;
    const float vy = -ax;
    const float k = 1.0f / (1.0f + az);

    const float fix[3][3] = {
        { 1.0f - vy * vy * k,  vx * vy * k,         vy },
        { vx * vy * k,         1.0f - vx * vx * k, -vx },
        { -vy,                 vx,                  1.0f - (vx * vx + vy * vy) * k },
    };

    MountingAlignment result;

    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            result.m[i][j] = fix[i][0] * current.m[0][j] +
                             fix[i][1] * current.m[1][j] +
                             fix[i][2] * current.m[2][j];

    /* The tilt altogether, not just this step's. */
    if(result.m[2][2] < min_cos)
        return false;

    *out = result;
    return true;
}

void mountingTilt(const MountingAlignment &alignment,
                  float *pitch_deg, float *roll_deg)
{
    /* Gravity as the unit sees it with the aircraft level. */
    const AttitudeVec3 down = { 0.0f, 0.0f, 1.0f };
    const AttitudeVec3 unit = mountingUnapply(alignment, down);

    /* As AttitudeFilter::accelAttitude() reads it. */
    if(pitch_deg != 0)
        *pitch_deg = atan2f(-unit.x, sqrtf(unit.y * unit.y + unit.z * unit.z)) *
                     DEG_PER_RAD;

    if(roll_deg != 0)
        *roll_deg = atan2f(unit.y, unit.z) * DEG_PER_RAD;
}

bool mountingIsIdentity(const MountingAlignment &alignment)
{
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            if(fabsf(alignment.m[i][j] - ((i == j) ? 1.0f : 0.0f)) > 1.0e-6f)
                return false;

    return true;
}
