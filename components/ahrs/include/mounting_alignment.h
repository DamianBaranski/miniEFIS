#pragma once

#include "attitude_filter.h"

/*
 * Mounting alignment - the unit as installed against the aircraft.
 *
 * The axis mapping (main/mounting.h) assumes the unit sits square in
 * the panel: display upright, face square to the line of flight. A
 * real panel is seldom quite that - raked back a few degrees, or the
 * unit a little rotated in its hole - and the horizon then shows that
 * error as a permanent pitch or bank with the aircraft level.
 *
 * Levelling measures it. With the aircraft level and still, gravity
 * should lie straight down the body z axis; whatever angle it actually
 * makes is the installation's. The alignment is the smallest rotation
 * that takes the measured direction onto body z, applied to every
 * sensor - accelerometer, gyro and magnetometer alike - so the attitude,
 * the turn coordinator and the heading all see the aircraft's axes, not
 * the unit's.
 *
 * Only tilt can be measured this way. A unit turned in yaw - its face
 * not square to the line of flight - is invisible to gravity and is not
 * corrected.
 *
 * Portable, like the rest of this component: no ESP-IDF dependency.
 */

struct MountingAlignment
{
    /* Unit axes (as mapped) to aircraft axes. Identity: none. */
    float m[3][3] = {
        { 1.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f },
    };
};

/* Unit axes to aircraft axes. */
AttitudeVec3 mountingApply(const MountingAlignment &alignment,
                           const AttitudeVec3 &unit);

/* Aircraft axes back to unit axes. */
AttitudeVec3 mountingUnapply(const MountingAlignment &alignment,
                             const AttitudeVec3 &aircraft);

/*
 * Level against gravity measured with the aircraft level: down is the
 * accelerometer's direction of gravity with the current alignment
 * already applied, so levelling again refines rather than starts over.
 *
 * False, *out untouched, if gravity is too weak to be gravity, or if
 * the result would tilt the unit more than max_tilt_deg against the
 * aircraft - more than that is a unit not mounted as a panel gauge, or
 * an aircraft that is not level.
 */
bool mountingLevel(const MountingAlignment &current,
                   const AttitudeVec3 &down,
                   float max_tilt_deg,
                   MountingAlignment *out);

/*
 * How the unit sits against the aircraft, degrees: the pitch and roll
 * it would show with the aircraft level, were it not aligned.
 */
void mountingTilt(const MountingAlignment &alignment,
                  float *pitch_deg, float *roll_deg);

bool mountingIsIdentity(const MountingAlignment &alignment);
