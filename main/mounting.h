#pragma once

#include "alignment.h"
#include "attitude_filter.h"

/*
 * Sensor axes to body axes, for the IMU and the magnetometer.
 *
 * The filter works in body axes: x forward, y right, z down, with the
 * gyro positive by the right hand rule about each axis, and the
 * accelerometer giving the direction gravity pulls - +1 g on whichever
 * axis currently points down.
 *
 * The sensor's axes, as the board has them:
 *
 *   sensor x = up the face of the display
 *   sensor y = screen right
 *   sensor z = into the panel, away from the viewer
 *
 * The instrument is meant to be mounted like a panel gauge: display
 * upright, facing the pilot. In that position the aircraft's nose points
 * into the panel, its right wing towards screen-right, and down is down
 * the face of the display:
 *
 *   body x (forward) = +sensor z        into the panel
 *   body y (right)   = +sensor y        screen right
 *   body z (down)    = -sensor x        screen down
 *
 * The accelerometer, like any, measures specific force: at rest it
 * reads +1 g on the axis pointing UP. The filter wants the opposite, the
 * direction of gravity, so its reading is mapped and then negated.
 *
 * An earlier version took the readings to mean x pointing down the face
 * and z out of the screen, with an accelerometer that reads +1 g down.
 * Sitting still the two are indistinguishable - which is how it got
 * that way - but they are 180 degrees apart about the right wing: roll
 * and yaw came out reversed. The horizon hid it, drawing roll mirrored
 * as well; the turn coordinator and the heading did not.
 *
 * This is the one place that knows how the sensors sit in the device.
 * To check it, watch the serial log or the screens:
 *
 *   upright, facing you       roll and pitch near zero
 *   flat on its back          pitch near -90, the nose pointing at the
 *                             ground
 *   top tilted towards you    pitch rises  (nose up)
 *   right edge lowered        roll rises   (right wing down)
 *   upright, turning right    heading rises, turn coordinator banks
 *                             right
 *
 * If a value moves the right way at first and then creeps back, that
 * axis' gyro sign is inverted relative to the accelerometer.
 *
 * A unit not quite square in the panel is levelled from the settings
 * screen, and that alignment applied on top of this mapping
 * (alignment.h): "body" here is always the aircraft's axes.
 */
static inline AttitudeVec3 sensor_to_body(float x, float y, float z)
{
    AttitudeVec3 body;

    body.x =  z;
    body.y =  y;
    body.z = -x;

    /* And from the unit as installed to the aircraft (alignment.h). */
    return alignment_apply(body);
}

/* A rate or a field: just the axes. */
template <typename Vector>
static inline AttitudeVec3 sensor_to_body(const Vector &sensor)
{
    return sensor_to_body(sensor.x, sensor.y, sensor.z);
}

/* The accelerometer: the axes, then specific force to gravity. */
template <typename Vector>
static inline AttitudeVec3 accel_to_body(const Vector &sensor)
{
    return sensor_to_body(-sensor.x, -sensor.y, -sensor.z);
}

/*
 * The magnetometer: its axes into the IMU's, then as above.
 *
 * It is not mounted the IMU's way round: x and y swapped and all three
 * reversed - a quarter turn and face down, against the IMU:
 *
 *   IMU x = -mag y        IMU y = -mag x        IMU z = -mag z
 *
 * Measured, not read off the board: tools/check_mag_axes.py turned the
 * unit about every axis and found the one mounting of the 24 under
 * which the field moves the way the gyro says it should (misfit 0.23,
 * the next best 1.16), with the field pointing down, as it does north
 * of the equator. Run it again if the board changes.
 */
template <typename Vector>
static inline AttitudeVec3 mag_to_body(const Vector &sensor)
{
    return sensor_to_body(-sensor.y, -sensor.x, -sensor.z);
}

/* The other way: body axes into the magnetometer's - for gravity's
 * direction, which the compass swing needs in the magnetometer's axes. */
static inline AttitudeVec3 body_to_mag(const AttitudeVec3 &aircraft)
{
    const AttitudeVec3 body = alignment_unapply(aircraft);

    /* mag_to_body() gives body = (-mag z, -mag x, +mag y). */
    AttitudeVec3 mag;

    mag.x = -body.y;
    mag.y =  body.z;
    mag.z = -body.x;

    return mag;
}
