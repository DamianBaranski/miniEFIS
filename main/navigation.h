#pragma once

#include "navigator.h"

/*
 * The instrument's estimates - attitude, heading, rate of turn and slip,
 * altitude and vertical speed - from one Navigator (components/ahrs),
 * and the only place any screen gets them from.
 *
 * The IMU task hands every sample to navigation_process_imu(); the
 * barometer is read from there too, at its own rate. Screens call
 * navigation_sample() and get all of it from the same instant.
 */

/*
 * One IMU sample, from the IMU task, in body axes: accelerometer in g,
 * gyro in deg/s. mag_uT is a fresh, calibrated magnetometer reading in
 * body axes, or NULL - each reading passed once only.
 */
void navigation_process_imu(const AttitudeVec3 &accel_g,
                            const AttitudeVec3 &gyro_dps,
                            const AttitudeVec3 *mag_uT,
                            float dt_s);

/* Everything, one consistent snapshot. Safe from any task.
 * vertical.valid is false with no barometer fitted. */
NavigationSample navigation_sample(void);

/* The barometer's last raw reading; false with none fitted. Any task. */
bool navigation_barometer(float *pressure_pa, float *temperature_c);

/*
 * Sea level pressure used to turn pressure into altitude - the QNH
 * setting on a real altimeter. Until it is set from a local report, the
 * standard atmosphere is assumed and the reading is pressure altitude.
 * Any task; the altitude moves at the next barometer reading, at once.
 */
void navigation_set_sea_level_pressure(float pascals);
float navigation_sea_level_pressure(void);

/* Re-measure the gyro bias; the unit must be held still. Any task. */
void navigation_recalibrate_gyro(void);
