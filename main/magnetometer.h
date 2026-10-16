#pragma once

#include "mag_calibration.h"
#include "mag_qmc5883.h"

/*
 * The magnetometer as the instrument uses it: its calibration, and the
 * calibration procedure.
 *
 * Its axes are not the IMU's; mag_to_body() in mounting.h takes them to
 * body axes. The calibration is
 * kept in NVS and survives a restart; the maths lives in
 * components/ahrs (mag_calibration.h), where the host tests cover it,
 * and this is the storage and the threading around it.
 *
 * Threading: magnetometer_heading_sample() runs in the IMU task, every
 * other function in the LVGL task. The calibration is swapped under a
 * lock, so a change takes effect on the next sample.
 */

/* Load the saved calibration. Call once nvs_flash_init() has run. */
void magnetometer_init(void);

MagCalibration magnetometer_calibration(void);
void magnetometer_clear_calibration(void);

/*
 * From the IMU task, with every fresh sample.
 *
 * Returns true with the corrected field in body axes when it is fit to
 * steer the heading: calibrated, not saturated, and no calibration in
 * progress - during one the samples go to the calibrator instead, and
 * the filter keeps its heading on the gyro. down is gravity's direction
 * in body axes, from the accelerometer: the calibration needs it.
 */
bool magnetometer_heading_sample(const mag_sample_t &sample,
                                 const AttitudeVec3 &down,
                                 AttitudeVec3 *body);

/* Corrected (when calibrated) and in body axes, for display. */
AttitudeVec3 magnetometer_to_body(const mag_sample_t &sample);

/* ---- calibration procedure ---------------------------------------------- */

/* Start collecting. The unit should then be turned once round, level
 * as installed, slowly, away from anything made of steel - a compass
 * swing (mag_calibration.h). */
void magnetometer_calibration_start(void);

bool magnetometer_calibrating(void);

MagCalibrator::Progress magnetometer_calibration_progress(void);

/* Adopt and save the result. False, and still collecting, if
 * progress().ready is not yet true. */
bool magnetometer_calibration_finish(void);

/*
 * The swing so far, for the Wi-Fi debug page to draw: false, *out
 * untouched, if no calibration is in progress. Any task; the work is
 * done on a copy, outside the lock the IMU task needs. 6 KB: allocate
 * it, do not put it on a stack.
 */
bool magnetometer_calibration_snapshot(MagCalibrator::Snapshot *out);

/* Stop collecting and keep the previous calibration. */
void magnetometer_calibration_cancel(void);
