#pragma once

#include "mounting_alignment.h"

/*
 * The unit's mounting alignment as the instrument uses it: kept in NVS,
 * applied to every sensor, and the levelling procedure that measures it.
 * The maths is in components/ahrs (mounting_alignment.h), where the host
 * tests cover it.
 *
 * mounting.h applies it when it maps sensor axes to body axes, so
 * everything downstream - the filter, every screen, the diagnostics -
 * sees the aircraft's axes.
 *
 * Threading: alignment_apply(), alignment_unapply() and
 * alignment_record() from any task, the IMU task in particular; the
 * rest from the LVGL task.
 */

/* Load the saved alignment. Call once nvs_flash_init() has run. */
void alignment_init(void);

AttitudeVec3 alignment_apply(const AttitudeVec3 &unit);
AttitudeVec3 alignment_unapply(const AttitudeVec3 &aircraft);

MountingAlignment alignment_get(void);
bool alignment_set(void);           /* other than the identity */
void alignment_clear(void);

/* ---- levelling ----------------------------------------------------------- */

typedef enum {
    ALIGNMENT_IDLE = 0,
    ALIGNMENT_MEASURING,
    ALIGNMENT_DONE,
    ALIGNMENT_MOVED,                /* not still: try again */
    ALIGNMENT_TOO_STEEP,            /* more than ALIGNMENT_MAX_TILT_DEG */
} alignment_state_t;

static const float ALIGNMENT_MAX_TILT_DEG = 20.0f;

/*
 * Start levelling: the aircraft level, the unit still. Gravity is
 * averaged for a couple of seconds; then the alignment is adopted and
 * saved, and the gyro re-measured so the horizon starts again from the
 * new level.
 */
void alignment_level_start(void);

/* Where levelling is; finishes it once measured. Call it from the LVGL
 * task while it is ALIGNMENT_MEASURING - every 100 ms or so. */
alignment_state_t alignment_level_poll(void);

/* From the IMU task, with every sample, in body axes as mapped - that
 * is, with the current alignment applied. */
void alignment_record(const AttitudeVec3 &accel_g,
                      const AttitudeVec3 &gyro_dps);
