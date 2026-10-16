#pragma once

#include "attitude_filter.h"
#include <stdint.h>

/*
 * Magnetometer calibration - what stands between a raw magnetometer
 * reading and a heading, once the reading is in the right axes.
 *
 * Calibration removes what the installation does to the field:
 *
 *   hard iron - a constant offset from magnetised parts nearby and from
 *               the sensor's own zero error
 *   soft iron - a gain that differs between the sensor's axes
 *
 * It is a compass swing, as an aircraft's is done: the unit, level as
 * it is installed, turned once round about the vertical. Turned that
 * way the field it sees traces a circle in the horizontal plane - offset
 * by the hard iron, squashed into an ellipse by the soft iron - and
 * fitting that ellipse gives both, for the two axes that lie in the
 * plane. "Horizontal" is taken from the accelerometer, sample by
 * sample, so it need not be exactly level: an aircraft on its wheels
 * sits nose up, and the swing is still a turn about the true vertical.
 *
 * The third axis never moves in such a swing, so its offset cannot be
 * seen. It comes instead from the local magnetic dip - the field
 * points down into the earth at a known angle - set in
 * MagCalibratorConfig. Getting that a few degrees wrong costs nothing
 * level and a degree or two of heading in a steep bank.
 *
 * Calibration is done in sensor axes, before they are mapped to body
 * axes.
 *
 * Portable, like the rest of this component: no ESP-IDF dependency.
 */

struct MagCalibration
{
    bool valid = false;

    /* Applied as (raw - offset) * scale, in sensor axes. */
    AttitudeVec3 offset = { 0.0f, 0.0f, 0.0f };
    AttitudeVec3 scale = { 1.0f, 1.0f, 1.0f };

    /* Strength of the field measured during calibration, before
     * normalising - for display, and as a sanity check on the part. */
    float field = 0.0f;
};

AttitudeVec3 magCorrect(const MagCalibration &calibration,
                        const AttitudeVec3 &sensor);

struct MagCalibratorConfig
{
    /*
     * The corrected field is scaled to this strength, in uT, whatever
     * was measured.
     *
     * Only the direction of the field matters for heading, but the
     * filter's plausibility window (AttitudeConfig::mag_min_norm and
     * mag_max_norm) is in uT. Normalising makes that window a relative
     * test against the calibrated field, so it keeps working whatever
     * the local field strength is and however accurate the part's own
     * scale factor turns out to be.
     */
    float nominal_field = 50.0f;

    /*
     * Magnetic dip (inclination) where the unit is used, degrees,
     * positive with the field pointing down - north of the equator. 67
     * across Poland, 64-68 over most of Europe; a world magnetic model
     * gives it for anywhere. It sets the offset of the axis the swing
     * cannot see.
     */
    float dip_deg = 67.0f;

    /* Samples to collect before a result is offered at all. */
    uint32_t min_samples = 200;

    /* How much of the circle has to have been covered, degrees. */
    float min_coverage_deg = 330.0f;

    /* How far the unit may tilt about during the swing, degrees: a
     * swing is a turn about the vertical, not a wobble. */
    float max_tilt_spread_deg = 10.0f;

    /* Smallest believable horizontal field, uT: less is a sensor that
     * is not working, or one buried in steel. */
    float min_radius = 1.0f;

    /* Worst fit tolerated: how far the samples lie off the ellipse, as
     * a fraction of its size. More is a field being disturbed as the
     * unit turns - a hangar's steel, a car alongside. */
    float max_fit_error = 0.10f;
};

class MagCalibrator
{
public:
    struct Progress
    {
        uint32_t samples = 0;

        /* Of the circle, 0..1, in 10 degree steps. */
        float coverage = 0.0f;

        /* How much the unit has tilted about meanwhile, degrees. */
        float tilt_spread_deg = 0.0f;

        /* The horizontal field as fitted, uT, and how well it fits. */
        float radius = 0.0f;
        float fit_error = 0.0f;

        bool ready = false;
    };

    MagCalibrator();
    explicit MagCalibrator(const MagCalibratorConfig &config);

    void reset();

    /*
     * One sample, in sensor axes, uncorrected, with the direction of
     * gravity - down - in the same axes, from the accelerometer.
     */
    void add(const AttitudeVec3 &sensor, const AttitudeVec3 &down);

    Progress progress() const;

    /*
     * The calibration, if progress().ready. Returns false and leaves
     * *out alone otherwise.
     */
    bool result(MagCalibration *out) const;

    /* Kept, thinned out as the swing goes on: enough for the fit and
     * the coverage, whatever its length. */
    static const int MAX_POINTS = 720;

    /*
     * The swing so far, to draw: every point kept, in the horizontal
     * plane the fit works in (uT, u along the first horizontal sensor
     * axis), and the ellipse fitted to them. 6 KB - not for the stack.
     */
    struct Snapshot
    {
        Progress progress;
        bool fitted = false;
        float centre_u = 0.0f, centre_v = 0.0f;
        float axis_u = 0.0f, axis_v = 0.0f;
        int count = 0;
        float u[MAX_POINTS];
        float v[MAX_POINTS];
    };

    void snapshot(Snapshot *out) const;

private:
    struct Fit
    {
        bool ok = false;
        AttitudeVec3 down;              /* mean gravity, unit */
        AttitudeVec3 u, v;              /* the horizontal plane */
        int vertical_axis = 2;          /* the sensor axis nearest down */
        float centre_u = 0.0f, centre_v = 0.0f;
        float axis_u = 0.0f, axis_v = 0.0f;     /* semi-axes, uT */
        float along_down = 0.0f;        /* mean field along down */
        float coverage = 0.0f;
        float fit_error = 1.0f;
        float tilt_spread_deg = 0.0f;
    };

    Fit fit() const;

    MagCalibratorConfig m_config;
    uint32_t m_samples;
    uint32_t m_stride;
    int m_count;
    AttitudeVec3 m_points[MAX_POINTS];
    AttitudeVec3 m_down_sum;
    uint32_t m_down_count;
};
