#include "magnetometer.h"
#include "mounting.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "nvs.h"

#include <string.h>

static const char *TAG = "magnetometer";

static const char *NVS_NAMESPACE = "magnetometer";
static const char *NVS_KEY = "settings";

/* What an older firmware also kept, about the axis mapping it used to
 * detect. Removed when the calibration is next saved. */
static const char *NVS_KEY_DETECTED = "detected";

/* Bump when the layout of Stored changes; an old blob is then ignored. */
static const uint32_t STORED_VERSION = 1;

/*
 * The layout is the one firmware with a configurable axis mapping
 * wrote, so a calibration saved then is still read: it is in sensor
 * axes, and the mapping has nothing to do with it. The mapping's three
 * bytes are written as "as mounted" and otherwise ignored.
 */
struct Stored
{
    uint32_t version;
    uint8_t unused_axes[3];
    uint8_t calibrated;
    float offset[3];
    float scale[3];
    float field;
};

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

/* Guarded by s_lock: written in the LVGL task, read in the IMU task. */
static MagCalibration s_calibration;
static MagCalibratorConfig calibrator_config(void)
{
    MagCalibratorConfig config;
    config.dip_deg = (float)CONFIG_MAG_DIP_DEG;
    return config;
}

/* Built in place: it holds 8 KB of samples, too much to pass about. */
static MagCalibrator s_calibrator(calibrator_config());
static bool s_calibrating = false;

static AttitudeVec3 to_vec(const mag_sample_t &sample)
{
    AttitudeVec3 v = { sample.x_uT, sample.y_uT, sample.z_uT };
    return v;
}

static void save(void)
{
    Stored stored;
    memset(&stored, 0, sizeof(stored));

    portENTER_CRITICAL(&s_lock);
    stored.version = STORED_VERSION;
    stored.unused_axes[0] = 0;          /* +X, +Y, +Z: as mounted */
    stored.unused_axes[1] = 2;
    stored.unused_axes[2] = 4;
    stored.calibrated = s_calibration.valid ? 1 : 0;
    stored.offset[0] = s_calibration.offset.x;
    stored.offset[1] = s_calibration.offset.y;
    stored.offset[2] = s_calibration.offset.z;
    stored.scale[0] = s_calibration.scale.x;
    stored.scale[1] = s_calibration.scale.y;
    stored.scale[2] = s_calibration.scale.z;
    stored.field = s_calibration.field;
    portEXIT_CRITICAL(&s_lock);

    nvs_handle_t handle;

    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);

    if(err == ESP_OK)
    {
        err = nvs_set_blob(handle, NVS_KEY, &stored, sizeof(stored));

        /* Gone already is fine. */
        nvs_erase_key(handle, NVS_KEY_DETECTED);

        if(err == ESP_OK)
            err = nvs_commit(handle);

        nvs_close(handle);
    }

    if(err != ESP_OK)
        ESP_LOGE(TAG, "could not save the calibration: %s",
                 esp_err_to_name(err));
}

void magnetometer_init(void)
{
    nvs_handle_t handle;

    if(nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK)
    {
        ESP_LOGI(TAG, "not calibrated");
        return;
    }

    Stored stored;
    size_t length = sizeof(stored);

    const esp_err_t err = nvs_get_blob(handle, NVS_KEY, &stored, &length);

    nvs_close(handle);

    if(err != ESP_OK || length != sizeof(stored) ||
       stored.version != STORED_VERSION)
    {
        ESP_LOGW(TAG, "saved calibration unreadable, not calibrated");
        return;
    }

    s_calibration.valid = stored.calibrated != 0;
    s_calibration.offset = { stored.offset[0], stored.offset[1], stored.offset[2] };
    s_calibration.scale = { stored.scale[0], stored.scale[1], stored.scale[2] };
    s_calibration.field = stored.field;

    ESP_LOGI(TAG, "%s", s_calibration.valid ? "calibrated" : "not calibrated");
}

MagCalibration magnetometer_calibration(void)
{
    portENTER_CRITICAL(&s_lock);
    const MagCalibration calibration = s_calibration;
    portEXIT_CRITICAL(&s_lock);

    return calibration;
}

void magnetometer_clear_calibration(void)
{
    portENTER_CRITICAL(&s_lock);
    s_calibration = MagCalibration();
    portEXIT_CRITICAL(&s_lock);

    save();
}

bool magnetometer_heading_sample(const mag_sample_t &sample,
                                 const AttitudeVec3 &down,
                                 AttitudeVec3 *body)
{
    const AttitudeVec3 raw = to_vec(sample);

    portENTER_CRITICAL(&s_lock);

    if(s_calibrating)
    {
        if(!sample.overflow)
            s_calibrator.add(raw, body_to_mag(down));

        portEXIT_CRITICAL(&s_lock);
        return false;
    }

    const MagCalibration calibration = s_calibration;

    portEXIT_CRITICAL(&s_lock);

    /* An uncalibrated magnetometer gives a heading off by anything up
     * to 180 degrees; better none at all. */
    if(!calibration.valid || sample.overflow)
        return false;

    *body = mag_to_body(magCorrect(calibration, raw));

    return true;
}

AttitudeVec3 magnetometer_to_body(const mag_sample_t &sample)
{
    return mag_to_body(magCorrect(magnetometer_calibration(),
                                  to_vec(sample)));
}

void magnetometer_calibration_start(void)
{
    portENTER_CRITICAL(&s_lock);
    s_calibrator.reset();
    s_calibrating = true;
    portEXIT_CRITICAL(&s_lock);

    ESP_LOGI(TAG, "calibration started");
}

bool magnetometer_calibrating(void)
{
    return s_calibrating;
}

MagCalibrator::Progress magnetometer_calibration_progress(void)
{
    portENTER_CRITICAL(&s_lock);
    const MagCalibrator::Progress progress = s_calibrator.progress();
    portEXIT_CRITICAL(&s_lock);

    return progress;
}

bool magnetometer_calibration_finish(void)
{
    MagCalibration result;

    portENTER_CRITICAL(&s_lock);

    const bool ok = s_calibrator.result(&result);

    if(ok)
    {
        s_calibration = result;
        s_calibrating = false;
    }

    portEXIT_CRITICAL(&s_lock);

    if(!ok)
        return false;

    save();

    ESP_LOGI(TAG,
             "calibrated: offset %+.2f %+.2f %+.2f uT, "
             "scale %.3f %.3f %.3f, field %.1f uT",
             result.offset.x, result.offset.y, result.offset.z,
             result.scale.x, result.scale.y, result.scale.z, result.field);

    return true;
}

void magnetometer_calibration_cancel(void)
{
    portENTER_CRITICAL(&s_lock);
    s_calibrating = false;
    portEXIT_CRITICAL(&s_lock);

    ESP_LOGI(TAG, "calibration cancelled");
}

bool magnetometer_calibration_snapshot(MagCalibrator::Snapshot *out)
{
    if(!s_calibrating)
        return false;

    /*
     * Copied out - into PSRAM, it is 9 KB - and the fit done on the copy,
     * so the IMU task never waits for it. Without the lock, too: copying
     * 9 KB into PSRAM with interrupts off would hold up the display's
     * refill interrupt long enough to blink the screen. The IMU task may
     * add a point meanwhile, and the copy have one point torn; this is
     * for a picture, and the next one is whole.
     */
    MagCalibrator *copy = (MagCalibrator *)heap_caps_malloc(
        sizeof(MagCalibrator), MALLOC_CAP_SPIRAM);

    if(copy == NULL)
        return false;

    memcpy((void *)copy, (const void *)&s_calibrator, sizeof(MagCalibrator));
    copy->snapshot(out);

    heap_caps_free(copy);

    return true;
}
