#include "alignment.h"
#include "navigation.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "nvs.h"

#include <math.h>
#include <string.h>

static const char *TAG = "alignment";

static const char *NVS_NAMESPACE = "alignment";
static const char *NVS_KEY = "matrix";

/* Bump when the layout of Stored changes; an old blob is then ignored. */
static const uint32_t STORED_VERSION = 1;

struct Stored
{
    uint32_t version;
    float m[3][3];
};

/*
 * Levelling: two seconds of samples at 100 Hz, all of them still.
 *
 * Still is judged by how much the readings move over those two seconds,
 * not by their size: the raw gyro carries its zero offset - 3.4 deg/s
 * on one unit, which a limit on the rate itself took for movement - and
 * the accelerometer its own scale error. A hand on the panel or a gust
 * rocking the aircraft moves them far more than this, and fails it, to
 * be tried again.
 */
static const uint32_t LEVEL_SAMPLES = 200;
static const float STILL_GYRO_SPREAD_DPS = 2.0f;
static const float STILL_ACCEL_SPREAD_G = 0.03f;

/* Anything this far from one g is not gravity alone, still or not. */
static const float GRAVITY_WINDOW_G = 0.2f;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

/* Guarded by s_lock. */
static MountingAlignment s_alignment;
static alignment_state_t s_state = ALIGNMENT_IDLE;
static uint32_t s_count = 0;
static AttitudeVec3 s_sum = { 0.0f, 0.0f, 0.0f };

/* Smallest and largest of each reading so far, x y z. */
static float s_gyro_min[3], s_gyro_max[3];
static float s_accel_min[3], s_accel_max[3];

static void spread_add(float *low, float *high, const AttitudeVec3 &v,
                       bool first)
{
    const float value[3] = { v.x, v.y, v.z };

    for(int i = 0; i < 3; i++)
    {
        if(first || value[i] < low[i]) low[i] = value[i];
        if(first || value[i] > high[i]) high[i] = value[i];
    }
}

static float spread_max(const float *low, const float *high)
{
    float worst = 0.0f;

    for(int i = 0; i < 3; i++)
        worst = fmaxf(worst, high[i] - low[i]);

    return worst;
}

static void save(const MountingAlignment &alignment)
{
    Stored stored;
    memset(&stored, 0, sizeof(stored));

    stored.version = STORED_VERSION;
    memcpy(stored.m, alignment.m, sizeof(stored.m));

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);

    if(err == ESP_OK)
    {
        err = nvs_set_blob(handle, NVS_KEY, &stored, sizeof(stored));

        if(err == ESP_OK)
            err = nvs_commit(handle);

        nvs_close(handle);
    }

    if(err != ESP_OK)
        ESP_LOGE(TAG, "could not save the alignment: %s",
                 esp_err_to_name(err));
}

static void log_tilt(const char *what, const MountingAlignment &alignment)
{
    float pitch, roll;
    mountingTilt(alignment, &pitch, &roll);

    ESP_LOGI(TAG, "%s: unit pitched %+.1f, rolled %+.1f deg", what, pitch,
             roll);
}

void alignment_init(void)
{
    nvs_handle_t handle;

    if(nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK)
    {
        ESP_LOGI(TAG, "not levelled");
        return;
    }

    Stored stored;
    size_t length = sizeof(stored);

    const esp_err_t err = nvs_get_blob(handle, NVS_KEY, &stored, &length);

    nvs_close(handle);

    if(err != ESP_OK || length != sizeof(stored) ||
       stored.version != STORED_VERSION)
    {
        ESP_LOGW(TAG, "saved alignment unreadable, not levelled");
        return;
    }

    MountingAlignment alignment;
    memcpy(alignment.m, stored.m, sizeof(alignment.m));

    portENTER_CRITICAL(&s_lock);
    s_alignment = alignment;
    portEXIT_CRITICAL(&s_lock);

    log_tilt("levelled", alignment);
}

MountingAlignment alignment_get(void)
{
    portENTER_CRITICAL(&s_lock);
    const MountingAlignment alignment = s_alignment;
    portEXIT_CRITICAL(&s_lock);

    return alignment;
}

AttitudeVec3 alignment_apply(const AttitudeVec3 &unit)
{
    return mountingApply(alignment_get(), unit);
}

AttitudeVec3 alignment_unapply(const AttitudeVec3 &aircraft)
{
    return mountingUnapply(alignment_get(), aircraft);
}

bool alignment_set(void)
{
    return !mountingIsIdentity(alignment_get());
}

void alignment_clear(void)
{
    const MountingAlignment none;

    portENTER_CRITICAL(&s_lock);
    s_alignment = none;
    s_state = ALIGNMENT_IDLE;
    portEXIT_CRITICAL(&s_lock);

    save(none);
    navigation_recalibrate_gyro();

    ESP_LOGI(TAG, "cleared");
}

void alignment_level_start(void)
{
    portENTER_CRITICAL(&s_lock);
    s_count = 0;
    s_sum = { 0.0f, 0.0f, 0.0f };
    s_state = ALIGNMENT_MEASURING;
    portEXIT_CRITICAL(&s_lock);

    ESP_LOGI(TAG, "levelling");
}

void alignment_record(const AttitudeVec3 &accel_g,
                      const AttitudeVec3 &gyro_dps)
{
    const float g = sqrtf(accel_g.x * accel_g.x + accel_g.y * accel_g.y +
                          accel_g.z * accel_g.z);

    portENTER_CRITICAL(&s_lock);

    if(s_state == ALIGNMENT_MEASURING && s_count < LEVEL_SAMPLES)
    {
        const bool first = (s_count == 0);

        spread_add(s_gyro_min, s_gyro_max, gyro_dps, first);
        spread_add(s_accel_min, s_accel_max, accel_g, first);

        if(spread_max(s_gyro_min, s_gyro_max) > STILL_GYRO_SPREAD_DPS ||
           spread_max(s_accel_min, s_accel_max) > STILL_ACCEL_SPREAD_G ||
           fabsf(g - 1.0f) > GRAVITY_WINDOW_G)
        {
            s_state = ALIGNMENT_MOVED;
        }
        else
        {
            s_sum.x += accel_g.x;
            s_sum.y += accel_g.y;
            s_sum.z += accel_g.z;
            s_count++;
        }
    }

    portEXIT_CRITICAL(&s_lock);
}

alignment_state_t alignment_level_poll(void)
{
    portENTER_CRITICAL(&s_lock);

    const alignment_state_t state = s_state;
    const bool measured =
        state == ALIGNMENT_MEASURING && s_count >= LEVEL_SAMPLES;
    const AttitudeVec3 sum = s_sum;
    const MountingAlignment current = s_alignment;

    portEXIT_CRITICAL(&s_lock);

    if(!measured)
        return state;

    /* Measured with the current alignment applied: this refines it. */
    MountingAlignment levelled;

    if(!mountingLevel(current, sum, ALIGNMENT_MAX_TILT_DEG, &levelled))
    {
        portENTER_CRITICAL(&s_lock);
        s_state = ALIGNMENT_TOO_STEEP;
        portEXIT_CRITICAL(&s_lock);

        ESP_LOGW(TAG, "not levelled: more than %.0f deg out",
                 ALIGNMENT_MAX_TILT_DEG);
        return ALIGNMENT_TOO_STEEP;
    }

    portENTER_CRITICAL(&s_lock);
    s_alignment = levelled;
    s_state = ALIGNMENT_DONE;
    portEXIT_CRITICAL(&s_lock);

    save(levelled);

    /* The attitude was the unit's; start it again from the aircraft's,
     * while everything is still. */
    navigation_recalibrate_gyro();

    log_tilt("levelled", levelled);

    return ALIGNMENT_DONE;
}
