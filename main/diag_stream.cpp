#include "diag_stream.h"
#include "navigation.h"
#include "compass_screen.h"
#include "magnetometer.h"
#include "sensor_readings.h"
#include "web_log.h"

#include "baro_bmp280.h"
#include "mag_qmc5883.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "diag_stream";

static const uint32_t PERIOD_MS = 50;           /* 20 lines a second */
static const uint32_t HEADER_EVERY = 50;        /* lines: 2.5 s */
static const int TASK_STACK_BYTES = 4096;
static const int TASK_PRIORITY = 1;
static const int TASK_CORE = 0;

/* Bump when columns change meaning; adding columns needs no bump, since
 * readers go by the names in the header. */
static const int FORMAT_VERSION = 2;

/* Everything, raw: what the diagnostics screen shows. */
static const char *const DIAG_COLUMNS =
    "t_ms,"
    "ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,"
    "mx_uT,my_uT,mz_uT,mx_raw,my_raw,mz_raw,mag_ovf,"
    "mf_uT,mr_uT,md_uT,mag_cal,"
    "a0_V,a1_V,a2_V,a3_V,"
    "p_Pa,t_C,alt_m,vs_ms,"
    "roll_deg,pitch_deg,hdg_deg,turn_dps,slip_deg,"
    "e_imu,e_mag,e_adc";

/*
 * Everything the compass screen's heading is made of, in the order it
 * is computed:
 *
 *   mx..mz      raw field, sensor axes
 *   off_*, sc_* calibration, sensor axes: (raw - off) * sc
 *   cal_field   field strength measured while calibrating
 *   mf..md      corrected, in body axes
 *   roll, pitch the attitude used to level it
 *   lf, lr, ld  levelled: forward and right along the horizon, and down.
 *               The heading is atan2(-lr, lf).
 *   h_uT        horizontal strength, the part that points north
 *   incl_deg    dip below the horizon
 *   mag_hdg     heading from this one sample, before any filtering
 *   hdg         the filter's heading, what the screen shows
 *   bias_z      gyro z bias, which the magnetometer is what corrects
 */
static const char *const COMP_COLUMNS =
    "t_ms,state,status,"
    "mx_uT,my_uT,mz_uT,mag_ovf,"
    "cal,off_x,off_y,off_z,sc_x,sc_y,sc_z,cal_field_uT,"
    "mf_uT,mr_uT,md_uT,b_uT,"
    "roll_deg,pitch_deg,"
    "lf_uT,lr_uT,ld_uT,h_uT,incl_deg,"
    "mag_hdg_deg,hdg_deg,bias_z_dps,"
    "e_mag";

static const char *const SENTENCE[2] = { "DIAG", "COMP" };
static const char *const *const COLUMNS[2] = { &DIAG_COLUMNS, &COMP_COLUMNS };

/* The profile streaming, or -1 for none. Written by the LVGL task. */
static volatile int s_profile = -1;

/* Appends the checksum and newline, then writes the whole line in one
 * call so other tasks' log output cannot split it. A copy goes to the
 * web server (wifi_debug.h), for reading the stream over Wi-Fi. */
static void emit(char *line, size_t used, size_t size)
{
    uint8_t checksum = 0;

    for(size_t i = 1; i < used; i++)
        checksum ^= (uint8_t)line[i];

    if(used + 5 > size)
        return;

    const int length = snprintf(line + used, size - used, "*%02X\n",
                                checksum);

    fputs(line, stdout);
    web_stream_append(line, used + (size_t)length);
}

static void send_header(int profile)
{
    char line[512];

    const int used = snprintf(line, sizeof(line), "$%sH,%d,%s",
                              SENTENCE[profile], FORMAT_VERSION,
                              *COLUMNS[profile]);

    if(used > 0 && used < (int)sizeof(line))
        emit(line, (size_t)used, sizeof(line));
}

/* "%.4g"-ish but always valid for the reader: nan for missing. */
static int put(char *line, size_t used, size_t size, const char *format,
               float value, bool present = true)
{
    if(used >= size)
        return 0;

    if(!present || !isfinite(value))
        return snprintf(line + used, size - used, ",nan");

    return snprintf(line + used, size - used, format, value);
}

static void send_diagnostics(void)
{
    sensor_readings_t r;
    sensor_readings_get(&r);

    const NavigationSample nav = navigation_sample();
    const AttitudeSample &attitude = nav.attitude;
    const TurnSample &turn = nav.turn;
    const VerticalChannelSample &air = nav.vertical;

    float pressure = 0.0f;
    float temperature = 0.0f;
    const bool baro = navigation_barometer(&pressure, &temperature);

    AttitudeVec3 body = { 0.0f, 0.0f, 0.0f };

    if(r.mag_valid)
        body = magnetometer_to_body(r.mag);

    char line[512];
    size_t n = 0;
    const size_t size = sizeof(line);

    n += snprintf(line, size, "$DIAG,%lu",
                  (unsigned long)(esp_timer_get_time() / 1000));

    n += put(line, n, size, ",%.4f", r.imu.accel_g.x, r.imu_valid);
    n += put(line, n, size, ",%.4f", r.imu.accel_g.y, r.imu_valid);
    n += put(line, n, size, ",%.4f", r.imu.accel_g.z, r.imu_valid);
    n += put(line, n, size, ",%.3f", r.imu.gyro_dps.x, r.imu_valid);
    n += put(line, n, size, ",%.3f", r.imu.gyro_dps.y, r.imu_valid);
    n += put(line, n, size, ",%.3f", r.imu.gyro_dps.z, r.imu_valid);

    n += put(line, n, size, ",%.2f", r.mag.x_uT, r.mag_valid);
    n += put(line, n, size, ",%.2f", r.mag.y_uT, r.mag_valid);
    n += put(line, n, size, ",%.2f", r.mag.z_uT, r.mag_valid);
    n += put(line, n, size, ",%.0f", (float)r.mag.raw[0], r.mag_valid);
    n += put(line, n, size, ",%.0f", (float)r.mag.raw[1], r.mag_valid);
    n += put(line, n, size, ",%.0f", (float)r.mag.raw[2], r.mag_valid);
    n += put(line, n, size, ",%.0f", r.mag.overflow ? 1.0f : 0.0f, r.mag_valid);

    n += put(line, n, size, ",%.2f", body.x, r.mag_valid);
    n += put(line, n, size, ",%.2f", body.y, r.mag_valid);
    n += put(line, n, size, ",%.2f", body.z, r.mag_valid);
    n += put(line, n, size, ",%.0f",
             magnetometer_calibration().valid ? 1.0f : 0.0f);

    for(int channel = 0; channel < ADS1115_CHANNELS; channel++)
        n += put(line, n, size, ",%.5f", r.adc_volts[channel],
                 r.adc_valid[channel]);

    n += put(line, n, size, ",%.1f", pressure, baro);
    n += put(line, n, size, ",%.2f", temperature, baro);
    n += put(line, n, size, ",%.2f", air.altitude_m, baro && air.valid);
    n += put(line, n, size, ",%.2f", air.vertical_speed_ms, baro && air.valid);

    n += put(line, n, size, ",%.2f", attitude.roll_deg);
    n += put(line, n, size, ",%.2f", attitude.pitch_deg);
    n += put(line, n, size, ",%.1f", attitude.heading_deg,
             attitude.heading_valid);
    n += put(line, n, size, ",%.2f", turn.turn_rate_dps, turn.valid);
    n += put(line, n, size, ",%.2f", turn.slip_deg, turn.valid);

    if(n < size)
        n += snprintf(line + n, size - n, ",%u,%u,%u",
                      (unsigned)r.imu_errors, (unsigned)r.mag_errors,
                      (unsigned)r.adc_errors);

    if(n >= size)
    {
        ESP_LOGW(TAG, "line too long, dropped");
        return;
    }

    emit(line, n, size);
}


static float deg(float radians)
{
    return radians * 57.29577951f;
}

static void send_compass(void)
{
    sensor_readings_t r;
    sensor_readings_get(&r);

    const NavigationSample nav = navigation_sample();
    const AttitudeSample &attitude = nav.attitude;
    const MagCalibration cal = magnetometer_calibration();
    const bool mag = r.mag_valid;

    /* The same steps the IMU task takes for the filter: correct, map to
     * body axes, then level with the attitude. */
    AttitudeVec3 body = { 0.0f, 0.0f, 0.0f };
    AttitudeVec3 level = { 0.0f, 0.0f, 0.0f };

    if(mag)
    {
        body = magnetometer_to_body(r.mag);
        level = AttitudeFilter::magLevel(body, attitude.roll_deg,
                                         attitude.pitch_deg);
    }

    const float horizontal = sqrtf(level.x * level.x + level.y * level.y);

    char line[512];
    size_t n = 0;
    const size_t size = sizeof(line);

    n += snprintf(line, size, "$COMP,%lu,%d,%d",
                  (unsigned long)(esp_timer_get_time() / 1000),
                  (int)compass_state(attitude.heading_valid),
                  (int)attitude.status);

    n += put(line, n, size, ",%.2f", r.mag.x_uT, mag);
    n += put(line, n, size, ",%.2f", r.mag.y_uT, mag);
    n += put(line, n, size, ",%.2f", r.mag.z_uT, mag);
    n += put(line, n, size, ",%.0f", r.mag.overflow ? 1.0f : 0.0f, mag);


    n += put(line, n, size, ",%.0f", cal.valid ? 1.0f : 0.0f);
    n += put(line, n, size, ",%.2f", cal.offset.x, cal.valid);
    n += put(line, n, size, ",%.2f", cal.offset.y, cal.valid);
    n += put(line, n, size, ",%.2f", cal.offset.z, cal.valid);
    n += put(line, n, size, ",%.4f", cal.scale.x, cal.valid);
    n += put(line, n, size, ",%.4f", cal.scale.y, cal.valid);
    n += put(line, n, size, ",%.4f", cal.scale.z, cal.valid);
    n += put(line, n, size, ",%.2f", cal.field, cal.valid);

    n += put(line, n, size, ",%.2f", body.x, mag);
    n += put(line, n, size, ",%.2f", body.y, mag);
    n += put(line, n, size, ",%.2f", body.z, mag);
    n += put(line, n, size, ",%.2f",
             sqrtf(body.x * body.x + body.y * body.y + body.z * body.z), mag);

    n += put(line, n, size, ",%.2f", attitude.roll_deg);
    n += put(line, n, size, ",%.2f", attitude.pitch_deg);

    n += put(line, n, size, ",%.2f", level.x, mag);
    n += put(line, n, size, ",%.2f", level.y, mag);
    n += put(line, n, size, ",%.2f", level.z, mag);
    n += put(line, n, size, ",%.2f", horizontal, mag);
    n += put(line, n, size, ",%.1f", deg(atan2f(level.z, horizontal)), mag);

    n += put(line, n, size, ",%.1f",
             AttitudeFilter::magHeading(body, attitude.roll_deg,
                                        attitude.pitch_deg), mag);
    n += put(line, n, size, ",%.1f", attitude.heading_deg,
             attitude.heading_valid);
    n += put(line, n, size, ",%.3f", nav.gyro_bias_dps.z);

    if(n < size)
        n += snprintf(line + n, size - n, ",%u", (unsigned)r.mag_errors);

    if(n >= size)
    {
        ESP_LOGW(TAG, "line too long, dropped");
        return;
    }

    emit(line, n, size);
}

static void stream_task(void *argument)
{
    (void)argument;

    TickType_t last_wake = xTaskGetTickCount();
    uint32_t lines = 0;
    int previous = -1;

    while(true)
    {
        (void)xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_MS));

        const int profile = s_profile;

        if(profile < 0)
        {
            previous = -1;
            continue;
        }

        /* A header first, so a reader that is already listening learns
         * the columns straight away. */
        if(profile != previous || lines % HEADER_EVERY == 0)
            send_header(profile);

        previous = profile;
        lines++;

        if(profile == DIAG_STREAM_COMPASS)
            send_compass();
        else
            send_diagnostics();
    }
}

void diag_stream_start(void)
{
    const BaseType_t created = xTaskCreatePinnedToCore(
        stream_task, "diag_stream", TASK_STACK_BYTES, NULL, TASK_PRIORITY,
        NULL, TASK_CORE);

    if(created != pdPASS)
        ESP_LOGE(TAG, "could not start the stream task");
}

void diag_stream_begin(diag_stream_profile_t profile)
{
    if(s_profile != (int)profile)
        ESP_LOGI(TAG, "streaming %s", SENTENCE[profile]);

    s_profile = (int)profile;
}

void diag_stream_end(diag_stream_profile_t profile)
{
    if(s_profile != (int)profile)
        return;

    ESP_LOGI(TAG, "stopped");
    s_profile = -1;
}
