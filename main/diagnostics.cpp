#include "diagnostics.h"
#include "altitude_widget.h"
#include "horizon_screen.h"
#include "magnetometer.h"
#include "mounting.h"
#include "navigation.h"
#include "baro_bmp280.h"
#include "display_st7701.h"
#include "lvgl_port.h"
#include "sensor_readings.h"
#include "web_log.h"

#include "esp_timer.h"
#include <stdio.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "horizon";

/* $TEL every tick, $SYS every second, headroom to the log every ten. */
static const uint32_t TICK_MS = 100;
static const uint32_t SYS_EVERY = 10;
static const uint32_t HEADROOM_EVERY = 100;

/* Each sentence's column names, repeated this often so that a reader
 * joining part way learns them. */
static const uint32_t HEADER_EVERY = 50;

static const int TASK_STACK_BYTES = 4096;
static const int TASK_PRIORITY = 1;         /* below the IMU and LVGL tasks */

/* Bump when a column changes meaning; added columns need no bump, since
 * readers go by the names in the header. */
static const int FORMAT_VERSION = 1;

static const char *const TEL_COLUMNS =
    "t_ms,roll_deg,pitch_deg,hdg_deg,status,accel_trust,tilt_sd_deg,"
    "hdg_sd_deg,p_dps,q_dps,r_dps,bias_x_dps,bias_y_dps,bias_z_dps,"
    "ax_g,ay_g,az_g,turn_dps,slip_deg,alt_m,vs_ms,alt_sd_m,vs_sd_ms,"
    "vbias_ms2,p_pa,t_c,mf_ut,mr_ut,md_ut,b_ut";

static const char *const SYS_COLUMNS =
    "t_ms,fps,panel_hz,draw_ms,bg_ms,symbols_ms,widget_ms,switch_ms,"
    "frame_ms,frames_cut,"
    "imu_n,imu_err,dt_min_ms,dt_max_ms,roll_pp_deg,pitch_pp_deg,"
    "a_min_g,a_max_g,w_max_dps,mag_err,adc_err,heap_kb,heap_min_kb";

struct Stats
{
    uint32_t samples;
    uint32_t read_errors;

    float roll_min, roll_max;
    float pitch_min, pitch_max;
    float accel_norm_min, accel_norm_max;
    float rate_max;
    uint32_t interval_min_us, interval_max_us;

    imu_sample_t last;
};

/* Writes found beside the framebuffers - see display_guard_check(). */
static volatile uint32_t s_guard_hits = 0;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static Stats s_stats;

static void stats_clear(Stats *stats)
{
    stats->samples = 0;
    stats->read_errors = 0;
    stats->roll_min = 1.0e9f;
    stats->roll_max = -1.0e9f;
    stats->pitch_min = 1.0e9f;
    stats->pitch_max = -1.0e9f;
    stats->accel_norm_min = 1.0e9f;
    stats->accel_norm_max = -1.0e9f;
    stats->rate_max = 0.0f;
    stats->interval_min_us = 0xFFFFFFFFu;
    stats->interval_max_us = 0;
}

static float vector_norm(const imu_vec3_t &v)
{
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

void diagnostics_record_imu(const imu_sample_t *sample,
                            esp_err_t read_status,
                            uint32_t interval_us)
{
    float roll = 0.0f;
    float pitch = 0.0f;

    /* Outside the critical section: this reads the filter's own
     * lock-free snapshot and must not hold a spinlock while doing so. */
    if(read_status == ESP_OK)
    {
        const AttitudeSample attitude = navigation_sample().attitude;

        roll = attitude.roll_deg;
        pitch = attitude.pitch_deg;
    }

    const float accel_norm =
        (read_status == ESP_OK) ? vector_norm(sample->accel_g) : 0.0f;
    const float rate =
        (read_status == ESP_OK) ? vector_norm(sample->gyro_dps) : 0.0f;

    portENTER_CRITICAL(&s_lock);

    if(read_status != ESP_OK)
    {
        s_stats.read_errors++;
        portEXIT_CRITICAL(&s_lock);
        return;
    }

    s_stats.samples++;
    s_stats.last = *sample;

    if(roll < s_stats.roll_min) s_stats.roll_min = roll;
    if(roll > s_stats.roll_max) s_stats.roll_max = roll;
    if(pitch < s_stats.pitch_min) s_stats.pitch_min = pitch;
    if(pitch > s_stats.pitch_max) s_stats.pitch_max = pitch;

    if(accel_norm < s_stats.accel_norm_min) s_stats.accel_norm_min = accel_norm;
    if(accel_norm > s_stats.accel_norm_max) s_stats.accel_norm_max = accel_norm;

    if(rate > s_stats.rate_max) s_stats.rate_max = rate;

    if(interval_us < s_stats.interval_min_us)
        s_stats.interval_min_us = interval_us;
    if(interval_us > s_stats.interval_max_us)
        s_stats.interval_max_us = interval_us;

    portEXIT_CRITICAL(&s_lock);
}

/* ---- telemetry ----------------------------------------------------------- */

/*
 * A sentence as the diag stream writes them - $NAME,values*XX, XX the
 * XOR of everything between $ and * - into the telemetry ring, for the
 * web page. Not to the serial console: ten lines a second of it is what
 * this replaced in the log.
 */
static void emit(char *line, int used, size_t size)
{
    if(used <= 0 || (size_t)used + 5 > size)
        return;

    uint8_t checksum = 0;

    for(int i = 1; i < used; i++)
        checksum ^= (uint8_t)line[i];

    used += snprintf(line + used, size - used, "*%02X\n", checksum);
    web_telemetry_append(line, (size_t)used);
}

static void send_header(const char *name, const char *columns)
{
    char line[512];
    const int used = snprintf(line, sizeof(line), "$%sH,%d,%s", name,
                              FORMAT_VERSION, columns);

    if(used < (int)sizeof(line))
        emit(line, used, sizeof(line));
}

/* ",value", or ",nan" for one that is missing. */
static int put(char *line, int used, size_t size, const char *format,
               float value, bool present = true)
{
    if(used < 0 || (size_t)used >= size)
        return 0;

    if(!present || !isfinite(value))
        return snprintf(line + used, size - used, ",nan");

    return snprintf(line + used, size - used, format, value);
}

/* Every estimate and sensor, now. */
static void send_telemetry(void)
{
    const NavigationSample nav = navigation_sample();
    const AttitudeSample &a = nav.attitude;
    const VerticalChannelSample &v = nav.vertical;

    sensor_readings_t r;
    sensor_readings_get(&r);

    const AttitudeVec3 accel = accel_to_body(r.imu.accel_g);
    const AttitudeVec3 mag = magnetometer_to_body(r.mag);
    const bool calibrated = magnetometer_calibration().valid;

    float pressure = 0.0f;
    float temperature = 0.0f;
    const bool baro = navigation_barometer(&pressure, &temperature);

    char line[512];
    const size_t size = sizeof(line);
    int n = snprintf(line, size, "$TEL,%lu",
                     (unsigned long)(esp_timer_get_time() / 1000));

    n += put(line, n, size, ",%.2f", a.roll_deg);
    n += put(line, n, size, ",%.2f", a.pitch_deg);
    n += put(line, n, size, ",%.1f", a.heading_deg, a.heading_valid);
    n += put(line, n, size, ",%.0f", (float)(int)a.status);
    n += put(line, n, size, ",%.2f", a.accel_trust);
    n += put(line, n, size, ",%.2f", a.tilt_sigma_deg);
    n += put(line, n, size, ",%.1f", a.heading_sigma_deg);
    n += put(line, n, size, ",%.2f", nav.rate_dps.x);
    n += put(line, n, size, ",%.2f", nav.rate_dps.y);
    n += put(line, n, size, ",%.2f", nav.rate_dps.z);
    n += put(line, n, size, ",%.3f", nav.gyro_bias_dps.x);
    n += put(line, n, size, ",%.3f", nav.gyro_bias_dps.y);
    n += put(line, n, size, ",%.3f", nav.gyro_bias_dps.z);
    n += put(line, n, size, ",%.3f", accel.x, r.imu_valid);
    n += put(line, n, size, ",%.3f", accel.y, r.imu_valid);
    n += put(line, n, size, ",%.3f", accel.z, r.imu_valid);
    n += put(line, n, size, ",%.2f", nav.turn.turn_rate_dps, nav.turn.valid);
    n += put(line, n, size, ",%.2f", nav.turn.slip_deg, nav.turn.valid);
    n += put(line, n, size, ",%.2f", v.altitude_m, v.valid);
    n += put(line, n, size, ",%.2f", v.vertical_speed_ms, v.valid);
    n += put(line, n, size, ",%.2f", v.altitude_sigma_m, v.valid);
    n += put(line, n, size, ",%.2f", v.vertical_speed_sigma_ms, v.valid);
    n += put(line, n, size, ",%.2f", v.accel_bias_ms2, v.valid);
    n += put(line, n, size, ",%.1f", pressure, baro);
    n += put(line, n, size, ",%.1f", temperature, baro);
    n += put(line, n, size, ",%.1f", mag.x, r.mag_valid && calibrated);
    n += put(line, n, size, ",%.1f", mag.y, r.mag_valid && calibrated);
    n += put(line, n, size, ",%.1f", mag.z, r.mag_valid && calibrated);
    n += put(line, n, size, ",%.1f",
             sqrtf(mag.x * mag.x + mag.y * mag.y + mag.z * mag.z),
             r.mag_valid && calibrated);

    emit(line, n, size);
}

/* The IMU's timing and steadiness over the last second, and the
 * display's - so a jerky screen can be told from a jumpy attitude. */
static void send_system(const Stats &stats, uint32_t period_ms)
{
    lvgl_port_stats_t display;
    lvgl_port_take_stats(&display);

    sensor_readings_t r;
    sensor_readings_get(&r);

    const bool imu = stats.samples > 0;
    const float per_second = 1000.0f / (float)period_ms;

    char line[512];
    const size_t size = sizeof(line);
    int n = snprintf(line, size, "$SYS,%lu",
                     (unsigned long)(esp_timer_get_time() / 1000));

    n += put(line, n, size, ",%.1f", (float)display.frames * per_second);
    n += put(line, n, size, ",%.1f", (float)display.vsyncs * per_second);
    /* The horizon's drawing, by part - zero on other screens. */
    uint32_t custom_us = 0, ground_us = 0, symbols_us = 0;
    uint32_t background_us = 0, screen_us = 0;
    horizon_screen_render_cost(&custom_us, &ground_us, &symbols_us,
                               &background_us, &screen_us);

    n += put(line, n, size, ",%.1f", display.draw_max_us / 1000.0f);
    n += put(line, n, size, ",%.1f", ground_us / 1000.0f);
    n += put(line, n, size, ",%.1f", symbols_us / 1000.0f);
    n += put(line, n, size, ",%.1f", altitude_widget_render_cost() / 1000.0f);
    n += put(line, n, size, ",%.1f", display.flush_wait_max_us / 1000.0f);
    n += put(line, n, size, ",%.1f", display.handler_max_us / 1000.0f);
    n += put(line, n, size, ",%.0f", (float)display.frames_cut);
    n += put(line, n, size, ",%.0f", (float)stats.samples);
    n += put(line, n, size, ",%.0f", (float)stats.read_errors);
    n += put(line, n, size, ",%.2f", stats.interval_min_us / 1000.0f, imu);
    n += put(line, n, size, ",%.2f", stats.interval_max_us / 1000.0f, imu);
    n += put(line, n, size, ",%.3f", stats.roll_max - stats.roll_min, imu);
    n += put(line, n, size, ",%.3f", stats.pitch_max - stats.pitch_min, imu);
    n += put(line, n, size, ",%.3f", stats.accel_norm_min, imu);
    n += put(line, n, size, ",%.3f", stats.accel_norm_max, imu);
    n += put(line, n, size, ",%.2f", stats.rate_max, imu);
    n += put(line, n, size, ",%.0f", (float)r.mag_errors);
    n += put(line, n, size, ",%.0f", (float)r.adc_errors);
    n += put(line, n, size, ",%.1f",
             heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024.0f);
    n += put(line, n, size, ",%.1f",
             heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL) / 1024.0f);

    emit(line, n, size);
}

/*
 * The least free stack each task has ever had, and the least free
 * internal heap. A task that runs out of stack trips its canary and the
 * whole unit restarts, so these are worth watching as code is added:
 * the deepest paths - a failed I2C read logging a warning, a float
 * printf - are rare, and a margin that looks fine in normal running can
 * be gone the first time one of them happens.
 */
static void report_headroom(void)
{
    static const char *const tasks[] = {
        "imu", "lvgl", "sensors", "imu_diag", "diag_stream", "esp_timer"
    };

    char line[192];
    int used = snprintf(line, sizeof(line), "free stack, least ever:");

    for(size_t i = 0; i < sizeof(tasks) / sizeof(tasks[0]); i++)
    {
        TaskHandle_t task = xTaskGetHandle(tasks[i]);

        if(task == NULL || used >= (int)sizeof(line))
            continue;

        used += snprintf(line + used, sizeof(line) - used, " %s %u",
                         tasks[i],
                         (unsigned)uxTaskGetStackHighWaterMark(task));
    }

    ESP_LOGI(TAG, "%s | internal heap min %u",
             line,
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL));
}

static void diagnostics_task(void *argument)
{
    (void)argument;

    uint32_t ticks = 0;
    TickType_t last_wake = xTaskGetTickCount();

    while(true)
    {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TICK_MS));
        ticks++;

        if(ticks % HEADER_EVERY == 1)
        {
            send_header("TEL", TEL_COLUMNS);
            send_header("SYS", SYS_COLUMNS);
        }

        send_telemetry();

        uint32_t guard_value = 0;
        const void *guard_address = NULL;
        const int hits = display_guard_check(&guard_value, &guard_address);

        if(hits > 0)
        {
            s_guard_hits = s_guard_hits + (uint32_t)hits;
            ESP_LOGE(TAG, "something wrote past a framebuffer: %d word(s), "
                          "first 0x%08lx at %p (%lu so far)",
                     hits, (unsigned long)guard_value, guard_address,
                     (unsigned long)s_guard_hits);
        }

        if(ticks % HEADROOM_EVERY == 0)
            report_headroom();

        if(ticks % SYS_EVERY != 0)
            continue;

        Stats stats;

        portENTER_CRITICAL(&s_lock);
        stats = s_stats;
        stats_clear(&s_stats);
        portEXIT_CRITICAL(&s_lock);

        /* Events still go to the log: the IMU falling silent is one. */
        if(stats.samples == 0)
            ESP_LOGW(TAG, "no IMU samples in %u ms, %u read errors",
                     (unsigned)(TICK_MS * SYS_EVERY),
                     (unsigned)stats.read_errors);

        send_system(stats, TICK_MS * SYS_EVERY);
    }
}

uint32_t diagnostics_guard_hits(void)
{
    return s_guard_hits;
}

void diagnostics_start(void)
{
    stats_clear(&s_stats);

    xTaskCreatePinnedToCore(diagnostics_task, "imu_diag", TASK_STACK_BYTES,
                            NULL, TASK_PRIORITY, NULL, 0);
}
