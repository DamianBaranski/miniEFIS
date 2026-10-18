#include "alignment.h"
#include "app_settings.h"
#include "can_bus.h"
#include "altitude_widget.h"
#include "horizon_screen.h"
#include "board.h"
#include "crash_report.h"
#include "diag_stream.h"
#include "diagnostics.h"
#include "magnetometer.h"
#include "mounting.h"
#include "navigation.h"
#include "route_store.h"
#include "sensor_readings.h"
#include "screen_manager.h"
#include "web_log.h"
#include "wifi_debug.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "main";

/*
 * IMU rate. The attitude filter integrates gyro rates, so the sample
 * interval is part of the measurement; 100 Hz is comfortably above the
 * movement rates a hand-held or airframe mounted display sees.
 */
static const uint32_t IMU_PERIOD_MS = 10;

/*
 * The magnetometer converts at 50 Hz, so it is read every other IMU
 * step. Each reading goes to the filter once, on the step it was read:
 * a Kalman filter counts every reading it is given as a new
 * measurement, so handing it the same one twice would make it trust
 * the magnetometer twice as much as it should.
 */
static const uint32_t MAG_EVERY_N_STEPS = 2;

static const int IMU_TASK_STACK_BYTES = 4096;
static const int IMU_TASK_PRIORITY = 3;
static const int IMU_TASK_CORE = 0;     /* away from LVGL on core 1 */

static void imu_task(void *argument)
{
    (void)argument;

    TickType_t last_wake = xTaskGetTickCount();
    uint32_t previous_us = esp_timer_get_time();

    uint32_t step = 0;
    AttitudeVec3 mag_body = { 0.0f, 0.0f, 0.0f };
    bool mag_fresh = false;

    while(true)
    {
        imu_sample_t sample;

        const esp_err_t status = imu_read(&sample);

        /* Measured, not assumed: an uneven interval is one of the things
         * worth seeing on the serial log, and the filters integrate over
         * it. */
        const uint32_t now_us = (uint32_t)esp_timer_get_time();
        const uint32_t interval_us = now_us - previous_us;
        previous_us = now_us;

        const float dt_s = (float)interval_us * 1.0e-6f;

        const AttitudeVec3 accel_body = accel_to_body(sample.accel_g);
        const AttitudeVec3 gyro_body = sensor_to_body(sample.gyro_dps);

        if(mag_present() && (step++ % MAG_EVERY_N_STEPS) == 0)
        {
            mag_sample_t mag;

            const esp_err_t mag_status = mag_read(&mag);

            /* Calibrated and mapped to body axes in there; until it is
             * calibrated there is no heading. */
            if(mag_status == ESP_OK && status == ESP_OK &&
               magnetometer_heading_sample(
                   mag, accel_body, &mag_body))
                mag_fresh = true;

            sensor_readings_record_mag(&mag, mag_status);
        }

        /*
         * Every estimate every screen shows, in one step: attitude and
         * heading, rate of turn and slip, altitude and vertical speed
         * (navigation.h). A magnetometer reading that arrives on a
         * failed IMU step waits for the next good one.
         */
        if(status == ESP_OK)
        {
            navigation_process_imu(accel_body, gyro_body,
                                   mag_fresh ? &mag_body : nullptr, dt_s);
            mag_fresh = false;

            alignment_record(accel_body, gyro_body);
        }

        diagnostics_record_imu(&sample, status, interval_us);
        sensor_readings_record_imu(&sample, status);

        /* A fixed period, rather than a delay after the work, so the
         * filter sees an even sample interval. */
        (void)xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_PERIOD_MS));
    }
}

/* Runs inside the LVGL task: widgets must not be built from app_main. */
static void build_ui(void)
{
    horizon_screen_create(lv_scr_act());

    /* Hides itself if no barometer answered, so it is created either
     * way. Units come from the settings screen. */
    altitude_widget_create(lv_scr_act());
    altitude_widget_set_units(app_settings().units == UNITS_AVIATION
                                  ? ALTITUDE_UNITS_AVIATION
                                  : ALTITUDE_UNITS_METRIC);

    /* The settings and diagnostics screens, and the double tap menu
     * that switches between them and this one. */
    screen_manager_create(lv_scr_act());

    /* Every screen is built up front, so this is close to the most LVGL
     * will ever hold - worth seeing when screens are added. */
    lv_mem_monitor_t memory;
    lv_mem_monitor(&memory);

    ESP_LOGI(TAG, "LVGL heap: %u of %u bytes used, %u%% fragmented",
             (unsigned)(memory.total_size - memory.free_size),
             (unsigned)memory.total_size, (unsigned)memory.frag_pct);
}

/* Settings live in NVS. A partition from an older layout, or a full
 * one, is wiped rather than left to fail every save. */
static void settings_storage_init(void)
{
    esp_err_t err = nvs_flash_init();

    if(err == ESP_ERR_NVS_NO_FREE_PAGES ||
       err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAG, "NVS unusable (%s), erasing it", esp_err_to_name(err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    ESP_ERROR_CHECK(err);
}

extern "C" void app_main(void)
{
    /* Before anything logs, so the web page has the whole boot log. */
    web_log_init();
    crash_report_init();
    settings_storage_init();
    app_settings_init();
    magnetometer_init();
    alignment_init();

    ESP_ERROR_CHECK(board_init());

    /* The saved settings that act on the hardware. */
    display_set_backlight(app_settings().brightness);
    navigation_set_sea_level_pressure(app_settings().qnh_pa);
    ESP_ERROR_CHECK(lvgl_port_start(build_ui));

    diagnostics_start();
    sensor_readings_start();
    can_bus_start();
    route_store_start();
    diag_stream_start();

    const BaseType_t created = xTaskCreatePinnedToCore(
        imu_task, "imu", IMU_TASK_STACK_BYTES, NULL, IMU_TASK_PRIORITY, NULL,
        IMU_TASK_CORE);

    if(created == pdPASS)
        ESP_LOGI(TAG, "MiniEFIS running");
    else
        ESP_LOGE(TAG, "could not start the IMU task");

    /*
     * Last: reaching it is what confirms a newly installed image. Even
     * without the IMU task, since Wi-Fi is how a fix gets installed.
     */
    wifi_debug_init();
}
