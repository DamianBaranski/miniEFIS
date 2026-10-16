#include "sensor_readings.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sensors";

/*
 * 10 Hz: as fast as anyone can read numbers off a screen. A full pass
 * over the four channels takes some 40 ms of that, almost all of it
 * waiting for conversions.
 */
static const uint32_t POLL_PERIOD_MS = 100;

/* 4 KB since the log is also copied for the web page (web_log.cpp): a
 * failed I2C read that logs a warning left 628 bytes at 3 KB. */
static const int TASK_STACK_BYTES = 4096;
static const int TASK_PRIORITY = 1;         /* below the IMU and LVGL tasks */
static const int TASK_CORE = 0;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static sensor_readings_t s_readings;

void sensor_readings_record_imu(const imu_sample_t *sample,
                                esp_err_t read_status)
{
    portENTER_CRITICAL(&s_lock);

    if(read_status == ESP_OK)
    {
        s_readings.imu = *sample;
        s_readings.imu_valid = true;
    }
    else
    {
        s_readings.imu_errors++;
    }

    portEXIT_CRITICAL(&s_lock);
}

void sensor_readings_record_mag(const mag_sample_t *sample,
                                esp_err_t read_status)
{
    portENTER_CRITICAL(&s_lock);

    if(read_status == ESP_OK)
    {
        s_readings.mag = *sample;
        s_readings.mag_valid = true;
    }
    else
    {
        s_readings.mag_errors++;
    }

    portEXIT_CRITICAL(&s_lock);
}

void sensor_readings_get(sensor_readings_t *out)
{
    if(out == NULL)
        return;

    portENTER_CRITICAL(&s_lock);
    *out = s_readings;
    portEXIT_CRITICAL(&s_lock);
}

static void poll_adc(void)
{
    for(uint8_t channel = 0; channel < ADS1115_CHANNELS; channel++)
    {
        int16_t raw = 0;

        /* Outside the lock: this waits for the conversion. */
        const esp_err_t err = ads1115_read(channel, &raw);

        portENTER_CRITICAL(&s_lock);

        if(err == ESP_OK)
        {
            s_readings.adc_raw[channel] = raw;
            s_readings.adc_volts[channel] = ads1115_to_volts(raw);
            s_readings.adc_valid[channel] = true;
        }
        else
        {
            s_readings.adc_errors++;
        }

        portEXIT_CRITICAL(&s_lock);
    }
}

static void sensor_task(void *argument)
{
    (void)argument;

    TickType_t last_wake = xTaskGetTickCount();

    while(true)
    {
        poll_adc();

        (void)xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(POLL_PERIOD_MS));
    }
}

void sensor_readings_start(void)
{
    if(!ads1115_present())
        return;

    const BaseType_t created = xTaskCreatePinnedToCore(
        sensor_task, "sensors", TASK_STACK_BYTES, NULL, TASK_PRIORITY, NULL,
        TASK_CORE);

    if(created != pdPASS)
        ESP_LOGE(TAG, "could not start the sensor polling task");
}
