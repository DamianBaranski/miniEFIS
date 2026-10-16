#include "navigation.h"
#include "baro_bmp280.h"

#include "esp_log.h"

static const char *TAG = "navigation";

/*
 * The barometer is read every fourth IMU step: 25 Hz against the IMU's
 * 100 Hz, which is what the sensor produces at x16 oversampling.
 *
 * Reading it faster would not give fresher data, it would hand the
 * filter the same conversion several times over - and the filter treats
 * every reading as an independent measurement, so repeats would make it
 * trust the barometer four times more than it should.
 */
static const uint32_t BARO_DECIMATION = 4;

/* Updated in the IMU task, read anywhere through its snapshot. */
static Navigator s_navigator;
static uint32_t s_step = 0;

/*
 * Set from the LVGL task, applied in the IMU task. The applied copy is
 * the one the altitude is currently relative to; when the two differ,
 * the next barometer reading moves the altitude by the difference at
 * once (VerticalChannel::shiftAltitude), as turning a real altimeter's
 * setting knob moves its needles.
 */
static volatile float s_sea_level_pa = BARO_SEA_LEVEL_STANDARD_PA;
static float s_applied_sea_level_pa = BARO_SEA_LEVEL_STANDARD_PA;

/* Last raw reading, for display and diagnostics. */
static volatile float s_pressure_pa = 0.0f;
static volatile float s_temperature_c = 0.0f;
static volatile bool s_have_pressure = false;

/* A fresh barometric altitude, or false on the steps between them. */
static bool read_barometer(float *altitude_m)
{
    if(!baro_present() || (++s_step % BARO_DECIMATION) != 0)
        return false;

    baro_sample_t air;

    if(baro_read(&air) != ESP_OK)
        return false;

    const float sea_level = s_sea_level_pa;

    if(sea_level != s_applied_sea_level_pa)
    {
        s_navigator.shiftAltitude(
            baro_altitude_m(air.pressure_pa, sea_level) -
            baro_altitude_m(air.pressure_pa, s_applied_sea_level_pa));
        s_applied_sea_level_pa = sea_level;
    }

    s_pressure_pa = air.pressure_pa;
    s_temperature_c = air.temperature_c;
    s_have_pressure = true;

    *altitude_m = baro_altitude_m(air.pressure_pa, sea_level);
    return true;
}

void navigation_process_imu(const AttitudeVec3 &accel_g,
                            const AttitudeVec3 &gyro_dps,
                            const AttitudeVec3 *mag_uT,
                            float dt_s)
{
    float altitude = 0.0f;
    const bool have_altitude = read_barometer(&altitude);

    s_navigator.update(accel_g, gyro_dps, mag_uT,
                       have_altitude ? &altitude : 0, dt_s);
}

NavigationSample navigation_sample(void)
{
    return s_navigator.sample();
}

bool navigation_barometer(float *pressure_pa, float *temperature_c)
{
    if(!baro_present() || !s_have_pressure)
        return false;

    if(pressure_pa != 0) *pressure_pa = s_pressure_pa;
    if(temperature_c != 0) *temperature_c = s_temperature_c;

    return true;
}

void navigation_set_sea_level_pressure(float pascals)
{
    /* 870 to 1085 hPa spans every pressure ever recorded at sea level. */
    if(pascals < 87000.0f || pascals > 108500.0f)
    {
        ESP_LOGW(TAG, "ignoring sea level pressure of %.0f Pa", pascals);
        return;
    }

    s_sea_level_pa = pascals;

    ESP_LOGI(TAG, "sea level pressure set to %.1f hPa", pascals / 100.0f);
}

float navigation_sea_level_pressure(void)
{
    return s_sea_level_pa;
}

void navigation_recalibrate_gyro(void)
{
    s_navigator.requestGyroCalibration();
}
