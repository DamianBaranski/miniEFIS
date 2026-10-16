#include "board.h"
#include "board_pins.h"

#include "esp_log.h"

static const char *TAG = "board";

esp_err_t board_init(void)
{
    esp_err_t err = board_i2c_init();

    if(err != ESP_OK)
        return err;

    /* Every expander line is an output on this board. */
    err = tca9554_init(0x00);

    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "IO expander init failed: %s", esp_err_to_name(err));
        return err;
    }

    /*
     * The vendor's driver pulls this line low before touching the IMU
     * and the display comes up unreliably without it; its exact function
     * is not documented on the schematic.
     */
    err = tca9554_write_pin(BOARD_EXIO_IMU_ENABLE, TCA9554_LOW);

    if(err != ESP_OK)
        return err;

    /* Log what actually answers before touching any registers: it tells a
     * wiring fault from a driver fault straight away. */
    board_i2c_scan();

    err = imu_init(NULL);

    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "IMU init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = display_init();

    if(err != ESP_OK)
        return err;

    err = touch_init();

    if(err != ESP_OK)
    {
        /* A dead touch panel is worth reporting but not fatal: the
         * instrument is readable without it. */
        ESP_LOGW(TAG, "touch init failed: %s", esp_err_to_name(err));
    }

    /* Optional: the board has no barometer of its own. */
    const esp_err_t baro_err = baro_init(NULL);

    if(baro_err != ESP_OK && baro_err != ESP_ERR_NOT_FOUND)
        ESP_LOGW(TAG, "barometer init failed: %s", esp_err_to_name(baro_err));

    /* Optional add-ons on the same bus. */
    const esp_err_t mag_err = mag_init();

    if(mag_err != ESP_OK && mag_err != ESP_ERR_NOT_FOUND)
        ESP_LOGW(TAG, "magnetometer init failed: %s", esp_err_to_name(mag_err));

    const esp_err_t adc_err = ads1115_init(ADS1115_RANGE_4V096);

    if(adc_err != ESP_OK && adc_err != ESP_ERR_NOT_FOUND)
        ESP_LOGW(TAG, "ADS1115 init failed: %s", esp_err_to_name(adc_err));

    ESP_LOGI(TAG, "board ready");
    return ESP_OK;
}
