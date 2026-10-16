#include "adc_ads1115.h"
#include "board_i2c.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ads1115";

/* ---- register map, 16 bits each, MSB first ---- */
static const uint8_t REG_CONVERSION = 0x00;
static const uint8_t REG_CONFIG = 0x01;

/* ---- config register ---- */
static const uint16_t CONFIG_START = 0x8000;        /* OS: write 1 to start,
                                                       reads 1 when idle */
static const uint16_t CONFIG_MUX_SINGLE_0 = 0x4000; /* AIN0 to GND; +1 per
                                                       channel in bits 14:12 */
static const int CONFIG_MUX_SHIFT = 12;
static const int CONFIG_RANGE_SHIFT = 9;
static const uint16_t CONFIG_SINGLE_SHOT = 0x0100;
static const uint16_t CONFIG_RATE_128SPS = 0x0080;
static const uint16_t CONFIG_COMPARATOR_OFF = 0x0003;

/*
 * One conversion at 128 SPS is 7.8 ms, and the internal oscillator is
 * allowed to run 10% slow, so 9 ms is the first point at which the
 * result can be expected.
 */
static const uint32_t CONVERSION_MS = 9;
static const int CONVERSION_POLLS = 5;

static const float FULL_SCALE_VOLTS[] = {
    6.144f, 4.096f, 2.048f, 1.024f, 0.512f, 0.256f
};

static i2c_master_dev_handle_t s_device = NULL;
static uint8_t s_address = 0;
static ads1115_range_t s_range = ADS1115_RANGE_4V096;

static esp_err_t write_register(uint8_t reg, uint16_t value)
{
    const uint8_t data[2] = { (uint8_t)(value >> 8), (uint8_t)value };

    return board_i2c_write(s_device, reg, data, sizeof(data));
}

static esp_err_t read_register(uint8_t reg, uint16_t *value)
{
    uint8_t data[2];

    const esp_err_t err = board_i2c_read(s_device, reg, data, sizeof(data));

    if(err == ESP_OK)
        *value = (uint16_t)((uint16_t)data[0] << 8 | data[1]);

    return err;
}

esp_err_t ads1115_init(ads1115_range_t range)
{
    s_address = 0;

    for(uint8_t candidate = ADS1115_ADDR_FIRST;
        candidate <= ADS1115_ADDR_LAST;
        candidate++)
    {
        if(board_i2c_probe(candidate) == ESP_OK)
        {
            s_address = candidate;
            break;
        }
    }

    if(s_address == 0)
    {
        ESP_LOGI(TAG, "no ADS1115 at 0x%02x..0x%02x",
                 ADS1115_ADDR_FIRST, ADS1115_ADDR_LAST);
        return ESP_ERR_NOT_FOUND;
    }

    esp_err_t err = board_i2c_add_device(s_address, &s_device);

    if(err != ESP_OK)
    {
        s_address = 0;
        return err;
    }

    /* There is no id to check, so read the config register instead: it
     * at least proves the part answers a register read, not just its
     * address. */
    uint16_t config = 0;

    err = read_register(REG_CONFIG, &config);

    if(err != ESP_OK)
    {
        s_address = 0;
        return err;
    }

    s_range = range;

    ESP_LOGI(TAG, "ADS1115 at 0x%02x, +-%.3f V range", s_address,
             FULL_SCALE_VOLTS[s_range]);

    return ESP_OK;
}

bool ads1115_present(void)
{
    return s_address != 0;
}

uint8_t ads1115_address(void)
{
    return s_address;
}

esp_err_t ads1115_read(uint8_t channel, int16_t *raw)
{
    if(raw == NULL || channel >= ADS1115_CHANNELS)
        return ESP_ERR_INVALID_ARG;

    if(s_address == 0)
        return ESP_ERR_INVALID_STATE;

    const uint16_t config =
        CONFIG_START |
        (uint16_t)(CONFIG_MUX_SINGLE_0 + (channel << CONFIG_MUX_SHIFT)) |
        (uint16_t)(s_range << CONFIG_RANGE_SHIFT) |
        CONFIG_SINGLE_SHOT |
        CONFIG_RATE_128SPS |
        CONFIG_COMPARATOR_OFF;

    esp_err_t err = write_register(REG_CONFIG, config);

    if(err != ESP_OK)
        return err;

    vTaskDelay(pdMS_TO_TICKS(CONVERSION_MS));

    /* The start bit reads back as 1 once the conversion is done. */
    for(int poll = 0; poll < CONVERSION_POLLS; poll++)
    {
        uint16_t status = 0;

        err = read_register(REG_CONFIG, &status);

        if(err != ESP_OK)
            return err;

        if((status & CONFIG_START) != 0)
        {
            uint16_t value = 0;

            err = read_register(REG_CONVERSION, &value);

            if(err == ESP_OK)
                *raw = (int16_t)value;

            return err;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return ESP_ERR_TIMEOUT;
}

float ads1115_to_volts(int16_t raw)
{
    return (float)raw * FULL_SCALE_VOLTS[s_range] / 32768.0f;
}
