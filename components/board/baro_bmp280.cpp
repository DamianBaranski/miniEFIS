#include "baro_bmp280.h"
#include "board_i2c.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "baro";

/* ---- register map ---- */
static const uint8_t REG_CALIBRATION = 0x88;    /* 0x88..0x9F, 24 bytes */
static const uint8_t REG_CHIP_ID = 0xD0;
static const uint8_t REG_RESET = 0xE0;
static const uint8_t REG_STATUS = 0xF3;
static const uint8_t REG_CTRL_MEAS = 0xF4;
static const uint8_t REG_CONFIG = 0xF5;
static const uint8_t REG_PRESSURE = 0xF7;       /* 0xF7..0xFC, 6 bytes */

static const uint8_t CHIP_ID_BMP280 = 0x58;
static const uint8_t CHIP_ID_BME280 = 0x60;

static const uint8_t RESET_COMMAND = 0xB6;
static const uint8_t MODE_NORMAL = 0x03;
static const uint8_t STATUS_CALIBRATING = 0x01;

/*
 * Factory calibration, read once. The compensation below is the
 * datasheet's fixed point form: it is exact, and integer only, where the
 * floating point variant in the same datasheet needs doubles that this
 * core has to emulate in software.
 */
struct Calibration
{
    uint16_t t1;
    int16_t t2;
    int16_t t3;
    uint16_t p1;
    int16_t p2;
    int16_t p3;
    int16_t p4;
    int16_t p5;
    int16_t p6;
    int16_t p7;
    int16_t p8;
    int16_t p9;
};

static i2c_master_dev_handle_t s_device = NULL;
static uint8_t s_address = 0;
static uint8_t s_chip_id = 0;
static Calibration s_cal;

static inline uint16_t u16_le(const uint8_t *raw)
{
    return (uint16_t)((uint16_t)raw[1] << 8 | raw[0]);
}

static inline int16_t s16_le(const uint8_t *raw)
{
    return (int16_t)u16_le(raw);
}

static esp_err_t read_calibration(void)
{
    uint8_t raw[24];

    const esp_err_t err =
        board_i2c_read(s_device, REG_CALIBRATION, raw, sizeof(raw));

    if(err != ESP_OK)
        return err;

    s_cal.t1 = u16_le(&raw[0]);
    s_cal.t2 = s16_le(&raw[2]);
    s_cal.t3 = s16_le(&raw[4]);
    s_cal.p1 = u16_le(&raw[6]);
    s_cal.p2 = s16_le(&raw[8]);
    s_cal.p3 = s16_le(&raw[10]);
    s_cal.p4 = s16_le(&raw[12]);
    s_cal.p5 = s16_le(&raw[14]);
    s_cal.p6 = s16_le(&raw[16]);
    s_cal.p7 = s16_le(&raw[18]);
    s_cal.p8 = s16_le(&raw[20]);
    s_cal.p9 = s16_le(&raw[22]);

    /*
     * t1 and p1 are unsigned and never zero on a working part, so this
     * catches a device that answers on the bus but returns nothing
     * usable - a half-soldered SDO pin, or a clone that needs a
     * different init.
     */
    if(s_cal.t1 == 0 || s_cal.p1 == 0)
    {
        ESP_LOGE(TAG, "calibration data is empty (t1=%u p1=%u)",
                 s_cal.t1, s_cal.p1);
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

void baro_config_default(baro_config_t *config)
{
    if(config == NULL)
        return;

    config->pressure_oversample = BARO_OVERSAMPLE_X16;
    config->temperature_oversample = BARO_OVERSAMPLE_X2;
    config->filter = BARO_FILTER_16;
}

esp_err_t baro_init(const baro_config_t *config)
{
    baro_config_t defaults;

    if(config == NULL)
    {
        baro_config_default(&defaults);
        config = &defaults;
    }

    /* Which address is strapped is a board wiring detail, so try both
     * rather than make the caller care. */
    const uint8_t candidates[] = { BARO_ADDR_PRIMARY, BARO_ADDR_SECONDARY };

    s_address = 0;

    for(size_t i = 0; i < sizeof(candidates); i++)
    {
        if(board_i2c_probe(candidates[i]) == ESP_OK)
        {
            s_address = candidates[i];
            break;
        }
    }

    if(s_address == 0)
    {
        ESP_LOGI(TAG, "no barometer at 0x%02x or 0x%02x",
                 BARO_ADDR_PRIMARY, BARO_ADDR_SECONDARY);
        return ESP_ERR_NOT_FOUND;
    }

    esp_err_t err = board_i2c_add_device(s_address, &s_device);

    if(err != ESP_OK)
        return err;

    err = board_i2c_read_byte(s_device, REG_CHIP_ID, &s_chip_id);

    if(err != ESP_OK)
        return err;

    if(s_chip_id != CHIP_ID_BMP280 && s_chip_id != CHIP_ID_BME280)
    {
        ESP_LOGE(TAG, "0x%02x answered but reports chip id 0x%02x, "
                      "expected 0x%02x (BMP280) or 0x%02x (BME280)",
                 s_address, s_chip_id, CHIP_ID_BMP280, CHIP_ID_BME280);
        s_address = 0;
        return ESP_ERR_NOT_SUPPORTED;
    }

    /* Start from a known state: the part keeps its mode across a warm
     * reset of the ESP32, so a rerun would otherwise inherit whatever
     * the last run left. */
    err = board_i2c_write_byte(s_device, REG_RESET, RESET_COMMAND);

    if(err != ESP_OK)
        return err;

    /* The datasheet allows 2 ms to copy the calibration out of NVM. */
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t status = STATUS_CALIBRATING;

    for(int attempt = 0; attempt < 10; attempt++)
    {
        if(board_i2c_read_byte(s_device, REG_STATUS, &status) != ESP_OK)
            break;

        if((status & STATUS_CALIBRATING) == 0)
            break;

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if((status & STATUS_CALIBRATING) != 0)
        ESP_LOGW(TAG, "still copying calibration after reset");

    err = read_calibration();

    if(err != ESP_OK)
        return err;

    err = board_i2c_write_byte(
        s_device, REG_CONFIG, (uint8_t)(config->filter << 2));

    if(err != ESP_OK)
        return err;

    /* CTRL_MEAS last: writing it starts conversions. */
    err = board_i2c_write_byte(
        s_device, REG_CTRL_MEAS,
        (uint8_t)((config->temperature_oversample << 5) |
                  (config->pressure_oversample << 2) |
                  MODE_NORMAL));

    if(err != ESP_OK)
        return err;

    ESP_LOGI(TAG, "%s at 0x%02x, chip id 0x%02x",
             (s_chip_id == CHIP_ID_BME280) ? "BME280" : "BMP280",
             s_address, s_chip_id);

    return ESP_OK;
}

bool baro_present(void)
{
    return s_address != 0;
}

uint8_t baro_address(void)
{
    return s_address;
}

uint8_t baro_chip_id(void)
{
    return s_chip_id;
}

/*
 * Datasheet compensation, fixed point.
 *
 * Temperature has to be computed first either way: it produces the
 * t_fine term the pressure formula depends on, which is why a pressure
 * reading always costs a temperature reading too.
 */
static int32_t compensate_temperature(int32_t adc, int32_t *t_fine)
{
    const int32_t var1 =
        ((((adc >> 3) - ((int32_t)s_cal.t1 << 1))) * ((int32_t)s_cal.t2)) >> 11;

    const int32_t var2 =
        (((((adc >> 4) - ((int32_t)s_cal.t1)) *
           ((adc >> 4) - ((int32_t)s_cal.t1))) >> 12) *
         ((int32_t)s_cal.t3)) >> 14;

    *t_fine = var1 + var2;

    return (*t_fine * 5 + 128) >> 8;        /* 0.01 degC */
}

static uint32_t compensate_pressure(int32_t adc, int32_t t_fine)
{
    int64_t var1 = (int64_t)t_fine - 128000;
    int64_t var2 = var1 * var1 * (int64_t)s_cal.p6;

    var2 = var2 + ((var1 * (int64_t)s_cal.p5) << 17);
    var2 = var2 + (((int64_t)s_cal.p4) << 35);

    var1 = ((var1 * var1 * (int64_t)s_cal.p3) >> 8) +
           ((var1 * (int64_t)s_cal.p2) << 12);

    var1 = ((((int64_t)1) << 47) + var1) * ((int64_t)s_cal.p1) >> 33;

    if(var1 == 0)
        return 0;                           /* would divide by zero */

    int64_t p = 1048576 - adc;

    p = (((p << 31) - var2) * 3125) / var1;

    var1 = (((int64_t)s_cal.p9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)s_cal.p8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)s_cal.p7) << 4);

    return (uint32_t)p;                     /* Q24.8 Pa */
}

esp_err_t baro_read(baro_sample_t *sample)
{
    if(sample == NULL)
        return ESP_ERR_INVALID_ARG;

    if(s_device == NULL || s_address == 0)
        return ESP_ERR_INVALID_STATE;

    /* Pressure and temperature are adjacent, so one burst keeps them
     * from straddling a conversion. */
    uint8_t raw[6];

    const esp_err_t err =
        board_i2c_read(s_device, REG_PRESSURE, raw, sizeof(raw));

    if(err != ESP_OK)
        return err;

    /* 20 bit results, left aligned in three bytes. */
    const int32_t adc_pressure =
        (int32_t)(((uint32_t)raw[0] << 12) |
                  ((uint32_t)raw[1] << 4) |
                  ((uint32_t)raw[2] >> 4));

    const int32_t adc_temperature =
        (int32_t)(((uint32_t)raw[3] << 12) |
                  ((uint32_t)raw[4] << 4) |
                  ((uint32_t)raw[5] >> 4));

    int32_t t_fine = 0;
    const int32_t temperature = compensate_temperature(adc_temperature, &t_fine);
    const uint32_t pressure = compensate_pressure(adc_pressure, t_fine);

    sample->temperature_c = (float)temperature / 100.0f;
    sample->pressure_pa = (float)pressure / 256.0f;

    return ESP_OK;
}

float baro_altitude_m(float pressure_pa, float sea_level_pa)
{
    if(pressure_pa <= 0.0f || sea_level_pa <= 0.0f)
        return 0.0f;

    /*
     * International standard atmosphere below the tropopause, with the
     * exponent R*L/(g*M) = 0.1902632 and the 44330 m scale height that
     * follows from a 288.15 K sea level temperature.
     */
    return 44330.0f *
           (1.0f - powf(pressure_pa / sea_level_pa, 0.1902632f));
}
