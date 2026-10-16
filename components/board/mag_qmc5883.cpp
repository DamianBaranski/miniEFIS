#include "mag_qmc5883.h"
#include "board_i2c.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "mag";

/* ---- QMC5883L register map ---- */
static const uint8_t L_REG_DATA = 0x00;         /* 0x00..0x05, x y z, LSB first */
static const uint8_t L_REG_STATUS = 0x06;
static const uint8_t L_REG_CONTROL_1 = 0x09;
static const uint8_t L_REG_CONTROL_2 = 0x0A;
static const uint8_t L_REG_SET_RESET = 0x0B;
static const uint8_t L_REG_CHIP_ID = 0x0D;

static const uint8_t L_CHIP_ID = 0xFF;

/*
 * Oversampling 512, range 8 G, 50 Hz, continuous:
 * OSR[7:6]=00 RNG[5:4]=01 ODR[3:2]=01 MODE[1:0]=01.
 */
static const uint8_t L_CONTROL_1 = 0x15;
static const uint8_t L_SOFT_RESET = 0x80;
static const uint8_t L_SET_RESET_PERIOD = 0x01;    /* as the datasheet asks */
static const float L_COUNTS_PER_GAUSS = 3000.0f;   /* 8 G range */

/* ---- QMC5883P register map ---- */
static const uint8_t P_REG_CHIP_ID = 0x00;
static const uint8_t P_REG_DATA = 0x01;         /* 0x01..0x06, x y z, LSB first */
static const uint8_t P_REG_STATUS = 0x09;
static const uint8_t P_REG_CONTROL_1 = 0x0A;
static const uint8_t P_REG_CONTROL_2 = 0x0B;
static const uint8_t P_REG_AXIS_SIGN = 0x29;

static const uint8_t P_CHIP_ID = 0x80;

/*
 * Down-sampling 8, oversampling 8, 50 Hz, normal mode:
 * OSR2[7:6]=11 OSR1[5:4]=00 ODR[3:2]=01 MODE[1:0]=01.
 */
static const uint8_t P_CONTROL_1 = 0xC5;
/* Set and reset on, range 8 G: RNG[3:2]=10 SET_RESET[1:0]=00. */
static const uint8_t P_CONTROL_2 = 0x08;
static const uint8_t P_SOFT_RESET = 0x80;
/* The datasheet's start-up sequence writes this without explaining it
 * beyond "define the sign of the X, Y and Z axes". */
static const uint8_t P_AXIS_SIGN = 0x06;
static const float P_COUNTS_PER_GAUSS = 3750.0f;   /* 8 G range */

/* Shared: data ready is bit 0 and overflow bit 1 on both parts. */
static const uint8_t STATUS_OVERFLOW = 0x02;

static const float MICROTESLA_PER_GAUSS = 100.0f;

static i2c_master_dev_handle_t s_device = NULL;
static mag_chip_t s_chip = MAG_CHIP_NONE;
static uint8_t s_address = 0;

static esp_err_t init_qmc5883l(void)
{
    uint8_t id = 0;

    esp_err_t err = board_i2c_read_byte(s_device, L_REG_CHIP_ID, &id);

    if(err != ESP_OK)
        return err;

    if(id != L_CHIP_ID)
    {
        ESP_LOGE(TAG, "0x%02x answered but reports chip id 0x%02x, "
                      "expected 0x%02x (QMC5883L)",
                 s_address, id, L_CHIP_ID);
        return ESP_ERR_NOT_SUPPORTED;
    }

    err = board_i2c_write_byte(s_device, L_REG_CONTROL_2, L_SOFT_RESET);

    if(err != ESP_OK)
        return err;

    vTaskDelay(pdMS_TO_TICKS(10));

    err = board_i2c_write_byte(s_device, L_REG_SET_RESET, L_SET_RESET_PERIOD);

    if(err != ESP_OK)
        return err;

    return board_i2c_write_byte(s_device, L_REG_CONTROL_1, L_CONTROL_1);
}

static esp_err_t init_qmc5883p(void)
{
    uint8_t id = 0;

    esp_err_t err = board_i2c_read_byte(s_device, P_REG_CHIP_ID, &id);

    if(err != ESP_OK)
        return err;

    if(id != P_CHIP_ID)
    {
        ESP_LOGE(TAG, "0x%02x answered but reports chip id 0x%02x, "
                      "expected 0x%02x (QMC5883P)",
                 s_address, id, P_CHIP_ID);
        return ESP_ERR_NOT_SUPPORTED;
    }

    err = board_i2c_write_byte(s_device, P_REG_CONTROL_2, P_SOFT_RESET);

    if(err != ESP_OK)
        return err;

    vTaskDelay(pdMS_TO_TICKS(10));

    err = board_i2c_write_byte(s_device, P_REG_AXIS_SIGN, P_AXIS_SIGN);

    if(err != ESP_OK)
        return err;

    err = board_i2c_write_byte(s_device, P_REG_CONTROL_2, P_CONTROL_2);

    if(err != ESP_OK)
        return err;

    /* CONTROL_1 last: writing the mode starts conversions. */
    return board_i2c_write_byte(s_device, P_REG_CONTROL_1, P_CONTROL_1);
}

esp_err_t mag_init(void)
{
    s_chip = MAG_CHIP_NONE;
    s_address = 0;

    mag_chip_t found = MAG_CHIP_NONE;

    if(board_i2c_probe(MAG_ADDR_QMC5883P) == ESP_OK)
    {
        found = MAG_CHIP_QMC5883P;
        s_address = MAG_ADDR_QMC5883P;
    }
    else if(board_i2c_probe(MAG_ADDR_QMC5883L) == ESP_OK)
    {
        found = MAG_CHIP_QMC5883L;
        s_address = MAG_ADDR_QMC5883L;
    }

    if(found == MAG_CHIP_NONE)
    {
        ESP_LOGI(TAG, "no magnetometer at 0x%02x or 0x%02x",
                 MAG_ADDR_QMC5883P, MAG_ADDR_QMC5883L);
        return ESP_ERR_NOT_FOUND;
    }

    esp_err_t err = board_i2c_add_device(s_address, &s_device);

    if(err != ESP_OK)
    {
        s_address = 0;
        return err;
    }

    err = (found == MAG_CHIP_QMC5883P) ? init_qmc5883p() : init_qmc5883l();

    if(err != ESP_OK)
    {
        s_address = 0;
        return err;
    }

    s_chip = found;

    ESP_LOGI(TAG, "%s at 0x%02x", mag_chip_name(), s_address);

    return ESP_OK;
}

bool mag_present(void)
{
    return s_chip != MAG_CHIP_NONE;
}

mag_chip_t mag_chip(void)
{
    return s_chip;
}

const char *mag_chip_name(void)
{
    switch(s_chip)
    {
        case MAG_CHIP_QMC5883L: return "QMC5883L";
        case MAG_CHIP_QMC5883P: return "QMC5883P";
        default:                return "none";
    }
}

uint8_t mag_address(void)
{
    return s_address;
}

esp_err_t mag_read(mag_sample_t *sample)
{
    if(sample == NULL)
        return ESP_ERR_INVALID_ARG;

    if(s_chip == MAG_CHIP_NONE)
        return ESP_ERR_INVALID_STATE;

    const bool is_p = (s_chip == MAG_CHIP_QMC5883P);

    /*
     * Data and status in one burst: the status register follows the
     * data on both parts (two reserved bytes between them on the P), so
     * the whole block is a single transaction. This is read from the IMU
     * task, where every transfer counts against the sample interval.
     */
    const uint8_t first = is_p ? P_REG_DATA : L_REG_DATA;
    const uint8_t status_reg = is_p ? P_REG_STATUS : L_REG_STATUS;

    uint8_t raw[9];
    const size_t length = (size_t)(status_reg - first + 1);

    const esp_err_t err = board_i2c_read(s_device, first, raw, length);

    if(err != ESP_OK)
        return err;

    const uint8_t status = raw[length - 1];

    const float counts_per_gauss =
        is_p ? P_COUNTS_PER_GAUSS : L_COUNTS_PER_GAUSS;
    const float scale = MICROTESLA_PER_GAUSS / counts_per_gauss;

    for(int axis = 0; axis < 3; axis++)
        sample->raw[axis] = (int16_t)((uint16_t)raw[axis * 2 + 1] << 8 |
                                      raw[axis * 2]);

    sample->x_uT = (float)sample->raw[0] * scale;
    sample->y_uT = (float)sample->raw[1] * scale;
    sample->z_uT = (float)sample->raw[2] * scale;
    sample->overflow = (status & STATUS_OVERFLOW) != 0;

    return ESP_OK;
}
