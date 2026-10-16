#include "imu_qmi8658.h"
#include "board_i2c.h"
#include "board_pins.h"

#include "esp_log.h"

static const char *TAG = "imu";

/* ---- register map ---- */
static const uint8_t REG_REVISION = 0x01;
static const uint8_t REG_CTRL1 = 0x02;      /* interface and oscillator  */
static const uint8_t REG_CTRL2 = 0x03;      /* accelerometer range / ODR */
static const uint8_t REG_CTRL3 = 0x04;      /* gyroscope range / ODR     */
static const uint8_t REG_CTRL5 = 0x06;      /* low pass filters          */
static const uint8_t REG_CTRL6 = 0x07;      /* AttitudeEngine            */
static const uint8_t REG_CTRL7 = 0x08;      /* sensor enable             */
static const uint8_t REG_AX_L = 0x35;       /* start of the data block   */

static const uint8_t CTRL2_ODR_MASK = 0x0F;
static const uint8_t CTRL3_ODR_MASK = 0x0F;
static const uint8_t CTRL2_RANGE_MASK = 0x70;
static const uint8_t CTRL3_RANGE_MASK = 0x70;
static const uint8_t CTRL5_ALPF_MASK = 0x06;
static const uint8_t CTRL5_GLPF_MASK = 0x60;
static const uint8_t RANGE_SHIFT = 4;
static const uint8_t CTRL5_ALPF_SHIFT = 1;
static const uint8_t CTRL5_GLPF_SHIFT = 5;
static const uint8_t CTRL5_ALPF_ENABLE = 0x01;
static const uint8_t CTRL5_GLPF_ENABLE = 0x10;

static i2c_master_dev_handle_t s_device = NULL;

/* Counts per unit, derived from the configured ranges. */
static float s_accel_scale = 0.0f;
static float s_gyro_scale = 0.0f;
static imu_vec3_t s_accel_offset = {0.0f, 0.0f, 0.0f};

void imu_config_default(imu_config_t *config)
{
    if(config == NULL)
        return;

    config->accel_range = IMU_ACCEL_RANGE_4G;
    config->gyro_range = IMU_GYRO_RANGE_512DPS;
    config->odr = IMU_ODR_1000HZ;
    config->lpf = IMU_LPF_2_66_PCT;

    /* Measured on the board this was written for; see the header. */
    config->accel_offset_g.x = 0.06f;
    config->accel_offset_g.y = 0.01f;
    config->accel_offset_g.z = 0.05f;
}

static float accel_scale_of(imu_accel_range_t range)
{
    switch(range)
    {
        case IMU_ACCEL_RANGE_2G:  return 2.0f / 32768.0f;
        case IMU_ACCEL_RANGE_4G:  return 4.0f / 32768.0f;
        case IMU_ACCEL_RANGE_8G:  return 8.0f / 32768.0f;
        case IMU_ACCEL_RANGE_16G: return 16.0f / 32768.0f;
    }

    return 4.0f / 32768.0f;
}

static float gyro_scale_of(imu_gyro_range_t range)
{
    switch(range)
    {
        case IMU_GYRO_RANGE_16DPS:   return 16.0f / 32768.0f;
        case IMU_GYRO_RANGE_32DPS:   return 32.0f / 32768.0f;
        case IMU_GYRO_RANGE_64DPS:   return 64.0f / 32768.0f;
        case IMU_GYRO_RANGE_128DPS:  return 128.0f / 32768.0f;
        case IMU_GYRO_RANGE_256DPS:  return 256.0f / 32768.0f;
        case IMU_GYRO_RANGE_512DPS:  return 512.0f / 32768.0f;
        case IMU_GYRO_RANGE_1024DPS: return 1024.0f / 32768.0f;
    }

    return 512.0f / 32768.0f;
}

/* Read-modify-write of one register field. */
static esp_err_t update_reg(uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t current = 0;

    const esp_err_t err = board_i2c_read_byte(s_device, reg, &current);

    if(err != ESP_OK)
        return err;

    current = (uint8_t)((current & (uint8_t)~mask) | (value & mask));

    return board_i2c_write_byte(s_device, reg, current);
}

esp_err_t imu_init(const imu_config_t *config)
{
    imu_config_t defaults;

    if(config == NULL)
    {
        imu_config_default(&defaults);
        config = &defaults;
    }

    if(s_device == NULL)
    {
        const esp_err_t err =
            board_i2c_add_device(BOARD_I2C_ADDR_QMI8658, &s_device);

        if(err != ESP_OK)
            return err;
    }

    uint8_t revision = 0;

    esp_err_t err = board_i2c_read_byte(s_device, REG_REVISION, &revision);

    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "QMI8658 not responding at 0x%02x",
                 BOARD_I2C_ADDR_QMI8658);
        return err;
    }

    ESP_LOGI(TAG, "QMI8658 revision 0x%02x", revision);

    /* Enable the oscillator and address auto-increment, which is what
     * makes the single burst read in imu_read() possible. */
    err = update_reg(REG_CTRL1, 0x41, 0x40);

    if(err != ESP_OK)
        return err;

    /* High speed internal clock, both sensors in full mode, no syncSample. */
    err = board_i2c_write_byte(s_device, REG_CTRL7, 0x43);

    if(err != ESP_OK)
        return err;

    /* No AttitudeEngine: this project does its own fusion. */
    err = board_i2c_write_byte(s_device, REG_CTRL6, 0x00);

    if(err != ESP_OK)
        return err;

    err = update_reg(REG_CTRL2, CTRL2_RANGE_MASK,
                     (uint8_t)(config->accel_range << RANGE_SHIFT));
    if(err != ESP_OK) return err;

    err = update_reg(REG_CTRL2, CTRL2_ODR_MASK, (uint8_t)config->odr);
    if(err != ESP_OK) return err;

    err = update_reg(REG_CTRL3, CTRL3_RANGE_MASK,
                     (uint8_t)(config->gyro_range << RANGE_SHIFT));
    if(err != ESP_OK) return err;

    err = update_reg(REG_CTRL3, CTRL3_ODR_MASK, (uint8_t)config->odr);
    if(err != ESP_OK) return err;

    /*
     * Both low pass filters live in CTRL5, so they are set in one
     * read-modify-write per field with the enable bits included.
     */
    err = update_reg(REG_CTRL5,
                     (uint8_t)(CTRL5_ALPF_MASK | CTRL5_ALPF_ENABLE),
                     (uint8_t)((config->lpf << CTRL5_ALPF_SHIFT) |
                               CTRL5_ALPF_ENABLE));
    if(err != ESP_OK) return err;

    err = update_reg(REG_CTRL5,
                     (uint8_t)(CTRL5_GLPF_MASK | CTRL5_GLPF_ENABLE),
                     (uint8_t)((config->lpf << CTRL5_GLPF_SHIFT) |
                               CTRL5_GLPF_ENABLE));
    if(err != ESP_OK) return err;

    s_accel_scale = accel_scale_of(config->accel_range);
    s_gyro_scale = gyro_scale_of(config->gyro_range);
    s_accel_offset = config->accel_offset_g;

    return ESP_OK;
}

esp_err_t imu_read(imu_sample_t *sample)
{
    if(sample == NULL)
        return ESP_ERR_INVALID_ARG;

    if(s_device == NULL)
        return ESP_ERR_INVALID_STATE;

    /*
     * AX_L(0x35)..GZ_H(0x40) are contiguous and auto-increment is on, so
     * one 12 byte transfer returns both sensors from the same instant.
     */
    uint8_t raw[12];

    const esp_err_t err = board_i2c_read(s_device, REG_AX_L, raw, sizeof(raw));

    if(err != ESP_OK)
        return err;

    const int16_t ax = (int16_t)((raw[1] << 8) | raw[0]);
    const int16_t ay = (int16_t)((raw[3] << 8) | raw[2]);
    const int16_t az = (int16_t)((raw[5] << 8) | raw[4]);
    const int16_t gx = (int16_t)((raw[7] << 8) | raw[6]);
    const int16_t gy = (int16_t)((raw[9] << 8) | raw[8]);
    const int16_t gz = (int16_t)((raw[11] << 8) | raw[10]);

    sample->accel_g.x = (float)ax * s_accel_scale - s_accel_offset.x;
    sample->accel_g.y = (float)ay * s_accel_scale - s_accel_offset.y;
    sample->accel_g.z = (float)az * s_accel_scale - s_accel_offset.z;

    sample->gyro_dps.x = (float)gx * s_gyro_scale;
    sample->gyro_dps.y = (float)gy * s_gyro_scale;
    sample->gyro_dps.z = (float)gz * s_gyro_scale;

    return ESP_OK;
}
