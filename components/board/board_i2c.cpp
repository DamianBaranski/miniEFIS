#include "board_i2c.h"
#include "board_pins.h"

#include "esp_log.h"
#include <stdio.h>

static const char *TAG = "board_i2c";

/* Transactions are short; a bus that does not answer quickly is faulty. */
static const int I2C_TIMEOUT_MS = 100;

static i2c_master_bus_handle_t s_bus = NULL;

esp_err_t board_i2c_init(void)
{
    if(s_bus != NULL)
        return ESP_OK;

    i2c_master_bus_config_t bus_config = {};

    bus_config.i2c_port = BOARD_I2C_PORT;
    bus_config.sda_io_num = (gpio_num_t)BOARD_I2C_SDA_PIN;
    bus_config.scl_io_num = (gpio_num_t)BOARD_I2C_SCL_PIN;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    const esp_err_t err = i2c_new_master_bus(&bus_config, &s_bus);

    if(err != ESP_OK)
        ESP_LOGE(TAG, "bus init failed: %s", esp_err_to_name(err));

    return err;
}

esp_err_t board_i2c_add_device(uint8_t address, i2c_master_dev_handle_t *out)
{
    if(s_bus == NULL)
        return ESP_ERR_INVALID_STATE;

    i2c_device_config_t device_config = {};

    device_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    device_config.device_address = address;
    device_config.scl_speed_hz = BOARD_I2C_FREQ_HZ;

    const esp_err_t err = i2c_master_bus_add_device(s_bus, &device_config, out);

    if(err != ESP_OK)
        ESP_LOGE(TAG, "adding device 0x%02x failed: %s", address,
                 esp_err_to_name(err));

    return err;
}

esp_err_t board_i2c_read(i2c_master_dev_handle_t device,
                         uint8_t reg,
                         uint8_t *data,
                         size_t length)
{
    const esp_err_t err =
        i2c_master_transmit_receive(device, &reg, 1, data, length,
                                    I2C_TIMEOUT_MS);

    if(err != ESP_OK)
        ESP_LOGW(TAG, "read of reg 0x%02x failed: %s", reg,
                 esp_err_to_name(err));

    return err;
}

esp_err_t board_i2c_write(i2c_master_dev_handle_t device,
                          uint8_t reg,
                          const uint8_t *data,
                          size_t length)
{
    /* Register address and payload have to go out as one transaction. */
    uint8_t buffer[9];

    if(length + 1 > sizeof(buffer))
        return ESP_ERR_INVALID_SIZE;

    buffer[0] = reg;

    for(size_t i = 0; i < length; i++)
        buffer[i + 1] = data[i];

    const esp_err_t err =
        i2c_master_transmit(device, buffer, length + 1, I2C_TIMEOUT_MS);

    if(err != ESP_OK)
        ESP_LOGW(TAG, "write to reg 0x%02x failed: %s", reg,
                 esp_err_to_name(err));

    return err;
}

esp_err_t board_i2c_write_byte(i2c_master_dev_handle_t device,
                               uint8_t reg,
                               uint8_t value)
{
    return board_i2c_write(device, reg, &value, 1);
}

esp_err_t board_i2c_read_byte(i2c_master_dev_handle_t device,
                              uint8_t reg,
                              uint8_t *value)
{
    return board_i2c_read(device, reg, value, 1);
}

esp_err_t board_i2c_probe(uint8_t address)
{
    if(s_bus == NULL)
        return ESP_ERR_INVALID_STATE;

    return i2c_master_probe(s_bus, address, I2C_TIMEOUT_MS);
}

void board_i2c_scan(void)
{
    if(s_bus == NULL)
    {
        ESP_LOGE(TAG, "scan requested before the bus was initialised");
        return;
    }

    char found[64];
    size_t used = 0;
    int count = 0;

    /* 0x00 is the general call address and 0x78 and up are reserved. */
    for(uint8_t address = 0x08; address < 0x78; address++)
    {
        if(i2c_master_probe(s_bus, address, I2C_TIMEOUT_MS) != ESP_OK)
            continue;

        count++;

        if(used + 6 < sizeof(found))
            used += snprintf(found + used, sizeof(found) - used,
                             "0x%02x ", address);
    }

    found[used] = '\0';

    ESP_LOGI(TAG, "scan found %d device(s): %s", count, found);
}
