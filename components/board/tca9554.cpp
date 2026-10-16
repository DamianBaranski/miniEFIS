#include "tca9554.h"
#include "board_i2c.h"
#include "board_pins.h"

/* Register map. Polarity inversion (0x02) is not used on this board. */
static const uint8_t REG_INPUT = 0x00;
static const uint8_t REG_OUTPUT = 0x01;
static const uint8_t REG_CONFIG = 0x03;

static i2c_master_dev_handle_t s_device = NULL;

static inline bool pin_valid(uint8_t pin)
{
    return pin >= TCA9554_PIN_MIN && pin <= TCA9554_PIN_MAX;
}

static inline uint8_t pin_mask(uint8_t pin)
{
    return (uint8_t)(1u << (pin - 1));
}

esp_err_t tca9554_init(uint8_t modes)
{
    if(s_device == NULL)
    {
        const esp_err_t err =
            board_i2c_add_device(BOARD_I2C_ADDR_TCA9554, &s_device);

        if(err != ESP_OK)
            return err;
    }

    return board_i2c_write_byte(s_device, REG_CONFIG, modes);
}

/*
 * Read-modify-write on one bit. The expander has no bit-set register, so
 * the current value has to be read back first; a failed read must not be
 * allowed to write a guess over the other seven lines, which is why the
 * error is propagated instead of ignored.
 */
static esp_err_t update_bit(uint8_t reg, uint8_t pin, bool set)
{
    uint8_t value = 0;

    const esp_err_t err = board_i2c_read_byte(s_device, reg, &value);

    if(err != ESP_OK)
        return err;

    if(set)
        value |= pin_mask(pin);
    else
        value &= (uint8_t)~pin_mask(pin);

    return board_i2c_write_byte(s_device, reg, value);
}

esp_err_t tca9554_set_mode(uint8_t pin, tca9554_mode_t mode)
{
    if(!pin_valid(pin))
        return ESP_ERR_INVALID_ARG;

    return update_bit(REG_CONFIG, pin, mode == TCA9554_INPUT);
}

esp_err_t tca9554_write_pin(uint8_t pin, tca9554_level_t level)
{
    if(!pin_valid(pin))
        return ESP_ERR_INVALID_ARG;

    return update_bit(REG_OUTPUT, pin, level == TCA9554_HIGH);
}

esp_err_t tca9554_read_pin(uint8_t pin, tca9554_level_t *level)
{
    if(!pin_valid(pin) || level == NULL)
        return ESP_ERR_INVALID_ARG;

    uint8_t value = 0;

    const esp_err_t err = board_i2c_read_byte(s_device, REG_INPUT, &value);

    if(err != ESP_OK)
        return err;

    *level = (value & pin_mask(pin)) ? TCA9554_HIGH : TCA9554_LOW;
    return ESP_OK;
}

esp_err_t tca9554_toggle_pin(uint8_t pin)
{
    if(!pin_valid(pin))
        return ESP_ERR_INVALID_ARG;

    uint8_t value = 0;

    /* Toggling reads back the output register, not the input: reading
     * the pin state would fold the pin's actual level back in. */
    const esp_err_t err = board_i2c_read_byte(s_device, REG_OUTPUT, &value);

    if(err != ESP_OK)
        return err;

    return board_i2c_write_byte(s_device, REG_OUTPUT,
                                (uint8_t)(value ^ pin_mask(pin)));
}

esp_err_t tca9554_write_port(uint8_t levels)
{
    return board_i2c_write_byte(s_device, REG_OUTPUT, levels);
}

esp_err_t tca9554_read_port(uint8_t *levels)
{
    if(levels == NULL)
        return ESP_ERR_INVALID_ARG;

    return board_i2c_read_byte(s_device, REG_INPUT, levels);
}
