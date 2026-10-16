#include "touch_cst820.h"
#include "board_i2c.h"
#include "board_pins.h"
#include "tca9554.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "touch";

/* ---- register map ---- */
static const uint8_t REG_GESTURE_ID = 0x01;
static const uint8_t REG_VERSION = 0x15;
static const uint8_t REG_CHIP_ID = 0xA7;
static const uint8_t REG_DISABLE_AUTOSLEEP = 0xFE;

static i2c_master_dev_handle_t s_device = NULL;

static esp_err_t touch_reset(void)
{
    esp_err_t err = tca9554_write_pin(BOARD_EXIO_TOUCH_RESET, TCA9554_LOW);

    if(err != ESP_OK)
        return err;

    vTaskDelay(pdMS_TO_TICKS(10));

    err = tca9554_write_pin(BOARD_EXIO_TOUCH_RESET, TCA9554_HIGH);

    if(err != ESP_OK)
        return err;

    /* The controller needs this long before it answers on I2C. */
    vTaskDelay(pdMS_TO_TICKS(50));

    return ESP_OK;
}

esp_err_t touch_init(void)
{
    if(s_device == NULL)
    {
        const esp_err_t err =
            board_i2c_add_device(BOARD_I2C_ADDR_CST820, &s_device);

        if(err != ESP_OK)
            return err;
    }

    gpio_config_t int_config = {};

    int_config.pin_bit_mask = 1ULL << BOARD_TOUCH_INT_PIN;
    int_config.mode = GPIO_MODE_INPUT;
    int_config.pull_up_en = GPIO_PULLUP_ENABLE;
    int_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    int_config.intr_type = GPIO_INTR_DISABLE;

    esp_err_t err = gpio_config(&int_config);

    if(err != ESP_OK)
        return err;

    err = touch_reset();

    if(err != ESP_OK)
        return err;

    /* Auto sleep would stop the panel answering until it is touched. */
    err = board_i2c_write_byte(s_device, REG_DISABLE_AUTOSLEEP, 0xFF);

    if(err != ESP_OK)
        return err;

    uint8_t version = 0;
    uint8_t ids[3] = {0};

    if(board_i2c_read_byte(s_device, REG_VERSION, &version) == ESP_OK &&
       board_i2c_read(s_device, REG_CHIP_ID, ids, sizeof(ids)) == ESP_OK)
    {
        ESP_LOGI(TAG,
                 "CST820 version 0x%02x, chip 0x%02x, project 0x%02x, fw 0x%02x",
                 version, ids[0], ids[1], ids[2]);
    }

    return ESP_OK;
}

esp_err_t touch_read(touch_state_t *state)
{
    if(state == NULL)
        return ESP_ERR_INVALID_ARG;

    state->pressed = false;
    state->x = 0;
    state->y = 0;
    state->gesture = TOUCH_GESTURE_NONE;

    if(s_device == NULL)
        return ESP_ERR_INVALID_STATE;

    /* Gesture, point count, then the 12-bit X and Y pair. */
    uint8_t raw[6] = {0};

    const esp_err_t err =
        board_i2c_read(s_device, REG_GESTURE_ID, raw, sizeof(raw));

    if(err != ESP_OK)
        return err;

    state->gesture = (touch_gesture_t)raw[0];

    if(raw[1] != 0)
    {
        state->pressed = true;
        state->x = (uint16_t)(((raw[2] & 0x0F) << 8) | raw[3]);
        state->y = (uint16_t)(((raw[4] & 0x0F) << 8) | raw[5]);
    }

    return ESP_OK;
}

const char *touch_gesture_name(touch_gesture_t gesture)
{
    switch(gesture)
    {
        case TOUCH_GESTURE_NONE:         return "none";
        case TOUCH_GESTURE_SWIPE_UP:     return "swipe up";
        case TOUCH_GESTURE_SWIPE_DOWN:   return "swipe down";
        case TOUCH_GESTURE_SWIPE_LEFT:   return "swipe left";
        case TOUCH_GESTURE_SWIPE_RIGHT:  return "swipe right";
        case TOUCH_GESTURE_SINGLE_CLICK: return "single click";
        case TOUCH_GESTURE_DOUBLE_CLICK: return "double click";
        case TOUCH_GESTURE_LONG_PRESS:   return "long press";
    }

    return "unknown";
}
