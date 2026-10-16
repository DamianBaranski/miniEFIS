#pragma once

#include "adc_ads1115.h"
#include "baro_bmp280.h"
#include "board_i2c.h"
#include "display_st7701.h"
#include "esp_err.h"
#include "imu_qmi8658.h"
#include "lvgl_port.h"
#include "mag_qmc5883.h"
#include "tca9554.h"
#include "touch_cst820.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Bring up every peripheral on the board, in the order they depend on
 * each other: the I2C bus first, then the IO expander that holds the
 * display and touch reset lines, then the devices themselves.
 *
 * Does not start LVGL - call lvgl_port_start() once the UI is ready to
 * be built.
 */
esp_err_t board_init(void);

#ifdef __cplusplus
}
#endif
