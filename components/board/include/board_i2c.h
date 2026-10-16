#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * The board's shared I2C bus.
 *
 * Each chip gets its own device handle from board_i2c_add_device(). The
 * register read below is a single transmit-receive transaction, so the
 * address write and the data read cannot be separated by another task
 * using the bus - the driver serialises transactions internally, which
 * matters here because the IMU is polled from one task while the touch
 * controller is read from another.
 */

esp_err_t board_i2c_init(void);

esp_err_t board_i2c_add_device(uint8_t address, i2c_master_dev_handle_t *out);

esp_err_t board_i2c_read(i2c_master_dev_handle_t device,
                         uint8_t reg,
                         uint8_t *data,
                         size_t length);

esp_err_t board_i2c_write(i2c_master_dev_handle_t device,
                          uint8_t reg,
                          const uint8_t *data,
                          size_t length);

esp_err_t board_i2c_write_byte(i2c_master_dev_handle_t device,
                               uint8_t reg,
                               uint8_t value);

esp_err_t board_i2c_read_byte(i2c_master_dev_handle_t device,
                              uint8_t reg,
                              uint8_t *value);

/* Does anything answer at this address? */
esp_err_t board_i2c_probe(uint8_t address);

/* Log every address that answers. Useful when a device is added or when
 * one stops responding: it distinguishes wiring faults from driver
 * faults before any register is touched. */
void board_i2c_scan(void);

#ifdef __cplusplus
}
#endif
