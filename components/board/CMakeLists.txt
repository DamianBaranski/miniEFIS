# Hardware for the Waveshare ESP32-S3 480x480 RGB panel board.
idf_component_register(
    SRCS
        "board.cpp"
        "board_i2c.cpp"
        "tca9554.cpp"
        "display_st7701.cpp"
        "touch_cst820.cpp"
        "imu_qmi8658.cpp"
        "baro_bmp280.cpp"
        "mag_qmc5883.cpp"
        "adc_ads1115.cpp"
        "lvgl_port.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        driver
        esp_lcd
        esp_timer
        lvgl)
