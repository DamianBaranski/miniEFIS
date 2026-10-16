idf_component_register(
    SRCS
        "main.cpp"
        "alignment.cpp"
        "app_settings.cpp"
        "crash_report.cpp"
        "diag_stream.cpp"
        "diagnostics.cpp"
        "fast_draw.cpp"
        "magnetometer.cpp"
        "navigation.cpp"
        "sensor_readings.cpp"
        "web_log.cpp"
        "wifi_debug.cpp"

        "screens/screen_manager.cpp"
        "screens/horizon_screen.cpp"
        "screens/altimeter_screen.cpp"
        "screens/turn_coordinator_screen.cpp"
        "screens/compass_screen.cpp"
        "screens/settings_screen.cpp"
        "screens/diagnostics_screen.cpp"
        "screens/update_screen.cpp"

        "widgets/altitude_widget.cpp"
    INCLUDE_DIRS "." "screens" "widgets"
    REQUIRES
        ahrs
        board
        lvgl
        nvs_flash
        espcoredump
        app_update
        esp_wifi
        esp_netif
        esp_event
        esp_http_server
        spi_flash)
