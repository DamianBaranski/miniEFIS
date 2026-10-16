# Attitude, heading and altitude estimation.
#
# Deliberately free of any ESP-IDF, LVGL or board dependency beyond the
# clock, so that the same sources build and run on a host PC - see
# test/CMakeLists.txt for the unit test build.
idf_component_register(
    SRCS "attitude_filter.cpp" "vertical_channel.cpp" "mag_calibration.cpp" "turn_coordinator.cpp" "navigator.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_timer)
