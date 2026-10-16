# Host build of the attitude filter and its tests.
#
# The filter has no ESP-IDF, LVGL or board dependency, so it can be
# built and exercised on a PC:
#
#   cmake -B build-host test && cmake --build build-host && ctest --test-dir build-host
#
cmake_minimum_required(VERSION 3.16)

project(ahrs_tests CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(ahrs
    ../components/ahrs/attitude_filter.cpp
    ../components/ahrs/vertical_channel.cpp
    ../components/ahrs/mag_calibration.cpp
    ../components/ahrs/turn_coordinator.cpp
    ../components/ahrs/navigator.cpp)
target_include_directories(ahrs PUBLIC ../components/ahrs/include)
target_compile_options(ahrs PRIVATE -Wall -Wextra)

enable_testing()

foreach(suite attitude_filter vertical_channel mag_calibration turn_coordinator navigator)
    add_executable(test_${suite} test_${suite}.cpp)
    target_link_libraries(test_${suite} PRIVATE ahrs m)
    target_compile_options(test_${suite} PRIVATE -Wall -Wextra)
    add_test(NAME ${suite} COMMAND test_${suite})
endforeach()
