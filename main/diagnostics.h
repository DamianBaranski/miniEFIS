#pragma once

#include "esp_err.h"
#include "imu_qmi8658.h"

/*
 * Telemetry for the whole pipeline - sensors, filters and display - for
 * the Wi-Fi debug page's charts (wifi_debug.h), from a low priority task
 * of its own.
 *
 * Two sentences, in the telemetry stream (web_log.h), not the log:
 *
 *   $TEL  ten a second: attitude, heading and the filter's view of
 *         them, rates and gyro bias, acceleration, turn rate and slip,
 *         altitude and vertical speed, pressure, temperature, field
 *   $SYS  once a second: frame rate and the cost of drawing, the IMU's
 *         sample count and interval, the attitude's peak to peak over
 *         the second (taken at the full 100 Hz, so a spike between two
 *         $TEL still shows), errors, heap
 *
 * Each preceded by $TELH / $SYSH with the column names, repeated every
 * few seconds; values are CSV, "nan" for missing, and every line ends
 * *XX, the XOR of everything between $ and *. The columns are listed in
 * diagnostics.cpp.
 *
 * Reading them, to tell where an unsteady display comes from:
 *
 *   accelerometer or rates jumping, pp large  -> the sensor or the I2C
 *   raw steady, pp large                      -> the filter, or an axis
 *                                                mapped with the wrong
 *                                                sign
 *   pp small, fps low or uneven               -> the display path
 *   dt_max large                              -> the IMU task starved
 *
 * The log keeps only events: a write caught past a framebuffer, the IMU
 * falling silent, and every ten seconds the stack and heap headroom.
 */

void diagnostics_start(void);

/* Writes caught beside the framebuffers since start-up. Should be 0. */
uint32_t diagnostics_guard_hits(void);

/* Call once per IMU sample, from the IMU task. */
void diagnostics_record_imu(const imu_sample_t *sample,
                            esp_err_t read_status,
                            uint32_t interval_us);
