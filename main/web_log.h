#pragma once

#include <stddef.h>
#include <stdint.h>

/*
 * Copies of the serial output in memory, for reading over Wi-Fi
 * (wifi_debug.h). Two of them, kept apart:
 *
 *   the log      every ESP_LOGx line, from start-up on, so the boot
 *                log - reset reason, crash summary - is there to read
 *                when the web page is first opened. 128 KB.
 *   the stream   the $DIAG/$COMP sentences for tools/sensor_plot.py
 *                (diag_stream.h). At 20 lines a second they would push
 *                everything else out of the log, so they have their own
 *                64 KB, about ten seconds of them.
 *   telemetry    the $TEL/$SYS sentences (diagnostics.h): every
 *                estimate and sensor ten times a second, whatever the
 *                screen, for the web page's charts. 128 KB, about two
 *                minutes.
 *
 * Both still go to the serial console as before.
 *
 * A reader keeps a position, the count of bytes written before what it
 * wants next, and asks for what follows. If it has fallen so far behind
 * that the start was overwritten, it gets the oldest text still held.
 */

/* Call first thing in app_main, so the boot log is kept. */
void web_log_init(void);

/*
 * Copies up to `size` bytes written from *position on into `buffer`,
 * and moves *position past them. Returns the number copied. Any task.
 */
size_t web_log_read(uint32_t *position, char *buffer, size_t size);
size_t web_stream_read(uint32_t *position, char *buffer, size_t size);

/* Where the next byte will go: a reader starting here gets only what
 * comes after. */
size_t web_telemetry_read(uint32_t *position, char *buffer, size_t size);

uint32_t web_log_end(void);
uint32_t web_stream_end(void);
uint32_t web_telemetry_end(void);

/* One whole sentence, newline included. From diag_stream's task. */
void web_stream_append(const char *text, size_t length);
void web_telemetry_append(const char *text, size_t length);
