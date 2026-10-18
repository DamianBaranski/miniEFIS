#pragma once

/*
 * Machine readable sensor stream on the serial console, for plotting on
 * a PC (tools/sensor_plot.py).
 *
 * Runs while a screen that wants it is showing - the compass screen,
 * or the compass page of settings - at 20 lines a second, mixed in with the normal log. Lines
 * look like NMEA sentences so they are easy to pick out and check:
 *
 *   $DIAGH,<version>,<name>,<name>,...*CS     diagnostics: column names
 *   $DIAG,<value>,<value>,...*CS              diagnostics: one sample
 *   $COMPH,... / $COMP,...                    compass: the same pair
 *
 * CS is the XOR of every character between '$' and '*', in hex. Each
 * header is repeated every few seconds, so a reader started part way
 * through learns the columns without a restart. Missing values - no
 * barometer, no heading yet - are "nan". The columns are listed in
 * diag_stream.cpp.
 *
 * The same lines can be read over Wi-Fi, from /stream on the Wi-Fi debug
 * web server (wifi_debug.h); tools/sensor_plot.py --url does that.
 *
 * A line is about 300 characters, 26 ms of a 115200 baud link, so 20 a
 * second takes about half of it and leaves room for the normal log. The
 * lines are written from a task of their own at the lowest priority, so
 * a slow link holds up nothing else.
 */

typedef enum {
    DIAG_STREAM_DIAGNOSTICS = 0,    /* every sensor, raw */
    DIAG_STREAM_COMPASS             /* everything the heading is made of */
} diag_stream_profile_t;

void diag_stream_start(void);

/*
 * From the LVGL task, as screens come and go. end() only stops the
 * stream if it is still the given profile: LVGL announces the new screen
 * before it lets go of the old one, so the old screen's end() arrives
 * after the new one's begin().
 */
void diag_stream_begin(diag_stream_profile_t profile);
void diag_stream_end(diag_stream_profile_t profile);
