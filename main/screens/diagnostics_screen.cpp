#include "diagnostics_screen.h"
#include "fonts.h"
#include "navigation.h"
#include "crash_report.h"
#include "diag_stream.h"
#include "diagnostics.h"
#include "sensor_readings.h"

#include "adc_ads1115.h"
#include "baro_bmp280.h"
#include "mag_qmc5883.h"

#include <math.h>
#include <stdio.h>

static const uint32_t UPDATE_PERIOD_MS = 200;

/*
 * A table: a caption column and three value columns, values right
 * aligned so the digits line up. Sized to stay inside the round panel -
 * at the top and bottom rows the visible width is about 360 px.
 */
static const lv_coord_t CAPTION_X = 66;
static const lv_coord_t CAPTION_WIDTH = 84;
static const lv_coord_t COLUMN_X[3] = { 150, 240, 330 };
static const lv_coord_t COLUMN_WIDTH = 84;

/* Line spacing: a section heading, a row, and the space after a section. */
static const lv_coord_t HEADING_STEP = 19;
static const lv_coord_t ROW_STEP = 22;
static const lv_coord_t SECTION_GAP = 5;

static const uint32_t SECTION_COLOUR = 0xFFD400;
static const uint32_t HEADER_COLOUR = 0x808890;
static const uint32_t VALUE_COLOUR = 0xFFFFFF;
static const uint32_t ABSENT_COLOUR = 0x606870;

struct Row
{
    lv_obj_t *cell[3];
};

static lv_obj_t *s_screen = NULL;

static Row s_accel;
static Row s_gyro;
static Row s_mag_uT;
static Row s_mag_raw;
static Row s_mag_norm;
static Row s_adc[ADS1115_CHANNELS];
static Row s_baro;
static lv_obj_t *s_errors = NULL;

static lv_obj_t *make_label(lv_coord_t x, lv_coord_t y, lv_coord_t width,
                            const lv_font_t *font, uint32_t colour,
                            lv_text_align_t align, const char *text)
{
    lv_obj_t *label = lv_label_create(s_screen);

    lv_obj_set_pos(label, x, y);
    lv_obj_set_width(label, width);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(colour), 0);
    lv_obj_set_style_text_align(label, align, 0);
    lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
    lv_label_set_text(label, text);

    return label;
}

/* Section title, with optional column headings beside it. */
static void make_section(lv_coord_t y, const char *title,
                         const char *h1, const char *h2, const char *h3)
{
    const char *headers[3] = { h1, h2, h3 };

    /* Titles may run into the first column: they share a line only
     * with the headings, which are short. */
    make_label(CAPTION_X, y, CAPTION_WIDTH + COLUMN_WIDTH,
               FONT_SMALL, SECTION_COLOUR, LV_TEXT_ALIGN_LEFT,
               title);

    for(int i = 0; i < 3; i++)
    {
        if(headers[i] != NULL)
            make_label(COLUMN_X[i], y, COLUMN_WIDTH, FONT_SMALL,
                       HEADER_COLOUR, LV_TEXT_ALIGN_RIGHT, headers[i]);
    }
}

static Row make_row(lv_coord_t y, const char *caption)
{
    make_label(CAPTION_X, y, CAPTION_WIDTH, FONT_BODY,
               HEADER_COLOUR, LV_TEXT_ALIGN_LEFT, caption);

    Row row;

    for(int i = 0; i < 3; i++)
        row.cell[i] = make_label(COLUMN_X[i], y, COLUMN_WIDTH,
                                 FONT_BODY, VALUE_COLOUR,
                                 LV_TEXT_ALIGN_RIGHT, "");

    return row;
}

static void set_cell(lv_obj_t *cell, const char *format, float value)
{
    char text[16];

    snprintf(text, sizeof(text), format, value);
    lv_label_set_text(cell, text);
}

static void set_row(const Row &row, const char *format,
                    float a, float b, float c)
{
    set_cell(row.cell[0], format, a);
    set_cell(row.cell[1], format, b);
    set_cell(row.cell[2], format, c);
}

static void clear_row(const Row &row, const char *first)
{
    lv_label_set_text(row.cell[0], first);
    lv_label_set_text(row.cell[1], "");
    lv_label_set_text(row.cell[2], "");
}

static void update(lv_timer_t *timer)
{
    (void)timer;

    /* Built once and kept, like every screen: only spend time on it
     * while it can be seen. */
    if(lv_scr_act() != s_screen)
        return;

    sensor_readings_t readings;
    sensor_readings_get(&readings);

    if(readings.imu_valid)
    {
        set_row(s_accel, "%+.3f", readings.imu.accel_g.x,
                readings.imu.accel_g.y, readings.imu.accel_g.z);
        set_row(s_gyro, "%+.2f", readings.imu.gyro_dps.x,
                readings.imu.gyro_dps.y, readings.imu.gyro_dps.z);
    }
    else
    {
        clear_row(s_accel, "---");
        clear_row(s_gyro, "---");
    }

    if(readings.mag_valid)
    {
        const mag_sample_t &mag = readings.mag;

        set_row(s_mag_uT, "%+.1f", mag.x_uT, mag.y_uT, mag.z_uT);
        set_row(s_mag_raw, "%.0f", (float)mag.raw[0], (float)mag.raw[1],
                (float)mag.raw[2]);

        set_cell(s_mag_norm.cell[0], "%.1f",
                 sqrtf(mag.x_uT * mag.x_uT + mag.y_uT * mag.y_uT +
                       mag.z_uT * mag.z_uT));
        lv_label_set_text(s_mag_norm.cell[1], "uT");
        lv_label_set_text(s_mag_norm.cell[2], mag.overflow ? "OVERFLOW" : "");
    }
    else
    {
        const char *missing = mag_present() ? "---" : "not fitted";

        clear_row(s_mag_uT, missing);
        clear_row(s_mag_raw, "");
        clear_row(s_mag_norm, "");
    }

    for(int channel = 0; channel < ADS1115_CHANNELS; channel++)
    {
        const Row &row = s_adc[channel];

        if(readings.adc_valid[channel])
        {
            set_cell(row.cell[0], "%.0f", (float)readings.adc_raw[channel]);
            set_cell(row.cell[1], "%.4f", readings.adc_volts[channel]);
            lv_label_set_text(row.cell[2], "");
        }
        else
        {
            clear_row(row, (channel == 0 && !ads1115_present())
                               ? "not fitted" : "---");
        }
    }

    if(baro_present())
    {
        float pressure = 0.0f;
        float temperature = 0.0f;
        navigation_barometer(&pressure, &temperature);

        set_cell(s_baro.cell[0], "%.2f", pressure / 100.0f);
        set_cell(s_baro.cell[1], "%.1f", temperature);
        set_cell(s_baro.cell[2], "%.1f", navigation_sample().vertical.altitude_m);
    }
    else
    {
        clear_row(s_baro, "not fitted");
    }

    char errors[64];

    snprintf(errors, sizeof(errors),
             "errors  imu %u  mag %u  adc %u  fb guard %u",
             (unsigned)readings.imu_errors, (unsigned)readings.mag_errors,
             (unsigned)readings.adc_errors,
             (unsigned)diagnostics_guard_hits());
    lv_label_set_text(s_errors, errors);
}

static void on_screen_loaded(lv_event_t *e)
{
    (void)e;
    diag_stream_begin(DIAG_STREAM_DIAGNOSTICS);
}

static void on_screen_unloaded(lv_event_t *e)
{
    (void)e;
    diag_stream_end(DIAG_STREAM_DIAGNOSTICS);
}

void diagnostics_screen_create(lv_obj_t *screen)
{
    s_screen = screen;

    lv_obj_t *title = lv_label_create(screen);

    lv_label_set_text(title, "DIAGNOSTICS");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_style_text_font(title, FONT_LARGE, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 40);

    /* Each line a step below the last, sized for its font: section
     * headings in FONT_SMALL, rows in FONT_BODY (fonts.h). */
    lv_coord_t y = 68;

    make_section(y, "IMU QMI8658", "X", "Y", "Z");
    y += HEADING_STEP;
    s_accel = make_row(y, "acc g");
    y += ROW_STEP;
    s_gyro = make_row(y, "gyro dps");
    y += ROW_STEP + SECTION_GAP;

    char mag_title[32];

    if(mag_present())
        snprintf(mag_title, sizeof(mag_title), "MAG %s", mag_chip_name());
    else
        snprintf(mag_title, sizeof(mag_title), "MAG");

    make_section(y, mag_title, "X", "Y", "Z");
    y += HEADING_STEP;
    s_mag_uT = make_row(y, "uT");
    y += ROW_STEP;
    s_mag_raw = make_row(y, "raw");
    y += ROW_STEP;
    s_mag_norm = make_row(y, "|B|");
    y += ROW_STEP + SECTION_GAP;

    char adc_title[32];

    if(ads1115_present())
        snprintf(adc_title, sizeof(adc_title), "ADS1115 0x%02X",
                 ads1115_address());
    else
        snprintf(adc_title, sizeof(adc_title), "ADS1115");

    make_section(y, adc_title, "raw", "V", NULL);
    y += HEADING_STEP;

    for(int channel = 0; channel < ADS1115_CHANNELS; channel++)
    {
        static const char *const names[ADS1115_CHANNELS] = {
            "A0", "A1", "A2", "A3"
        };

        s_adc[channel] = make_row(y, names[channel]);
        y += ROW_STEP;
    }

    y += SECTION_GAP;

    make_section(y, "BARO BMP280", "hPa", "C", "alt m");
    y += HEADING_STEP;
    s_baro = make_row(y, "");
    y += ROW_STEP + SECTION_GAP;

    s_errors = make_label(0, y, 480, FONT_SMALL,
                          HEADER_COLOUR, LV_TEXT_ALIGN_CENTER, "");
    y += HEADING_STEP;

    /* Why the unit last restarted, and what crashed if anything did:
     * a restart away from the serial console leaves no other trace. */
    lv_obj_t *crash = make_label(0, y, 480, FONT_SMALL,
                                 ABSENT_COLOUR, LV_TEXT_ALIGN_CENTER,
                                 crash_report_text());
    lv_label_set_long_mode(crash, LV_LABEL_LONG_WRAP);

    /* Stream to the PC only while this screen is up. */
    lv_obj_add_event_cb(screen, on_screen_loaded, LV_EVENT_SCREEN_LOADED, NULL);
    lv_obj_add_event_cb(screen, on_screen_unloaded, LV_EVENT_SCREEN_UNLOADED,
                        NULL);

    lv_timer_create(update, UPDATE_PERIOD_MS, NULL);
}
