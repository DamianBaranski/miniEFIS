#include "update_screen.h"

#include "display_st7701.h"
#include "lvgl_port.h"

#include "esp_attr.h"
#include "esp_log.h"

#include <stdio.h>
#include <string.h>

static const char *TAG = "update_screen";

/*
 * Everything below that the interrupt reads is DRAM_ATTR and every
 * function it calls IRAM_ATTR: flash and PSRAM may be unreadable at the
 * moment it runs. Hence no LVGL, no library calls, no floating point.
 */

static const int WIDTH = BOARD_LCD_WIDTH;

/* RGB565, as the panel takes it. */
static const uint16_t BLACK = 0x0000;
static const uint16_t WHITE = 0xFFFF;
static const uint16_t YELLOW = 0xFEA0;          /* 0xFFD400 */
static const uint16_t GREY = 0x8451;            /* 0x808890 */
static const uint16_t RED = 0xFA08;             /* 0xFF4040 */
static const uint16_t BAR_BACK = 0x31C8;        /* 0x303840 */
static const uint16_t BAR_FILL = 0x1C3A;        /* 0x1987D1 */

/* A line of text: where, how big, what colour - and the text itself,
 * which the update's task rewrites. */
struct TextLine
{
    int y;
    int scale;
    uint16_t colour;
    char text[24];
};

static const int LINE_COUNT = 3;

static DRAM_ATTR TextLine s_lines[LINE_COUNT] = {
    /* 4x is 404 px for the title: inside the glass at that height. */
    { 150, 4, YELLOW, "UPDATING FIRMWARE" },
    { 205, 4, WHITE, "RECEIVING" },
    { 300, 3, GREY, "DO NOT SWITCH OFF" },
};

static const int BAR_X = 100;
static const int BAR_Y = 260;
static const int BAR_WIDTH = 280;
static const int BAR_HEIGHT = 16;

/* 0..100, the whole bar. */
static DRAM_ATTR volatile int s_bar_percent = 0;

/*
 * 5x7 capitals, digits and a few signs, one byte per row, bit 4 the
 * leftmost column - all the update ever says. Indexed by character
 * minus ' '; anything missing draws as a space.
 */
static const int GLYPH_FIRST = ' ';
static const int GLYPH_LAST = 'Z';

static DRAM_ATTR const uint8_t GLYPHS[GLYPH_LAST - GLYPH_FIRST + 1][7] = {
    /* ' ' */ { 0, 0, 0, 0, 0, 0, 0 },
    /* '!' */ { 0, 0, 0, 0, 0, 0, 0 },
    /* '"' */ { 0, 0, 0, 0, 0, 0, 0 },
    /* '#' */ { 0, 0, 0, 0, 0, 0, 0 },
    /* '$' */ { 0, 0, 0, 0, 0, 0, 0 },
    /* '%' */ { 0x18, 0x19, 0x02, 0x04, 0x08, 0x13, 0x03 },
    /* '&' */ { 0, 0, 0, 0, 0, 0, 0 },
    /* ''' */ { 0, 0, 0, 0, 0, 0, 0 },
    /* '(' */ { 0, 0, 0, 0, 0, 0, 0 },
    /* ')' */ { 0, 0, 0, 0, 0, 0, 0 },
    /* '*' */ { 0, 0, 0, 0, 0, 0, 0 },
    /* '+' */ { 0, 0, 0, 0, 0, 0, 0 },
    /* ',' */ { 0, 0, 0, 0, 0, 0, 0 },
    /* '-' */ { 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00 },
    /* '.' */ { 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C },
    /* '/' */ { 0, 0, 0, 0, 0, 0, 0 },
    /* '0' */ { 0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E },
    /* '1' */ { 0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E },
    /* '2' */ { 0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F },
    /* '3' */ { 0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E },
    /* '4' */ { 0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02 },
    /* '5' */ { 0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E },
    /* '6' */ { 0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E },
    /* '7' */ { 0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08 },
    /* '8' */ { 0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E },
    /* '9' */ { 0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C },
    /* ':' */ { 0, 0, 0, 0, 0, 0, 0 },
    /* ';' */ { 0, 0, 0, 0, 0, 0, 0 },
    /* '<' */ { 0, 0, 0, 0, 0, 0, 0 },
    /* '=' */ { 0, 0, 0, 0, 0, 0, 0 },
    /* '>' */ { 0, 0, 0, 0, 0, 0, 0 },
    /* '?' */ { 0, 0, 0, 0, 0, 0, 0 },
    /* '@' */ { 0, 0, 0, 0, 0, 0, 0 },
    /* 'A' */ { 0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11 },
    /* 'B' */ { 0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E },
    /* 'C' */ { 0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E },
    /* 'D' */ { 0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E },
    /* 'E' */ { 0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F },
    /* 'F' */ { 0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10 },
    /* 'G' */ { 0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0F },
    /* 'H' */ { 0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11 },
    /* 'I' */ { 0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E },
    /* 'J' */ { 0x07, 0x02, 0x02, 0x02, 0x02, 0x12, 0x0C },
    /* 'K' */ { 0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11 },
    /* 'L' */ { 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F },
    /* 'M' */ { 0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11 },
    /* 'N' */ { 0x11, 0x11, 0x19, 0x15, 0x13, 0x11, 0x11 },
    /* 'O' */ { 0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E },
    /* 'P' */ { 0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10 },
    /* 'Q' */ { 0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D },
    /* 'R' */ { 0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11 },
    /* 'S' */ { 0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E },
    /* 'T' */ { 0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04 },
    /* 'U' */ { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E },
    /* 'V' */ { 0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04 },
    /* 'W' */ { 0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0A },
    /* 'X' */ { 0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11 },
    /* 'Y' */ { 0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04 },
    /* 'Z' */ { 0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F },
};

/* Glyph cell: 5 columns and one of spacing. */
static const int CELL = 6;

static IRAM_ATTR int text_length(const char *text)
{
    int length = 0;

    while(length < (int)sizeof(TextLine::text) && text[length] != '\0')
        length++;

    return length;
}

/* One pixel row of one line of text, centred, into row[]. */
static IRAM_ATTR void draw_text_row(uint16_t *row, const TextLine &line,
                                    int glyph_row)
{
    const int length = text_length(line.text);
    const int width = (length * CELL - 1) * line.scale;

    int x = (WIDTH - width) / 2;

    for(int i = 0; i < length; i++, x += CELL * line.scale)
    {
        const int c = line.text[i];

        if(c < GLYPH_FIRST || c > GLYPH_LAST)
            continue;

        const uint8_t bits = GLYPHS[c - GLYPH_FIRST][glyph_row];

        if(bits == 0)
            continue;

        for(int column = 0; column < 5; column++)
        {
            if((bits & (0x10 >> column)) == 0)
                continue;

            const int start = x + column * line.scale;

            for(int s = 0; s < line.scale; s++)
            {
                const int px = start + s;

                if(px >= 0 && px < WIDTH)
                    row[px] = line.colour;
            }
        }
    }
}

static IRAM_ATTR void draw_row(uint16_t *row, int y)
{
    uint32_t *words = (uint32_t *)row;

    for(int i = 0; i < WIDTH / 2; i++)
        words[i] = ((uint32_t)BLACK << 16) | BLACK;

    for(int l = 0; l < LINE_COUNT; l++)
    {
        const TextLine &line = s_lines[l];
        const int glyph_row = (y - line.y) / line.scale;

        if(y >= line.y && glyph_row < 7)
            draw_text_row(row, line, glyph_row);
    }

    if(y >= BAR_Y && y < BAR_Y + BAR_HEIGHT)
    {
        const int filled = BAR_WIDTH * s_bar_percent / 100;

        for(int x = 0; x < BAR_WIDTH; x++)
            row[BAR_X + x] = (x < filled) ? BAR_FILL : BAR_BACK;
    }
}

/* The panel's bounce buffer callback: whole rows, since the bounce
 * buffer is a whole number of them and starts on a row. */
static IRAM_ATTR bool fill(esp_lcd_panel_handle_t panel, void *buffer,
                           int pos_px, int len_bytes, void *context)
{
    (void)panel;
    (void)context;

    uint16_t *pixels = (uint16_t *)buffer;
    const int count = len_bytes / 2;

    for(int done = 0; done < count; done += WIDTH)
        draw_row(pixels + done, (pos_px + done) / WIDTH);

    return false;
}

static void set_line(int index, const char *text, uint16_t colour)
{
    /* Written while the interrupt may be reading it: a frame with half
     * of one word and half of the next is harmless, and the next frame
     * is whole. */
    strlcpy(s_lines[index].text, text, sizeof(s_lines[index].text));
    s_lines[index].colour = colour;
}

bool update_screen_begin(void)
{
    if(lvgl_port_stop(1000) != ESP_OK)
    {
        ESP_LOGW(TAG, "LVGL did not stop; the instruments stay up");
        return false;
    }

    const esp_err_t err = display_generate(fill, NULL);

    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "could not switch the panel: %s", esp_err_to_name(err));
        return false;
    }

    return true;
}

void update_screen_progress(update_screen_phase_t phase, uint8_t percent)
{
    if(percent > 100)
        percent = 100;

    char text[24];

    switch(phase)
    {
        case UPDATE_SCREEN_RECEIVING:
            snprintf(text, sizeof(text), "RECEIVING %u%%", (unsigned)percent);
            /* Receiving is about a tenth of the time. */
            s_bar_percent = percent / 10;
            break;

        case UPDATE_SCREEN_WRITING:
            snprintf(text, sizeof(text), "INSTALLING %u%%", (unsigned)percent);
            s_bar_percent = 10 + percent * 90 / 100;
            break;

        default:
            snprintf(text, sizeof(text), "RESTARTING");
            s_bar_percent = 100;
            break;
    }

    set_line(1, text, WHITE);
}

void update_screen_failed(void)
{
    set_line(1, "UPDATE FAILED", RED);
    set_line(2, "RESTARTING", GREY);
}
