#include "web_log.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* In PSRAM - nothing here is in a hurry. */
struct TextRing
{
    char *text;
    size_t size;
    uint32_t written;           /* bytes ever written */
    SemaphoreHandle_t lock;

    bool init(size_t bytes)
    {
        text = (char *)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);
        size = bytes;
        written = 0;
        lock = xSemaphoreCreateMutex();

        return text != NULL && lock != NULL;
    }

    bool ready() const
    {
        return text != NULL && lock != NULL;
    }

    /* Under the lock. */
    void append(const char *data, size_t length)
    {
        for(size_t i = 0; i < length; i++)
            text[(written + i) % size] = data[i];

        written += length;
    }

    size_t read(uint32_t *position, char *buffer, size_t capacity)
    {
        if(!ready() || xSemaphoreTake(lock, portMAX_DELAY) != pdTRUE)
            return 0;

        const uint32_t oldest = (written > size) ? written - size : 0;

        /* Fallen behind, or a position from before a restart. */
        if(*position < oldest || *position > written)
            *position = oldest;

        size_t length = written - *position;

        if(length > capacity)
            length = capacity;

        for(size_t i = 0; i < length; i++)
            buffer[i] = text[(*position + i) % size];

        *position += length;

        xSemaphoreGive(lock);

        return length;
    }

    uint32_t end()
    {
        if(!ready() || xSemaphoreTake(lock, portMAX_DELAY) != pdTRUE)
            return 0;

        const uint32_t position = written;

        xSemaphoreGive(lock);

        return position;
    }
};

/* About four minutes of the diagnostics' 600 bytes a second, so the
 * boot log is still there when the page is first opened. */
static const size_t LOG_BYTES = 128 * 1024;

/* About ten seconds of sentences at 20 a second. */
static const size_t STREAM_BYTES = 64 * 1024;

/* About two minutes of telemetry at its 2.5 KB a second. */
static const size_t TELEMETRY_BYTES = 128 * 1024;

static TextRing s_log;
static TextRing s_stream;
static TextRing s_telemetry;

/*
 * Formatting happens under the lock, into a static line, rather than
 * into a buffer on the caller's stack: every task logs, and the
 * smallest stacks have about 1 KB to spare (README, "Unexpected
 * restarts").
 */
static char s_line[256];

static vprintf_like_t s_console = NULL;

static int capture(const char *format, va_list args)
{
    va_list copy;
    va_copy(copy, args);

    /* Never wait long: a line missing from the web copy is better than
     * the IMU task held up behind a slow one. */
    if(xSemaphoreTake(s_log.lock, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        const int length = vsnprintf(s_line, sizeof(s_line), format, copy);

        if(length > 0)
            s_log.append(s_line, ((size_t)length < sizeof(s_line))
                                     ? (size_t)length : sizeof(s_line) - 1);

        xSemaphoreGive(s_log.lock);
    }

    va_end(copy);

    return s_console(format, args);
}

void web_log_init(void)
{
    if(!s_stream.init(STREAM_BYTES))
        ESP_LOGE("web_log", "no memory for the stream copy");

    if(!s_telemetry.init(TELEMETRY_BYTES))
        ESP_LOGE("web_log", "no memory for telemetry");

    if(!s_log.init(LOG_BYTES))
    {
        ESP_LOGE("web_log", "no memory for the log copy");
        return;
    }

    s_console = esp_log_set_vprintf(capture);
}

size_t web_log_read(uint32_t *position, char *buffer, size_t size)
{
    return s_log.read(position, buffer, size);
}

size_t web_stream_read(uint32_t *position, char *buffer, size_t size)
{
    return s_stream.read(position, buffer, size);
}

uint32_t web_log_end(void)
{
    return s_log.end();
}

uint32_t web_stream_end(void)
{
    return s_stream.end();
}

void web_stream_append(const char *text, size_t length)
{
    if(!s_stream.ready() ||
       xSemaphoreTake(s_stream.lock, pdMS_TO_TICKS(5)) != pdTRUE)
        return;

    s_stream.append(text, length);

    xSemaphoreGive(s_stream.lock);
}

size_t web_telemetry_read(uint32_t *position, char *buffer, size_t size)
{
    return s_telemetry.read(position, buffer, size);
}

uint32_t web_telemetry_end(void)
{
    return s_telemetry.end();
}

void web_telemetry_append(const char *text, size_t length)
{
    if(!s_telemetry.ready() ||
       xSemaphoreTake(s_telemetry.lock, pdMS_TO_TICKS(5)) != pdTRUE)
        return;

    s_telemetry.append(text, length);

    xSemaphoreGive(s_telemetry.lock);
}
