#include "lvgl_port.h"
#include "board_pins.h"
#include "display_st7701.h"
#include "touch_cst820.h"

#include "esp_lcd_panel_rgb.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "lvgl";

static const uint32_t TICK_PERIOD_MS = 2;
static const uint32_t HANDLER_PERIOD_MS = 5;
static const int TASK_STACK_BYTES = 8192;
static const int TASK_PRIORITY = 2;
static const int TASK_CORE = 1;         /* away from the IMU task on core 0 */
/* Longest a frame switch can take: two panel frames, with margin. */
static const int FRAME_TIMEOUT_MS = 100;

static lv_disp_drv_t s_disp_drv;
static lv_disp_draw_buf_t s_draw_buf;
static lvgl_port_ui_cb_t s_build_ui = NULL;

/*
 * Every vertical sync, and every frame the panel's interrupt finished
 * copying out of a framebuffer, since start-up. Equal while the panel
 * keeps up; a frame cut short - its refills fell behind, and the DMA
 * was restarted at the next vertical sync - shows as one more vsync
 * than frames, and on the screen as a flicker.
 */
static volatile uint32_t s_vsyncs_total = 0;
static volatile uint32_t s_frames_total = 0;
static SemaphoreHandle_t s_frame_copied = NULL;

/* The framebuffer the panel is showing: the last one flushed. */
static const void *volatile s_shown_frame = NULL;

/* lvgl_port_stop(): asked for, and answered by the LVGL task. */
static volatile bool s_stop_requested = false;
static SemaphoreHandle_t s_stopped = NULL;

static portMUX_TYPE s_stats_lock = portMUX_INITIALIZER_UNLOCKED;
static lvgl_port_stats_t s_stats;
static int64_t s_handler_started = 0;

/* Runs in an ISR, in IRAM: the panel's interrupt is IRAM-safe
 * (CONFIG_LCD_RGB_ISR_IRAM_SAFE) and keeps running while flash is
 * written, which the driver only allows with callbacks in IRAM. */
static IRAM_ATTR bool on_vsync(esp_lcd_panel_handle_t panel,
                     const esp_lcd_rgb_panel_event_data_t *event,
                     void *user_ctx)
{
    (void)panel;
    (void)event;
    (void)user_ctx;

    s_stats.vsyncs++;
    s_vsyncs_total = s_vsyncs_total + 1;

    return false;
}

/* Runs in an ISR, in IRAM: a whole frame copied out to the panel. */
static IRAM_ATTR bool on_frame_copied(esp_lcd_panel_handle_t panel,
                                      const esp_lcd_rgb_panel_event_data_t *event,
                                      void *user_ctx)
{
    (void)panel;
    (void)event;
    (void)user_ctx;

    s_frames_total = s_frames_total + 1;

    BaseType_t task_woken = pdFALSE;

    if(s_frame_copied != NULL)
        xSemaphoreGiveFromISR(s_frame_copied, &task_woken);

    return task_woken == pdTRUE;
}

/*
 * Hand the finished framebuffer to the panel, and give LVGL the other
 * one back only once the panel has stopped reading it.
 *
 * With two framebuffers in PSRAM, LVGL draws a complete frame into the
 * one it is given and the flush is just a pointer swap. In bounce
 * buffer mode the swap does not happen when it is asked for: the
 * panel's interrupt copies the picture into the bounce buffers ahead of
 * the scan, and only picks up the new framebuffer when it starts
 * copying the next frame - which is when it calls on_frame_copied().
 * Until then it is still reading the old one.
 *
 * Releasing the old buffer to LVGL straight away - or at the next
 * VSYNC, which is no better, since the interrupt is ahead of it - lets
 * LVGL start the next frame in a buffer still being shown. The panel
 * then scans out a half drawn picture: the background repainted, the
 * horizon lines and numbers not yet. That is a blink, and the faster
 * the drawing, the more of the frame it covers. So the flush waits for
 * the frame boundary the driver actually switches at. The switch
 * itself lands between whole frames, so nothing tears either.
 */
static void flush_cb(lv_disp_drv_t *drv, const lv_area_t *area,
                     lv_color_t *pixels)
{
    const int64_t wait_started = esp_timer_get_time();
    const uint32_t drawing_us = (uint32_t)(wait_started - s_handler_started);

    display_draw_bitmap(area->x1, area->y1, area->x2, area->y2, pixels);

    /*
     * A frame copied from here on was copied from the new buffer. One
     * that slipped in between the swap and this read is missed, and
     * costs a frame of waiting - never a buffer released too early.
     */
    const uint32_t frames_before = s_frames_total;

    while(s_frames_total == frames_before)
    {
        if(xSemaphoreTake(s_frame_copied, pdMS_TO_TICKS(FRAME_TIMEOUT_MS))
               != pdTRUE)
        {
            /* The panel is not refreshing. Carry on rather than freeze
             * the UI, but say so once. */
            static bool warned = false;

            if(!warned)
            {
                warned = true;
                ESP_LOGW(TAG, "panel not switching frames within %d ms",
                         (int)FRAME_TIMEOUT_MS);
            }

            break;
        }
    }

    const uint32_t waited_us = (uint32_t)(esp_timer_get_time() - wait_started);

    s_shown_frame = pixels;
    lv_disp_flush_ready(drv);

    portENTER_CRITICAL(&s_stats_lock);
    s_stats.frames++;
    if(waited_us > s_stats.flush_wait_max_us)
        s_stats.flush_wait_max_us = waited_us;
    if(drawing_us > s_stats.draw_max_us)
        s_stats.draw_max_us = drawing_us;
    portEXIT_CRITICAL(&s_stats_lock);
}

void lvgl_port_take_stats(lvgl_port_stats_t *stats)
{
    if(stats == NULL)
        return;

    /* Read apart, so either may be one frame ahead of the other. */
    const uint32_t vsyncs = s_vsyncs_total;
    const uint32_t frames = s_frames_total;

    portENTER_CRITICAL(&s_stats_lock);
    *stats = s_stats;
    stats->frames_cut = (vsyncs > frames) ? vsyncs - frames : 0;
    s_stats.frames = 0;
    s_stats.handler_max_us = 0;
    s_stats.vsyncs = 0;
    s_stats.flush_wait_max_us = 0;
    s_stats.draw_max_us = 0;
    portEXIT_CRITICAL(&s_stats_lock);
}

static void touch_read_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    (void)drv;

    touch_state_t touch;

    if(touch_read(&touch) != ESP_OK || !touch.pressed)
    {
        data->state = LV_INDEV_STATE_REL;
        return;
    }

    data->point.x = touch.x;
    data->point.y = touch.y;
    data->state = LV_INDEV_STATE_PR;
}

static void tick_cb(void *argument)
{
    (void)argument;
    lv_tick_inc(TICK_PERIOD_MS);
}

static esp_err_t display_register(void)
{
    /*
     * LVGL draws straight into the panel's two PSRAM framebuffers, so
     * there is no intermediate buffer and no copy: with full_refresh the
     * flush just hands the finished buffer to the RGB peripheral.
     */
    void *frame_buffer_1 = NULL;
    void *frame_buffer_2 = NULL;

    const esp_err_t err = esp_lcd_rgb_panel_get_frame_buffer(
        display_panel(), 2, &frame_buffer_1, &frame_buffer_2);

    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "no framebuffers: %s", esp_err_to_name(err));
        return err;
    }

    s_frame_copied = xSemaphoreCreateBinary();

    if(s_frame_copied == NULL)
        return ESP_ERR_NO_MEM;

    esp_lcd_rgb_panel_event_callbacks_t callbacks = {};
    callbacks.on_vsync = on_vsync;
    callbacks.on_bounce_frame_finish = on_frame_copied;

    const esp_err_t callback_err =
        esp_lcd_rgb_panel_register_event_callbacks(display_panel(),
                                                   &callbacks, NULL);

    if(callback_err != ESP_OK)
    {
        ESP_LOGW(TAG, "no panel callbacks (%s); frames may blink",
                 esp_err_to_name(callback_err));
    }

    lv_disp_draw_buf_init(&s_draw_buf, frame_buffer_1, frame_buffer_2,
                          BOARD_LCD_WIDTH * BOARD_LCD_HEIGHT);

    lv_disp_drv_init(&s_disp_drv);

    s_disp_drv.hor_res = BOARD_LCD_WIDTH;
    s_disp_drv.ver_res = BOARD_LCD_HEIGHT;
    s_disp_drv.flush_cb = flush_cb;
    s_disp_drv.draw_buf = &s_draw_buf;
    s_disp_drv.full_refresh = 1;

    return lv_disp_drv_register(&s_disp_drv) != NULL
        ? ESP_OK
        : ESP_FAIL;
}

static void input_register(void)
{
    static lv_indev_drv_t indev_drv;

    lv_indev_drv_init(&indev_drv);

    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touch_read_cb;

    lv_indev_drv_register(&indev_drv);
}

static esp_err_t tick_start(void)
{
    esp_timer_create_args_t timer_args = {};

    timer_args.callback = tick_cb;
    timer_args.name = "lvgl_tick";

    esp_timer_handle_t timer = NULL;

    const esp_err_t err = esp_timer_create(&timer_args, &timer);

    if(err != ESP_OK)
        return err;

    return esp_timer_start_periodic(timer, TICK_PERIOD_MS * 1000);
}

static void lvgl_task(void *argument)
{
    (void)argument;

    lv_init();

    if(display_register() != ESP_OK)
    {
        ESP_LOGE(TAG, "display registration failed, task exiting");
        vTaskDelete(NULL);
        return;
    }

    input_register();

    if(tick_start() != ESP_OK)
        ESP_LOGE(TAG, "tick timer failed; LVGL animations will not run");

    if(s_build_ui != NULL)
        s_build_ui();

    ESP_LOGI(TAG, "LVGL running");

    while(true)
    {
        if(s_stop_requested)
        {
            ESP_LOGI(TAG, "LVGL stopped");
            xSemaphoreGive(s_stopped);
            vTaskSuspend(NULL);
        }

        const int64_t started_us = esp_timer_get_time();
        s_handler_started = started_us;

        lv_timer_handler();

        const uint32_t elapsed_us =
            (uint32_t)(esp_timer_get_time() - started_us);

        portENTER_CRITICAL(&s_stats_lock);
        if(elapsed_us > s_stats.handler_max_us)
            s_stats.handler_max_us = elapsed_us;
        portEXIT_CRITICAL(&s_stats_lock);

        vTaskDelay(pdMS_TO_TICKS(HANDLER_PERIOD_MS));
    }
}

const void *lvgl_port_shown_frame(void)
{
    return s_shown_frame;
}

esp_err_t lvgl_port_stop(uint32_t timeout_ms)
{
    if(s_stopped == NULL)
        return ESP_ERR_INVALID_STATE;

    s_stop_requested = true;

    return (xSemaphoreTake(s_stopped, pdMS_TO_TICKS(timeout_ms)) == pdTRUE)
               ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t lvgl_port_start(lvgl_port_ui_cb_t build_ui)
{
    s_build_ui = build_ui;
    s_stopped = xSemaphoreCreateBinary();

    const BaseType_t created = xTaskCreatePinnedToCore(
        lvgl_task, "lvgl", TASK_STACK_BYTES, NULL, TASK_PRIORITY, NULL,
        TASK_CORE);

    return (created == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
}
