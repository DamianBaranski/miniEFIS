#include "display_st7701.h"
#include "board_pins.h"
#include "tca9554.h"

#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_idf_version.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "display";

static const ledc_timer_t BACKLIGHT_TIMER = LEDC_TIMER_0;
static const ledc_channel_t BACKLIGHT_CHANNEL = LEDC_CHANNEL_0;
static const ledc_timer_bit_t BACKLIGHT_RESOLUTION = LEDC_TIMER_10_BIT;
static const uint32_t BACKLIGHT_MAX_DUTY = (1u << 10) - 1u;

static spi_device_handle_t s_spi = NULL;
static esp_lcd_panel_handle_t s_panel = NULL;
static uint8_t s_backlight_percent = 50;

/* -------------------------------------------------------------------------- */
/* Register interface                                                         */
/* -------------------------------------------------------------------------- */

/*
 * The ST7701 takes a 9-bit word on this link: one leading bit selecting
 * command or data, then the byte itself. That is expressed as a 1-bit
 * SPI command phase plus an 8-bit address phase with no payload, which
 * is why both helpers below send zero-length transactions.
 */
static void write_command(uint8_t command)
{
    spi_transaction_t transaction = {};

    transaction.cmd = 0;
    transaction.addr = command;

    spi_device_transmit(s_spi, &transaction);
}

static void write_data(uint8_t data)
{
    spi_transaction_t transaction = {};

    transaction.cmd = 1;
    transaction.addr = data;

    spi_device_transmit(s_spi, &transaction);
}

static void chip_select(bool enable)
{
    tca9554_write_pin(BOARD_EXIO_LCD_CS,
                      enable ? TCA9554_LOW : TCA9554_HIGH);
    vTaskDelay(pdMS_TO_TICKS(10));
}

static void panel_reset(void)
{
    tca9554_write_pin(BOARD_EXIO_LCD_RESET, TCA9554_LOW);
    vTaskDelay(pdMS_TO_TICKS(10));
    tca9554_write_pin(BOARD_EXIO_LCD_RESET, TCA9554_HIGH);
    vTaskDelay(pdMS_TO_TICKS(50));
}

static esp_err_t register_link_init(void)
{
    spi_bus_config_t bus_config = {};

    bus_config.mosi_io_num = BOARD_LCD_SPI_MOSI_PIN;
    bus_config.miso_io_num = -1;
    bus_config.sclk_io_num = BOARD_LCD_SPI_CLK_PIN;
    bus_config.quadwp_io_num = -1;
    bus_config.quadhd_io_num = -1;
    bus_config.max_transfer_sz = 64;

    esp_err_t err =
        spi_bus_initialize(BOARD_LCD_SPI_HOST, &bus_config, SPI_DMA_CH_AUTO);

    if(err != ESP_OK)
        return err;

    spi_device_interface_config_t device_config = {};

    device_config.command_bits = 1;
    device_config.address_bits = 8;
    device_config.mode = 0;
    device_config.clock_speed_hz = BOARD_LCD_SPI_FREQ_HZ;
    device_config.spics_io_num = -1;        /* CS is on the IO expander */
    device_config.queue_size = 1;

    return spi_bus_add_device(BOARD_LCD_SPI_HOST, &device_config, &s_spi);
}

/*
 * Panel initialisation sequence, as supplied by the board vendor.
 *
 * These are undocumented ST7701 factory registers (gamma curves, power
 * rails, gate timing) and the values are specific to this glass. They
 * are kept verbatim; there is nothing to derive them from.
 */
static void panel_registers_init(void)
{
    chip_select(true);
    write_command(0xFF);
    write_data(0x77);
    write_data(0x01);
    write_data(0x00);
    write_data(0x00);
    write_data(0x10);

    write_command(0xC0);
    write_data(0x3B);//Scan line	
    write_data(0x00);

    write_command(0xC1);
    write_data(0x0B);	//VBP
    write_data(0x02);

    write_command(0xC2);
    write_data(0x07);
    write_data(0x02);

    write_command(0xCC);
    write_data(0x10);

    write_command(0xCD);//RGB format
    write_data(0x08);

    write_command(0xB0); // IPS   
    write_data(0x00); // 255 
    write_data(0x11); // 251    
    write_data(0x16); // 247  down
    write_data(0x0e); // 239    
    write_data(0x11); // 231    
    write_data(0x06); // 203    
    write_data(0x05); // 175 
    write_data(0x09); // 147    
    write_data(0x08); // 108    
    write_data(0x21); // 80  
    write_data(0x06); // 52   
    write_data(0x13); // 24    
    write_data(0x10); // 16    
    write_data(0x29); // 8    down
    write_data(0x31); // 4    
    write_data(0x18); // 0   

    write_command(0xB1);//  IPS	   
    write_data(0x00);//  255 
    write_data(0x11);//  251
    write_data(0x16);//  247   down
    write_data(0x0e);//  239
    write_data(0x11);//  231
    write_data(0x07);//  203    
    write_data(0x05);//  175
    write_data(0x09);//  147  
    write_data(0x09);//  108  
    write_data(0x21);//  80 
    write_data(0x05);//  52   
    write_data(0x13);//  24 
    write_data(0x11);//  16 
    write_data(0x2a);//  8  down 
    write_data(0x31);//  4  
    write_data(0x18);//  0  

    write_command(0xFF);
    write_data(0x77);
    write_data(0x01);
    write_data(0x00);
    write_data(0x00);
    write_data(0x11);

    write_command(0xB0);  //VOP  3.5375+ *x 0.0125
    write_data(0x6d);  //5D
  
    write_command(0xB1); 	//VCOM amplitude setting  
    write_data(0x37);  //
  
    write_command(0xB2); 	//VGH Voltage setting  
    write_data(0x81);	//12V

    write_command(0xB3);
    write_data(0x80);

    write_command(0xB5); 	//VGL Voltage setting  
    write_data(0x43);	//-8.3V

    write_command(0xB7);
    write_data(0x85);

    write_command(0xB8);
    write_data(0x20);

    write_command(0xC1);
    write_data(0x78);

    write_command(0xC2);
    write_data(0x78);

    write_command(0xD0);
    write_data(0x88);

    write_command(0xE0);
    write_data(0x00);
    write_data(0x00);
    write_data(0x02);

    write_command(0xE1);
    write_data(0x03);	
    write_data(0xA0);
    write_data(0x00);	
    write_data(0x00);
    write_data(0x04);	
    write_data(0xA0);
    write_data(0x00);	
    write_data(0x00);	
    write_data(0x00);
    write_data(0x20);
    write_data(0x20);

    write_command(0xE2);
    write_data(0x00);	
    write_data(0x00);	  
    write_data(0x00);	
    write_data(0x00);	
    write_data(0x00);	
    write_data(0x00);	
    write_data(0x00);	
    write_data(0x00);	
    write_data(0x00);	
    write_data(0x00);	
    write_data(0x00);	
    write_data(0x00);	
    write_data(0x00);

    write_command(0xE3);
    write_data(0x00);
    write_data(0x00);
    write_data(0x11);
    write_data(0x00);

    write_command(0xE4);
    write_data(0x22);
    write_data(0x00);

    write_command(0xE5);		
    write_data(0x05);	
    write_data(0xEC);	
    write_data(0xA0);
    write_data(0xA0);
    write_data(0x07);	
    write_data(0xEE);	
    write_data(0xA0);
    write_data(0xA0);
    write_data(0x00);
    write_data(0x00);	
    write_data(0x00);
    write_data(0x00);
    write_data(0x00);
    write_data(0x00);	
    write_data(0x00);
    write_data(0x00);

    write_command(0xE6);
    write_data(0x00);
    write_data(0x00);
    write_data(0x11);
    write_data(0x00);

    write_command(0xE7);
    write_data(0x22);
    write_data(0x00);

    write_command(0xE8);		
    write_data(0x06);	
    write_data(0xED);	
    write_data(0xA0);
    write_data(0xA0);
    write_data(0x08);	
    write_data(0xEF);	
    write_data(0xA0); 
    write_data(0xA0);
    write_data(0x00);	
    write_data(0x00);	
    write_data(0x00);
    write_data(0x00);
    write_data(0x00);	
    write_data(0x00);	
    write_data(0x00);
    write_data(0x00);

    write_command(0xEB);
    write_data(0x00); 	
    write_data(0x00);
    write_data(0x40);
    write_data(0x40);
    write_data(0x00);
    write_data(0x00);
    write_data(0x00);  

    write_command(0xED);  
    write_data(0xFF); 
    write_data(0xFF);  
    write_data(0xFF); 	
    write_data(0xBA); 		
    write_data(0x0A); 	
    write_data(0xBF); 	
    write_data(0x45); 	
    write_data(0xFF); 
    write_data(0xFF);  
    write_data(0x54); 	
    write_data(0xFB); 	
    write_data(0xA0); 	
    write_data(0xAB); 	
    write_data(0xFF); 
    write_data(0xFF); 
    write_data(0xFF); 

    write_command(0xEF);
    write_data(0x10); 
    write_data(0x0D); 
    write_data(0x04); 
    write_data(0x08); 
    write_data(0x3F); 
    write_data(0x1F);

    write_command(0xFF);
    write_data(0x77);
    write_data(0x01);
    write_data(0x00);
    write_data(0x00);
    write_data(0x13);

    write_command(0xEF);
    write_data(0x08);

    write_command(0xFF);
    write_data(0x77);
    write_data(0x01);
    write_data(0x00);
    write_data(0x00);
    write_data(0x00);


    write_command(0x36);
    write_data(0x00);

    write_command(0x3A);
    write_data(0x66);

    write_command(0x11);
  
    vTaskDelay(pdMS_TO_TICKS(480));

    write_command(0x20); //
    vTaskDelay(pdMS_TO_TICKS(120));
    write_command(0x29); 
    chip_select(false);
}

/* -------------------------------------------------------------------------- */
/* RGB panel                                                                  */
/* -------------------------------------------------------------------------- */

/*
 * fill == NULL: the normal panel, streaming from two framebuffers in
 * PSRAM. Otherwise no framebuffer at all, and every bounce buffer is
 * filled by fill (display_generate()).
 */
static esp_err_t rgb_panel_init(esp_lcd_rgb_panel_bounce_buf_fill_cb_t fill,
                                void *context)
{
    esp_lcd_rgb_panel_config_t config = {};

    config.clk_src = LCD_CLK_SRC_DEFAULT;
    config.timings.pclk_hz = BOARD_LCD_RGB_FREQ_HZ;
    config.timings.h_res = BOARD_LCD_WIDTH;
    config.timings.v_res = BOARD_LCD_HEIGHT;
    config.timings.hsync_pulse_width = BOARD_LCD_RGB_HPW;
    config.timings.hsync_back_porch = BOARD_LCD_RGB_HBP;
    config.timings.hsync_front_porch = BOARD_LCD_RGB_HFP;
    config.timings.vsync_pulse_width = BOARD_LCD_RGB_VPW;
    config.timings.vsync_back_porch = BOARD_LCD_RGB_VBP;
    config.timings.vsync_front_porch = BOARD_LCD_RGB_VFP;

    config.data_width = BOARD_LCD_RGB_DATA_WIDTH;
    config.bits_per_pixel = BOARD_LCD_RGB_PIXEL_BITS;
    config.num_fbs = BOARD_LCD_RGB_FB_NUM;
    config.bounce_buffer_size_px = BOARD_LCD_RGB_BOUNCE_PX;

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 4, 0)
    config.psram_trans_align = 64;
#endif

    config.hsync_gpio_num = BOARD_LCD_PIN_HSYNC;
    config.vsync_gpio_num = BOARD_LCD_PIN_VSYNC;
    config.de_gpio_num = BOARD_LCD_PIN_DE;
    config.pclk_gpio_num = BOARD_LCD_PIN_PCLK;
    config.disp_gpio_num = BOARD_LCD_PIN_DISP;

    config.data_gpio_nums[0] = BOARD_LCD_PIN_DATA0;
    config.data_gpio_nums[1] = BOARD_LCD_PIN_DATA1;
    config.data_gpio_nums[2] = BOARD_LCD_PIN_DATA2;
    config.data_gpio_nums[3] = BOARD_LCD_PIN_DATA3;
    config.data_gpio_nums[4] = BOARD_LCD_PIN_DATA4;
    config.data_gpio_nums[5] = BOARD_LCD_PIN_DATA5;
    config.data_gpio_nums[6] = BOARD_LCD_PIN_DATA6;
    config.data_gpio_nums[7] = BOARD_LCD_PIN_DATA7;
    config.data_gpio_nums[8] = BOARD_LCD_PIN_DATA8;
    config.data_gpio_nums[9] = BOARD_LCD_PIN_DATA9;
    config.data_gpio_nums[10] = BOARD_LCD_PIN_DATA10;
    config.data_gpio_nums[11] = BOARD_LCD_PIN_DATA11;
    config.data_gpio_nums[12] = BOARD_LCD_PIN_DATA12;
    config.data_gpio_nums[13] = BOARD_LCD_PIN_DATA13;
    config.data_gpio_nums[14] = BOARD_LCD_PIN_DATA14;
    config.data_gpio_nums[15] = BOARD_LCD_PIN_DATA15;

    if(fill == NULL)
    {
        /* The framebuffers are far too large for internal RAM. */
        config.flags.fb_in_psram = true;
    }
    else
    {
        config.num_fbs = 0;
        config.flags.no_fb = true;
    }

    esp_err_t err = esp_lcd_new_rgb_panel(&config, &s_panel);

    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "RGB panel init failed: %s", esp_err_to_name(err));
        return err;
    }

    if(fill != NULL)
    {
        esp_lcd_rgb_panel_event_callbacks_t callbacks = {};
        callbacks.on_bounce_empty = fill;

        err = esp_lcd_rgb_panel_register_event_callbacks(s_panel, &callbacks,
                                                         context);

        if(err != ESP_OK)
            return err;
    }

    err = esp_lcd_panel_reset(s_panel);

    if(err != ESP_OK)
        return err;

    return esp_lcd_panel_init(s_panel);
}

/* -------------------------------------------------------------------------- */
/* Backlight                                                                  */
/* -------------------------------------------------------------------------- */

static esp_err_t backlight_init(void)
{
    ledc_timer_config_t timer_config = {};

    timer_config.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_config.duty_resolution = BACKLIGHT_RESOLUTION;
    timer_config.timer_num = BACKLIGHT_TIMER;
    timer_config.freq_hz = BOARD_LCD_BACKLIGHT_FREQ_HZ;
    timer_config.clk_cfg = LEDC_AUTO_CLK;

    esp_err_t err = ledc_timer_config(&timer_config);

    if(err != ESP_OK)
        return err;

    ledc_channel_config_t channel_config = {};

    channel_config.gpio_num = BOARD_LCD_BACKLIGHT_PIN;
    channel_config.speed_mode = LEDC_LOW_SPEED_MODE;
    channel_config.channel = BACKLIGHT_CHANNEL;
    channel_config.timer_sel = BACKLIGHT_TIMER;
    channel_config.duty = 0;
    channel_config.hpoint = 0;

    err = ledc_channel_config(&channel_config);

    if(err != ESP_OK)
        return err;

    return display_set_backlight(s_backlight_percent);
}

esp_err_t display_set_backlight(uint8_t percent)
{
    if(percent > 100)
        return ESP_ERR_INVALID_ARG;

    s_backlight_percent = percent;

    const uint32_t duty = (BACKLIGHT_MAX_DUTY * percent) / 100u;

    esp_err_t err =
        ledc_set_duty(LEDC_LOW_SPEED_MODE, BACKLIGHT_CHANNEL, duty);

    if(err != ESP_OK)
        return err;

    return ledc_update_duty(LEDC_LOW_SPEED_MODE, BACKLIGHT_CHANNEL);
}

uint8_t display_get_backlight(void)
{
    return s_backlight_percent;
}

/* -------------------------------------------------------------------------- */

/* ---- framebuffer guards ---------------------------------------------------- */

/*
 * The two framebuffers are 64-byte aligned, which leaves gaps beside
 * them, and the heap hands those gaps to later small allocations. One
 * was the touch controller's I2C device record - and a unit in use
 * crashed when that record was found overwritten with 0xFFFFFFFF, the
 * colour of white pixels, four bytes past the end of the first
 * framebuffer.
 *
 * What wrote there was not found: ten minutes of drawing every screen
 * under a hardware watchpoint, and hundreds of flash writes, never
 * reproduced it. So the gaps are filled with guard blocks holding a
 * known pattern instead. Nothing live can sit there any more, and if
 * anything ever writes past a framebuffer again it lands in a guard,
 * where display_guard_check() reports it rather than the unit crashing
 * some time later somewhere unrelated.
 */
static const uint32_t GUARD_PATTERN = 0x5AA5C33Cu;
static const size_t GUARD_BYTES = 16;
static const size_t GUARD_REACH = 128;      /* bytes past each buffer watched */
static const int GUARD_TRIES = 32;
static const int GUARD_MAX = 16;

static uint32_t *s_guards[GUARD_MAX];
static int s_guard_count = 0;

static void guard_fill(uint32_t *guard)
{
    for(size_t i = 0; i < GUARD_BYTES / sizeof(uint32_t); i++)
        guard[i] = GUARD_PATTERN;
}

static void guards_place(void)
{
    void *fb[2] = { NULL, NULL };
    const size_t fb_bytes =
        (size_t)BOARD_LCD_WIDTH * BOARD_LCD_HEIGHT * (BOARD_LCD_RGB_PIXEL_BITS / 8);

    if(esp_lcd_rgb_panel_get_frame_buffer(s_panel, 2, &fb[0], &fb[1]) != ESP_OK)
        return;

    /* Small blocks until the gaps beside the buffers are used up; keep
     * those that landed in one, give the rest back. */
    void *spare[GUARD_TRIES];
    int spares = 0;

    for(int i = 0; i < GUARD_TRIES && s_guard_count < GUARD_MAX; i++)
    {
        uint8_t *block = (uint8_t *)heap_caps_malloc(GUARD_BYTES,
                                                     MALLOC_CAP_SPIRAM);

        if(block == NULL)
            break;

        bool beside = false;

        for(int b = 0; b < 2; b++)
        {
            const uint8_t *end = (const uint8_t *)fb[b] + fb_bytes;

            if(block >= end && block < end + GUARD_REACH)
                beside = true;
        }

        if(beside)
        {
            s_guards[s_guard_count++] = (uint32_t *)block;
            guard_fill((uint32_t *)block);
        }
        else
        {
            spare[spares++] = block;
        }
    }

    for(int i = 0; i < spares; i++)
        heap_caps_free(spare[i]);

    ESP_LOGI(TAG, "%d guard block(s) beside the framebuffers", s_guard_count);
}

int display_guard_check(uint32_t *value, const void **address)
{
    int hits = 0;

    for(int g = 0; g < s_guard_count; g++)
    {
        for(size_t i = 0; i < GUARD_BYTES / sizeof(uint32_t); i++)
        {
            const uint32_t word = ((volatile uint32_t *)s_guards[g])[i];

            if(word == GUARD_PATTERN)
                continue;

            if(hits == 0)
            {
                if(value != NULL) *value = word;
                if(address != NULL) *address = &s_guards[g][i];
            }

            hits++;
        }

        /* Re-arm, so the next hit is a new one. */
        guard_fill(s_guards[g]);
    }

    return hits;
}

int display_guard_count(void)
{
    return s_guard_count;
}

esp_err_t display_init(void)
{
    panel_reset();

    esp_err_t err = register_link_init();

    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI register link failed: %s", esp_err_to_name(err));
        return err;
    }

    panel_registers_init();

    err = rgb_panel_init(NULL, NULL);

    if(err != ESP_OK)
        return err;

    /* Straight after the framebuffers exist, before anything else can
     * be allocated beside them. */
    guards_place();

    err = backlight_init();

    if(err != ESP_OK)
        return err;

    ESP_LOGI(TAG, "ST7701 ready, %dx%d", BOARD_LCD_WIDTH, BOARD_LCD_HEIGHT);
    return ESP_OK;
}

esp_lcd_panel_handle_t display_panel(void)
{
    return s_panel;
}

esp_err_t display_draw_bitmap(int x_start, int y_start,
                              int x_end, int y_end,
                              const void *pixels)
{
    /*
     * esp_lcd takes an exclusive end coordinate, LVGL gives an inclusive
     * one.
     */
    x_end += 1;
    y_end += 1;

    if(x_end > BOARD_LCD_WIDTH)  x_end = BOARD_LCD_WIDTH;
    if(y_end > BOARD_LCD_HEIGHT) y_end = BOARD_LCD_HEIGHT;

    return esp_lcd_panel_draw_bitmap(s_panel, x_start, y_start,
                                     x_end, y_end, pixels);
}

/*
 * The panel without framebuffers: delete it, and make it again with the
 * bounce buffers only, filled by fill. The ST7701 itself is left alone -
 * it only sees the RGB bus - so the picture just changes source.
 *
 * Nothing on this path touches PSRAM or flash: the bounce buffers are
 * internal RAM, the interrupt is IRAM-safe (CONFIG_LCD_RGB_ISR_IRAM_SAFE),
 * and fill has to be IRAM code reading internal RAM. So the picture
 * holds steady whatever the flash is doing.
 *
 * One way: there is no going back to the framebuffers short of a
 * restart.
 */
esp_err_t display_generate(esp_lcd_rgb_panel_bounce_buf_fill_cb_t fill,
                           void *context)
{
    if(fill == NULL)
        return ESP_ERR_INVALID_ARG;

    if(s_panel != NULL)
    {
        /* The framebuffers go with it, and the guards beside them stop
         * meaning anything. */
        s_guard_count = 0;

        const esp_err_t err = esp_lcd_panel_del(s_panel);
        s_panel = NULL;

        if(err != ESP_OK)
            return err;
    }

    return rgb_panel_init(fill, context);
}
