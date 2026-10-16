#pragma once

/*
 * Every pin and bus assignment for this board, in one place.
 *
 * Waveshare ESP32-S3 480x480 round RGB panel: ST7701 display driven over
 * a 16-bit RGB bus (registers loaded over SPI), CST820 capacitive touch
 * and a QMI8658 IMU on I2C, with a TCA9554 expander supplying the reset
 * and chip-select lines the ESP32 pins ran out for.
 */

/* ---- I2C (touch, IMU, IO expander) ---- */
#define BOARD_I2C_PORT              0
#define BOARD_I2C_SDA_PIN           15
#define BOARD_I2C_SCL_PIN           7
#define BOARD_I2C_FREQ_HZ           400000

#define BOARD_I2C_ADDR_TCA9554      0x20
#define BOARD_I2C_ADDR_CST820       0x15
#define BOARD_I2C_ADDR_QMI8658      0x6B

/* ---- TCA9554 expander lines (1..8) ---- */
#define BOARD_EXIO_LCD_RESET        1
#define BOARD_EXIO_TOUCH_RESET      2
#define BOARD_EXIO_LCD_CS           3
/* Driven low before the IMU is initialised, as the vendor driver does;
 * the schematic does not document what it gates. */
#define BOARD_EXIO_IMU_ENABLE       8

/* ---- ST7701 register interface (SPI) ---- */
#define BOARD_LCD_SPI_HOST          SPI2_HOST
#define BOARD_LCD_SPI_CLK_PIN       2
#define BOARD_LCD_SPI_MOSI_PIN      1
#define BOARD_LCD_SPI_FREQ_HZ       40000000

/* ---- Backlight (LEDC PWM) ---- */
#define BOARD_LCD_BACKLIGHT_PIN     6
#define BOARD_LCD_BACKLIGHT_FREQ_HZ 20000

/* ---- Touch ---- */
#define BOARD_TOUCH_INT_PIN         16

/* ---- RGB panel geometry and timing ---- */
#define BOARD_LCD_WIDTH             480
#define BOARD_LCD_HEIGHT            480
#define BOARD_LCD_RGB_DATA_WIDTH    16
#define BOARD_LCD_RGB_PIXEL_BITS    16
/*
 * 12 MHz gives about 44 Hz refresh, which is ample for an instrument and
 * leaves more PSRAM bandwidth for drawing: the panel's DMA reads a whole
 * framebuffer every frame, and at 16 MHz that competed with LVGL's
 * writes hard enough to be the limit on redraw time.
 */
#define BOARD_LCD_RGB_FREQ_HZ       (12 * 1000 * 1000)
#define BOARD_LCD_RGB_HPW           8
#define BOARD_LCD_RGB_HBP           10
#define BOARD_LCD_RGB_HFP           50
#define BOARD_LCD_RGB_VPW           3
#define BOARD_LCD_RGB_VBP           8
#define BOARD_LCD_RGB_VFP           8

/*
 * Two framebuffers in PSRAM, plus a bounce buffer. The bounce buffer
 * stops the panel drifting when PSRAM is busy; its size must divide the
 * screen evenly (width * height = size * N, N even).
 */
#define BOARD_LCD_RGB_FB_NUM        2
#define BOARD_LCD_RGB_BOUNCE_PX     (BOARD_LCD_HEIGHT * 10)

/* ---- RGB data pins ---- */
#define BOARD_LCD_PIN_HSYNC         38
#define BOARD_LCD_PIN_VSYNC         39
#define BOARD_LCD_PIN_DE            40
#define BOARD_LCD_PIN_PCLK          41
#define BOARD_LCD_PIN_DISP          (-1)

#define BOARD_LCD_PIN_DATA0         5
#define BOARD_LCD_PIN_DATA1         45
#define BOARD_LCD_PIN_DATA2         48
#define BOARD_LCD_PIN_DATA3         47
#define BOARD_LCD_PIN_DATA4         21
#define BOARD_LCD_PIN_DATA5         14
#define BOARD_LCD_PIN_DATA6         13
#define BOARD_LCD_PIN_DATA7         12
#define BOARD_LCD_PIN_DATA8         11
#define BOARD_LCD_PIN_DATA9         10
#define BOARD_LCD_PIN_DATA10        9
#define BOARD_LCD_PIN_DATA11        46
#define BOARD_LCD_PIN_DATA12        3
#define BOARD_LCD_PIN_DATA13        8
#define BOARD_LCD_PIN_DATA14        18
#define BOARD_LCD_PIN_DATA15        17
