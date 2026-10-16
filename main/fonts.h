#pragma once

#include "lvgl.h"

/*
 * Text sizes, by role - the one place to make every screen's text
 * bigger or smaller. Each needs its CONFIG_LV_FONT_MONTSERRAT_<size> in
 * sdkconfig.defaults; LVGL's default font (CONFIG_LV_FONT_DEFAULT_*),
 * for labels that set none, is FONT_BODY's size too.
 */
#define FONT_SMALL  (&lv_font_montserrat_16)    /* hints, captions, headings */
#define FONT_BODY   (&lv_font_montserrat_18)    /* labels, buttons, table values */
#define FONT_LARGE  (&lv_font_montserrat_20)    /* readouts, titles */
#define FONT_HUGE   (&lv_font_montserrat_28)    /* dial numbers, compass letters */
