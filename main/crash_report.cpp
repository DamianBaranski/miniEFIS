#include "crash_report.h"

#include "esp_core_dump.h"
#include "esp_log.h"
#include "esp_system.h"

#include <stdio.h>

static const char *TAG = "crash";

static char s_text[96] = "";

static const char *reset_reason_name(esp_reset_reason_t reason)
{
    switch(reason)
    {
        case ESP_RST_POWERON:   return "power on";
        case ESP_RST_EXT:       return "reset pin";
        case ESP_RST_SW:        return "software restart";
        case ESP_RST_PANIC:     return "crash (panic)";
        case ESP_RST_INT_WDT:   return "interrupt watchdog";
        case ESP_RST_TASK_WDT:  return "task watchdog";
        case ESP_RST_WDT:       return "watchdog";
        case ESP_RST_DEEPSLEEP: return "deep sleep wake";
        case ESP_RST_BROWNOUT:  return "brownout (supply dipped)";
        case ESP_RST_SDIO:      return "SDIO";
        case ESP_RST_USB:       return "USB";
        case ESP_RST_JTAG:      return "JTAG";
        default:                return "unknown";
    }
}

void crash_report_init(void)
{
    const esp_reset_reason_t reason = esp_reset_reason();

    if(reason == ESP_RST_POWERON || reason == ESP_RST_EXT ||
       reason == ESP_RST_SW || reason == ESP_RST_USB)
        ESP_LOGI(TAG, "reset reason: %s", reset_reason_name(reason));
    else
        ESP_LOGW(TAG, "reset reason: %s", reset_reason_name(reason));

    int used = snprintf(s_text, sizeof(s_text), "last reset: %s",
                        reset_reason_name(reason));

    if(esp_core_dump_image_check() != ESP_OK)
        return;

    esp_core_dump_summary_t summary;

    if(esp_core_dump_get_summary(&summary) != ESP_OK)
        return;

    /* In the form addr2line takes, so it can be pasted straight in:
     *   xtensa-esp32s3-elf-addr2line -pfiaC -e miniefis.elf ... */
    char backtrace[16 * 11 + 1];
    int length = 0;

    for(uint32_t i = 0; i < summary.exc_bt_info.depth && i < 16; i++)
        length += snprintf(backtrace + length, sizeof(backtrace) - length,
                           " 0x%08lx", (unsigned long)summary.exc_bt_info.bt[i]);

    ESP_LOGW(TAG, "stored crash: task '%s', PC 0x%08lx, cause %lu%s",
             summary.exc_task, (unsigned long)summary.exc_pc,
             (unsigned long)summary.ex_info.exc_cause,
             summary.exc_bt_info.corrupted ? ", backtrace corrupted" : "");
    ESP_LOGW(TAG, "stored crash backtrace:%s", backtrace);

    if(used > 0 && used < (int)sizeof(s_text))
        snprintf(s_text + used, sizeof(s_text) - used,
                 "\nlast crash: %s at 0x%08lx", summary.exc_task,
                 (unsigned long)summary.exc_pc);
}

const char *crash_report_text(void)
{
    return s_text;
}
