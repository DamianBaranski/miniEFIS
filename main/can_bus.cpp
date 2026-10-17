#include "can_bus.h"

#include "board_pins.h"

#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <string.h>

static const char *TAG = "can";

/* Below the IMU task (3): a burst of frames must not delay a sample. */
static const int TASK_PRIORITY = 2;
static const int TASK_CORE = 0;
static const int TASK_STACK_BYTES = 4096;

/* About 80 frames/s on the bus: 32 is 0.4 s of them, ample for the
 * task being held up by Wi-Fi or a flash write. */
static const uint32_t RX_QUEUE_LENGTH = 32;

/* How long a receive waits, so alerts and statistics get looked at. */
static const TickType_t RECEIVE_TIMEOUT = pdMS_TO_TICKS(50);

/* Bus errors are logged at most this often: a bus with no transceiver,
 * or the wrong bit rate, makes them continuously. */
static const int64_t ERROR_LOG_INTERVAL_US = 10 * 1000 * 1000;

/* The protocol's identifiers (docs/CAN_PROTOCOL.md). */
enum {
    ID_DIFF_PRESSURE    = 0x100,
    ID_OAT              = 0x101,

    ID_GPS_POSITION     = 0x200,
    ID_GPS_VELOCITY     = 0x201,
    ID_GPS_ALTITUDE     = 0x202,
    ID_GPS_QUALITY      = 0x203,
    ID_GPS_TIME         = 0x204,

    ID_ENGINE_BASE      = 0x300,
    ID_ENGINE_STRIDE    = 0x20,     /* engine 2 at 0x320 */
    ENGINE_FAST         = 0x0,
    ENGINE_SLOW         = 0x1,
    ENGINE_EGT_1_4      = 0x2,
    ENGINE_CHT_1_4      = 0x3,
    ENGINE_CYL_5_6      = 0x4,
    ENGINE_FUEL         = 0x5,
    ENGINE_TOTALS       = 0x6,

    ID_HEARTBEAT_BASE   = 0x700,
};

/* Every message in the protocol is a full 8 bytes. */
static const uint8_t FRAME_LENGTH = 8;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static can_data_t s_data;

/* ---- little-endian fields, with the "not available" sentinels ---------- */

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

static int16_t get_i16(const uint8_t *p)
{
    return (int16_t)get_u16(p);
}

static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int32_t get_i32(const uint8_t *p)
{
    return (int32_t)get_u32(p);
}

static float scaled_u16(const uint8_t *p, float resolution)
{
    const uint16_t raw = get_u16(p);
    return (raw == 0xFFFF) ? NAN : raw * resolution;
}

static float scaled_i16(const uint8_t *p, float resolution)
{
    const int16_t raw = get_i16(p);
    return (raw == INT16_MIN) ? NAN : raw * resolution;
}

static float scaled_i32(const uint8_t *p, float resolution)
{
    const int32_t raw = get_i32(p);
    return (raw == INT32_MIN) ? NAN : raw * resolution;
}

static double scaled_i32_d(const uint8_t *p, double resolution)
{
    const int32_t raw = get_i32(p);
    return (raw == INT32_MIN) ? NAN : raw * resolution;
}

static float scaled_u8(uint8_t raw, float resolution)
{
    return (raw == 0xFF) ? NAN : raw * resolution;
}

static void stamp(can_stamp_t *s, int64_t now_us)
{
    s->rx_us = now_us;
    s->count++;
}

/* ---- decoding, under the lock ------------------------------------------- */

static bool decode_air(uint32_t id, const uint8_t *d, int64_t now_us)
{
    if(id == ID_DIFF_PRESSURE)
    {
        can_air_data_t &air = s_data.air;
        const uint8_t sequence = d[7];

        if(air.stamp.count > 0)
            air.sequence_gaps += (uint8_t)(sequence - air.sequence - 1);

        air.diff_pa = scaled_i32(&d[0], 0.01f);
        air.sensor_temp_c = scaled_i16(&d[4], 0.01f);
        air.status = d[6];
        air.sequence = sequence;
        stamp(&air.stamp, now_us);
        return true;
    }

    if(id == ID_OAT)
    {
        s_data.oat.oat_c = scaled_i16(&d[0], 0.01f);
        s_data.oat.humidity_pct = scaled_u16(&d[2], 0.01f);
        stamp(&s_data.oat.stamp, now_us);
        return true;
    }

    return false;
}

static bool decode_gps(uint32_t id, const uint8_t *d, int64_t now_us)
{
    can_gps_t &gps = s_data.gps;

    switch(id)
    {
    case ID_GPS_POSITION:
        gps.latitude_deg = scaled_i32_d(&d[0], 1e-7);
        gps.longitude_deg = scaled_i32_d(&d[4], 1e-7);
        stamp(&gps.position_stamp, now_us);
        return true;

    case ID_GPS_VELOCITY:
        gps.ground_speed_ms = scaled_u16(&d[0], 0.01f);
        gps.track_deg = scaled_u16(&d[2], 0.01f);
        gps.vertical_speed_ms = scaled_i16(&d[4], 0.01f);
        gps.velocity_fix_sequence = d[6];
        gps.fix_type = d[7];
        stamp(&gps.velocity_stamp, now_us);
        return true;

    case ID_GPS_ALTITUDE:
        gps.altitude_msl_m = scaled_i32(&d[0], 0.001f);
        gps.geoid_separation_m = scaled_i16(&d[4], 0.1f);
        gps.altitude_fix_sequence = d[6];
        stamp(&gps.altitude_stamp, now_us);
        return true;

    case ID_GPS_QUALITY:
        gps.satellites = d[0];
        gps.hdop = scaled_u8(d[1], 0.1f);
        gps.horizontal_accuracy_m = scaled_u16(&d[2], 0.01f);
        gps.vertical_accuracy_m = scaled_u16(&d[4], 0.01f);
        gps.speed_accuracy_ms = scaled_u16(&d[6], 0.01f);
        stamp(&gps.quality_stamp, now_us);
        return true;

    case ID_GPS_TIME:
        gps.utc_s = get_u32(&d[0]);
        gps.utc_ms = get_u16(&d[4]);
        gps.time_flags = d[6];
        stamp(&gps.time_stamp, now_us);
        return true;

    default:
        return false;
    }
}

static bool decode_engine(uint32_t id, const uint8_t *d, int64_t now_us)
{
    if(id < ID_ENGINE_BASE ||
       id >= ID_ENGINE_BASE + CAN_ENGINES * ID_ENGINE_STRIDE)
        return false;

    const uint32_t index = (id - ID_ENGINE_BASE) / ID_ENGINE_STRIDE;
    can_engine_t &engine = s_data.engine[index];

    switch((id - ID_ENGINE_BASE) % ID_ENGINE_STRIDE)
    {
    case ENGINE_FAST:
        engine.rpm = scaled_u16(&d[0], 1.0f);
        engine.manifold_kpa = scaled_u16(&d[2], 0.01f);
        engine.oil_pressure_kpa = scaled_u16(&d[4], 0.1f);
        engine.fuel_pressure_kpa = scaled_u16(&d[6], 0.1f);
        stamp(&engine.fast_stamp, now_us);
        return true;

    case ENGINE_SLOW:
        engine.oil_temp_c = scaled_i16(&d[0], 0.1f);
        engine.coolant_temp_c = scaled_i16(&d[2], 0.1f);
        engine.fuel_flow_lph = scaled_u16(&d[4], 0.01f);
        engine.bus_volts = scaled_u16(&d[6], 0.01f);
        stamp(&engine.slow_stamp, now_us);
        return true;

    case ENGINE_EGT_1_4:
        for(int i = 0; i < 4; i++)
            engine.egt_c[i] = scaled_i16(&d[2 * i], 0.1f);
        stamp(&engine.egt_stamp, now_us);
        return true;

    case ENGINE_CHT_1_4:
        for(int i = 0; i < 4; i++)
            engine.cht_c[i] = scaled_i16(&d[2 * i], 0.1f);
        stamp(&engine.cht_stamp, now_us);
        return true;

    case ENGINE_CYL_5_6:
        engine.egt_c[4] = scaled_i16(&d[0], 0.1f);
        engine.egt_c[5] = scaled_i16(&d[2], 0.1f);
        engine.cht_c[4] = scaled_i16(&d[4], 0.1f);
        engine.cht_c[5] = scaled_i16(&d[6], 0.1f);
        stamp(&engine.cyl56_stamp, now_us);
        return true;

    case ENGINE_FUEL:
        engine.fuel_left_l = scaled_u16(&d[0], 0.1f);
        engine.fuel_right_l = scaled_u16(&d[2], 0.1f);
        engine.battery_amps = scaled_i16(&d[4], 0.01f);
        engine.intake_temp_c = scaled_i16(&d[6], 0.1f);
        stamp(&engine.fuel_stamp, now_us);
        return true;

    case ENGINE_TOTALS:
        engine.engine_time_s = get_u32(&d[0]);
        engine.fuel_used_l = scaled_u16(&d[4], 0.1f);
        engine.endurance_min = scaled_u16(&d[6], 1.0f);
        stamp(&engine.totals_stamp, now_us);
        return true;

    default:
        return false;
    }
}

static bool decode_heartbeat(uint32_t id, const uint8_t *d, int64_t now_us)
{
    if(id < ID_HEARTBEAT_BASE || id >= ID_HEARTBEAT_BASE + CAN_NODES)
        return false;

    can_node_t &node = s_data.node[id - ID_HEARTBEAT_BASE];

    node.state = d[0];
    node.device_type = d[1];
    node.firmware = get_u16(&d[2]);
    node.uptime_s = get_u32(&d[4]);
    stamp(&node.stamp, now_us);
    return true;
}

static void receive(const twai_message_t &message, int64_t now_us)
{
    portENTER_CRITICAL(&s_lock);

    s_data.bus.frames++;

    if(message.extd || message.rtr)
    {
        s_data.bus.unknown_frames++;
    }
    else
    {
        const uint32_t id = message.identifier;
        const bool known = (id >= ID_DIFF_PRESSURE && id <= ID_OAT) ||
                           (id >= ID_GPS_POSITION && id <= ID_GPS_TIME) ||
                           (id >= ID_ENGINE_BASE &&
                            id < ID_ENGINE_BASE + CAN_ENGINES * ID_ENGINE_STRIDE &&
                            (id - ID_ENGINE_BASE) % ID_ENGINE_STRIDE <= ENGINE_TOTALS) ||
                           (id >= ID_HEARTBEAT_BASE &&
                            id < ID_HEARTBEAT_BASE + CAN_NODES);

        if(!known)
            s_data.bus.unknown_frames++;
        else if(message.data_length_code != FRAME_LENGTH)
            s_data.bus.bad_length++;
        else
            (void)(decode_air(id, message.data, now_us) ||
                   decode_gps(id, message.data, now_us) ||
                   decode_engine(id, message.data, now_us) ||
                   decode_heartbeat(id, message.data, now_us));
    }

    portEXIT_CRITICAL(&s_lock);
}

/* ---- the bus ------------------------------------------------------------- */

static void set_state(can_bus_state_t state)
{
    portENTER_CRITICAL(&s_lock);
    s_data.bus.state = state;
    portEXIT_CRITICAL(&s_lock);
}

/* Errors, bus-off and recovery. */
static void handle_alerts(int64_t now_us)
{
    static int64_t s_last_error_log_us = -ERROR_LOG_INTERVAL_US;

    uint32_t alerts = 0;

    if(twai_read_alerts(&alerts, 0) != ESP_OK || alerts == 0)
        return;

    portENTER_CRITICAL(&s_lock);

    if(alerts & TWAI_ALERT_BUS_ERROR)
        s_data.bus.bus_errors++;
    if(alerts & (TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_RX_FIFO_OVERRUN))
        s_data.bus.rx_lost++;
    if(alerts & TWAI_ALERT_BUS_OFF)
        s_data.bus.bus_off_count++;

    portEXIT_CRITICAL(&s_lock);

    if(alerts & TWAI_ALERT_BUS_OFF)
    {
        /* Can't happen from transmitting - nothing is - but receive
         * errors raise the error count too, and a bus with no
         * transceiver or the wrong bit rate gets here. */
        set_state(CAN_BUS_BUS_OFF);
        ESP_LOGW(TAG, "bus off - recovering");
        twai_initiate_recovery();
    }
    else if(alerts & TWAI_ALERT_BUS_RECOVERED)
    {
        if(twai_start() == ESP_OK)
        {
            set_state(CAN_BUS_RUNNING);
            ESP_LOGI(TAG, "bus recovered");
        }
    }
    else if(alerts & TWAI_ALERT_ERR_PASS)
    {
        set_state(CAN_BUS_ERROR_PASSIVE);
    }
    else if(alerts & TWAI_ALERT_ERR_ACTIVE)
    {
        set_state(CAN_BUS_RUNNING);
    }

    if((alerts & (TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_FIFO_OVERRUN |
                  TWAI_ALERT_RX_QUEUE_FULL)) &&
       now_us - s_last_error_log_us >= ERROR_LOG_INTERVAL_US)
    {
        s_last_error_log_us = now_us;

        can_bus_stats_t stats;
        portENTER_CRITICAL(&s_lock);
        stats = s_data.bus;
        portEXIT_CRITICAL(&s_lock);

        ESP_LOGW(TAG, "bus errors %lu, frames lost %lu (no transceiver, "
                 "termination, or bit rate?)",
                 (unsigned long)stats.bus_errors,
                 (unsigned long)stats.rx_lost);
    }
}

/* Once a second: the controller's error counters and the frame rate. */
static void update_statistics(int64_t now_us)
{
    static int64_t s_second_start_us = 0;
    static uint32_t s_frames_at_start = 0;

    if(now_us - s_second_start_us < 1000000)
        return;

    twai_status_info_t info;
    const bool have_info = (twai_get_status_info(&info) == ESP_OK);

    portENTER_CRITICAL(&s_lock);

    s_data.bus.frames_per_s = s_data.bus.frames - s_frames_at_start;
    s_frames_at_start = s_data.bus.frames;

    if(have_info)
    {
        s_data.bus.tx_error_counter = info.tx_error_counter;
        s_data.bus.rx_error_counter = info.rx_error_counter;
    }

    portEXIT_CRITICAL(&s_lock);

    s_second_start_us = now_us;
}

static void can_task(void *argument)
{
    (void)argument;

    for(;;)
    {
        twai_message_t message;

        if(twai_receive(&message, RECEIVE_TIMEOUT) == ESP_OK)
            receive(message, esp_timer_get_time());

        const int64_t now_us = esp_timer_get_time();
        handle_alerts(now_us);
        update_statistics(now_us);
    }
}

void can_bus_start(void)
{
    twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)BOARD_CAN_TX_PIN, (gpio_num_t)BOARD_CAN_RX_PIN,
        TWAI_MODE_NORMAL);

    /* Receive only: nothing can be queued for sending. Normal mode, not
     * listen-only, so frames are still acknowledged. */
    general.tx_queue_len = 0;
    general.rx_queue_len = RX_QUEUE_LENGTH;
    general.alerts_enabled = TWAI_ALERT_BUS_ERROR | TWAI_ALERT_BUS_OFF |
                             TWAI_ALERT_BUS_RECOVERED | TWAI_ALERT_ERR_PASS |
                             TWAI_ALERT_ERR_ACTIVE | TWAI_ALERT_RX_QUEUE_FULL |
                             TWAI_ALERT_RX_FIFO_OVERRUN;

    /* 500 kbit/s, 87.5 % sample point as the protocol sets it: 16 quanta
     * of 125 ns, 1 sync + 13 + 2. */
    static_assert(BOARD_CAN_BITRATE == 500000, "timing is for 500 kbit/s");
    twai_timing_config_t timing = TWAI_TIMING_CONFIG_500KBITS();
    timing.quanta_resolution_hz = 8000000;
    timing.brp = 0;
    timing.tseg_1 = 13;
    timing.tseg_2 = 2;
    timing.sjw = 2;

    const twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&general, &timing, &filter);

    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "driver install failed: %s", esp_err_to_name(err));
        return;
    }

    /* With no transceiver fitted, RX would float - read as dominant, a
     * bus stuck busy. Pulled up it reads as an idle bus instead. */
    gpio_pullup_en((gpio_num_t)BOARD_CAN_RX_PIN);

    err = twai_start();

    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "start failed: %s", esp_err_to_name(err));
        twai_driver_uninstall();
        return;
    }

    set_state(CAN_BUS_RUNNING);

    if(xTaskCreatePinnedToCore(can_task, "can", TASK_STACK_BYTES, NULL,
                               TASK_PRIORITY, NULL, TASK_CORE) != pdPASS)
    {
        ESP_LOGE(TAG, "could not start the receive task");
        twai_stop();
        twai_driver_uninstall();
        set_state(CAN_BUS_OFF_DRIVER);
        return;
    }

    ESP_LOGI(TAG, "receiving, %d kbit/s, TX GPIO%d RX GPIO%d",
             BOARD_CAN_BITRATE / 1000, BOARD_CAN_TX_PIN, BOARD_CAN_RX_PIN);
}

void can_bus_get(can_data_t *out)
{
    if(out == NULL)
        return;

    portENTER_CRITICAL(&s_lock);
    *out = s_data;
    portEXIT_CRITICAL(&s_lock);
}

bool can_fresh(const can_stamp_t *stamp, int64_t now_us, uint32_t period_ms)
{
    if(stamp == NULL || stamp->count == 0)
        return false;

    int64_t timeout_us = 3 * (int64_t)period_ms * 1000;

    if(timeout_us < 500000)
        timeout_us = 500000;

    return now_us - stamp->rx_us <= timeout_us;
}
