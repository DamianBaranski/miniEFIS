#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * CAN bus receiver: the air data unit, the GPS and the engine units, as
 * docs/CAN_PROTOCOL.md describes them.
 *
 * Receive only. A task of its own takes every frame off the bus,
 * decodes it and keeps the latest value of each message here, with the
 * time it arrived; nothing is ever transmitted. The controller still
 * acknowledges the frames it receives, as every CAN node must - a
 * sensor alone on the bus with the EFIS would otherwise resend forever.
 *
 * Values are in the units on the wire (SI: Pa, m/s, deg C, m), scaled.
 * A field the sender marked "not available" is NAN. Whether a message is
 * still current is the reader's call, by can_fresh() against its period:
 * nothing here goes stale by itself.
 */

#define CAN_ENGINES     2
#define CAN_CYLINDERS   6
#define CAN_NODES       16

/* When a message last arrived, and how many have. */
typedef struct {
    int64_t rx_us;              /* esp_timer time; 0: never */
    uint32_t count;
} can_stamp_t;

/* 0x100 DIFF_PRESSURE, 20 Hz. */
typedef struct {
    can_stamp_t stamp;
    float diff_pa;
    float sensor_temp_c;
    uint8_t status;             /* bit0 sensor fault, bit1 over range */
    uint8_t sequence;
    uint32_t sequence_gaps;     /* frames lost, by the counter */
} can_air_data_t;

/* 0x101 OAT, 2 Hz. */
typedef struct {
    can_stamp_t stamp;
    float oat_c;
    float humidity_pct;
} can_oat_t;

typedef struct {
    /* 0x200 GPS_POSITION */
    can_stamp_t position_stamp;
    double latitude_deg;
    double longitude_deg;

    /* 0x201 GPS_VELOCITY */
    can_stamp_t velocity_stamp;
    float ground_speed_ms;
    float track_deg;
    float vertical_speed_ms;
    uint8_t velocity_fix_sequence;
    uint8_t fix_type;           /* 0 none, 2 2D, 3 3D, 4 SBAS/DGPS, 5 RTK */

    /* 0x202 GPS_ALTITUDE */
    can_stamp_t altitude_stamp;
    float altitude_msl_m;
    float geoid_separation_m;
    uint8_t altitude_fix_sequence;

    /* 0x203 GPS_QUALITY */
    can_stamp_t quality_stamp;
    uint8_t satellites;         /* 0xFF: not available */
    float hdop;
    float horizontal_accuracy_m;
    float vertical_accuracy_m;
    float speed_accuracy_ms;

    /* 0x204 GPS_TIME */
    can_stamp_t time_stamp;
    uint32_t utc_s;             /* seconds since 1970 */
    uint16_t utc_ms;
    uint8_t time_flags;         /* bit0 time valid, bit1 date valid */
} can_gps_t;

typedef struct {
    /* 0x300 ENGINE_FAST */
    can_stamp_t fast_stamp;
    float rpm;
    float manifold_kpa;
    float oil_pressure_kpa;
    float fuel_pressure_kpa;

    /* 0x301 ENGINE_SLOW */
    can_stamp_t slow_stamp;
    float oil_temp_c;
    float coolant_temp_c;
    float fuel_flow_lph;
    float bus_volts;

    /* 0x302 cylinders 1-4, 0x304 cylinders 5-6 */
    can_stamp_t egt_stamp;
    can_stamp_t cht_stamp;
    can_stamp_t cyl56_stamp;
    float egt_c[CAN_CYLINDERS];
    float cht_c[CAN_CYLINDERS];

    /* 0x305 FUEL_ELECTRICAL */
    can_stamp_t fuel_stamp;
    float fuel_left_l;
    float fuel_right_l;
    float battery_amps;
    float intake_temp_c;

    /* 0x306 ENGINE_TOTALS */
    can_stamp_t totals_stamp;
    uint32_t engine_time_s;     /* 0xFFFFFFFF: not available */
    float fuel_used_l;
    float endurance_min;
} can_engine_t;

/* 0x700 + node HEARTBEAT, 1 Hz. */
typedef struct {
    can_stamp_t stamp;
    uint8_t state;              /* 0 booting, 1 running, 2 degraded, 3 fault */
    uint8_t device_type;        /* 2 GPS, 3 air data, 4 engine unit */
    uint16_t firmware;          /* major << 8 | minor */
    uint32_t uptime_s;
} can_node_t;

typedef enum {
    CAN_BUS_OFF_DRIVER = 0,     /* not started: no driver, or it failed */
    CAN_BUS_RUNNING,            /* error active */
    CAN_BUS_ERROR_PASSIVE,
    CAN_BUS_BUS_OFF,            /* recovering */
} can_bus_state_t;

typedef struct {
    can_bus_state_t state;
    uint32_t frames;            /* every frame received */
    uint32_t unknown_frames;    /* an ID not in the protocol, or extended */
    uint32_t bad_length;        /* a known ID with the wrong DLC */
    uint32_t bus_errors;
    uint32_t rx_lost;           /* queue full or FIFO overrun */
    uint32_t bus_off_count;
    uint32_t tx_error_counter;
    uint32_t rx_error_counter;
    uint32_t frames_per_s;      /* measured over the last 250 ms */
    int rx_level;               /* the RX pin now: 1 recessive (idle) */
} can_bus_stats_t;

typedef struct {
    can_bus_stats_t bus;
    can_air_data_t air;
    can_oat_t oat;
    can_gps_t gps;
    can_engine_t engine[CAN_ENGINES];
    can_node_t node[CAN_NODES];
} can_data_t;

/* Message periods, for can_fresh(). */
#define CAN_PERIOD_AIR_MS           50
#define CAN_PERIOD_OAT_MS           500
#define CAN_PERIOD_GPS_MS           200     /* the slowest the GPS may run */
#define CAN_PERIOD_GPS_SLOW_MS      1000
#define CAN_PERIOD_ENGINE_FAST_MS   100
#define CAN_PERIOD_ENGINE_SLOW_MS   500
#define CAN_PERIOD_FUEL_MS          1000
#define CAN_PERIOD_TOTALS_MS        5000
#define CAN_PERIOD_HEARTBEAT_MS     1000

/* Installs the driver and starts the receiving task. */
void can_bus_start(void);

/* A consistent copy of everything. Safe from any task. */
void can_bus_get(can_data_t *out);


/*
 * Whether a message is current at now_us: it arrived within 3 periods,
 * and never less than 500 ms - the protocol's timeout.
 */
bool can_fresh(const can_stamp_t *stamp, int64_t now_us, uint32_t period_ms);
