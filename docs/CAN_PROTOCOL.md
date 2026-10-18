# MiniEFIS CAN bus protocol

One bus for the EFIS and the boxes that feed it: a GPS, an air data
unit (differential pressure from the pitot-static system) and a custom
engine monitoring unit (EMU). Each sensor node broadcasts its own data
at a fixed rate. The MiniEFIS only receives: it never transmits a data
frame. There is no master, no polling, no configuration over the bus and
no alarm messages.

Each box is set up locally, by its own button, jumper or USB. Limits and
warnings are decided on the EFIS from the values it receives.

Status: receiving is implemented (`main/can_bus.cpp`, shown on the debug
web page and at `GET /can`). The HSI screen uses the GPS messages
(0x200-0x202); the other screens and the attitude filter do not use the
data yet, and the CAN BUS setting is not there yet.

## 1. Physical layer

| Item | Value |
| --- | --- |
| Standard | CAN 2.0A, classic frames, 11-bit identifiers (the ESP32-S3 TWAI controller has no CAN FD) |
| Bit rate | 500 kbit/s, sample point 87.5 % |
| Topology | One linear bus, stubs under 0.3 m, 120 ohm termination at both physical ends only |
| Cable | Twisted pair for CANH/CANL plus +12 V and GND, shielded and grounded at one end |
| Connector | DB9 per CiA 303-1: pin 2 CANL, pin 7 CANH, pin 3 GND, pin 9 +12 V |
| Transceiver | 3.3 V part on every node, e.g. SN65HVD230 or TCAN330. Not a 5 V part such as the TJA1050 run from 3.3 V: it still receives, but it can't drive its acknowledge onto the bus. The EFIS then reads back its own ACK as recessive, counts a bit error and drops every frame |

At 500 kbit/s an 8-byte frame, worst-case bit stuffing included, takes
about 270 us. The message set below comes to roughly 80 frames/s,
about 2 % bus load. That leaves room for a second engine and more, and means 250 kbit/s still works if the cable run gets long.

### MiniEFIS wiring

| Signal | GPIO | Transceiver pin |
| --- | --- | --- |
| TWAI TX | 20 | D / TXD |
| TWAI RX | 19 | R / RXD |

GPIO19/20 are the ESP32-S3's internal USB pins (D-/D+). The USB
connector isn't fitted on this unit, so they're free, and CAN doesn't
touch UART0 (IO43/IO44). Flashing and the serial console stay on UART0
and work with CAN running.

* **TX on GPIO20.** At reset the ROM enables the USB D+ pull-up on
  GPIO20. That holds the transceiver's TXD high (recessive) from
  power-on until the driver starts, so a booting EFIS never disturbs the
  bus.
* **Releasing the USB pads.** When the TWAI driver routes GPIO19/20,
  ESP-IDF detaches them from USB-Serial-JTAG itself (`gpio_ll_func_sel`
  clears `USB_SERIAL_JTAG_USB_PAD_ENABLE`). The firmware only has to
  stop using USB-Serial-JTAG as a console:
  `CONFIG_ESP_CONSOLE_SECONDARY_NONE=y` instead of
  `CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG=y`. The primary console
  stays on UART0.
* **Board check.** Make sure the unfitted connector's footprint leaves
  no ESD diodes or series resistors loading D+/D-. If parts are there,
  500 kbit/s is normally unaffected, but check.

**Receive-only, but not listen-only.** The TWAI controller runs in
normal mode with no transmit queue. It sends no frames of its own, but
it still drives the ACK bit for every frame it receives correctly. That
matters when the EFIS sits on a bus with only one sensor node: without
an ACK the sender retransmits forever and eventually goes error-passive.
Listen-only mode would break such a bus. TX must therefore be wired to
the transceiver, not left open.

## 2. Conventions

* **Byte order:** little-endian. That is native for the ESP32 and for
  most Cortex-M parts the other nodes will run on.
* **Values:** scaled integers, never floats. Each field lists its type
  (`u8`/`u16`/`u32` unsigned, `i16`/`i32` signed) and its resolution.
  The physical value is the raw value multiplied by the resolution.
* **Not available:** a field the sender cannot fill is set to its
  sentinel. Unsigned fields use all ones (`0xFF`, `0xFFFF`,
  `0xFFFFFFFF`). Signed fields use the most negative value (`0x8000`,
  `0x80000000`). Receivers show a sentinel as "no data", never as a
  number. This lets an engine unit with two CHT probes use the same
  frame as one with four.
* **Frame length:** the DLC is always the full length listed. A frame
  with a shorter DLC is discarded.
* **Freshness:** each periodic message has a timeout of 3x its period,
  with a minimum of 500 ms. Past the timeout the receiver treats every
  field of that message as not available. On the EFIS that means a red
  cross or dashes, never a frozen value.
* **Units:** SI and aviation-neutral on the wire (Pa, m/s, deg C, m).
  Knots, feet, inHg and deg F are display choices made on the EFIS.
* **Directions:** angles are in degrees. Heading and track run 0-360,
  clockwise from north. Pitch is positive nose up, roll positive right
  wing down, and vertical speed positive up.

## 3. Identifiers and priority

A lower identifier wins arbitration. The ranges are laid out so the
most important data goes first:

| Range | Use |
| --- | --- |
| `0x100`-`0x13F` | Air data |
| `0x200`-`0x23F` | GPS |
| `0x300`-`0x33F` | Engine 1. Engine 2 is the same set at `+0x20` (`0x320`...) |
| `0x700`-`0x70F` | Heartbeats, `0x700 + node` |

The heartbeat range follows the CANopen layout, so ordinary CAN analyser
tools decode it sensibly.

### Node IDs

| Node | Device |
| --- | --- |
| 1 | MiniEFIS (receives only, never sends) |
| 2 | GPS |
| 3 | Air data unit |
| 4 | Engine monitoring unit, engine 1 |
| 5 | Engine monitoring unit, engine 2 |
| 6-15 | Spare |

## 4. Messages

Offsets are byte offsets in the data field.

### 4.1 Air data unit (node 3)

The unit reports raw differential pressure (pitot minus static).
Airspeed is worked out on the EFIS, which also has the static pressure
and holds the calibration: zero offset and the pitot position-error
table. That way the calibration lives in one place and is set from the
Settings screen.

Suitable sensors: Sensirion SDP810-500Pa or SDP3x, or TE MS4525DO.

**`0x100` DIFF_PRESSURE**, 20 Hz

| Offset | Type | Field | Resolution |
| --- | --- | --- | --- |
| 0 | i32 | Differential pressure | 0.01 Pa |
| 4 | i16 | Sensor temperature | 0.01 deg C |
| 6 | u8 | Status: bit0 sensor fault, bit1 over range | - |
| 7 | u8 | Sequence counter, +1 per frame | - |

Resolution matters at the slow end: 30 km/h is only about 42 Pa. A
signed value is used because a sensor reads slightly negative at zero
airspeed.

**`0x101` OAT**, 2 Hz. Optional: send it if the unit has a probe.

| Offset | Type | Field | Resolution |
| --- | --- | --- | --- |
| 0 | i16 | Outside air temperature | 0.01 deg C |
| 2 | u16 | Relative humidity | 0.01 % |
| 4 | u32 | Reserved, `0xFFFFFFFF` | - |

### 4.2 GPS (node 2)

The GPS sends one burst per fix, in this order: `0x200`, `0x201`,
`0x202`. `0x200` carries no sequence counter because it has no room
for one. `0x201` and `0x202` both carry the fix sequence, so a receiver
pairs a position with the velocity and altitude of the same fix.

**`0x200` GPS_POSITION**, 5-10 Hz

| Offset | Type | Field | Resolution |
| --- | --- | --- | --- |
| 0 | i32 | Latitude, north positive | 1e-7 deg (about 1 cm) |
| 4 | i32 | Longitude, east positive | 1e-7 deg |

**`0x201` GPS_VELOCITY**, same rate

| Offset | Type | Field | Resolution |
| --- | --- | --- | --- |
| 0 | u16 | Ground speed | 0.01 m/s |
| 2 | u16 | True track | 0.01 deg |
| 4 | i16 | Vertical velocity, up positive | 0.01 m/s |
| 6 | u8 | Fix sequence | - |
| 7 | u8 | Fix type: 0 none, 2 2D, 3 3D, 4 3D+SBAS/DGPS, 5 RTK | - |

**`0x202` GPS_ALTITUDE**, same rate

| Offset | Type | Field | Resolution |
| --- | --- | --- | --- |
| 0 | i32 | Altitude above mean sea level | 1 mm |
| 4 | i16 | Geoid separation | 0.1 m |
| 6 | u8 | Fix sequence | - |
| 7 | u8 | Reserved, `0xFF` | - |

**`0x203` GPS_QUALITY**, 1 Hz

| Offset | Type | Field | Resolution |
| --- | --- | --- | --- |
| 0 | u8 | Satellites used | - |
| 1 | u8 | HDOP | 0.1 |
| 2 | u16 | Horizontal accuracy estimate | 0.01 m |
| 4 | u16 | Vertical accuracy estimate | 0.01 m |
| 6 | u16 | Speed accuracy estimate | 0.01 m/s |

**`0x204` GPS_TIME**, 1 Hz

| Offset | Type | Field | Resolution |
| --- | --- | --- | --- |
| 0 | u32 | UTC, seconds since 1970-01-01 | 1 s |
| 4 | u16 | Milliseconds | 1 ms |
| 6 | u8 | Flags: bit0 time valid, bit1 date valid, bit2 leap seconds known | - |
| 7 | u8 | Reserved, `0xFF` | - |

The EFIS may use `0x204` to set its clock.

`tools/gps_emulator.py` emulates this node through a USB-CAN adapter.
It flies a circle or a straight line, can report no fix, and sends the
heartbeat too.

### 4.3 Engine monitoring unit (node 4, engine 1)

For engine 2, add `0x20` to every identifier and use node 5. The layout
covers up to six cylinders. Channels a given engine doesn't have are
sent as sentinels: a Rotax 912 has two CHTs, while a Lycoming O-320 has
four EGTs and four CHTs.

**`0x300` ENGINE_FAST**, 10 Hz. These values change quickly and the
pilot watches them.

| Offset | Type | Field | Resolution |
| --- | --- | --- | --- |
| 0 | u16 | Engine RPM | 1 rpm |
| 2 | u16 | Manifold pressure | 0.01 kPa |
| 4 | u16 | Oil pressure | 0.1 kPa |
| 6 | u16 | Fuel pressure | 0.1 kPa |

**`0x301` ENGINE_SLOW**, 2 Hz

| Offset | Type | Field | Resolution |
| --- | --- | --- | --- |
| 0 | i16 | Oil temperature | 0.1 deg C |
| 2 | i16 | Coolant temperature | 0.1 deg C |
| 4 | u16 | Fuel flow | 0.01 L/h |
| 6 | u16 | Bus voltage | 0.01 V |

**`0x302` EGT_1_4**, 2 Hz

| Offset | Type | Field | Resolution |
| --- | --- | --- | --- |
| 0 | i16 | EGT cylinder 1 | 0.1 deg C |
| 2 | i16 | EGT cylinder 2 | 0.1 deg C |
| 4 | i16 | EGT cylinder 3 | 0.1 deg C |
| 6 | i16 | EGT cylinder 4 | 0.1 deg C |

**`0x303` CHT_1_4**, 2 Hz. Same layout as `0x302`, for CHT cylinders
1-4.

**`0x304` CYL_5_6**, 2 Hz. Six-cylinder engines only.

| Offset | Type | Field | Resolution |
| --- | --- | --- | --- |
| 0 | i16 | EGT cylinder 5 | 0.1 deg C |
| 2 | i16 | EGT cylinder 6 | 0.1 deg C |
| 4 | i16 | CHT cylinder 5 | 0.1 deg C |
| 6 | i16 | CHT cylinder 6 | 0.1 deg C |

**`0x305` FUEL_ELECTRICAL**, 1 Hz

| Offset | Type | Field | Resolution |
| --- | --- | --- | --- |
| 0 | u16 | Fuel quantity, left / main | 0.1 L |
| 2 | u16 | Fuel quantity, right / aux | 0.1 L |
| 4 | i16 | Battery current, charge positive | 0.01 A |
| 6 | i16 | Carburettor / intake air temperature | 0.1 deg C |

**`0x306` ENGINE_TOTALS**, 0.2 Hz (every 5 s)

| Offset | Type | Field | Resolution |
| --- | --- | --- | --- |
| 0 | u32 | Engine time, counted while RPM is above the run threshold | 1 s |
| 4 | u16 | Fuel used since power-up | 0.1 L |
| 6 | u16 | Fuel remaining at the current flow (endurance) | 1 min |

## 5. Heartbeat

**`0x700 + node` HEARTBEAT**, 1 Hz, from every sensor node

| Offset | Type | Field |
| --- | --- | --- |
| 0 | u8 | State: 0 booting, 1 running, 2 degraded, 3 fault |
| 1 | u8 | Device type: 2 GPS, 3 air data, 4 engine unit |
| 2 | u16 | Firmware version, major.minor as `major << 8 \| minor` |
| 4 | u32 | Uptime, seconds |

The EFIS lists the nodes it hears on the Diagnostics screen. A node
that misses three heartbeats is shown as gone.

## 6. Using it on the MiniEFIS

* **Setting.** "CAN BUS On/Off" in a CAN BUS section of the Settings
  screen, stored in NVS and on by default. Off stops the driver, for
  bench work with no bus attached. It isn't needed for upgrades: see
  section 7.
* **Driver.** The ESP-IDF TWAI driver in normal mode, on TX GPIO20 and
  RX GPIO19, with the transmit queue length set to 0 so nothing can be
  queued (see "Receive-only" above). Its acceptance filter is left
  open, and the receive task sorts frames by ID. The receive task runs
  on core 0, at a priority below the IMU task. It only copies values,
  with a timestamp, into a structure guarded by a spinlock, the same
  pattern as `sensor_readings`. Screens read that structure. After
  bus-off the driver is recovered and restarted, and the event is
  logged to the web log.
* **Airspeed.** The EFIS works out IAS and TAS from `0x100`, its own
  static pressure and the OAT. The pitot zero offset is captured from a
  button on the Settings screen, on the ground with no wind, and stored
  on the EFIS. The air data unit itself needs no configuration.
* **Airspeed into the attitude filter.** TAS feeds `accel_turn_speed_ms`
  (`components/ahrs/include/attitude_filter.h`). The filter can then
  subtract the centripetal acceleration in a turn, so a sustained turn
  no longer leans the horizon towards level, which is today's main
  limitation (see "Estimation" in `README.md`). With no air data, or
  with stale air data, it falls back to 0 as now.
* **GPS.** Ground speed and track go on the compass screen. GPS altitude
  is used as a cross-check against the baro altitude, not as a
  replacement for it.
* **Engine.** A new engine screen, added to the swipe order after the
  compass. Green, amber and red ranges for each value are set on the
  EFIS, and the EFIS raises any warning from them itself. Nothing on the
  bus tells it a value is out of range.
