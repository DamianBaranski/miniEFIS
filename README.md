# MiniEFIS

A small electronic flight instrument - artificial horizon, altimeter,
turn coordinator and compass on one display - for the Waveshare ESP32-S3
board with the 480x480 round RGB panel (ST7701 display, CST820 touch,
QMI8658 IMU). Attitude and heading come from a Kalman filter over the
gyroscope, accelerometer and magnetometer, altitude and vertical speed
from a second one over the barometer, and every screen reads the same
estimates - see "Estimation" below.

The instrument is meant to be mounted like a panel gauge: display
upright and facing the pilot, nose pointing into the panel. That
orientation is what `sensor_to_body()` in `main/mounting.h` encodes, and it
is the only place that knows how the IMU sits in the device - lay the
board flat on its back instead and it will correctly report the nose
pointing at the ground.

ESP-IDF project - no Arduino core, no Arduino libraries.

## Requirements

* ESP32-S3 with **octal PSRAM** - the two 480x480x16bpp framebuffers are
  900 KB and the RGB bus streams straight out of them
* Linux or macOS with `git`, `bash` and `cmake` 3.16 or newer

ESP-IDF itself is not a prerequisite: if it is missing, the configure step
installs it.

## Build

```sh
mkdir -p build && cd build
cmake ..
make                # firmware
make flash          # flash it (ESPPORT=/dev/ttyUSB0 to choose a port)
make monitor        # serial console
make host-tests     # build and run the attitude filter tests on this PC
```

That is the whole sequence - there is no `export.sh` to remember and no
`idf.py` step. `idf.py` is a wrapper that sets up the cross toolchain, the
target and ESP-IDF's Python environment before calling cmake; the top
level `CMakeLists.txt` does that itself, so a plain `cmake ..` is enough.
It will, in order:

1. look for ESP-IDF in `IDF_PATH`, `~/esp/esp-idf` and `/opt/esp-idf`;
2. clone the pinned release into `~/esp/esp-idf` if there is none, and
   repair the submodules of a checkout that was interrupted;
3. run `install.sh` if the toolchain has not been downloaded yet;
4. source `export.sh` in a subshell and adopt the environment it sets;
5. hand over to ESP-IDF's own build system.

The first configure therefore downloads a few GB and takes several
minutes. Afterwards it is a normal cmake project. Useful knobs:

| Option | Meaning |
| --- | --- |
| `-DIDF_PATH=/path/to/esp-idf` | use an existing installation |
| `-DESP_IDF_AUTO_INSTALL=OFF` | fail instead of downloading anything |
| `-DESP_IDF_VERSION_TAG=v5.3.6` | release to install (default `v5.4.4`) |
| `-DESP_IDF_INSTALL_DIR=...` | where to install it (default `~/esp/esp-idf`) |
| `-DIDF_TARGET=esp32s3` | chip to build for |

An ESP-IDF that is already installed is used as it is found, whatever its
version, as long as it is v5.2 or newer.

`sdkconfig.defaults` carries the settings this board needs: octal PSRAM
at 80 MHz, instructions and constants relocated out of flash so a flash
read cannot stall the RGB bus mid-frame, a 1 kHz FreeRTOS tick and a 3 MB
app partition.

## Host tests

The estimators have no ESP-IDF, LVGL or board dependency, so they
build and run on a PC. The tests drive them with simulated sensors and
check the numbers that matter: attitude tracking, rejection of linear
acceleration, gyro bias handling, tilt-compensated heading, steadiness
under sensor noise, behaviour at steep pitch and upside down, a minute
of sustained coordinated turn, recovery from a gyro upset, a magnetic
disturbance, and whether the covariance means what it says. The
navigator test flies the whole chain - attitude, heading, turn rate,
slip, altitude - through a standard rate turn and a climb. The turn
coordinator is checked against coordinated, slipping and skidding
turns, the vertical channel against climbs, accelerometer errors and
barometer dropouts, the compass swing against a simulated sensor
with known hard and soft iron errors, and the mounting alignment
against units tilted in the panel.

`make host-tests` from the build directory runs them. They are also a
standalone cmake project, which is useful when there is no ESP-IDF around
at all:

```sh
cmake -B build-host test
cmake --build build-host
ctest --test-dir build-host --output-on-failure
```

## Layout

```
components/ahrs/      the estimators: attitude and altitude Kalman filters, turn
                      coordinator, the Navigator that runs them together,
                      magnetometer calibration, mounting alignment - portable
                      C++, no dependencies
components/board/     hardware: I2C, IO expander, display, touch, IMU, barometer,
                      magnetometer, ADC, LVGL port
components/lvgl/      LVGL 8.3, vendored without its docs, tests, demos and
                      examples (configured from sdkconfig, NOT from its own
                      lv_conf.h - see sdkconfig.defaults). The demos and
                      examples include paths are dropped from
                      env_support/cmake/esp.cmake to match; turning a demo on
                      needs them back
main/                 start-up, navigation (what every screen reads), sensors,
                      settings, Wi-Fi debug, logging, fast drawing
main/screens/         one file pair per screen, the menu that switches them
                      (screen_manager), and the update screen
main/widgets/         pieces drawn on a screen: the altitude panel
tools/                PC side: live sensor plot from the serial stream
test/                 host build of components/ahrs plus its tests
```

Three tasks run: the IMU task on core 0 samples the sensor at 100 Hz and
steps the filters, LVGL owns core 1 and redraws at 20 Hz, and an esp_timer
supplies LVGL's tick. The filters publish their estimates through one
lock-free snapshot (`main/navigation.h`), so neither side blocks the
other.

## Where to change things

| To change | Edit |
| --- | --- |
| Pins, panel timing, I2C addresses | `components/board/include/board_pins.h` |
| Barometer oversampling and filtering | `baro_config_t`, `components/board/include/baro_bmp280.h` |
| Altitude/vertical speed filter (noise model) | `VerticalChannelConfig`, `components/ahrs/include/vertical_channel.h` |
| What every screen reads, barometer rate, QNH | `main/navigation.{h,cpp}` |
| Altitude widget, units (m or ft) | `main/widgets/altitude_widget.{h,cpp}` |
| LVGL settings (fonts, heap, ...) | `sdkconfig.defaults`, **not** `lv_conf.h` |
| Sensor ranges, rate, filtering, accel offset | `imu_config_t`, `components/board/include/imu_qmi8658.h` |
| How the IMU and magnetometer are mounted (axis mapping) | `sensor_to_body()`, `mag_to_body()` in `main/mounting.h` |
| Panel pixel clock and timing | `components/board/include/board_pins.h` |
| Attitude filter noise model, gates, turn handling | `AttitudeConfig`, `components/ahrs/include/attitude_filter.h` |
| Telemetry: what is sent, how often | `main/diagnostics.cpp` |
| Reset reason and crash summary | `main/crash_report.cpp` |
| Instrument appearance | `main/screens/horizon_screen.cpp` |
| Screens, the menu, double tap timing | `main/screens/screen_manager.cpp` |
| Altimeter dial, vertical speed scale | `main/screens/altimeter_screen.cpp` |
| What is saved, defaults, save delay | `main/app_settings.cpp` |
| Compass card appearance | `main/screens/compass_screen.cpp` |
| Turn coordinator appearance, symbol scale | `main/screens/turn_coordinator_screen.cpp` |
| Turn rate and ball damping | `TurnCoordinatorConfig`, `components/ahrs/include/turn_coordinator.h` |
| Magnetometer calibration UI | `main/screens/settings_screen.cpp` |
| Calibration acceptance (coverage, tilt, fit), magnetic dip | `MagCalibratorConfig`, `components/ahrs/include/mag_calibration.h`; dip also `CONFIG_MAG_DIP_DEG` |
| Diagnostics screen layout | `main/screens/diagnostics_screen.cpp` |
| Wi-Fi network, access point password | `make menuconfig`, "Wi-Fi debug", or the web page |
| Wi-Fi debug web page, update checks | `main/wifi_debug.cpp` |
| What the display shows during an update | `main/screens/update_screen.cpp` |
| Text sizes on every screen | `main/fonts.h` (and the fonts enabled in `sdkconfig.defaults`) |
| How much log the web page keeps | `WEB_LOG_SIZE`, `main/web_log.cpp` |
| Magnetometer / ADC settings | `components/board/mag_qmc5883.cpp`, `adc_ads1115.cpp` |
| Pitch number font and spacing | `PITCH_LABEL_FONT`, `PITCH_LABEL_GAP` in `main/screens/horizon_screen.cpp` |
| Roll arc size | `ROLL_SCALE_RADIUS_FRACTION` in `main/screens/horizon_screen.cpp` |

## Barometer (optional)

A BMP280 or BME280 on the I2C bus is detected automatically at 0x76 or
0x77 and gives pressure, temperature and pressure altitude. Nothing
depends on it: `baro_init()` returning `ESP_ERR_NOT_FOUND` is a normal
outcome, and `baro_present()` says whether to show altitude.

Measured on a BMP280 at the default x16 oversampling with the on-chip
IIR filter at 16, over 25 s sitting still:

| | |
| --- | --- |
| Pressure | 100691.7 Pa, sd 1.24 Pa, 5.3 Pa peak to peak |
| Altitude | sd 0.106 m, 0.40 m peak to peak |
| Temperature | 25.3 C, stable to 0.1 C |

The 1.24 Pa figure is the datasheet's own noise number for this
oversampling, so the part is performing as specified. Note the
temperature is the die, not the air: it reads a degree or two above
ambient through self-heating.

`baro_altitude_m()` takes the sea level pressure, so passing
`BARO_SEA_LEVEL_STANDARD_PA` gives pressure altitude and passing the
local QNH gives altitude above sea level.

What is deliberately *not* built on it yet: vertical speed cannot be had
by differentiating this - 0.1 m of noise at 4 Hz would swamp it - and
needs a filter over altitude, vertical speed and vertical accelerometer
bias, fed by the attitude-rotated accelerometer. That belongs in
`components/ahrs`, where the host tests can exercise it.

## Estimation

Every number on every screen comes from one `Navigator`
(`components/ahrs/navigator.h`), stepped once per IMU sample in the IMU
task and read by the screens as one snapshot (`main/navigation.h`), so
the horizon, the compass, the turn coordinator and the altimeter always
show the same instant.

* **Attitude and heading** - `AttitudeFilter`, a multiplicative extended
  Kalman filter. The state is the attitude quaternion and the three
  gyro biases; a 6x6 covariance over the attitude error and the bias
  error decides how far each measurement moves each. The gyro predicts;
  the accelerometer's direction corrects roll and pitch; the
  magnetometer's tilt compensated heading corrects heading, about the
  vertical only, so a disturbed field can never tilt the horizon.
  Readings the covariance calls implausible fail a chi-square gate and
  are ignored; turning flight - a centripetal acceleration the
  accelerometer cannot tell from gravity - is detected from the rates
  and flown on the gyro; the gyro bias is learned only in steady
  conditions; and an estimate that is clearly wrong (a gyro upset, a
  lasting change of field) opens its own covariance to be corrected.
  Its uncertainty grows with rotation (`gyro_scale_noise`): after a turn,
  where the gyro and the magnetometer never quite agree how far it
  went, the heading settles on the magnetometer within half a second
  instead of holding the gyro's answer and moving seconds later.
* **Turn rate and slip** - `TurnCoordinator`, from the filter's bias
  corrected rates and attitude.
* **Altitude and vertical speed** - `VerticalChannel`, a three state
  Kalman filter (altitude, vertical speed, accelerometer bias) driven by
  the vertical acceleration the attitude gives and corrected by the
  barometer, with its own gate.

Each magnetometer and barometer reading is handed to the filters once:
a Kalman filter counts every reading as a new measurement.

Against the Madgwick filter and fixed-gain vertical loop it replaced,
on the same simulated sensors (`test/`):

| | Before | Now |
| --- | --- | --- |
| Standard rate turn, 1 min: worst bank error | 15.2 deg | 1.0 deg |
| Standard rate turn: heading error | 30.8 deg | 2.4 deg |
| Standard rate turn: false vertical speed | 0.87 m/s | 0.08 m/s |
| 30 deg bank held 1 min: worst bank error | 12.2 deg | 0.14 deg |
| 0.25 g held 3 s: false pitch | 14.1 deg | 0 |
| Plausible but 70 deg wrong field, 1 s: heading moved | 31 deg | 0 |
| Horizon jitter, noisy sensor at rest | 0.45 deg p-p | 0.15 deg p-p |

The filters' uncertainty is on the serial log (`sd`, below) and in the
snapshot (`tilt_sigma_deg`, `heading_sigma_deg`, `altitude_sigma_m`,
`vertical_speed_sigma_ms`).

## Screens and touch

Double tap the background of any screen to open the menu, which
switches between the artificial horizon, the altimeter, the turn
coordinator, the compass, settings and diagnostics; a tap outside the menu closes it.
A swipe left or right steps through the four instruments - horizon,
altimeter, turn coordinator, compass, and round again - but not
settings or diagnostics; a swipe that starts on a control does not
also press it. LVGL 8 has no double
click event, so `main/screens/screen_manager.cpp` builds one from two short clicks within
500 ms and 60 px of each other, caught on the screen object itself.
Controls take their own taps, so anything drawn on a screen that should
let taps through - the horizon, the compass card, the altitude panel -
is made non-clickable.

Every screen is built once and kept. Switching only loads another one,
so leaving the horizon does not reset the attitude filter. All of them
together take about 33 KB of LVGL's heap, which is why
`CONFIG_LV_MEM_SIZE_KILOBYTES` is 64: at the default 32 the firmware
crashed while building the menu. The start-up log prints the figure.

* **Altimeter** - a two pointer dial (long pointer hundreds, a turn per
  1000; short pointer thousands) with the altitude as a number, and the
  vertical speed as a bar along the left edge, green up for climb and
  orange down for sink, full scale 10 m/s or 2000 ft/min. Below the hub
  is the QNH window: - and + step it a hectopascal (hold to repeat), STD
  sets 1013.25. A new setting moves the altitude at once, as turning a
  real altimeter's knob does: the vertical channel is shifted by the
  change (`VerticalChannel::shiftAltitude`) rather than left to work it
  off as a measurement error, with a false climb while it did. About
  40 ms a frame, redrawn only when the reading moves.
* **Turn coordinator** - a rear view aircraft symbol that banks with the
  rate of turn, wings on the lower marks for a standard rate turn
  (3 deg/s, two minutes for a full circle), the rate as a number, and
  the slip ball below. The rate is the earth frame yaw rate, from the
  bias corrected gyro and the attitude, so it reads the true turn at
  any bank and nothing for a roll alone; the ball is the apparent
  gravity's angle across the aircraft. Both are damped like the real
  instrument (0.5 s and 0.3 s). The maths is `TurnCoordinator` in
  `components/ahrs`, with host tests. 29 ms a frame, only redrawn when
  something moves.
* **Compass** - a heading indicator: the card turns under a fixed
  lubber line and aircraft symbol, with the heading below it. It only
  redraws when the heading moves, and costs 35 ms a frame when it does.
  Without a calibrated magnetometer it says so instead of guessing.
* **Settings** - units (m and m/s, or ft and ft/min), backlight,
  mounting alignment, Wi-Fi debug, and the magnetometer's calibration.

  **Set level**, under MOUNTING, is for a unit not square in the panel:
  with the aircraft level and still it averages gravity for two seconds,
  and the rotation that takes it straight down is applied to every
  sensor from then on (`main/alignment.cpp`, applied in `main/mounting.h`;
  the maths in `components/ahrs/mounting_alignment.cpp`, with host
  tests). The horizon, turn coordinator and heading then all work in the
  aircraft's axes. It corrects pitch and roll of the installation up to
  20 degrees, not yaw, which gravity cannot see. Pressing it again
  refines it; the gyro is re-measured after each change.

Settings are kept across restarts, and so is the screen last shown: the
unit comes back where it was left. QNH, units, backlight and screen
live in `main/app_settings.cpp`, written to NVS three seconds after the
last change rather than on every one - holding the QNH button would
otherwise write flash a dozen times a second. The magnetometer and the
mounting alignment keep their own records (`main/magnetometer.cpp`,
`main/alignment.cpp`).
* **Diagnostics** - the raw readings: accelerometer and gyro in sensor
  axes, the magnetometer in uT and counts, the four ADC channels in
  counts and volts, the barometer, and read error counts.

## Magnetometer and ADC (optional)

Both are probed at start-up and, like the barometer, a missing part is
not an error:

* **QMC5883** magnetometer. The L variant (0x0D) and the P variant
  (0x2C) have different register maps; both are supported, and the boot
  log says which answered. Run at 50 Hz, range 8 G.
* **ADS1115** ADC at 0x48-0x4B, four single-ended channels, +-4.096 V
  range, 128 SPS.

The ADC is polled at 10 Hz by a low priority task in
`main/sensor_readings.cpp` - an ADS1115 conversion takes 9 ms to wait
for, which the IMU and LVGL tasks cannot spare. The magnetometer is
read in the IMU task, every other step to match its 50 Hz output, and
fed to the attitude filter for heading.

### Setting it up

The magnetometer is not mounted the IMU's way round: its x and y are
swapped and all three reversed. `mag_to_body()` in `main/mounting.h`,
the one place that knows how the sensors sit, takes it into body axes.
That was measured, not read off the board: `tools/check_mag_axes.py`
records the unit being turned about every axis, and finds which of the
24 possible mountings makes the field move the way the gyro says,
choosing between each and its exact opposite by the field pointing
down. Run it again if the board changes:

```sh
python3 tools/check_mag_axes.py --url http://192.168.0.124   # Diagnostics screen open
python3 tools/check_mag_axes.py --self-test
```

What the firmware has to measure is the calibration, from the settings
screen, kept in NVS.

Tap **Calibrate**, then - with the unit installed and level, or the
aircraft standing on level ground - turn it slowly once round, a full
circle, away from steel, hangars and other aircraft, until the bar is
full and it shows READY, and Save. That is a compass swing, as an
aircraft's compass is swung: an aircraft cannot be turned through
every orientation, and does not need to be. Until this has been done
there is no heading at all, since an uncalibrated one can be wrong by
anything up to 180 degrees.

The live F/R/D readout on the settings screen checks the result: held
level, D should be the largest and positive (north of the equator);
nose north, F positive and R near zero; turning right, HDG goes up.

Turned level about the vertical, the field the magnetometer sees traces
a circle: off centre by the hard iron, squashed into an ellipse by any
difference in gain between its axes. The calibration fits that ellipse
in the horizontal plane - taken from the accelerometer, so an aircraft
sitting nose up on its wheels is fine - and its centre and semi-axes
are the hard iron and the gains of the two horizontal axes. The
vertical axis never moves in a swing, so its offset is taken from the
local magnetic dip instead, `CONFIG_MAG_DIP_DEG` (menuconfig,
"Compass"; 67 for Poland): a degree wrong there costs nothing level and
about two degrees of heading at 20 degrees of bank. The swing is
refused if the unit was tilted about by more than 10 degrees while
turning, or if the field was disturbed on the way round. The corrected
field is scaled to a nominal 50 uT, which keeps the filter's
plausibility window (`mag_min_norm`, `mag_max_norm`) working as a test
against the calibrated field whatever the part's own scale accuracy.
The maths is in `components/ahrs/mag_calibration.{h,cpp}`; the host
tests swing a simulated unit with hard and soft iron, level and 10
degrees nose up, and check the heading it then gives all round:
within 0.7 degrees level from 51 uncalibrated.

Saving writes flash, which pauses PSRAM and with it the panel's DMA; the
picture may flicker for a frame. `CONFIG_LCD_RGB_RESTART_IN_VSYNC` puts
it back in register at the next frame rather than leaving it shifted.

## Wi-Fi debug

For a unit with no USB or serial connection: **Settings > Wi-Fi debug**
switches on a web page with the log and firmware updates. The setting is
kept, so the unit comes back up with Wi-Fi after a restart.

It joins the network set in menuconfig (`CONFIG_WIFI_DEBUG_SSID`,
`CONFIG_WIFI_DEBUG_PASSWORD`) or saved from the web page. If it cannot
within 20 s, or none is set, it also opens an access point of its own,
`MiniEFIS-XXXX`, password `CONFIG_WIFI_DEBUG_AP_PASSWORD` (default
`miniefis`), with the page at http://192.168.4.1, and keeps trying the
network every minute while nobody is on the access point. The settings
screen shows the address to open.

A network saved from the page is kept across restarts. Switching Wi-Fi
debug off forgets it, so off and on again starts over from the
menuconfig network, or the access point if there is none - the way out
of a mistyped password.

The page has:

* **the log**, live, from start-up on - every `ESP_LOGx` line, kept in
  a 128 KB ring in PSRAM (`main/web_log.cpp`).
* the **`$DIAG`/`$COMP` stream**, at `/stream`, in a 64 KB ring of its
  own so it does not push the log out: `tools/sensor_plot.py --url
  http://<address>` plots it as it does from the serial port (see
  "Plotting sensor data on a PC").
* **firmware update**: choose `build/miniefis.bin` and
  Install. The image is checked - an ESP32-S3 application of this
  project - before anything is erased, written to the other OTA slot,
  verified, and the unit restarts into it.
* **restart**, the **core dump** of the last crash (for
  `idf.py coredump-info --core coredump.elf`), and the network to join.

Two more addresses, for looking at the screens without the unit in
front of you: `/screenshot.bmp` is the picture on the panel now, and
`/screen?id=N` switches to screen N (0 horizon, 1 altimeter, 2 turn
coordinator, 3 compass, 4 settings, 5 diagnostics):

```sh
curl "http://192.168.0.124/screen?id=3"
curl -o compass.bmp http://192.168.0.124/screenshot.bmp
```

A new image boots on trial (`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`). It
confirms itself once `app_main` has finished - and, if Wi-Fi debug is
on, once its web server is up again. One that crashes or hangs before
then is replaced by the previous image at the next restart, so a bad
update cannot take away the only way to install a good one.

**There is no login.** Anyone on the network, or on the access point,
can install firmware: switch it off when it is not needed.

The display holds steady through an update. Writing flash disturbs
anything the panel reads from PSRAM, and its framebuffers are in PSRAM,
so the picture used to jump all through an update. Now, once the
upload's header shows it is this project's firmware, the update stops
LVGL and switches the panel to generating its own pixels
(`display_generate()`, `main/screens/update_screen.cpp`): no framebuffer, and
each bounce buffer filled in the panel's IRAM-safe interrupt from a
5x7 font and a few values in internal RAM. Nothing the picture needs
is in PSRAM or flash. The internal RAM for it is the panel's own
bounce buffers, freed and taken again.

It shows "UPDATING FIRMWARE", what it is doing - receiving, installing,
restarting - with a percentage and a bar, and "DO NOT SWITCH OFF". The
upload is held in PSRAM first - about 2 s over Wi-Fi - and then written
in one go, about 12 s, then the restart. A wrong file is turned away
from its header, before the display is touched; one that fails later -
a broken upload, an image that does not verify - shows UPDATE FAILED
and restarts into the firmware that was running, which was never
touched.

Code and constants also run from PSRAM (XIP, `sdkconfig.defaults`),
which drew faster than running them from flash.

Restarts go through `restart()` in `main/wifi_debug.cpp`, not
`esp_restart()`. ESP-IDF 5.2 turns the cache off on the S3 before
stopping the second core, and with LVGL drawing on that core nearly all
the time it crashed on two restarts in six; core 1 is now parked first.

The partition table changed for this (two 3 MB app slots, `otadata`,
the core dump moved to 0x620000), as did the bootloader. An existing
unit therefore needs **one last flash over USB** - `make flash`, which
writes all three - before updates over Wi-Fi work. Until then the page
refuses updates with "no OTA partition".

## Telemetry and charts

Everything the instrument estimates and measures goes out as telemetry,
not into the log: `$TEL` ten times a second (attitude, heading and the
filter's uncertainty and accelerometer weight, rates, gyro bias,
acceleration, turn rate and slip, altitude, vertical speed, pressure,
temperature, magnetic field) and `$SYS` once a second (frame rate and
drawing time, the IMU's sample count and interval, the attitude's peak
to peak over the second, errors, heap). The Wi-Fi debug page charts
them live - the last 30 s, minute or five minutes, with a pause - and
`/telemetry?from=0` serves them raw, CSV after a `$TELH`/`$SYSH` header
line with the column names. `main/diagnostics.h` has the format.

The page's Compass section follows a calibration: during a swing it
draws every point the calibrator has kept, in the horizontal plane,
with the ellipse fitted to them and its centre - the hard iron - and
how far the swing is from READY (circle covered, tilt, fit; `/compass`
serves it as JSON). At any time it also draws the calibrated field
levelled with the attitude: turned round, the dots should follow the
green circle, centred on the origin, and their centre is what is left
of the hard iron. Two charts beside it give the field strength and dip,
and the filter's heading against each sample's own.

The log keeps the events: start-up, Wi-Fi, updates, warnings and
errors, a write caught past a framebuffer, and every ten seconds the
least free stack of each task and the least free internal heap.

Reading the charts, to tell where an unsteady display comes from:

| Symptom | Meaning |
| --- | --- |
| acceleration or rates jumping, attitude jitter large | the sensor or the I2C transfer |
| sensors steady but attitude jitter large | the filter, or an axis mapped with the wrong sign |
| jitter small but fps low or uneven | the display path, not the attitude |
| IMU interval max large | the IMU task is being starved |
| gyro bias drifting | gyro offset being learned, expected |
| accelerometer weight dropping to 0 | readings rejected as not gravity: a knock, a pull |

If the attitude moves the right way initially but then creeps back the
wrong way, that axis' gyro sign is inverted relative to the
accelerometer - fix it in `sensor_to_body()` in `main/mounting.h`.

## Plotting sensor data on a PC

Two screens stream their data on the serial console, 20 times a second,
mixed in with the normal log - the format is in `main/diag_stream.h`:

* **Diagnostics** sends `$DIAG` lines: every sensor, raw.
* **Compass** sends `$COMP` lines: everything the heading is made of,
  in the order it is computed - the raw field, the axis mapping, the
  calibration, the corrected field in body axes, the roll and pitch used
  to level it, the levelled forward, right and down components, the
  heading from that one sample, the filter's heading, and why there is
  no heading when there is none.

`tools/sensor_plot.py` reads them and draws the matching view,
following the screen on the device.

The **magnetometer view** charts each axis and |B| over time, and the
XY, XZ and YZ projections. Turned through every orientation, the
projections trace circles; an off-centre circle is hard iron, an
ellipse soft iron, and the dashed fit is what the calibration on the
settings screen would produce.

The **compass view** shows the horizontal field in the level frame -
forward up, right across - with the aircraft at the reference point in
the middle and an arrow to magnetic north: the angle from the nose to
it is the heading. Turned level through a full circle, the samples
should circle the reference point; the chart measures how far off
centre they are and what that costs in heading. Below it, the two raw
sensor axes mapped to forward and right are drawn against the
calibration's centre and the circle it expects, so a reading the
calibration never saw stands out. Alongside: the filter's heading and
each sample's own, the levelled components, and roll, pitch and dip.

```sh
pip install -r tools/requirements.txt
python3 tools/sensor_plot.py                    # live, follows the screen
python3 tools/sensor_plot.py --view compass     # stay on one view
python3 tools/sensor_plot.py --body             # magnetometer view, body axes
python3 tools/sensor_plot.py --csv log          # record: log_DIAG.csv, log_COMP.csv
python3 tools/sensor_plot.py --save mag.png --duration 30   # no window
python3 tools/sensor_plot.py --url http://192.168.4.1       # over Wi-Fi
```

`--url` reads the same lines from the Wi-Fi debug web server instead of
the serial port; the address is on the unit's settings screen. It polls
five times a second and starts from the newest, so it lags by up to
about 200 ms, and like the serial stream it only has data while the
Diagnostics or Compass screen is showing.

The script opens the port without resetting the device. If matplotlib
fails to import with a NumPy 1.x/2.x error, the system matplotlib and a
pip installed NumPy disagree; run it from a virtual environment.

## Unexpected restarts

The start-up log, and the bottom of the diagnostics screen, say why the
unit last reset. A crash also writes a core dump to the `coredump`
partition, and the next start-up prints the crashed task and its
backtrace from it - so a restart that happened away from the PC can
still be traced. Decode the backtrace against the ELF that was running:

```sh
xtensa-esp32s3-elf-addr2line -pfiaC -e build/miniefis.elf 0x4200b46f ...
```

**One crash was traced**, from a core dump taken on a unit in use: the
LVGL task faulted reading the touch controller's I2C device record,
which had been overwritten with 0xFFFFFFFF. The I2C driver had put that
record in PSRAM, in the 64 bytes of alignment padding the heap left
between the two framebuffers - four bytes past the end of the first,
where a stray white pixel would land. What wrote there was found later:
`fast_draw_line()` worked out each row's span in float and converted it
to screen coordinates before checking it was empty. A line within a
hair of level or vertical - the pitch ladder and the bank pointer
whenever the roll passes within a tenth of a degree of zero - divides by
almost nothing there, and a bound of 100000 wrapped to -31072: the span
began tens of kilobytes before the row. It crashed the unit again on the
horizon screen, through the PSRAM heap it had overwritten with yellow;
a host harness with the real drawing code wrote outside the framebuffer
in 76 of 4001 frames at rolls within 0.2 degrees of level. Empty rows
are now rejected before the conversion.

The earlier defences stay: `CONFIG_I2C_ISR_IRAM_SAFE` keeps the I2C
driver's records in internal RAM; and the gaps beside the framebuffers are filled with
guard blocks (`display_st7701.cpp`) that the diagnostics task checks
four times a second. A hit is logged as "something wrote past a
framebuffer", with the value and address, and counted as "fb guard" on
the diagnostics screen - it should stay at 0, and if it ever does not,
that is the lead to follow.

Every ten seconds the serial log also reports the least free stack each
task has had. Measured, including the deepest path - a failed I2C read
logging a warning, which costs about 1 KB - every task keeps at least
1 KB spare.

**Opening the serial port resets the board.** DTR and RTS drive the
reset and boot pins through the usual auto-program circuit, which is how
flashing works - and the board's CH343 bridge reports itself as a modem,
so on Linux ModemManager opens it about five seconds after every plug-in
to probe it. To stop that:

```sh
echo 'ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", ENV{ID_MM_DEVICE_IGNORE}="1"' \
    | sudo tee /etc/udev/rules.d/99-esp-no-modemmanager.rules
sudo udevadm control --reload-rules
```

Serial monitors reset it too when they connect. To watch the log
without restarting, the order matters: Linux asserts both lines when
the port opens, and releasing DTR before RTS passes through RTS alone,
which is reset. Release RTS first, then DTR - `SerialLines._open()` in
`tools/sensor_plot.py` does this, and is the pattern to copy.

## Measured on hardware

Figures from the board itself, for reference when changing any of this:

| | |
| --- | --- |
| Accelerometer zero-g offset | +0.06, +0.01, +0.05 g - corrected in `imu_config_t` |
| Accelerometer scale | within 2% of unity once the offset is removed |
| Gyro zero offset | +3.2, +1.2, -0.3 deg/s - measured at every start-up |
| Attitude at rest, mounted | roll -0.3, pitch +0.1 deg |
| Attitude jitter at rest | 0.10-0.19 deg peak to peak |
| IMU sample interval | 6-13 ms, nominal 10 |
| Redraw | 36 ms: background 17, lines and pitch numbers 6, readout panel 5, labels and LVGL 8 |
| Frame rate | 24 fps, panel refreshing at 48 Hz |

### How the frame rate got there

It started at 16 fps with an 80 ms redraw. Four things mattered, all of
them found by measuring rather than guessing - the breakdown on the
serial log exists for exactly this:

* **The screen was being filled twice.** LVGL painted the screen
  background, then the widget painted over every pixel of it. Answering
  the cover check so LVGL skips the screen: -17 ms.
* **Sky and ground were painted separately**, so a third of the screen
  was written twice and half of it through LVGL's polygon masks. The
  horizon is a straight line, so each row is split at one point: filling
  two runs per row writes each pixel exactly once, with no masks. Paired
  with the point above: -20 ms.
* **LVGL's line drawing cost 300 microseconds a line** against 13
  microseconds' worth of pixels, nearly all of it mask setup, and the
  instrument draws about thirty lines a frame. Filling the rotated
  rectangles directly: -8 ms.
* **The corners are not wired to anything** on a round panel. Skipping a
  fifth of the pixels: -4 ms.

The pitch ladder numbers came later and cost 4 ms through
`lv_draw_label`, almost all of it text layout for a two-digit number.
Placing the glyphs directly with `lv_draw_letter` brought that back to
2.5 ms.

That took the redraw from 80 ms to 35 ms - and the frame rate from 16 to
20, because the horizon was only asking for a redraw every 50 ms. Once
the drawing was fast enough, our own timer had quietly become the limit.

### Why the old buffer is held until the panel lets go

The panel's interrupt copies the picture into the bounce buffers ahead
of the scan, and picks up a newly flushed framebuffer only when it
starts copying the next frame. Until then it still reads the old one.
Handing that buffer back to LVGL at the flush, or at the next VSYNC,
let LVGL start the next frame in a buffer still on the screen: the
panel showed the background repainted and the horizon lines and
numbers not yet drawn - a blink, which got worse as the drawing got
faster. `flush_cb()` in `components/board/lvgl_port.cpp` therefore waits
for the driver's "frame copied" callback before releasing it. The log's
`cut` counts frames the panel had to cut short instead (its refills
fell behind); it should stay at 0.

### Why 24 and not more

Frames are quantised by the panel. The flush waits for the panel to
switch to the new frame - which it does only at a frame boundary - so
a frame costs `ceil(draw / panel_period)` periods:
at 48 Hz that is 20.8 ms a step, and a 35 ms redraw takes two of them -
41.6 ms, or 24 fps. Anything between 21 and 41 ms of drawing produces
exactly 24 fps, which is why the measured total sits at 41.0 ms with 10
ms of headroom rather than scraping the boundary.

The next step up is one period, 48 fps, which needs the whole redraw
inside 20.8 ms. The background fill alone is 17 ms and is bound by PSRAM
write bandwidth - measured at 88 ns per pixel, unchanged by writing two
pixels per store, while the panel's DMA reads a whole framebuffer every
period. So 48 fps is not reachable by tuning; it needs the screen to
stop being repainted in full every frame, which means partial redraw
instead of `full_refresh`, and that is a change of architecture rather
than a tweak.

## Limits

This is a prototype instrument, not an aviation-grade AHRS. Without
airspeed or GPS it cannot tell a sustained acceleration from a tilt;
it can only recognise the ones that look like acceleration:

| Held | False pitch, worst | Accelerometer alone |
| --- | --- | --- |
| 0.5 g, 2 s | 0.0 deg (rejected) | 26.6 deg |
| 0.25 g, 3 s | 0.0 deg (rejected) | 14.0 deg |
| 0.1 g, 10 s | 5.9 deg - half of it after 1.7 s | 5.7 deg |
| 0.05 g, 30 s | 3.1 deg | 2.9 deg |

The accelerometer is used all the time, moving or not: the filter
assumes no airspeed (`accel_turn_speed_ms = 0`), right for a unit in the
hand, where turning it makes no acceleration worth the name. In an
aircraft a turn does: at speed V and turn rate w the aircraft is pushed
towards the centre at V w, and in a coordinated turn the accelerometer
reads "level" while it is banked - by 8 degrees in a standard rate turn
at 100 km/h, 17 at 200. With speed 0 the filter cannot tell, and a
sustained turn leans the horizon towards level at the rate the gravity
update corrects. Setting `accel_turn_speed_ms` to the aircraft's speed
makes the filter fly turns faster than about 0.6 deg/s on the gyro
instead (the flight tests run that way); a measured airspeed would let
it subtract the push outright.
For flight use, feed the same data into an AHRS with air data or GPS
aiding.
