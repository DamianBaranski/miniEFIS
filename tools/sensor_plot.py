#!/usr/bin/env python3
"""
Live charts from the instrument's serial stream.

The firmware streams 20 samples a second on the serial console while
one of two screens is showing (main/diag_stream.h has the format):

  Diagnostics screen -> $DIAG lines, every sensor raw
  Compass screen     -> $COMP lines, everything the heading is made of

This reads them and draws the matching view, switching by itself when
the screen on the device changes.

Magnetometer view ($DIAG):
  top      the field on each axis and |B|, over time
  bottom   the field projected onto the XY, XZ and YZ planes, with the
           min/max fit the firmware's calibration would make. Turned
           through every orientation the points trace circles; off
           centre is hard iron, an ellipse is soft iron.

Compass view ($COMP):
  left, big    the horizontal field in the level frame - forward up,
               right to the right - with the aircraft at the reference
               point in the middle. The field points to magnetic north,
               and the angle from the nose to it is the heading. Turned
               level through a full circle the points should circle the
               reference point; any offset from it is heading error.
  left, small  the two raw sensor axes that are mapped to forward and
               right, with the calibration's centre - the reference
               point the correction subtracts - and the circle the
               calibration expects.
  right        heading (the filter's, and from each sample alone), the
               levelled components it is computed from, and the roll and
               pitch used to level them.

    pip install -r tools/requirements.txt
    python3 tools/sensor_plot.py                   # live, follows the screen
    python3 tools/sensor_plot.py --view compass    # stay on one view
    python3 tools/sensor_plot.py --body            # magnetometer view, body axes
    python3 tools/sensor_plot.py --csv log         # record: log_DIAG.csv, log_COMP.csv
    python3 tools/sensor_plot.py --save hdg.png --duration 30   # no window
    python3 tools/sensor_plot.py --url http://192.168.4.1       # over Wi-Fi

Opening the port does not restart the device - see SerialLines for why
that takes care.

--url reads the same sentences over Wi-Fi, from the unit's Wi-Fi debug
web server (main/wifi_debug.h), polling /stream for what is new. The
address is on the unit's settings screen.
"""

import argparse
import collections
import csv
import http.client
import math
import sys
import threading
import time
import urllib.parse

# Magnetometer view: sensor axes before any correction, which is what
# calibration fits - or the calibrated field in body axes.
RAW_COLUMNS = ("mx_uT", "my_uT", "mz_uT")
BODY_COLUMNS = ("mf_uT", "mr_uT", "md_uT")

# The filter's plausibility window for the corrected field
# (AttitudeConfig::mag_min_norm, mag_max_norm): outside it the sample is
# thrown away and the heading coasts on the gyro, then goes invalid.
FIELD_WINDOW_UT = (15.0, 90.0)

# How much of a circle the trail has to cover before its centre means
# anything.
CENTRING_MIN_COVERAGE_DEG = 270.0

COMPASS_STATES = ("OK", "NO MAGNETOMETER", "NOT CALIBRATED", "CALIBRATING",
                  "NO HEADING")
FILTER_STATUS = ("NO DATA", "GYRO CAL", "FUSED", "GYRO ONLY")
AXIS_NAMES = {1: "+X", -1: "-X", 2: "+Y", -2: "-Y", 3: "+Z", -3: "-Z"}

# Which magnetometer axis is forward and right (main/mounting.h,
# mag_to_body): forward = -z, right = -x. Streams from firmware that
# still had a configurable mapping carry it as mount_f and mount_r,
# which win.
MOUNT_FORWARD = -3
MOUNT_RIGHT = -1

SENTENCES = ("DIAG", "COMP")


def checksum_ok(line):
    """NMEA style: XOR of everything between '$' and '*'."""
    star = line.rfind("*")

    if not line.startswith("$") or star < 0 or len(line) < star + 3:
        return False

    value = 0
    for char in line[1:star]:
        value ^= ord(char)

    try:
        return value == int(line[star + 1:star + 3], 16)
    except ValueError:
        return False


def to_float(text):
    try:
        return float(text)
    except ValueError:
        return math.nan


def finite(values):
    return [v for v in values if math.isfinite(v)]


def fit(values):
    """The firmware's calibration: centre and radius from the extremes."""
    values = finite(values)

    if not values:
        return math.nan, math.nan

    low, high = min(values), max(values)
    return (high + low) / 2.0, (high - low) / 2.0


def coverage_deg(angles):
    """How much of the circle a set of directions spans: 360 less the
    widest gap between neighbours."""
    angles = sorted(a % 360.0 for a in finite(angles))

    if len(angles) < 2:
        return 0.0

    gaps = [b - a for a, b in zip(angles, angles[1:])]
    gaps.append(angles[0] + 360.0 - angles[-1])
    return 360.0 - max(gaps)


def break_wraps(times, values, limit=180.0):
    """Insert gaps where an angle wraps round, so 359 -> 1 is not a line
    straight down the chart. The gap goes into both lists, which have to
    stay the same length."""
    out_t, out_v = [], []
    for i, (t, value) in enumerate(zip(times, values)):
        if i and math.isfinite(value) and math.isfinite(values[i - 1]) \
                and abs(value - values[i - 1]) > limit:
            out_t.append(t)
            out_v.append(math.nan)
        out_t.append(t)
        out_v.append(value)
    return out_t, out_v


class SerialLines:
    """Lines from the serial console."""

    def __init__(self, port, baud):
        self.link = self._open(port, baud)

    @staticmethod
    def _open(port, baud):
        import serial

        def open_at(rate):
            """
            Open without resetting the device.

            DTR and RTS drive its boot and reset pins through the usual
            auto-program circuit: RTS asserted with DTR released pulls
            reset. Linux asserts both when the port opens, and pyserial
            then applies DTR before RTS - so asking for both released
            passes through exactly that state and resets the board.
            Releasing RTS first, then DTR, goes through DTR alone, which
            only touches the boot pin and does nothing while running.
            """
            link = serial.Serial()
            link.port = port
            link.baudrate = rate
            link.timeout = 0.5
            link.dtr = True
            link.rts = False
            link.open()
            link.dtr = False
            return link

        # Open at another rate first. The board's CH343 bridge can keep a
        # stale line setting when asked for the rate the kernel already
        # believes it has, and then everything arrives as garbage.
        open_at(9600 if baud != 9600 else 19200).close()

        return open_at(baud)

    def __iter__(self):
        while True:
            raw = self.link.readline()

            if raw:
                yield raw.decode("ascii", errors="replace")


class HttpLines:
    """
    Lines from the Wi-Fi debug web server's /stream.

    Each reply carries what was written since the position asked for,
    and the position to ask for next. Starting from "end" skips the ten
    seconds or so the unit still holds from before.
    """

    POLL_S = 0.2

    def __init__(self, url):
        parts = urllib.parse.urlsplit(url if "://" in url
                                      else "http://" + url)
        self.host = parts.hostname
        self.port = parts.port or 80
        self.connection = None

    def _get(self, position):
        if self.connection is None:
            self.connection = http.client.HTTPConnection(
                self.host, self.port, timeout=5)

        try:
            self.connection.request("GET", f"/stream?from={position}")
            reply = self.connection.getresponse()
            body = reply.read()
            return body, reply.getheader("X-Log-Next")
        except (OSError, http.client.HTTPException):
            self.connection.close()
            self.connection = None
            raise

    def __iter__(self):
        position = "end"
        partial = ""

        while True:
            try:
                body, following = self._get(position)
            except (OSError, http.client.HTTPException) as error:
                print(f"{self.host}: {error}, retrying", file=sys.stderr)
                time.sleep(2.0)
                continue

            # A restart starts the count again: the unit then answers
            # with the oldest it has, and the half line held is stale.
            if following is not None:
                if position != "end" and int(following) < int(position):
                    partial = ""
                position = following

            lines = (partial + body.decode("ascii", errors="replace"))
            lines = lines.split("\n")
            partial = lines.pop()

            for line in lines:
                yield line

            # A full reply means more is waiting.
            if len(body) < 4096:
                time.sleep(self.POLL_S)


class Stream:
    """Reads the sentences in a background thread and keeps recent ones."""

    def __init__(self, lines, keep, csv_prefix):
        self.columns = {}
        self.samples = {kind: collections.deque(maxlen=keep)
                        for kind in SENTENCES}
        self.latest_kind = None
        self.lock = threading.Lock()
        self.received = 0
        self.rejected = 0
        self.csv_prefix = csv_prefix
        self.csv = {}
        self.lines = lines

        thread = threading.Thread(target=self._run, daemon=True)
        thread.start()

    def _run(self):
        for line in self.lines:
            line = line.strip()
            start = line.find("$")

            if start < 0:
                continue

            line = line[start:]
            name = line[1:line.find(",")] if "," in line else ""
            kind = name[:-1] if name.endswith("H") else name

            if kind not in SENTENCES:
                continue

            if not checksum_ok(line):
                self.rejected += 1
                continue

            fields = line[:line.rfind("*")].split(",")

            if name.endswith("H"):
                self._header(kind, fields[2:])
            elif kind in self.columns:
                self._sample(kind, fields[1:])

    def _header(self, kind, names):
        if self.columns.get(kind) == names:
            return

        self.columns[kind] = names

        if self.csv_prefix:
            handle = open(f"{self.csv_prefix}_{kind}.csv", "w", newline="")
            writer = csv.writer(handle)
            writer.writerow(names)
            self.csv[kind] = (handle, writer)

    def _sample(self, kind, values):
        names = self.columns[kind]

        if len(values) != len(names):
            self.rejected += 1
            return

        sample = {name: to_float(value) for name, value in zip(names, values)}

        with self.lock:
            self.samples[kind].append(sample)
            self.latest_kind = kind
            self.received += 1

        if kind in self.csv:
            handle, writer = self.csv[kind]
            writer.writerow(values)
            handle.flush()

    def snapshot(self, kind):
        with self.lock:
            return list(self.samples[kind])


class MagnetometerView:
    """$DIAG: the field on each axis, and its projections."""

    kind = "DIAG"

    def __init__(self, figure, stream, args):
        self.stream = stream
        self.columns = BODY_COLUMNS if args.body else RAW_COLUMNS
        self.window_s = args.window
        self.scatter_points = args.points

        from matplotlib.patches import Ellipse

        grid = figure.add_gridspec(2, 3, height_ratios=(1, 1.3))
        self.series = figure.add_subplot(grid[0, :])
        self.planes = [figure.add_subplot(grid[1, i]) for i in range(3)]

        # Body axes are forward, right and down; the sensor's are its own.
        names = ("F", "R", "D") if self.columns == BODY_COLUMNS \
            else ("x", "y", "z")
        self.names = names

        self.lines = {}
        for name, colour in zip(names + ("|B|",),
                                ("tab:red", "tab:green", "tab:blue", "black")):
            (self.lines[name],) = self.series.plot(
                [], [], color=colour, label=name,
                linewidth=1.0 if name != "|B|" else 1.6)

        self.series.set_xlabel("time, s")
        self.series.set_ylabel("uT")
        self.series.grid(True, alpha=0.3)
        self.series.legend(loc="upper left", ncol=4)

        self.pairs = ((names[0], names[1], 0, 1), (names[0], names[2], 0, 2),
                      (names[1], names[2], 1, 2))
        self.dots, self.latest, self.circles, self.notes = [], [], [], []

        for axes, (h, v, _, _) in zip(self.planes, self.pairs):
            (dots,) = axes.plot([], [], ".", markersize=2, alpha=0.4)
            (latest,) = axes.plot([], [], "o", color="tab:red", markersize=7)
            circle = Ellipse((0, 0), 0, 0, fill=False, linestyle="--",
                             color="tab:orange", linewidth=1.5)
            axes.add_patch(circle)
            axes.axhline(0, color="grey", linewidth=0.5)
            axes.axvline(0, color="grey", linewidth=0.5)
            axes.set_xlabel(f"{h}, uT")
            axes.set_ylabel(f"{v}, uT")
            axes.set_aspect("equal", adjustable="datalim")
            axes.grid(True, alpha=0.3)
            note = axes.text(0.02, 0.98, "", transform=axes.transAxes,
                             va="top", family="monospace", fontsize=8)

            self.dots.append(dots)
            self.latest.append(latest)
            self.circles.append(circle)
            self.notes.append(note)

    def update(self, samples):
        t_end = samples[-1]["t_ms"] / 1000.0
        axes_data = [[s[c] for s in samples] for c in self.columns]
        t = [s["t_ms"] / 1000.0 - t_end for s in samples]
        norm = [math.sqrt(x * x + y * y + z * z)
                for x, y, z in zip(*axes_data)]

        first = next((i for i, v in enumerate(t) if v >= -self.window_s), 0)

        for name, values in zip(self.names + ("|B|",), axes_data + [norm]):
            self.lines[name].set_data(t[first:], values[first:])

        self.series.set_xlim(-self.window_s, 0)
        self.series.relim()
        self.series.autoscale_view(scalex=False)

        recent = [values[-self.scatter_points:] for values in axes_data]
        centres, radii = zip(*(fit(values) for values in recent))

        for i, (h_name, v_name, h, v) in enumerate(self.pairs):
            self.dots[i].set_data(recent[h], recent[v])
            self.latest[i].set_data([recent[h][-1]], [recent[v][-1]])

            if all(math.isfinite(x) for x in (centres[h], centres[v],
                                              radii[h], radii[v])):
                self.circles[i].set_center((centres[h], centres[v]))
                self.circles[i].width = 2 * radii[h]
                self.circles[i].height = 2 * radii[v]

            self.notes[i].set_text(
                f"offset {h_name} {centres[h]:+7.1f}  {v_name} {centres[v]:+7.1f}\n"
                f"radius {h_name} {radii[h]:7.1f}  {v_name} {radii[v]:7.1f}")

            self.planes[i].relim()
            self.planes[i].autoscale_view()

        latest = samples[-1]
        heading = latest.get("hdg_deg", math.nan)

        return (f"{'calibrated, body axes' if self.columns == BODY_COLUMNS else 'raw, sensor axes'}"
                f"   |B| now {norm[-1]:.1f} uT"
                f"   fit radius {sum(radii) / 3.0:.1f} uT"
                f"   heading {'---' if math.isnan(heading) else f'{heading:.0f}'}"
                f"   calibrated {'yes' if latest.get('mag_cal') == 1 else 'no'}")


class CompassView:
    """$COMP: where the heading comes from."""

    kind = "COMP"

    def __init__(self, figure, stream, args):
        self.stream = stream
        self.window_s = args.window
        self.trail_points = args.points

        from matplotlib.patches import Ellipse

        grid = figure.add_gridspec(3, 3, width_ratios=(1.25, 1, 1),
                                   height_ratios=(1, 1, 1))

        # ---- horizontal field, level frame ------------------------------
        self.plane = figure.add_subplot(grid[0:2, 0])
        axes = self.plane
        axes.set_title("horizontal field, level frame", fontsize=10)
        axes.set_xlabel("right, uT")
        axes.set_ylabel("forward, uT")
        # Limits are set by hand each frame, so keep it square by the box.
        axes.set_aspect("equal", adjustable="box")
        axes.grid(True, alpha=0.3)
        axes.axhline(0, color="grey", linewidth=0.5)
        axes.axvline(0, color="grey", linewidth=0.5)

        (self.trail,) = axes.plot([], [], ".", markersize=2, alpha=0.35,
                                  color="tab:blue", label="recent samples")
        self.reference_circle = Ellipse((0, 0), 0, 0, fill=False,
                                        linestyle="--", color="grey",
                                        linewidth=1.0)
        axes.add_patch(self.reference_circle)
        (self.reference,) = axes.plot([0], [0], "+", color="black",
                                      markersize=16, markeredgewidth=2,
                                      label="reference point (0, 0)")
        (self.trail_centre,) = axes.plot([], [], "x", color="tab:orange",
                                         markersize=10, markeredgewidth=2,
                                         label="centre of the samples")
        (self.north,) = axes.plot([], [], color="tab:red", linewidth=2.5,
                                  label="field: magnetic north")
        (self.north_tip,) = axes.plot([], [], "o", color="tab:red",
                                      markersize=8)
        (self.nose,) = axes.plot([], [], color="black", linewidth=2.0,
                                 label="nose")
        self.north_label = axes.text(0, 0, "N", color="tab:red",
                                     fontsize=14, fontweight="bold",
                                     ha="center", va="center")
        self.plane_note = axes.text(0.02, 0.02, "", transform=axes.transAxes,
                                    va="bottom", family="monospace",
                                    fontsize=8)
        axes.legend(loc="upper right", fontsize=7)

        # ---- raw sensor axes behind forward and right -------------------
        self.raw = figure.add_subplot(grid[2, 0])
        self.raw.set_title("raw sensor axes used for forward and right",
                           fontsize=10)
        self.raw.set_aspect("equal", adjustable="datalim")
        self.raw.grid(True, alpha=0.3)
        (self.raw_dots,) = self.raw.plot([], [], ".", markersize=2, alpha=0.35)
        (self.raw_latest,) = self.raw.plot([], [], "o", color="tab:red",
                                           markersize=6)
        (self.raw_centre,) = self.raw.plot([], [], "+", color="black",
                                           markersize=14, markeredgewidth=2,
                                           label="calibration centre")
        self.raw_circle = Ellipse((0, 0), 0, 0, fill=False, linestyle="--",
                                  color="tab:orange", linewidth=1.5,
                                  label="calibrated circle")
        self.raw.add_patch(self.raw_circle)
        self.raw.legend(loc="upper right", fontsize=7)

        # ---- time series --------------------------------------------------
        self.heading = figure.add_subplot(grid[0, 1:])
        self.heading.set_ylabel("heading, deg")
        self.heading.set_ylim(0, 360)
        self.heading.set_yticks(range(0, 361, 90))
        self.heading.grid(True, alpha=0.3)
        (self.hdg_line,) = self.heading.plot([], [], color="black",
                                             linewidth=1.6, label="filter (shown)")
        (self.mag_hdg_line,) = self.heading.plot([], [], ".", markersize=3,
                                                 color="tab:red",
                                                 label="this sample alone")
        self.heading.legend(loc="upper left", fontsize=8, ncol=2)

        self.level = figure.add_subplot(grid[1, 1:], sharex=self.heading)
        self.level.set_ylabel("levelled, uT")
        self.level.grid(True, alpha=0.3)
        self.level_lines = {}
        for name, colour, label in (("lf_uT", "tab:red", "forward"),
                                    ("lr_uT", "tab:green", "right"),
                                    ("ld_uT", "tab:blue", "down"),
                                    ("h_uT", "black", "horizontal |H|")):
            (self.level_lines[name],) = self.level.plot(
                [], [], color=colour, label=label,
                linewidth=1.6 if name == "h_uT" else 1.0)
        self.level.legend(loc="upper left", fontsize=8, ncol=4)

        self.tilt = figure.add_subplot(grid[2, 1:], sharex=self.heading)
        self.tilt.set_ylabel("deg")
        self.tilt.set_xlabel("time, s")
        self.tilt.grid(True, alpha=0.3)
        self.tilt_lines = {}
        for name, colour, label in (("roll_deg", "tab:purple", "roll"),
                                    ("pitch_deg", "tab:brown", "pitch"),
                                    ("incl_deg", "tab:cyan", "field dip")):
            (self.tilt_lines[name],) = self.tilt.plot([], [], color=colour,
                                                      label=label)
        self.tilt.legend(loc="upper left", fontsize=8, ncol=3)

    def update(self, samples):
        latest = samples[-1]
        t_end = latest["t_ms"] / 1000.0
        t = [s["t_ms"] / 1000.0 - t_end for s in samples]
        first = next((i for i, v in enumerate(t) if v >= -self.window_s), 0)
        window = samples[first:]
        tw = t[first:]

        def column(name, rows=window):
            return [s[name] for s in rows]

        # ---- time series ----
        self.hdg_line.set_data(*break_wraps(tw, column("hdg_deg")))
        self.mag_hdg_line.set_data(tw, column("mag_hdg_deg"))
        self.heading.set_xlim(-self.window_s, 0)

        for name, line in self.level_lines.items():
            line.set_data(tw, column(name))
        for name, line in self.tilt_lines.items():
            line.set_data(tw, column(name))
        for axes in (self.level, self.tilt):
            axes.relim()
            axes.autoscale_view(scalex=False)

        # ---- level plane: right across, forward up ----
        trail = samples[-self.trail_points:]
        right = column("lr_uT", trail)
        forward = column("lf_uT", trail)
        self.trail.set_data(right, forward)

        lf, lr, h = latest["lf_uT"], latest["lr_uT"], latest["h_uT"]
        mean_h = sum(finite(column("h_uT", trail))) / max(
            1, len(finite(column("h_uT", trail))))

        if math.isfinite(lf) and math.isfinite(lr):
            self.north.set_data([0, lr], [0, lf])
            self.north_tip.set_data([lr], [lf])
            self.north_label.set_position((lr * 1.15, lf * 1.15))
            self.north_label.set_visible(True)
        else:
            self.north.set_data([], [])
            self.north_tip.set_data([], [])
            self.north_label.set_visible(False)

        reach = max(mean_h, h if math.isfinite(h) else 0, 1.0)
        self.nose.set_data([0, 0], [0, reach * 0.6])
        self.reference_circle.set_center((0, 0))
        self.reference_circle.width = self.reference_circle.height = 2 * mean_h

        # Where the samples are centred: with a good calibration, turning
        # level through a circle keeps this on the reference point. Only
        # meaningful once they go most of the way round - a cluster's
        # "centre" is just where it is.
        covered = coverage_deg(math.degrees(math.atan2(-r, f))
                               for r, f in zip(right, forward))
        notes = []

        if covered >= CENTRING_MIN_COVERAGE_DEG:
            cr, _ = fit(right)
            cf, _ = fit(forward)
            offset = math.hypot(cr, cf)
            self.trail_centre.set_data([cr], [cf])
            error = math.degrees(math.asin(min(offset / mean_h, 1.0))) \
                if mean_h > 0 else math.nan
            notes.append(f"samples centred {offset:4.1f} uT off reference:"
                         f" up to {error:.0f} deg heading error")
        else:
            self.trail_centre.set_data([], [])
            notes.append(f"samples cover {covered:3.0f} deg - turn it level "
                         "through a full circle to check centring")

        notes.append(f"mean |H| {mean_h:5.1f} uT")

        b = latest["b_uT"]
        if math.isfinite(b) and not FIELD_WINDOW_UT[0] <= b <= FIELD_WINDOW_UT[1]:
            notes.append(f"|B| {b:.0f} uT outside {FIELD_WINDOW_UT[0]:.0f}-"
                         f"{FIELD_WINDOW_UT[1]:.0f}: filter rejects it")

        self.plane_note.set_text("\n".join(notes))

        limit = reach * 1.35
        self.plane.set_xlim(-limit, limit)
        self.plane.set_ylim(-limit, limit)

        # ---- raw sensor axes mapped to forward and right ----
        f_code = latest.get("mount_f", MOUNT_FORWARD)
        r_code = latest.get("mount_r", MOUNT_RIGHT)

        if math.isfinite(f_code) and math.isfinite(r_code):
            f_code, r_code = int(f_code), int(r_code)
            axis_columns = ("mx_uT", "my_uT", "mz_uT")
            offsets = ("off_x", "off_y", "off_z")
            scales = ("sc_x", "sc_y", "sc_z")
            fi, ri = abs(f_code) - 1, abs(r_code) - 1
            fs, rs = math.copysign(1, f_code), math.copysign(1, r_code)

            # Signed as mounted, so this is the same way round as the
            # level plane: right across, forward up.
            raw_r = [rs * s[axis_columns[ri]] for s in trail]
            raw_f = [fs * s[axis_columns[fi]] for s in trail]
            self.raw_dots.set_data(raw_r, raw_f)
            self.raw_latest.set_data([raw_r[-1]], [raw_f[-1]])
            self.raw.set_xlabel(f"sensor {AXIS_NAMES[r_code]} (right), uT")
            self.raw.set_ylabel(f"sensor {AXIS_NAMES[f_code]} (fwd), uT")

            if latest["cal"] == 1:
                centre_r = rs * latest[offsets[ri]]
                centre_f = fs * latest[offsets[fi]]
                # The calibration maps a radius of 50 / scale onto the
                # nominal field, so that is the circle it expects.
                self.raw_centre.set_data([centre_r], [centre_f])
                self.raw_circle.set_center((centre_r, centre_f))
                self.raw_circle.width = 2 * 50.0 / latest[scales[ri]]
                self.raw_circle.height = 2 * 50.0 / latest[scales[fi]]
                self.raw_circle.set_visible(True)
            else:
                self.raw_centre.set_data([], [])
                self.raw_circle.set_visible(False)

            self.raw.relim()
            self.raw.autoscale_view()

        # ---- status ----
        state = latest["state"]
        status = latest["status"]
        hdg = latest["hdg_deg"]
        mag_hdg = latest["mag_hdg_deg"]
        diff = (mag_hdg - hdg + 180) % 360 - 180 \
            if math.isfinite(hdg) and math.isfinite(mag_hdg) else math.nan

        def show(value, fmt):
            return "---" if not math.isfinite(value) else format(value, fmt)

        return (f"{COMPASS_STATES[int(state)] if math.isfinite(state) and int(state) < len(COMPASS_STATES) else '?'}"
                f"   HDG {show(hdg, '03.0f')}"
                f"   sample {show(mag_hdg, '03.0f')} ({show(diff, '+.1f')})"
                f"   |B| {show(latest['b_uT'], '.1f')}  |H| {show(latest['h_uT'], '.1f')} uT"
                f"   dip {show(latest['incl_deg'], '.0f')}"
                f"   cal field {show(latest['cal_field_uT'], '.1f')} uT"
                f"   z bias {show(latest['bias_z_dps'], '+.2f')}"
                f"   {FILTER_STATUS[int(status)] if math.isfinite(status) and int(status) < 4 else ''}")


VIEWS = {"mag": MagnetometerView, "compass": CompassView}
VIEW_FOR_KIND = {"DIAG": "mag", "COMP": "compass"}


class Window:
    """Holds the figure, and swaps views when the stream changes kind."""

    def __init__(self, stream, args):
        import matplotlib.pyplot as plt

        self.plt = plt
        self.stream = stream
        self.args = args
        self.fixed = args.view if args.view != "auto" else None
        self.figure = plt.figure(figsize=(15, 9))
        self.view = None
        self.title = self.figure.suptitle("", fontsize=10)
        self._build(self.fixed or "mag")

    def _build(self, name):
        for axes in list(self.figure.axes):
            axes.remove()
        self.view = VIEWS[name](self.figure, self.stream, self.args)
        self.figure.tight_layout(rect=(0, 0, 1, 0.96))

    def update(self, _frame=None):
        kind = self.stream.latest_kind

        if self.fixed is None and kind and VIEW_FOR_KIND[kind] != \
                ("mag" if isinstance(self.view, MagnetometerView) else "compass"):
            self._build(VIEW_FOR_KIND[kind])

        samples = self.stream.snapshot(self.view.kind)

        if not samples:
            screen = "Diagnostics" if self.view.kind == "DIAG" else "Compass"
            self.title.set_text(f"waiting for ${self.view.kind} lines - open "
                                f"the {screen} screen on the device")
            return

        status = self.view.update(samples)
        span = max((samples[-1]["t_ms"] - samples[0]["t_ms"]) / 1000.0, 1e-3)
        rate = (len(samples) - 1) / span if len(samples) > 1 else 0.0

        self.title.set_text(f"{status}   |   {rate:.0f}/s   "
                            f"bad lines {self.stream.rejected}")

    def run(self):
        from matplotlib.animation import FuncAnimation

        self._animation = FuncAnimation(self.figure, self.update,
                                        interval=200, cache_frame_data=False)
        self.plt.show()

    def save(self, path):
        self.update()
        self.figure.savefig(path, dpi=100)


def main():
    parser = argparse.ArgumentParser(
        description="Chart the instrument's $DIAG and $COMP streams, "
                    "from the serial port or over Wi-Fi.")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--url", help="read over Wi-Fi from the unit's "
                                      "Wi-Fi debug web server instead, "
                                      "e.g. http://192.168.4.1")
    parser.add_argument("--view", choices=("auto", "mag", "compass"),
                        default="auto",
                        help="auto follows the screen showing on the device")
    parser.add_argument("--window", type=float, default=20.0,
                        help="seconds of history in the time series")
    parser.add_argument("--points", type=int, default=3000,
                        help="samples kept for the projections and trails")
    parser.add_argument("--body", action="store_true",
                        help="magnetometer view: the calibrated field in "
                             "body axes instead of the raw readings")
    parser.add_argument("--csv", metavar="PREFIX",
                        help="record every column, to PREFIX_DIAG.csv and "
                             "PREFIX_COMP.csv")
    parser.add_argument("--save", help="collect for --duration seconds, "
                                       "save the chart to this file, exit")
    parser.add_argument("--duration", type=float, default=20.0)
    args = parser.parse_args()

    if args.save:
        import matplotlib
        matplotlib.use("Agg")

    keep = max(args.points, int(args.window * 25))
    lines = (HttpLines(args.url) if args.url
             else SerialLines(args.port, args.baud))
    stream = Stream(lines, keep, args.csv)
    window = Window(stream, args)

    if args.save:
        time.sleep(args.duration)
        window.update()
        window.save(args.save)
        print(f"{stream.received} samples, {stream.rejected} bad lines, "
              f"saved {args.save}")
        if stream.received == 0:
            print("no samples: open the Diagnostics or Compass screen on "
                  "the device", file=sys.stderr)
            sys.exit(1)
    else:
        window.run()


if __name__ == "__main__":
    main()
