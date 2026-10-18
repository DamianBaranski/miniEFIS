#!/usr/bin/env python3
"""
An engine monitoring unit (EMU) on the CAN bus, for testing the MiniEFIS
without an engine.

Sends what the engine unit (node 4, engine 1; node 5, engine 2) sends
in docs/CAN_PROTOCOL.md, through any USB-CAN adapter python-can drives:

  0x300 ENGINE_FAST      10 Hz  RPM, manifold, oil and fuel pressure
  0x301 ENGINE_SLOW       2 Hz  oil and coolant temperature, fuel flow, volts
  0x302 EGT_1_4           2 Hz
  0x303 CHT_1_4           2 Hz
  0x304 CYL_5_6           2 Hz  six-cylinder engines only
  0x305 FUEL_ELECTRICAL   1 Hz  tanks, battery current, carburettor air
  0x306 ENGINE_TOTALS   every 5 s  engine time, fuel used, endurance
  0x704 HEARTBEAT         1 Hz

Engine 2 (--engine 2) is the same set at +0x20, heartbeat 0x705.

The engine is a simple model: RPM follows a flight - start, warm-up,
run-up, take-off, climb, cruise, descent, idle, and round again - or
stays at --rpm. Pressures follow RPM and oil temperature; temperatures
and fuel flow follow power, each with its own lag, so a cold start
warms up as a real one does. --profile picks the engine, and with it
the channels sent: a Rotax 912 has two EGT and two CHT probes and a
coolant temperature, so the rest go as "not available"; a Lycoming
O-320 four of each; an IO-540 six.

--fault puts in what the EFIS should warn about: low oil pressure, one
hot cylinder, a broken EGT probe, a dead alternator, little fuel.

    pip install python-can
    python3 tools/emu_emulator.py -i seeedstudio -c /dev/ttyUSB0
    python3 tools/emu_emulator.py -i seeedstudio -c /dev/ttyUSB0 \\
        --profile o320 --fault cht --fault alternator
    python3 tools/emu_emulator.py --rpm 5000            # socketcan can0
    python3 tools/emu_emulator.py --time-scale 10       # the flight 10x faster

Adapters as for tools/gps_emulator.py, whose notes apply here too - in
particular, one emulator per adapter: the USB-CAN Analyzer mixes up the
frames of two programs writing to it. To run the GPS as well, use a
second adapter.
"""

import argparse
import math
import random
import struct
import sys
import time

try:
    import can
except ImportError:
    sys.exit("python-can is needed: pip install python-can")

from gps_emulator import (NA_I16, NA_U16, SEEED_GAP_S, SEEED_SETTLE_S,
                          Sender, clamp_i16, clamp_u16)

DEVICE_TYPE_ENGINE = 4
FIRMWARE = (1 << 8) | 0             # 1.0

ID_ENGINE_BASE = 0x300
ID_ENGINE_STRIDE = 0x20
ID_HEARTBEAT_BASE = 0x700

FAST, SLOW, EGT_1_4, CHT_1_4, CYL_5_6, FUEL, TOTALS = range(7)

# Below this the engine is not running: no engine time, no charging.
RUN_RPM = 500

# What each engine has, and how it behaves. Temperatures deg C,
# pressures kPa, flows L/h, tanks L; "idle" and "full" are at idle and
# full power, in between by power.
PROFILES = {
    "rotax912": dict(
        name="Rotax 912 ULS", cylinders=4, egt_probes=2, cht_probes=2, coolant=True,
        carburettor=True, idle_rpm=1800, runup_rpm=4000, cruise_rpm=5000,
        max_rpm=5700, oil_temp=(70, 100), coolant_temp=(60, 95),
        cht=(80, 115), egt=(500, 780), oil_pressure=(200, 450),
        fuel_pressure=30, flow=(4, 27), tanks=(70, None)),
    "o320": dict(
        name="Lycoming O-320", cylinders=4, egt_probes=4, cht_probes=4, coolant=False,
        carburettor=True, idle_rpm=700, runup_rpm=1800, cruise_rpm=2400,
        max_rpm=2700, oil_temp=(65, 95), coolant_temp=None,
        cht=(110, 200), egt=(450, 730), oil_pressure=(180, 480),
        fuel_pressure=25, flow=(8, 40), tanks=(75, 75)),
    "io540": dict(
        name="Lycoming IO-540", cylinders=6, egt_probes=6, cht_probes=6, coolant=False,
        carburettor=False, idle_rpm=650, runup_rpm=1800, cruise_rpm=2450,
        max_rpm=2700, oil_temp=(65, 95), coolant_temp=None,
        cht=(110, 210), egt=(450, 740), oil_pressure=(180, 500),
        fuel_pressure=150, flow=(12, 95), tanks=(170, 170)),
}

# The flight: (phase, seconds, where RPM goes). Round again after the
# last, from the engine start.
FLIGHT = [
    ("off", 5, "off"),
    ("start", 3, "start"),
    ("warm-up", 120, "idle"),
    ("run-up", 30, "runup"),
    ("idle", 20, "idle"),
    ("take-off", 60, "max"),
    ("climb", 240, 0.92),
    ("cruise", 900, "cruise"),
    ("descent", 240, 0.6),
    ("idle", 60, "idle"),
    ("shutdown", 10, "off"),
]

FAULTS = {
    "oil-pressure": "oil pressure falling to a third",
    "cht": "cylinder 2 running 70 C hot",
    "egt-probe": "EGT 1 probe broken: not available",
    "alternator": "no charging: battery discharging, voltage falling",
    "fuel-low": "tanks at 8 % at start: the engine stops when they are dry",
}

# Lags, s: how fast each follows power.
TAU_EGT = 6.0
TAU_CHT = 90.0
TAU_OIL = 240.0
TAU_COOLANT = 150.0
TAU_RPM = 1.5

# Per cylinder spread, deg C, so they don't all read the same.
EGT_SPREAD = (0, 18, -12, 25, -20, 8)
CHT_SPREAD = (0, 9, -6, 12, -4, 5)


def lag(value, target, dt, tau):
    return value + (target - value) * min(1.0, dt / tau)


def between(pair, fraction):
    return pair[0] + (pair[1] - pair[0]) * fraction


class Engine:
    """The engine model, stepped through time."""

    def __init__(self, args):
        self.p = PROFILES[args.profile]
        self.args = args
        self.faults = set(args.fault or [])
        self.oat = args.oat

        self.rpm = 0.0
        self.egt = [self.oat] * 6
        self.cht = [self.oat] * 6
        self.oil_temp = self.oat
        self.coolant_temp = self.oat
        self.volts = 12.6
        self.current = 0.0
        self.flow = 0.0
        self.oil_factor = 1.0

        fill = 0.08 if "fuel-low" in self.faults else 1.0
        self.tanks = [None if size is None else size * fill
                      for size in self.p["tanks"]]
        self.fuel_used = 0.0
        self.engine_time = args.hobbs * 3600.0
        self.run_time = 0.0
        self.phase = "fixed"

    def target_rpm(self, t):
        """(phase, RPM wanted) at t seconds, from --rpm or the flight."""
        if self.args.rpm is not None:
            return "fixed", float(self.args.rpm)

        total = sum(step[1] for step in FLIGHT)
        # Round again from the start, not from the engine off.
        if t >= total:
            t = FLIGHT[0][1] + (t - total) % (total - FLIGHT[0][1])

        for phase, seconds, rpm in FLIGHT:
            if t < seconds:
                break
            t -= seconds

        p = self.p
        if rpm == "off":
            return phase, 0.0
        if rpm == "start":
            return phase, 250.0 if t < 1.5 else float(p["idle_rpm"])
        if rpm in ("idle", "runup", "cruise", "max"):
            return phase, float(p[rpm + "_rpm"])
        return phase, p["idle_rpm"] + (p["max_rpm"] - p["idle_rpm"]) * rpm

    def power(self):
        """0 at idle, 1 at full, by RPM."""
        p = self.p
        fraction = (self.rpm - p["idle_rpm"]) / (p["max_rpm"] - p["idle_rpm"])
        return max(0.0, min(1.0, fraction)) ** 1.5

    def running(self):
        return self.rpm >= RUN_RPM

    def step(self, t, dt):
        p = self.p
        self.phase, wanted = self.target_rpm(t)
        if self.fuel_remaining() <= 0.0:
            self.phase, wanted = "no fuel", 0.0

        # Stops faster than it spins up.
        self.rpm = lag(self.rpm, wanted, dt,
                       TAU_RPM if wanted > self.rpm else 0.7)
        if wanted == 0.0 and self.rpm < 50:
            self.rpm = 0.0

        running = self.running()
        power = self.power() if running else 0.0

        # Temperatures: towards their running values, or back to the
        # outside air with the engine off.
        for i in range(6):
            egt = (between(p["egt"], power) + EGT_SPREAD[i]) if running \
                else self.oat
            cht = (between(p["cht"], power) + CHT_SPREAD[i]) if running \
                else self.oat
            if i == 1 and "cht" in self.faults and running:
                cht += 70.0
            self.egt[i] = lag(self.egt[i], egt, dt, TAU_EGT)
            self.cht[i] = lag(self.cht[i], cht, dt, TAU_CHT)

        self.oil_temp = lag(self.oil_temp, between(p["oil_temp"], power)
                            if running else self.oat, dt, TAU_OIL)
        if p["coolant"]:
            self.coolant_temp = lag(
                self.coolant_temp, between(p["coolant_temp"], power)
                if running else self.oat, dt, TAU_COOLANT)

        self.flow = between(p["flow"], power) if running else 0.0
        burnt = self.flow * dt / 3600.0
        self.fuel_used += burnt
        self.draw_fuel(burnt)

        if running:
            self.engine_time += dt
            self.run_time += dt
        else:
            self.run_time = 0.0

        # Charging after the start, tapering off as the battery fills.
        if running and "alternator" not in self.faults:
            self.current = 1.0 + 12.0 * math.exp(-self.run_time / 60.0)
            self.volts = lag(self.volts, 14.2, dt, 3.0)
        elif running or self.rpm > 0:
            self.current = -8.0 if running else -120.0     # cranking
            self.volts = lag(self.volts, 11.9 if running else 10.5, dt,
                             60.0 if running else 0.5)
        else:
            self.current = -1.5
            self.volts = lag(self.volts, 12.5, dt, 5.0)

        if "oil-pressure" in self.faults and running:
            self.oil_factor = lag(self.oil_factor, 0.33, dt, 30.0)

    def draw_fuel(self, litres):
        """From the fuller tank, as a selector on BOTH roughly does."""
        tanks = [i for i, q in enumerate(self.tanks) if q is not None]
        if not tanks:
            return
        i = max(tanks, key=lambda k: self.tanks[k])
        self.tanks[i] = max(0.0, self.tanks[i] - litres)

    # ---- what is sent ------------------------------------------------------

    def noise(self, sigma):
        return random.gauss(0.0, sigma) if self.args.noise else 0.0

    def oil_pressure(self):
        if not self.running():
            return 0.0
        p = self.p
        # Up with RPM, down as the oil thins when it warms.
        fraction = ((self.rpm - p["idle_rpm"]) /
                    (p["cruise_rpm"] - p["idle_rpm"]))
        by_rpm = between(p["oil_pressure"], max(0.0, min(1.0, fraction)))
        thinning = 1.0 + 0.4 * max(0.0, (60.0 - self.oil_temp) / 45.0)
        return max(0.0, by_rpm * thinning * self.oil_factor)

    def manifold(self):
        """kPa: the throttle's vacuum, near ambient at full power."""
        if not self.running():
            return self.args.pressure
        return min(self.args.pressure,
                   28.0 + (self.args.pressure - 28.0) * self.power() ** 0.6)

    def fast_frame(self):
        fuel_pressure = (self.p["fuel_pressure"] if self.running()
                         else self.p["fuel_pressure"] * 0.8)  # boost pump
        return struct.pack(
            "<HHHH", clamp_u16(self.rpm + self.noise(8)),
            clamp_u16((self.manifold() + self.noise(0.2)) * 100),
            clamp_u16((self.oil_pressure() + self.noise(3)) * 10),
            clamp_u16((fuel_pressure + self.noise(0.5)) * 10))

    def slow_frame(self):
        coolant = (clamp_i16((self.coolant_temp + self.noise(0.2)) * 10)
                   if self.p["coolant"] else NA_I16)
        return struct.pack(
            "<hhHH", clamp_i16((self.oil_temp + self.noise(0.2)) * 10),
            coolant, clamp_u16(max(0.0, self.flow + self.noise(0.2)) * 100),
            clamp_u16((self.volts + self.noise(0.02)) * 100))

    def temperatures(self, values, first, count, fitted, broken=None):
        """Four i16 at 0.1 deg C, from cylinder first; not fitted or
        broken probes as not available."""
        out = []
        for i in range(first, first + count):
            if i >= fitted or i == broken:
                out.append(NA_I16)
            else:
                out.append(clamp_i16((values[i] + self.noise(1.5)) * 10))
        return out

    def egt_frame(self):
        broken = 0 if "egt-probe" in self.faults else None
        return struct.pack("<hhhh", *self.temperatures(
            self.egt, 0, 4, self.p["egt_probes"], broken))

    def cht_frame(self):
        return struct.pack("<hhhh", *self.temperatures(
            self.cht, 0, 4, self.p["cht_probes"]))

    def cylinders_5_6_frame(self):
        broken = 0 if "egt-probe" in self.faults else None
        return struct.pack(
            "<hhhh", *(self.temperatures(self.egt, 4, 2, self.p["egt_probes"], broken)
                       + self.temperatures(self.cht, 4, 2, self.p["cht_probes"])))

    def fuel_frame(self):
        tanks = [NA_U16 if q is None else clamp_u16(q * 10)
                 for q in self.tanks]
        # Carburettor air: a little under the outside air, cooled by the
        # fuel evaporating.
        carb = (clamp_i16((self.oat - 4.0 * self.power() + self.noise(0.2))
                          * 10) if self.p["carburettor"] else NA_I16)
        return struct.pack("<HHhh", tanks[0], tanks[1],
                           clamp_i16((self.current + self.noise(0.1)) * 100),
                           carb)

    def fuel_remaining(self):
        return sum(q for q in self.tanks if q is not None)

    def totals_frame(self):
        endurance = (clamp_u16(self.fuel_remaining() / self.flow * 60.0)
                     if self.flow > 0.5 else NA_U16)
        return struct.pack("<IHH", int(self.engine_time) & 0xFFFFFFFF,
                           clamp_u16(self.fuel_used * 10), endurance)

    def status(self):
        p = self.p
        cht = [c for i, c in enumerate(self.cht) if i < p["cht_probes"]]
        egt = [e for i, e in enumerate(self.egt) if i < p["egt_probes"]]
        return ("%-9s rpm %4.0f  map %5.1f kPa  oil %3.0f kPa %3.0f C  "
                "CHT %3.0f  EGT %3.0f  FF %4.1f L/h  %4.1f V %+5.1f A  "
                "fuel %5.1f L"
                % (self.phase, self.rpm, self.manifold(),
                   self.oil_pressure(), self.oil_temp, max(cht), max(egt),
                   self.flow, self.volts, self.current,
                   self.fuel_remaining()))


def heartbeat_frame(uptime_s, faults):
    state = 2 if faults else 1          # degraded with a broken probe & co
    return struct.pack("<BBHI", state, DEVICE_TYPE_ENGINE, FIRMWARE,
                       int(uptime_s) & 0xFFFFFFFF)


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Emulate the MiniEFIS CAN engine monitoring unit "
                    "(node 4, or 5 for engine 2).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        epilog="faults: " + "; ".join("%s - %s" % item
                                      for item in FAULTS.items()))

    bus = parser.add_argument_group("adapter")
    bus.add_argument("-i", "--interface", default="socketcan",
                     help="python-can interface: socketcan, seeedstudio, "
                          "slcan, gs_usb, pcan, ...")
    bus.add_argument("-c", "--channel", default="can0",
                     help="can0, /dev/ttyUSB0, /dev/ttyACM0, ...")
    bus.add_argument("-b", "--bitrate", type=int, default=500000,
                     help="bit rate (socketcan: set with ip link instead)")

    engine = parser.add_argument_group("engine")
    engine.add_argument("--engine", type=int, choices=(1, 2), default=1,
                        help="engine 1 (node 4, 0x300) or 2 (node 5, 0x320)")
    engine.add_argument("--profile", choices=PROFILES, default="rotax912")
    engine.add_argument("--rpm", type=float,
                        help="hold this RPM instead of flying the flight")
    engine.add_argument("--time-scale", type=float, default=1.0,
                        help="run the flight, and the engine's warming, "
                             "this many times faster")
    engine.add_argument("--fault", action="append", choices=FAULTS,
                        help="put in a fault; may be given more than once")
    engine.add_argument("--oat", type=float, default=15.0,
                        help="outside air temperature, deg C")
    engine.add_argument("--pressure", type=float, default=101.3,
                        help="outside air pressure, kPa: the manifold "
                             "pressure at full power and engine off")
    engine.add_argument("--hobbs", type=float, default=412.3,
                        help="engine time at the start, hours")
    engine.add_argument("--noise", action="store_true",
                        help="add a little sensor noise")
    engine.add_argument("--duration", type=float, default=0.0,
                        help="stop after this many seconds; 0 runs until "
                             "Ctrl-C")

    return parser.parse_args()


def main():
    args = parse_arguments()

    if args.time_scale <= 0:
        sys.exit("--time-scale must be positive")

    try:
        bus = can.Bus(interface=args.interface, channel=args.channel,
                      bitrate=args.bitrate)
    except (can.CanError, OSError, ValueError, ImportError) as error:
        sys.exit("could not open %s %s: %s"
                 % (args.interface, args.channel, error))

    if args.interface == "seeedstudio":
        time.sleep(SEEED_SETTLE_S)
        bus.init_frame()
        time.sleep(SEEED_SETTLE_S)

    sender = Sender(bus, SEEED_GAP_S if args.interface == "seeedstudio"
                    else 0.0)
    engine = Engine(args)
    node = 3 + args.engine
    base = ID_ENGINE_BASE + (args.engine - 1) * ID_ENGINE_STRIDE

    # (period s, frame): each sent on its own clock.
    schedule = [
        (0.1, lambda: sender.send(base + FAST, engine.fast_frame())),
        (0.5, lambda: sender.send(base + SLOW, engine.slow_frame())),
        (0.5, lambda: sender.send(base + EGT_1_4, engine.egt_frame())),
        (0.5, lambda: sender.send(base + CHT_1_4, engine.cht_frame())),
        (1.0, lambda: sender.send(base + FUEL, engine.fuel_frame())),
        (5.0, lambda: sender.send(base + TOTALS, engine.totals_frame())),
        (1.0, lambda: sender.send(ID_HEARTBEAT_BASE + node, heartbeat_frame(
            time.monotonic() - start, engine.faults))),
    ]
    if engine.p["cylinders"] > 4:
        schedule.append((0.5, lambda: sender.send(
            base + CYL_5_6, engine.cylinders_5_6_frame())))

    faults = ", ".join(sorted(engine.faults)) or "none"
    print("%s, engine %d (node %d, 0x%03X) on %s %s, faults: %s - "
          "Ctrl-C to stop"
          % (engine.p["name"], args.engine, node, base, args.interface,
             args.channel, faults))

    start = time.monotonic()
    due = [start] * len(schedule)
    next_print = start
    last = start

    try:
        while True:
            now = time.monotonic()
            if args.duration and now - start >= args.duration:
                break

            engine.step((now - start) * args.time_scale,
                         (now - last) * args.time_scale)
            last = now

            for k, (period, send) in enumerate(schedule):
                if now >= due[k]:
                    send()
                    due[k] += period
                    # Fell behind (the adapter blocked): don't burst.
                    if due[k] < now:
                        due[k] = now + period

            if now >= next_print:
                line = "%6.0f s  %s  sent %d" % (now - start, engine.status(),
                                                 sender.sent)
                if sender.failed:
                    line += ("  FAILED %d (%s) - EFIS connected and "
                             "running? termination?"
                             % (sender.failed, sender.last_error))
                print(line, flush=True)
                next_print += 1.0

            time.sleep(max(0.0, min(due) - time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()

    print("sent %d frames, %d failed" % (sender.sent, sender.failed))


if __name__ == "__main__":
    main()
