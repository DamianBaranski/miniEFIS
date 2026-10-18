#!/usr/bin/env python3
"""
A GPS on the CAN bus, for testing the MiniEFIS without one.

Sends what the GPS node (node 2) sends in docs/CAN_PROTOCOL.md, through
any USB-CAN adapter python-can drives:

  0x200 GPS_POSITION   } one burst per fix, at --rate (default 10 Hz),
  0x201 GPS_VELOCITY   } velocity and altitude carrying the same fix
  0x202 GPS_ALTITUDE   } sequence number
  0x203 GPS_QUALITY      1 Hz
  0x204 GPS_TIME         1 Hz, UTC from this computer's clock
  0x702 HEARTBEAT        1 Hz

The simulated aircraft flies a circle - or a straight line with
--radius 0 - at a steady ground speed, climbing at --climb, with
optional noise on the position. --gpx flies a GPX file's route instead,
leg after leg as the MiniEFIS's HSI expects: --offset puts it to one
side of each leg, --above over the path between the waypoints'
heights. --fix none emulates a GPS without a fix,
sending its fields as "not available". The debug page's CAN bus section
then shows each message arriving and its values.

    pip install python-can
    python3 tools/gps_emulator.py                         # socketcan can0
    python3 tools/gps_emulator.py -i seeedstudio -c /dev/ttyUSB0
    python3 tools/gps_emulator.py -i slcan -c /dev/ttyACM0
    python3 tools/gps_emulator.py -i gs_usb -c 0
    python3 tools/gps_emulator.py -i pcan -c PCAN_USBBUS1
    python3 tools/gps_emulator.py --radius 0 --track 270 --speed 40
    python3 tools/gps_emulator.py --fix none
    python3 tools/gps_emulator.py --gpx trip.gpx --speed 60 --offset 300

Adapters:
  socketcan  candleLight / gs_usb firmware, PEAK, Kvaser and most others
             on Linux appear as a network interface. Bring it up first,
             at the bus's bit rate:
                 sudo ip link set can0 up type can bitrate 500000
  seeedstudio  The "USB-CAN Analyzer" sold under many names: a CH340 USB
             serial chip (1a86:7523, /dev/ttyUSB0) in front of the CAN
             controller, spoken to at 2 Mbaud. It doesn't report back
             whether a frame got onto the bus, so sends never fail here
             even with nothing connected: check the EFIS's debug page.
             Run one instance per adapter: two writing to the same
             serial port interleave their bytes, and the adapter then
             sends frames with one message's ID on another's data, or
             loses its set-up altogether.
             It also misses the set-up python-can sends the moment the
             port opens, and keeps the bit rate it had before (800
             kbit/s seen): the EFIS then counts one bus error per frame
             and receives nothing. The script sends the set-up again
             after a pause. And frames written to it back to back come
             out with one's ID on another's data, so it waits 2 ms
             after each.
  slcan      CANable and other Lawicel-protocol adapters, a serial port.
  gs_usb     candleLight without the kernel driver (needs pyusb).
  pcan       PEAK, through PEAK's own library.

The bus needs a termination at each end: 120 ohm on the adapter (most
have a jumper or switch) and on the far end of the MiniEFIS's
transceiver. With only the adapter and the EFIS on the bus, the EFIS's
acknowledgements are what let frames through; if it is not connected or
not running, sends fail and the script says so.
"""

import argparse
import math
import random
import struct
import sys
import time
import xml.etree.ElementTree as ElementTree

try:
    import can
except ImportError:
    sys.exit("python-can is needed: pip install python-can")

NODE = 2
DEVICE_TYPE_GPS = 2
FIRMWARE = (1 << 8) | 0             # 1.0

# The pause either side of the USB-CAN Analyzer's second set-up frame.
SEEED_SETTLE_S = 0.5

# The pause after each frame to the USB-CAN Analyzer: written back to
# back, it puts one frame's ID on another's data.
SEEED_GAP_S = 0.002

ID_POSITION = 0x200
ID_VELOCITY = 0x201
ID_ALTITUDE = 0x202
ID_QUALITY = 0x203
ID_TIME = 0x204
ID_HEARTBEAT = 0x700 + NODE

FIX_TYPES = {"none": 0, "2d": 2, "3d": 3, "sbas": 4, "rtk": 5}

# "Not available", as the protocol has it.
NA_U8 = 0xFF
NA_U16 = 0xFFFF
NA_I16 = -0x8000
NA_I32 = -0x80000000

EARTH_RADIUS_M = 6371000.0


def clamp_u16(value):
    return max(0, min(0xFFFE, int(round(value))))


def clamp_i16(value):
    return max(-0x7FFF, min(0x7FFF, int(round(value))))


class Flight:
    """The simulated aircraft: where it is at time t."""

    def __init__(self, args):
        self.lat0 = args.lat
        self.lon0 = args.lon
        self.alt0 = args.alt
        self.speed = args.speed
        self.radius = args.radius
        self.track0 = args.track
        self.climb = args.climb
        self.noise = args.noise
        self.clockwise = not args.left

    def at(self, t):
        """(lat, lon, altitude, track, vertical speed) at t seconds."""
        if self.radius > 0:
            # Round a circle that starts here, heading track0.
            turn_rate = self.speed / self.radius          # rad/s
            sign = 1.0 if self.clockwise else -1.0
            track = math.radians(self.track0) + sign * turn_rate * t
            centre = math.radians(self.track0) + sign * math.pi / 2
            cn = self.radius * math.cos(centre)
            ce = self.radius * math.sin(centre)
            # From the centre back out to the aircraft.
            out = track - sign * math.pi / 2
            north = cn + self.radius * math.cos(out)
            east = ce + self.radius * math.sin(out)
        else:
            track = math.radians(self.track0)
            north = self.speed * t * math.cos(track)
            east = self.speed * t * math.sin(track)

        if self.noise > 0:
            north += random.gauss(0.0, self.noise)
            east += random.gauss(0.0, self.noise)

        lat = self.lat0 + math.degrees(north / EARTH_RADIUS_M)
        lon = self.lon0 + math.degrees(
            east / (EARTH_RADIUS_M * math.cos(math.radians(self.lat0))))
        alt = self.alt0 + self.climb * t

        return lat, lon, alt, math.degrees(track) % 360.0, self.climb


def destination(lat, lon, bearing, distance):
    """Where a great circle from (lat, lon) at bearing (deg) reaches
    after distance (m)."""
    p1, l1, b = math.radians(lat), math.radians(lon), math.radians(bearing)
    d = distance / EARTH_RADIUS_M
    p2 = math.asin(math.sin(p1) * math.cos(d) +
                   math.cos(p1) * math.sin(d) * math.cos(b))
    l2 = l1 + math.atan2(math.sin(b) * math.sin(d) * math.cos(p1),
                         math.cos(d) - math.sin(p1) * math.sin(p2))
    return math.degrees(p2), math.degrees(l2)


def distance_bearing(lat1, lon1, lat2, lon2):
    """Great circle distance (m) and initial bearing (deg)."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    a = (math.sin((p2 - p1) / 2) ** 2 +
         math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2)
    distance = 2 * EARTH_RADIUS_M * math.atan2(math.sqrt(a),
                                               math.sqrt(1 - a))
    bearing = math.degrees(math.atan2(
        math.sin(dl) * math.cos(p2),
        math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) *
        math.cos(dl)))
    return distance, bearing % 360.0


def read_gpx(path):
    """[(lat, lon, ele or None, name)]: the first route, else the
    waypoints, else the track - the order the MiniEFIS takes them in."""
    root = ElementTree.parse(path).getroot()

    def local(tag):
        return tag.rsplit("}", 1)[-1]

    def children(element, name):
        return [c for c in element if local(c.tag) == name]

    def all_named(name):
        return [e for e in root.iter() if local(e.tag) == name]

    routes = all_named("rte")
    points = (children(routes[0], "rtept") if routes else []) or \
        all_named("wpt") or all_named("trkpt")

    result = []
    for number, point in enumerate(points, 1):
        ele = children(point, "ele")
        name = children(point, "name")
        result.append((float(point.get("lat")), float(point.get("lon")),
                       float(ele[0].text) if ele else None,
                       name[0].text.strip() if name else "WP%d" % number))
    return result


class RouteFlight:
    """Flies a GPX route leg after leg, round again after the last."""

    def __init__(self, args):
        self.points = read_gpx(args.gpx)
        if len(self.points) < 2:
            sys.exit("%s: need at least two points" % args.gpx)

        self.speed = args.speed
        self.offset = args.offset
        self.above = args.above
        self.alt = args.alt
        self.noise = args.noise
        self.heights = all(p[2] is not None for p in self.points)

        # (start, length, bearing) of each leg, closed back to the start.
        self.legs = []
        for a, b in zip(self.points, self.points[1:] + self.points[:1]):
            length, bearing = distance_bearing(a[0], a[1], b[0], b[1])
            self.legs.append((a, b, length, bearing))
        self.total = sum(leg[2] for leg in self.legs)

        names = " - ".join(p[3] for p in self.points)
        print("flying %s, %.1f NM round" % (names, self.total / 1852.0))

    def at(self, t):
        """(lat, lon, altitude, track, vertical speed) at t seconds."""
        along = (self.speed * t) % self.total
        for a, b, length, bearing in self.legs:
            if along <= length:
                break
            along -= length

        lat, lon = destination(a[0], a[1], bearing, along)
        remaining, track = distance_bearing(lat, lon, b[0], b[1])
        if remaining < 1.0:
            track = bearing

        # To the right of the leg: square to the track there.
        side = self.offset + (random.gauss(0.0, self.noise)
                              if self.noise > 0 else 0.0)
        if side != 0.0:
            lat, lon = destination(lat, lon, track + 90.0, side)

        if self.heights:
            fraction = along / length if length > 0 else 1.0
            alt = a[2] + (b[2] - a[2]) * fraction + self.above
            climb = (b[2] - a[2]) / length * self.speed if length > 0 else 0.0
        else:
            alt, climb = self.alt, 0.0

        return lat, lon, alt, track, climb


class Sender:
    """Sends frames, counting and reporting the ones the bus refused."""

    def __init__(self, bus, gap_s):
        self.bus = bus
        self.gap_s = gap_s
        self.sent = 0
        self.failed = 0
        self.last_error = None

    def send(self, can_id, data):
        assert len(data) == 8, hex(can_id)
        message = can.Message(arbitration_id=can_id, data=data,
                              is_extended_id=False)
        try:
            self.bus.send(message, timeout=0.05)
            self.sent += 1
            if self.gap_s > 0:
                time.sleep(self.gap_s)
        except can.CanError as error:
            self.failed += 1
            self.last_error = str(error)


def position_frame(lat, lon, have_fix):
    if not have_fix:
        return struct.pack("<ii", NA_I32, NA_I32)
    return struct.pack("<ii", int(round(lat * 1e7)), int(round(lon * 1e7)))


def velocity_frame(speed, track, climb, sequence, fix_type, have_fix):
    if not have_fix:
        return struct.pack("<HHhBB", NA_U16, NA_U16, NA_I16, sequence,
                           fix_type)
    return struct.pack("<HHhBB", clamp_u16(speed * 100),
                       clamp_u16(track * 100) % 36000,
                       clamp_i16(climb * 100), sequence, fix_type)


def altitude_frame(alt, geoid, sequence, have_altitude):
    if not have_altitude:
        return struct.pack("<ihBB", NA_I32, NA_I16, sequence, NA_U8)
    return struct.pack("<ihBB", int(round(alt * 1000)),
                       clamp_i16(geoid * 10), sequence, NA_U8)


def quality_frame(args, have_fix):
    if not have_fix:
        return struct.pack("<BBHHH", args.satellites if args.satellites
                           else 0, NA_U8, NA_U16, NA_U16, NA_U16)
    return struct.pack("<BBHHH", args.satellites,
                       min(0xFE, int(round(args.hdop * 10))),
                       clamp_u16(args.hacc * 100),
                       clamp_u16(args.vacc * 100),
                       clamp_u16(0.3 * 100))


def time_frame(now):
    seconds = int(now)
    milliseconds = int((now - seconds) * 1000)
    flags = 0b111       # time valid, date valid, leap seconds known
    return struct.pack("<IHBB", seconds, milliseconds, flags, NA_U8)


def heartbeat_frame(uptime_s):
    return struct.pack("<BBHI", 1, DEVICE_TYPE_GPS, FIRMWARE,
                       int(uptime_s) & 0xFFFFFFFF)


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Emulate the MiniEFIS CAN GPS node (node 2).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    bus = parser.add_argument_group("adapter")
    bus.add_argument("-i", "--interface", default="socketcan",
                     help="python-can interface: socketcan, slcan, gs_usb, "
                          "pcan, kvaser, ...")
    bus.add_argument("-c", "--channel", default="can0",
                     help="can0, /dev/ttyACM0, PCAN_USBBUS1, ...")
    bus.add_argument("-b", "--bitrate", type=int, default=500000,
                     help="bit rate (socketcan: set with ip link instead)")

    flight = parser.add_argument_group("flight")
    flight.add_argument("--lat", type=float, default=52.1657,
                        help="start latitude, deg")
    flight.add_argument("--lon", type=float, default=20.9671,
                        help="start longitude, deg")
    flight.add_argument("--alt", type=float, default=300.0,
                        help="start altitude above MSL, m")
    flight.add_argument("--speed", type=float, default=33.0,
                        help="ground speed, m/s (33 m/s = 64 kt)")
    flight.add_argument("--track", type=float, default=90.0,
                        help="initial true track, deg")
    flight.add_argument("--radius", type=float, default=1000.0,
                        help="circle radius, m; 0 flies straight")
    flight.add_argument("--left", action="store_true",
                        help="circle to the left (default right)")
    flight.add_argument("--climb", type=float, default=0.0,
                        help="vertical speed, m/s, up positive")
    flight.add_argument("--noise", type=float, default=0.0,
                        help="position noise, m (1 sigma)")
    flight.add_argument("--gpx", metavar="FILE",
                        help="fly this GPX file's route (start at its "
                             "first point) instead of a circle")
    flight.add_argument("--offset", type=float, default=0.0,
                        help="with --gpx: this far right of each leg, m "
                             "(negative left)")
    flight.add_argument("--above", type=float, default=0.0,
                        help="with --gpx: this far above the path between "
                             "the waypoints' heights, m (if every one has "
                             "<ele>; --alt otherwise)")

    gps = parser.add_argument_group("receiver")
    gps.add_argument("--rate", type=float, default=10.0,
                     help="fixes per second (protocol: 5-10)")
    gps.add_argument("--fix", choices=FIX_TYPES, default="3d",
                     help="fix type to report")
    gps.add_argument("--satellites", type=int, default=12)
    gps.add_argument("--hdop", type=float, default=0.8)
    gps.add_argument("--hacc", type=float, default=1.5,
                     help="horizontal accuracy, m")
    gps.add_argument("--vacc", type=float, default=2.5,
                     help="vertical accuracy, m")
    gps.add_argument("--geoid", type=float, default=33.0,
                     help="geoid separation, m")
    gps.add_argument("--duration", type=float, default=0.0,
                     help="stop after this many seconds; 0 runs until Ctrl-C")

    return parser.parse_args()


def main():
    args = parse_arguments()

    if args.rate <= 0:
        sys.exit("--rate must be positive")

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

    flight = RouteFlight(args) if args.gpx else Flight(args)
    sender = Sender(bus, SEEED_GAP_S if args.interface == "seeedstudio"
                    else 0.0)
    fix_type = FIX_TYPES[args.fix]
    have_fix = fix_type >= 2
    have_altitude = fix_type >= 3

    print("GPS node %d on %s %s, %g fixes/s, fix %s - Ctrl-C to stop"
          % (NODE, args.interface, args.channel, args.rate, args.fix))

    start = time.monotonic()
    period = 1.0 / args.rate
    next_fix = start
    next_second = start
    sequence = 0

    try:
        while True:
            now = time.monotonic()
            t = now - start

            if args.duration and t >= args.duration:
                break

            if now >= next_fix:
                lat, lon, alt, track, climb = flight.at(t)

                # The burst, in the order the protocol sets.
                sender.send(ID_POSITION, position_frame(lat, lon, have_fix))
                sender.send(ID_VELOCITY, velocity_frame(
                    args.speed, track, climb, sequence, fix_type, have_fix))
                sender.send(ID_ALTITUDE, altitude_frame(
                    alt, args.geoid, sequence, have_altitude))

                sequence = (sequence + 1) & 0xFF
                next_fix += period

                # Fell behind (the adapter blocked): don't burst to catch up.
                if next_fix < now:
                    next_fix = now + period

            if now >= next_second:
                sender.send(ID_QUALITY, quality_frame(args, have_fix))
                sender.send(ID_TIME, time_frame(time.time()))
                sender.send(ID_HEARTBEAT, heartbeat_frame(t))
                next_second += 1.0

                lat, lon, alt, track, _ = flight.at(t)
                line = ("%6.0f s  %.6f %.6f  alt %6.1f m  track %5.1f  "
                        "gs %.1f m/s  sent %d"
                        % (t, lat, lon, alt, track, args.speed, sender.sent))
                if sender.failed:
                    line += ("  FAILED %d (%s) - EFIS connected and "
                             "running? termination?"
                             % (sender.failed, sender.last_error))
                print(line, flush=True)

            time.sleep(max(0.0, min(next_fix, next_second)
                           - time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()

    print("sent %d frames, %d failed" % (sender.sent, sender.failed))


if __name__ == "__main__":
    main()
