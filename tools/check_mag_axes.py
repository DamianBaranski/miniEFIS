#!/usr/bin/env python3
"""
Is the magnetometer really mounted the same way round as the IMU?

The firmware assumes it is (main/mounting.h): the magnetometer's
reported x, y, z are taken to point the same way as the IMU's. This
checks that against the gyro, which knows how the unit actually turned.

While the unit rotates, the earth's field stays put, so in the sensor's
own axes it turns the opposite way:

    dm/dt = -w x m

with w the gyro's rate. Each of the 24 ways the magnetometer could sit
relative to the IMU makes the readings turn some way; only the right
one makes them turn the way the gyro says. Rotation cannot tell a
mounting from the same one with every axis reversed, so the pair is
told apart by the dip: north of the equator the field points down, so
its component along gravity - which the accelerometer gives - is
positive.

On the unit: open the Diagnostics screen (it streams $DIAG). Then

    python3 tools/check_mag_axes.py --url http://192.168.0.124

hold it still for the first three seconds, then turn it slowly about
every axis - roll, pitch and yaw, each both ways - for the rest of the
recording, away from steel.

    python3 tools/check_mag_axes.py --self-test     # check the maths
"""

import argparse
import itertools
import math
import random
import sys
import time

AXES = "xyz"


def rotations():
    """All 48 signed permutations, as (matrix, determinant)."""
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            m = [[0] * 3 for _ in range(3)]
            for row in range(3):
                m[row][perm[row]] = signs[row]
            det = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                   - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                   + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
            yield m, det


def describe(m):
    """Which magnetometer axis lands on each IMU axis, e.g. 'x=+x y=-y'."""
    parts = []
    for row in range(3):
        for col in range(3):
            if m[row][col]:
                sign = "+" if m[row][col] > 0 else "-"
                parts.append(f"imu {AXES[row]} = {sign}mag {AXES[col]}")
    return ", ".join(parts)


def apply(m, v):
    return [sum(m[r][c] * v[c] for c in range(3)) for r in range(3)]


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def score(samples, m, min_rate_dps=15.0):
    """
    Relative misfit of dm/dt = -w x m over the moving parts of the
    recording, for the mounting m. Each sample is (t_s, gyro_dps[3] in
    IMU sensor axes, bias removed, field[3] in magnetometer axes, hard
    iron removed, accel_g[3] in IMU sensor axes). 0 is a perfect fit; 1
    is no better than predicting no change at all.
    """
    residual = 0.0
    total = 0.0

    for (t0, w0, m0, _), (t1, w1, m1, _) in zip(samples, samples[1:]):
        dt = t1 - t0
        if dt <= 0 or dt > 0.2:
            continue

        w = [math.radians(0.5 * (a + b)) for a, b in zip(w0, w1)]
        if math.degrees(math.sqrt(sum(c * c for c in w))) < min_rate_dps:
            continue

        a = apply(m, m0)
        b = apply(m, m1)
        mid = [0.5 * (p + q) for p, q in zip(a, b)]
        predicted = [-c * dt for c in cross(w, mid)]
        measured = [q - p for p, q in zip(a, b)]

        residual += sum((p - q) ** 2 for p, q in zip(predicted, measured))
        total += sum(q * q for q in measured)

    return residual / total if total > 0 else float("nan")


def downward(samples, m):
    """Mean component of the field along gravity, for mounting m: the
    accelerometer reads +1 g on whichever axis points up, as any
    accelerometer does, so down is against it."""
    total = 0.0
    for _, _, field, accel in samples:
        norm = math.sqrt(sum(c * c for c in accel)) or 1.0
        total -= sum(f * a for f, a in zip(apply(m, field), accel)) / norm
    return total / len(samples)


def analyse(samples):
    results = sorted(((score(samples, m), m, det) for m, det in rotations()),
                     key=lambda r: r[0])

    # Each fit ties with its exact negative; keep the one whose field
    # points down, as it does north of the equator.
    kept = []
    for misfit, m, det in results:
        negative = [[-c for c in row] for row in m]
        if any(k[1] in (m, negative) for k in kept):
            continue
        if downward(samples, m) < 0:
            m, det = negative, -det
        kept.append((misfit, m, det))
    results = kept
    identity = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

    print("\nbest fits (misfit: 0 perfect, 1 no fit at all):")
    for misfit, m, det in results[:5]:
        kind = "rotation" if det > 0 else "MIRROR"
        tag = "   <- what the firmware assumes" if m == identity else ""
        print(f"  {misfit:6.3f}  {kind:8s}  {describe(m)}{tag}")

    rank = next((i for i, r in enumerate(results) if r[1] == identity),
                None)
    best, runner = results[0][0], results[1][0]

    print()
    if not math.isfinite(best):
        print("no movement recorded - turn the unit while it records")
        return 2
    if results[0][1] == identity and runner > 2.0 * best:
        print("the magnetometer IS mounted like the IMU")
        return 0
    if results[0][1] == identity:
        print("the firmware's assumption fits best, but not clearly - "
              "turn the unit about every axis and try again")
        return 1
    where = (f"ranks {rank + 1} of {len(results)}" if rank is not None
             else "has the field pointing up")
    print(f"the magnetometer is NOT mounted like the IMU (the assumption "
          f"{where}); the best fit is:\n  {describe(results[0][1])}")
    return 1


def record(url, seconds):
    sys.path.insert(0, __file__.rsplit("/", 1)[0])
    import sensor_plot

    stream = sensor_plot.Stream(sensor_plot.HttpLines(url), 100000, None)
    print(f"recording {seconds:.0f} s: hold still for 3 s, then turn the "
          f"unit about every axis")

    start = time.time()
    while time.time() - start < seconds:
        time.sleep(1.0)
        print(f"  {time.time() - start:4.0f} s, {stream.received} samples",
              end="\r", flush=True)
    print()

    rows = stream.snapshot("DIAG")
    if len(rows) < 100:
        sys.exit("too few $DIAG samples - is the Diagnostics screen open?")

    # Gyro bias: the still start.
    t_first = rows[0]["t_ms"]
    still = [r for r in rows if r["t_ms"] - t_first < 2500]
    bias = [sum(r[k] for r in still) / len(still)
            for k in ("gx_dps", "gy_dps", "gz_dps")]

    # Hard iron: the middle of what was seen on each magnetometer axis.
    keys = ("mx_uT", "my_uT", "mz_uT")
    centre = [0.5 * (max(r[k] for r in rows) + min(r[k] for r in rows))
              for k in keys]

    return [(r["t_ms"] / 1000.0,
             [r["gx_dps"] - bias[0], r["gy_dps"] - bias[1],
              r["gz_dps"] - bias[2]],
             [r[k] - c for k, c in zip(keys, centre)],
             [r["ax_g"], r["ay_g"], r["az_g"]])
            for r in rows
            if all(math.isfinite(r[k]) for k in keys + ("gx_dps", "ax_g"))]


def self_test():
    """Simulated rotation with a known mounting: must find it."""
    random.seed(1)
    true = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]     # y and z reversed
    true_t = [list(col) for col in zip(*true)]     # IMU -> magnetometer

    field = [20.0, 0.0, 44.0]       # in the IMU frame at the start
    samples = []
    t = 0.0
    m_imu = field[:]
    g_imu = [0.0, 0.0, -1.0]        # +1 g up, as the accelerometer reads it
    for step in range(2000):
        phase = step * 0.01
        w = [60 * math.sin(phase * 0.7), 50 * math.sin(phase * 1.1 + 1),
             70 * math.sin(phase * 0.5 + 2)]
        wr = [math.radians(c) for c in w]
        m_imu = [a - b * 0.01 for a, b in zip(m_imu, cross(wr, m_imu))]
        g_imu = [a - b * 0.01 for a, b in zip(g_imu, cross(wr, g_imu))]
        t += 0.01
        if step % 5 == 0:
            m_mag = apply(true_t, m_imu)
            m_mag = [c + random.gauss(0, 0.3) for c in m_mag]
            gyro = [c + random.gauss(0, 0.5) for c in w]
            samples.append((t, gyro, m_mag, g_imu[:]))

    print("self test: magnetometer y and z reversed against the IMU")
    ok = analyse(samples) == 1 and \
        min(rotations(), key=lambda r: (score(samples, r[0]),
                                        -downward(samples, r[0])))[0] == true
    print("\nself test", "PASSED" if ok else "FAILED")
    return 0 if ok else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("--url", help="the unit's Wi-Fi debug address")
    parser.add_argument("--seconds", type=float, default=40.0)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    if args.self_test:
        sys.exit(self_test())
    if not args.url:
        parser.error("--url is needed (or --self-test)")

    sys.exit(analyse(record(args.url, args.seconds)))


if __name__ == "__main__":
    main()
