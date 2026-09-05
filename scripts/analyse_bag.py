#!/usr/bin/env python3
"""Analyse a corvid pose_check bag: detection health, image staleness, pose accuracy.

Runs anywhere with a stdlib Python 3 — no ROS, no deps:

    uv run corvid/scripts/analyse_bag.py corvid/bags/pose_check_6

Humble's rosbag2 does not store message definitions, so both messages are decoded
from raw CDR by hand. The VehicleLocalPosition field offsets were found empirically
and are recorded in PROJECT_STATE.md GOTCHAS; they are correct for the px4_msgs
version in the workshop image and will silently produce nonsense against another.

Everything that compares the two topics matches them in SIM time. Arrival-time
matching is meaningless here — it produced poses claiming 2.6 m when PX4 said 0.5 m.
"""
import bisect
import sqlite3
import statistics as st
import struct
import sys

# --- VehicleLocalPosition, empirical CDR offsets. Blob length 220. ---------------
# 4-byte CDR header, then timestamp u64 @4, timestamp_sample @12, four bools @20,
# x @24, y @28, z @32, dist_bottom @164. Alignment is relative to payload start (4).
OFF_TS_SAMPLE, OFF_X, OFF_Y, OFF_Z, OFF_DIST = 12, 24, 28, 32, 164

HOVER_LO, HOVER_HI = 2.45, 2.75   # steady-hover altitude band, metres
MATCH_TOL = 0.05                  # max sim-time gap when pairing messages, seconds


class Cdr:
    """Minimal little-endian CDR reader."""

    def __init__(self, buf):
        self.b, self.p = buf, 4

    def _align(self, n):
        off = (self.p - 4) % n
        if off:
            self.p += n - off

    def u32(self):
        self._align(4)
        v = struct.unpack_from('<I', self.b, self.p)[0]
        self.p += 4
        return v

    def i32(self):
        self._align(4)
        v = struct.unpack_from('<i', self.b, self.p)[0]
        self.p += 4
        return v

    def f64(self):
        self._align(8)
        v = struct.unpack_from('<d', self.b, self.p)[0]
        self.p += 8
        return v

    def string(self):
        n = self.u32()
        s = self.b[self.p:self.p + n - 1].decode('utf-8', 'replace')
        self.p += n
        return s


def read_bag(path):
    """-> (detections, vlp). Both lists carry sim time AND bag receive (wall) time."""
    path = path.rstrip('/')
    name = path.split('/')[-1]
    con = sqlite3.connect(f"{path}/{name}_0.db3")
    topics = {r[0]: r[1] for r in con.execute("select name, id from topics")}

    def rows(topic):
        return con.execute(
            "select timestamp, data from messages where topic_id=? order by timestamp",
            (topics[topic],))

    dets = []
    for wall, blob in rows('/marker_detections'):
        r = Cdr(blob)
        sim = r.i32() + r.u32() * 1e-9        # header.stamp, image acquisition time
        frame = r.string()
        markers = []
        for _ in range(r.u32()):
            mid = r.u32()
            x, y, z = r.f64(), r.f64(), r.f64()
            r.f64(), r.f64(), r.f64(), r.f64()   # quaternion, unused here
            markers.append((mid, x, y, z))
        dets.append({'sim': sim, 'wall': wall * 1e-9, 'frame': frame, 'markers': markers})

    vlp = []
    for wall, blob in rows('/fmu/out/vehicle_local_position'):
        vlp.append({
            'sim': struct.unpack_from('<Q', blob, OFF_TS_SAMPLE)[0] * 1e-6,
            'wall': wall * 1e-9,
            'x': struct.unpack_from('<f', blob, OFF_X)[0],
            'y': struct.unpack_from('<f', blob, OFF_Y)[0],
            'z': struct.unpack_from('<f', blob, OFF_Z)[0],
            'dist': struct.unpack_from('<f', blob, OFF_DIST)[0],
        })
    return dets, vlp


def nearest(seq, key, value, tol):
    """Nearest entry in seq by seq[i][key], or None if further away than tol."""
    keys = [s[key] for s in seq]
    i = bisect.bisect_left(keys, value)
    cands = [j for j in (i - 1, i) if 0 <= j < len(seq)]
    if not cands:
        return None
    j = min(cands, key=lambda j: abs(seq[j][key] - value))
    return seq[j] if abs(seq[j][key] - value) <= tol else None


def summary(dets, vlp):
    span = dets[-1]['sim'] - dets[0]['sim']
    hits = [d for d in dets if d['markers']]
    print(f"detection msgs {len(dets)}   vlp msgs {len(vlp)}")
    print(f"frame_id(s): {sorted({d['frame'] for d in dets})}")
    print(f"sim span {dets[0]['sim']:.2f} -> {dets[-1]['sim']:.2f}  ({span:.1f} s)")
    print(f"with a detection {len(hits)}   empty {len(dets) - len(hits)}")
    print(f"publish {len(dets)/span:.2f} Hz   detection {len(hits)/span:.2f} Hz")


def staleness(dets, vlp):
    """How old is a frame by the time its detection is published?

    Do NOT calibrate sim->wall with a single offset: sim time does not run at
    real-time factor 1.0, so a constant offset drifts and yields garbage. PX4
    telemetry has negligible lag, so the PX4 message recorded nearest in WALL time
    tells us what sim time it actually was when the detection landed.
    """
    print("\nSTALENESS  (sim_now - image stamp)")
    samples = []
    for d in dets:
        v = nearest(vlp, 'wall', d['wall'], 0.1)
        if v:
            samples.append(v['sim'] - d['sim'])
    if not samples:
        print("  no overlapping samples")
        return
    s = sorted(samples)
    print(f"  median {st.median(s)*1000:7.1f} ms   "
          f"p05 {s[len(s)//20]*1000:7.1f}   p95 {s[len(s)*19//20]*1000:7.1f} ms")
    q = len(samples) // 4
    for k in range(4):
        seg = samples[k*q:(k+1)*q]
        if seg:
            print(f"  quarter {k+1}: {st.median(seg)*1000:7.1f} ms")
    print("  Flat is healthy (~one frame, 80 ms). Growth quarter on quarter is a")
    print("  queue — that is the parameter_bridge bug, see the post-mortem.")


def pose(dets, vlp):
    """Vision z against the rangefinder, matched in sim time."""
    rows = []
    for d in dets:
        if not d['markers']:
            continue
        v = nearest(vlp, 'sim', d['sim'], MATCH_TOL)
        if not v:
            continue
        _, x, y, z = d['markers'][0]
        rows.append({'sim': d['sim'], 'z': z, 'dist': v['dist'],
                     'lateral': (x*x + y*y) ** 0.5})
    if not rows:
        print("\nPOSE  no matched detections")
        return
    err = [r['z'] - r['dist'] for r in rows]
    print(f"\nPOSE vs rangefinder, sim-matched, {len(rows)} pairs")
    print(f"  pose_z - dist_bottom: median {st.median(err)*100:+.1f} cm   "
          f"mean {st.mean(err)*100:+.1f} cm   stdev {st.pstdev(err)*100:.1f} cm")
    print(f"  altitude {min(r['dist'] for r in rows):.2f} - "
          f"{max(r['dist'] for r in rows):.2f} m")

    print("\n  by altitude band:")
    for lo, hi in [(1.0, 2.0), (2.0, 2.5), (2.5, 3.0), (3.0, 6.0)]:
        seg = [r for r in rows if lo <= r['dist'] < hi]
        if len(seg) < 5:
            continue
        e = [r['z'] - r['dist'] for r in seg]
        print(f"   {lo:4.1f}-{hi:4.1f} m  n={len(seg):4d}  "
              f"median {st.median(e)*100:+7.1f} cm  stdev {st.pstdev(e)*100:5.1f}")

    # The histogram is the point. A median over a bimodal distribution describes
    # none of the data, and that is exactly how the "-7.4 cm camera offset" was
    # believed for two days. See the-offset-was-never-an-offset.md.
    hov = [r['z'] - r['dist'] for r in rows if HOVER_LO <= r['dist'] <= HOVER_HI]
    if len(hov) >= 20:
        print(f"\n  HOVER-BAND HISTOGRAM ({len(hov)} samples), cm")
        print("  Two separated humps = planar pose ambiguity, not a fixed offset.")
        cm = [e * 100 for e in hov]
        lo = int(min(cm) // 10 * 10)
        hi = int(max(cm) // 10 * 10) + 10
        for k in range(lo, hi, 10):
            n = sum(1 for v in cm if k <= v < k + 10)
            bar = '#' * int(n / len(cm) * 120)
            print(f"   {k:+5d}..{k+10:+5d} {bar:<48} {n}")


def timeline(dets, vlp):
    print("\nTIMELINE, 10 s buckets")
    print(f"  {'window':>16} {'msgs':>5} {'hits':>5} {'rate':>6} {'altitude':>14}")
    t0, t1 = dets[0]['sim'], dets[-1]['sim']
    t = t0
    while t < t1:
        seg = [d for d in dets if t <= d['sim'] < t + 10]
        if seg:
            alts = [v['dist'] for v in vlp if t <= v['sim'] < t + 10]
            hits = sum(1 for d in seg if d['markers'])
            band = f"{min(alts):.2f}-{max(alts):.2f}" if alts else "-"
            print(f"  {t-t0:6.1f}-{t+10-t0:6.1f}s {len(seg):5d} {hits:5d} "
                  f"{hits/len(seg)*100:5.0f}% {band:>14}")
        t += 10


def stability(vlp):
    """Was the vehicle actually steady? Kills 'the drone was tilting' theories."""
    h = [v for v in vlp if HOVER_LO <= v['dist'] <= HOVER_HI]
    if len(h) < 50:
        print("\nHOVER STABILITY  too few samples")
        return
    xs = [v['x'] for v in h]
    ys = [v['y'] for v in h]
    al = [v['dist'] for v in h]
    print(f"\nHOVER STABILITY  {len(h)/100:.1f} s in band")
    print(f"  x drift {max(xs)-min(xs):.3f} m   y drift {max(ys)-min(ys):.3f} m")
    print(f"  x std {st.pstdev(xs):.3f}   y std {st.pstdev(ys):.3f}   "
          f"alt std {st.pstdev(al):.3f}")


def main():
    if len(sys.argv) != 2:
        sys.exit(f"usage: {sys.argv[0]} <path/to/bag_dir>")
    path = sys.argv[1]
    dets, vlp = read_bag(path)
    if not dets or not vlp:
        sys.exit("bag is missing one of the two topics")
    print(f"=== {path.rstrip('/').split('/')[-1]} ===")
    summary(dets, vlp)
    staleness(dets, vlp)
    pose(dets, vlp)
    timeline(dets, vlp)
    stability(vlp)


if __name__ == '__main__':
    main()
