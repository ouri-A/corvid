#!/usr/bin/env python3
"""Spin test: where does a bot actually pivot, relative to its tag?

    uv run corvid/scripts/spin_test.py corvid/bags/spin_1_oct15
    uv run corvid/scripts/spin_test.py corvid/bags/spin_2_bot1 --marker 1 --scale 0.6985

Record a bag with /marker_detections, /botN/state and /botN/heading while the bot
sits still, turns in place (sent by hand on /botN/turn), sits still, turns again.
The tag is fixed to the bot, so between two still poses its centre has swung
round the real pivot. Each still pose gives the tag's position p and the bot's
heading t on the floor; with d the tag centre's offset from the pivot in the
bot's own frame (forward, left), every pose satisfies p = c + R(t) d for one
fixed pivot c. Two poses give p2 - p1 = (R(t2) - R(t1)) d, which is solvable
for d whenever the bot turned. All turns together are solved by least squares,
and each turn alone is printed too, so the spread shows how repeatable it is.

It also prints each turn as the gyro measured it (/botN/heading) against the
camera, which separates "the bot coasts past the gyro's stop" from "the gyro
under-reads".

The detector solves every tag at one size; --scale corrects a tag of another
real size (0.095 / 0.136 = 0.6985 for the small tags). The tag's top edge is
taken to face the bot's front, as mounted.

Stdlib only: Humble's rosbag2 does not store message definitions, so the CDR is
decoded by hand, as in analyse_bag.py. Light enough to run on the laptop.
"""
import argparse
import math
import sqlite3
import struct
import sys

SETTLE_S = 0.3        # ignore frames this close to a turn starting or ending
MIN_FRAMES = 5        # a still stretch needs at least this many detections
MIN_TURN_DEG = 30.0   # pairs that turned less than this are too ill-conditioned
DISAGREE_DEG = 10.0   # gyro and camera further apart than this: the gyro was not tracking
STATE_GAP_S = 1.0     # /botN/state silent this long: the bot dropped off or rebooted


class Cdr:
    """Minimal little-endian CDR reader; alignment is relative to the payload."""

    def __init__(self, buf):
        self.b, self.p = buf, 4

    def _align(self, n):
        off = (self.p - 4) % n
        if off:
            self.p += n - off

    def _read(self, fmt, size):
        self._align(size)
        v = struct.unpack_from('<' + fmt, self.b, self.p)[0]
        self.p += size
        return v

    def u32(self):
        return self._read('I', 4)

    def i32(self):
        return self._read('i', 4)

    def f64(self):
        return self._read('d', 8)

    def string(self):
        n = self.u32()
        s = self.b[self.p:self.p + n - 1].decode('utf-8', 'replace')
        self.p += n
        return s


def read_bag(path, marker_id, ns):
    path = path.rstrip('/')
    name = path.split('/')[-1]
    con = sqlite3.connect('file:%s/%s_0.db3?mode=ro' % (path, name), uri=True)
    topics = {r[0]: r[1] for r in con.execute('select name, id from topics')}
    for t in ('/marker_detections', ns + '/state', ns + '/heading'):
        if t not in topics:
            sys.exit('bag has no %s' % t)

    def rows(topic):
        return con.execute(
            'select timestamp, data from messages where topic_id=? order by timestamp',
            (topics[topic],))

    dets = []      # (image stamp s, (x, y, z), (qx, qy, qz, qw))
    frames = 0
    for _wall, blob in rows('/marker_detections'):
        frames += 1
        c = Cdr(blob)
        sec, nsec = c.i32(), c.u32()
        c.string()
        for _ in range(c.u32()):
            i = c.u32()
            pos = (c.f64(), c.f64(), c.f64())
            quat = (c.f64(), c.f64(), c.f64(), c.f64())
            if i == marker_id:
                dets.append((sec + nsec * 1e-9, pos, quat))
    state = [(w * 1e-9, blob[4]) for w, blob in rows(ns + '/state')]
    heading = [(w * 1e-9, struct.unpack_from('<f', blob, 4)[0])
               for w, blob in rows(ns + '/heading')]
    turns = [(w * 1e-9, struct.unpack_from('<f', blob, 4)[0])
             for w, blob in rows(ns + '/turn')] if ns + '/turn' in topics else []
    return frames, dets, state, heading, turns


# ---------- small vector helpers ----------

def rot(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def col(m, j):
    return (m[0][j], m[1][j], m[2][j])


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def unit(a):
    n = math.sqrt(dot(a, a))
    return tuple(x / n for x in a)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def median(v):
    s = sorted(v)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def still_stretches(state):
    """(start, end) receive times of each run of consecutive idle (0) states."""
    out, start, last = [], None, None
    for t, v in state:
        if v == 0:
            if start is None:
                start = t
            last = t
        elif start is not None:
            out.append((start, last))
            start = None
    if start is not None:
        out.append((start, last))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--marker', type=int, default=0)
    ap.add_argument('--bot', type=int, default=0)
    ap.add_argument('--scale', type=float, default=1.0,
                    help='real tag size / size the detector assumed')
    a = ap.parse_args()
    ns = '/bot%d' % a.bot

    frames, dets, state, heading, turns = read_bag(a.bag, a.marker, ns)
    print('%s: %d frames, %d with tag id %d, %d state, %d heading, %d turn messages'
          % (a.bag, frames, len(dets), a.marker, len(state), len(heading), len(turns)))
    if not dets:
        sys.exit('tag id %d never seen' % a.marker)
    if not turns:
        print('NOTE: no %s/turn messages in the bag, so the commanded angles are not '
              'measured here' % ns)

    # The floor plane, from the tag's own normal (its z axis), averaged. Floor
    # axes e1, e2 with the normal n make a right-handed set, so angles in the
    # plane are counter-clockwise seen from above, like the bot's heading.
    n = unit(tuple(sum(col(rot(q), 2)[k] for _, _, q in dets) for k in range(3)))
    e1 = unit(tuple(x - dot((1.0, 0.0, 0.0), n) * nk for x, nk in zip((1.0, 0.0, 0.0), n)))
    e2 = cross(n, e1)

    def floor(p):
        p = tuple(a.scale * x for x in p)
        return dot(p, e1), dot(p, e2)

    def yaw(q):
        f = col(rot(q), 1)          # tag +y = the bot's forward
        return math.atan2(dot(f, e2), dot(f, e1))

    poses = []
    for s0, s1 in still_stretches(state):
        lo, hi = s0 + SETTLE_S, s1 - SETTLE_S
        pick = [(p, q) for t, p, q in dets if lo <= t <= hi]
        hd = [h for t, h in heading if lo <= t <= hi]
        if len(pick) < MIN_FRAMES or not hd:
            continue
        xy = [floor(p) for p, _ in pick]
        ys = [yaw(q) for _, q in pick]
        cx, cy = sum(x for x, _ in xy) / len(xy), sum(y for _, y in xy) / len(xy)
        cyaw = math.atan2(sum(math.sin(y) for y in ys), sum(math.cos(y) for y in ys))
        spread = max(math.hypot(x - cx, y - cy) for x, y in xy)
        yspread = max(abs(wrap(y - cyaw)) for y in ys)
        poses.append(dict(t0=s0, t1=s1, n=len(pick), x=cx, y=cy, yaw=cyaw,
                          gyro=median(hd), spread=spread, yspread=yspread))

    print('\nstill stretches (tag centre on the floor, camera axes; yaw = bot heading):')
    for i, p in enumerate(poses):
        print('  %d  %6.1f-%6.1f s  %3d frames  tag (%+.4f, %+.4f) m  yaw %+7.2f deg  '
              'gyro %+8.2f deg  | wobble %.1f mm, %.2f deg'
              % (i, p['t0'] - poses[0]['t0'], p['t1'] - poses[0]['t0'], p['n'], p['x'],
                 p['y'], math.degrees(p['yaw']), math.degrees(p['gyro']),
                 p['spread'] * 1000, math.degrees(p['yspread'])))

    pairs = []
    for p1, p2 in zip(poses, poses[1:]):
        g = p2['gyro'] - p1['gyro']
        cam_wrapped = wrap(p2['yaw'] - p1['yaw'])
        gap = max((b - a for (a, _), (b, _) in zip(state, state[1:])
                   if p1['t1'] <= a and b <= p2['t0'] + 1e-6), default=0.0)
        # The camera sees heading only modulo 360, so the gyro is used to unwrap
        # it - but only when the gyro can be trusted for this turn: no reboot in
        # between (heading restarts at 0) and the two agree once wrapped.
        note = ''
        if gap > STATE_GAP_S:
            note = 'bot silent %.1f s in between (rebooted?): gyro angle meaningless' % gap
        elif abs(math.degrees(wrap(cam_wrapped - g))) > DISAGREE_DEG:
            note = 'gyro and camera disagree: gyro NOT tracking the real rotation'
        cam = cam_wrapped if note else g + wrap(cam_wrapped - g)
        if abs(math.degrees(cam_wrapped)) < MIN_TURN_DEG and not note:
            continue
        c1, s1 = math.cos(p1['yaw']), math.sin(p1['yaw'])
        c2, s2 = math.cos(p2['yaw']), math.sin(p2['yaw'])
        # (R(t2) - R(t1)) d = dp, with R(t) = [[c, -s], [s, c]]
        A = [[c2 - c1, -(s2 - s1)], [s2 - s1, c2 - c1]]
        b = [p2['x'] - p1['x'], p2['y'] - p1['y']]
        det_ = A[0][0] * A[1][1] - A[0][1] * A[1][0]
        d = ((A[1][1] * b[0] - A[0][1] * b[1]) / det_,
             (-A[1][0] * b[0] + A[0][0] * b[1]) / det_)
        pairs.append(dict(A=A, b=b, d=d, gyro=g, cam=cam, note=note))

    if not pairs:
        sys.exit('no turn of %.0f deg or more between two still stretches' % MIN_TURN_DEG)

    print('\nturns (camera = what the bot really turned; gyro = what it measured):')
    for i, p in enumerate(pairs):
        if p['note']:
            print('  %d  gyro %+8.2f deg  camera %+8.2f deg (mod 360)  ** %s **'
                  '  | offset alone: %+.1f mm forward, %+.1f mm left'
                  % (i, math.degrees(p['gyro']), math.degrees(p['cam']), p['note'],
                     p['d'][0] * 1000, p['d'][1] * 1000))
        else:
            print('  %d  gyro %+8.2f deg  camera %+8.2f deg  camera - gyro %+6.2f deg (%+.1f%%)'
                  '  | offset alone: %+.1f mm forward, %+.1f mm left'
                  % (i, math.degrees(p['gyro']), math.degrees(p['cam']),
                     math.degrees(p['cam'] - p['gyro']),
                     100 * (p['cam'] - p['gyro']) / p['gyro'],
                     p['d'][0] * 1000, p['d'][1] * 1000))
    print('  (the offset needs only the camera, so it uses every turn, flagged or not)')

    # Least squares over every turn: (sum A^T A) d = sum A^T b.
    m = [[0.0, 0.0], [0.0, 0.0]]
    v = [0.0, 0.0]
    for p in pairs:
        A, b = p['A'], p['b']
        for i in range(2):
            v[i] += A[0][i] * b[0] + A[1][i] * b[1]
            for j in range(2):
                m[i][j] += A[0][i] * A[0][j] + A[1][i] * A[1][j]
    det_ = m[0][0] * m[1][1] - m[0][1] * m[1][0]
    fwd = (m[1][1] * v[0] - m[0][1] * v[1]) / det_
    left = (-m[1][0] * v[0] + m[0][0] * v[1]) / det_

    # With that offset, where was the pivot in each still pose? If the bot turns
    # in place these all coincide; their scatter is how much it wanders.
    piv = []
    for p in poses:
        c, s = math.cos(p['yaw']), math.sin(p['yaw'])
        piv.append((p['x'] - (c * fwd - s * left), p['y'] - (s * fwd + c * left)))
    px = sum(x for x, _ in piv) / len(piv)
    py = sum(y for _, y in piv) / len(piv)
    print('\npivot in each still pose, from the fitted offset (should coincide):')
    for i, (x, y) in enumerate(piv):
        print('  %d  (%+.4f, %+.4f) m   %.1f mm from the mean' % (
            i, x, y, 1000 * math.hypot(x - px, y - py)))

    print('\nRESULT  tag centre relative to the pivot: %+.1f mm forward, %+.1f mm left'
          % (fwd * 1000, left * 1000))
    print('        ros2 run corvid_swarm coordinator --ros-args '
          '-p tag_offset_forward_m:=%.4f -p tag_offset_left_m:=%.4f' % (fwd, left))


if __name__ == '__main__':
    main()
