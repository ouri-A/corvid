"""Offline test of corvid_swarm/coordinator.py: fake ROS, a simulated bot and a tilted overhead camera.

No ROS needed. Fakes just enough of rclpy and the message types to run the real node
against a simulated bot (turns with a 4% error, drifts while driving, 5 s turn timeout)
and a camera 1.7 m up tilted 26 deg like the real mount, 8 fps, 0.1 s latency.

    uv run --with numpy corvid/scripts/coordinator_offline_sim.py

Each scenario prints the node's own log, how the run ended, and the TRUE distance
from the simulated pivot to the goal.
"""
import math, sys, types, importlib
import numpy as np
from types import SimpleNamespace as NS

T = [0.0]  # sim clock

def mod(name):
    m = types.ModuleType(name); sys.modules[name] = m; return m

# ---- fake ROS modules ----
rclpy = mod('rclpy'); rclpy.init=lambda **k: None; rclpy.spin=lambda n: None; rclpy.try_shutdown=lambda: None
node_mod = mod('rclpy.node'); qos = mod('rclpy.qos')
class QoSProfile:
    def __init__(self, **k): self.k = k
qos.QoSProfile = QoSProfile
qos.HistoryPolicy = NS(KEEP_LAST=1); qos.ReliabilityPolicy = NS(BEST_EFFORT=2); qos.DurabilityPolicy = NS(VOLATILE=2)

class Pub:
    def __init__(self, topic): self.topic, self.msgs = topic, []
    def publish(self, m): self.msgs.append((T[0], m))
class Clock:
    def now(self):
        return NS(nanoseconds=int(T[0]*1e9), to_msg=lambda: NS(sec=int(T[0]), nanosec=int((T[0]%1)*1e9)))
LOG = []
class Logger:
    def info(self, s): LOG.append((T[0], 'I', s))
    def warn(self, s): LOG.append((T[0], 'W', s))
OVERRIDES = {}
class Node:
    def __init__(self, name): self.pubs, self.subs, self.srvs, self.timers = {}, {}, {}, []
    def declare_parameter(self, n, d): return NS(value=OVERRIDES.get(n, d))
    def create_publisher(self, t, topic, q): p = Pub(topic); self.pubs[topic] = p; return p
    def create_subscription(self, t, topic, cb, q): self.subs[topic] = cb
    def create_service(self, t, name, cb): self.srvs[name] = cb
    def create_timer(self, p, cb): self.timers.append(cb)
    def get_clock(self): return Clock()
    def get_logger(self): return Logger()
    def destroy_node(self): pass
node_mod.Node = Node

def ns_factory(**fields):
    return NS(**fields)
std = mod('std_msgs'); stdm = mod('std_msgs.msg')
stdm.Float32 = lambda data=0.0: NS(data=data); stdm.UInt8 = lambda data=0: NS(data=data)
mod('std_srvs'); srv = mod('std_srvs.srv'); srv.Trigger = object
mod('corvid_msgs'); cm = mod('corvid_msgs.msg')
class CoordinatorStatus:
    PHASE_DISABLED=0; PHASE_MEASURING=1; PHASE_TURNING=2; PHASE_DRIVING=3
    PHASE_WAITING_FOR_TAGS=4; PHASE_STOPPING=5; PHASE_ARRIVED=6; PHASE_GAVE_UP=7
    def __init__(self):
        self.header = NS(stamp=NS(sec=0, nanosec=0), frame_id='')
        self.bot_pose = NS(position=NS(x=0,y=0,z=0), orientation=NS(x=0,y=0,z=0,w=1))
        self.target = NS(x=0,y=0,z=0); self.goal = NS(x=0,y=0,z=0)
cm.CoordinatorStatus = CoordinatorStatus; cm.MarkerDetectionArray = object

import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'corvid_swarm', 'corvid_swarm'))
import coordinator as C

# ---- world: camera 1.2 m up looking straight down ----
TILT = math.radians(26.0)   # the real VX-500 mount is tilted ~26 deg (spin_1_oct15)
R0 = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1.0]])
RX = np.array([[1, 0, 0], [0, math.cos(TILT), -math.sin(TILT)], [0, math.sin(TILT), math.cos(TILT)]])
R_CW = RX @ R0
CAM = np.array([0.2, -0.9, 1.7])
def to_cam(v): return R_CW @ np.asarray(v, dtype=float)
def tag_rot_cam(heading, bot):
    if bot:   # top edge (tag y) = bot forward
        ty = np.array([math.cos(heading), math.sin(heading), 0.0])
        tx = np.array([math.sin(heading), -math.cos(heading), 0.0])
    else:     # target: tag x along 'heading'
        tx = np.array([math.cos(heading), math.sin(heading), 0.0])
        ty = np.array([-math.sin(heading), math.cos(heading), 0.0])
    tz = np.array([0, 0, 1.0])
    return np.column_stack([to_cam(tx), to_cam(ty), to_cam(tz)])
def det(i, pos_w, R, scale_report):
    p = R_CW @ (np.asarray(pos_w, dtype=float) - CAM)
    p = p * scale_report
    q = C.quaternion_from_rotation(R)
    return NS(id=i, pose=NS(position=NS(x=p[0], y=p[1], z=p[2]),
                            orientation=NS(x=q[0], y=q[1], z=q[2], w=q[3])))

class Bot:
    """Pivot at (x,y), heading; tag 3 cm ahead of the pivot, 9 cm up."""
    def __init__(s, x, y, h, turn_err=0.04, drift=0.03, frozen_gyro=False):
        s.x, s.y, s.h = x, y, h; s.mode = 0; s.turn_goal = 0; s.speed = 0; s.last_drive = -9
        s.gyro = h; s.turn_err = turn_err; s.drift = drift; s.frozen = frozen_gyro
    def tag_world(s):
        return np.array([s.x + 0.03*math.cos(s.h), s.y + 0.03*math.sin(s.h), 0.09])
    def on_turn(s, a):
        s.mode = 1; s.turn_goal = s.h + a*(1+s.turn_err); s.dir = 1 if a > 0 else -1; s.turn_t0 = T[0]
    def on_drive(s, v):
        if v == 0.0: s.mode = 0; return
        if s.mode == 1: return
        s.mode = 2; s.speed = v; s.last_drive = T[0]
    def step(s, dt):
        if s.mode == 1:
            s.h += s.dir*1.5*dt
            reached = (s.turn_goal - s.h)*s.dir <= 0 and not s.frozen
            if reached or T[0] - s.turn_t0 > 5.0: s.mode = 0   # firmware's 5 s timeout
        elif s.mode == 2:
            if T[0] - s.last_drive > 0.5: s.mode = 0
            else:
                s.h += s.drift*dt   # slow heading drift
                s.x += s.speed*dt*math.cos(s.h); s.y += s.speed*dt*math.sin(s.h)

def run(bot, target_xy, target_heading, overrides, hide=None, label='', stop_at=None, restart_at=None):
    OVERRIDES.clear(); OVERRIDES.update(overrides); LOG.clear(); T[0] = 0.0
    importlib.reload(C)
    n = C.Coordinator()
    tick = n.timers[0]; on_det = n.subs['marker_detections']; on_state = n.subs['/bot0/state']
    turn_pub, drive_pub, status_pub = n.pubs['/bot0/turn'], n.pubs['/bot0/drive'], n.pubs['~/status']
    seen_turn = seen_drive = 0
    pending = []   # (deliver_time, msg)
    dt = 0.005; next_cam = 0.0; next_state = 0.0; next_tick = 0.0
    started = False; end = None
    while T[0] < 60:
        # deliver commands to bot (WiFi ~10 ms)
        for t, m in turn_pub.msgs[seen_turn:]: bot.on_turn(m.data)
        seen_turn = len(turn_pub.msgs)
        for t, m in drive_pub.msgs[seen_drive:]: bot.on_drive(m.data)
        seen_drive = len(drive_pub.msgs)
        bot.step(dt)
        if T[0] >= next_cam:
            ds = []
            hidden = hide and hide[0] <= T[0] <= hide[1]
            if not hidden:
                ds.append(det(0, bot.tag_world(), tag_rot_cam(bot.h, True), 1.0))
            ds.append(det(3, np.array([*target_xy, 0.0]), tag_rot_cam(target_heading, False), 0.136/0.095))
            msg = NS(header=NS(stamp=NS(sec=int(T[0]), nanosec=int(round((T[0]%1)*1e9))), frame_id='vx500'), detections=ds)
            pending.append((T[0] + 0.1, msg)); next_cam += 0.125
        for item in [p for p in pending if p[0] <= T[0]]:
            on_det(item[1]); pending.remove(item)
        if T[0] >= next_state:
            on_state(NS(data=bot.mode)); next_state += 0.1
        if T[0] >= next_tick:
            tick(); next_tick += 0.05
        if not started and T[0] > 1.0:
            r = n.srvs['~/start'](None, NS(success=False, message=''))
            print(label, 'START ->', r.success, r.message); started = True
        if stop_at and abs(T[0]-stop_at) < dt/2:
            r = n.srvs['~/stop'](None, NS(success=False, message='')); print(label, 'STOP ->', r.message, '| bot mode', bot.mode)
            end = T[0]
        if restart_at and abs(T[0]-restart_at) < dt/2:
            r = n.srvs['~/start'](None, NS(success=False, message='')); print(label, 'START again ->', r.success, r.message)
            r = n.srvs['~/start'](None, NS(success=False, message='')); print(label, 'START twice ->', r.success, r.message)
            end = None
        if started and n._phase in (6, 7) and end is None:
            end = T[0]
        if end and T[0] > end + 1: break
        T[0] += dt
    # true pivot vs true goal
    tx, ty = target_xy
    ang = n._goal_angle
    # goal angle is in target tag frame; target tag x along target_heading in world
    gx = tx + 0.25*math.cos(target_heading + ang); gy = ty + 0.25*math.sin(target_heading + ang)
    err = math.hypot(bot.x - gx, bot.y - gy)
    for t, k, s in LOG:
        if not s.startswith(('driving bot0', 'TAG SIZES', 'tag offset')): print('  %6.2f %s %s' % (t, k, s))
    print(label, 'END phase', C.PHASE_NAMES[n._phase], '| true pivot-goal error %.1f cm' % (err*100),
          '| bot mode', bot.mode, '| t=%.1f' % T[0])
    last = status_pub.msgs[-1][1]
    print('   last status: phase', last.phase, 'round', last.round, 'dist', last.distance_to_goal_m, 'reason', last.reason)
    return n, err

base = dict(tag_offset_forward_m=0.03)
run(Bot(0.0, 0.0, 0.0), (0.6, 0.0), 0.3, base, label='[ahead]')
run(Bot(0.0, 0.0, 0.0), (0.0, 0.6), -1.0, base, label='[left]')
run(Bot(0.0, 0.0, 0.0), (0.0, -0.6), 2.0, base, label='[right]')
run(Bot(0.3, 0.2, 2.5), (-0.2, -0.3), 0.0, base, label='[behind]')
run(Bot(0.0, 0.0, 0.0), (0.6, 0.1), 0.0, dict(base, extra_delay_s=0.3), label='[delay0.3]')
run(Bot(0.0, 0.0, 0.0), (0.7, 0.0), 0.0, base, hide=(3.5, 4.5), label='[hide 1s]')
run(Bot(0.0, 0.0, 0.0), (0.7, 0.0), 0.0, base, hide=(3.5, 8.0), label='[hide 4.5s]')
run(Bot(0.0, 0.0, 0.0, frozen_gyro=True), (0.0, 0.6), 0.0, base, label='[frozen gyro]')
run(Bot(0.0, 0.0, 0.0), (0.6, 0.0), 0.0, dict(base, tag_offset_forward_m=0.0), label='[offset unset, bot has 3 cm]')

n,_ = run(Bot(0.0, 0.0, 0.0), (0.7, 0.0), 0.0, base, label='[stop mid-drive]', stop_at=3.0)
n,_ = run(Bot(0.0, 0.0, 0.0), (0.7, 0.0), 0.0, base, label='[stop then restart]', stop_at=3.0, restart_at=3.6)
