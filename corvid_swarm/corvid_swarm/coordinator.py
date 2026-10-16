"""Corvid's swarm coordinator - one bot to one point.

Watches the overhead camera's marker detections and, on /coordinator/start,
drives one bot to a goal on a ring round a target tag. Each round: with the bot
stopped, measure where it is and which way it faces; send a RELATIVE turn,
which the bot measures off on its own gyro; stream a slow straight drive, which
the bot holds on its gyro; stop it from the camera; measure again. The run ends
when the bot is within tolerance, or after max_rounds.

Nothing here runs on the drone or touches the drone's packages.

Frames. Everything is computed from tag poses in the camera optical frame, and
there is no world frame. The bot's tag and the target's tag are in the same
image at the same instant, so where the goal lies relative to the bot is a
difference of two poses from one picture. The camera may move between frames -
it will, on the drone - without changing any answer.

Tag mounting. A bot's tag is stuck with its TOP edge towards the bot's front
(the IMU end). The detector's tag axes are x towards the tag's right edge, y
towards its top edge, z up out of the paper. The bot's body frame (REP-103: x
forward, y left, z up) is therefore the tag frame turned +90 degrees about z:
body x = tag y, body y = -tag x. That one rotation is TAG_TO_BODY below.

Tag sizes. The detector solves every tag at one size, 0.136 m, which is right
for id 0 and nothing else. A pose scales linearly with the size the solver is
told and its rotation does not change, so a tag of a different real size is
corrected exactly by scaling its position. /marker_detections, bags and the
debug image therefore show the small tags about 43% too far away; only this
node's numbers are right.

Clocks. A detection's age is this node's clock minus the image's stamp, so the
node must run on the machine whose clock stamped the image - true at home, where
camera, detector and coordinator share the laptop. If they are ever split across
machines (laptop and Orin), the clocks need syncing first or the stop point is off.
In the lab the detector runs on the Orin and this node on the laptop, and the two
clocks are not synced. arrival_latency_s covers that: when it is >= 0, every
image stamp is ignored and each detection is taken as having been seen that long
before it ARRIVED here, which keeps every comparison on this node's own clock.
0.3 s is the drone's measured detection age (hover_4). Off (-1) by default, so
nothing changes at home.
"""

import collections
import math
import time

import numpy as np
import rclpy
from corvid_msgs.msg import CoordinatorStatus, MarkerDetectionArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, UInt8
from std_srvs.srv import Trigger

NAN = float('nan')

# /botN/state values, as the firmware publishes them.
BOT_IDLE = 0

# The bot's body axes written in tag axes, one per column. Body x (forward) is
# the tag's +y (top edge), body y (left) is the tag's -x, z is shared.
TAG_TO_BODY = np.array([
    [0.0, -1.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
])

PHASE_NAMES = {
    CoordinatorStatus.PHASE_DISABLED: 'disabled',
    CoordinatorStatus.PHASE_MEASURING: 'measuring',
    CoordinatorStatus.PHASE_TURNING: 'turning',
    CoordinatorStatus.PHASE_DRIVING: 'driving',
    CoordinatorStatus.PHASE_WAITING_FOR_TAGS: 'waiting for tags',
    CoordinatorStatus.PHASE_STOPPING: 'stopping',
    CoordinatorStatus.PHASE_ARRIVED: 'arrived',
    CoordinatorStatus.PHASE_GAVE_UP: 'gave up',
}

RUNNING_PHASES = (
    CoordinatorStatus.PHASE_MEASURING,
    CoordinatorStatus.PHASE_TURNING,
    CoordinatorStatus.PHASE_DRIVING,
    CoordinatorStatus.PHASE_WAITING_FOR_TAGS,
    CoordinatorStatus.PHASE_STOPPING,
)


# ---------- geometry: plain functions, no ROS, so they can be tested alone ----------

def rotation_from_quaternion(x, y, z, w):
    """Unit quaternion (x, y, z, w) to a 3x3 rotation matrix."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quaternion_from_rotation(m):
    """3x3 rotation matrix to (x, y, z, w). Same four-branch method as the detector."""
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s,
                (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        return (0.25 * s, (m[0, 1] + m[1, 0]) / s,
                (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s)
    if m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        return ((m[0, 1] + m[1, 0]) / s, 0.25 * s,
                (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s)
    s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
    return ((m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s,
            0.25 * s, (m[1, 0] - m[0, 1]) / s)


def tag_pose(detection, scale):
    """Tag centre (scaled to the tag's real size) and rotation, camera frame."""
    p = detection.pose.position
    q = detection.pose.orientation
    return (scale * np.array([p.x, p.y, p.z]),
            rotation_from_quaternion(q.x, q.y, q.z, q.w))


def bot_pivot(tag_position, tag_rotation, offset_forward, offset_left):
    """The bot's pivot and body rotation, camera frame.

    offset_* is where the TAG centre sits relative to the pivot, in the bot's
    body frame, so the pivot is the tag centre minus that offset turned into
    the camera frame.
    """
    body = tag_rotation @ TAG_TO_BODY
    pivot = tag_position - body @ np.array([offset_forward, offset_left, 0.0])
    return pivot, body


def ring_angle(pivot, target_position, target_rotation):
    """Angle of the pivot round the target, in the target tag's own plane.

    Taken in the target tag's frame so it does not depend on the camera: the
    same angle names the same spot on the floor however the drone has turned.
    """
    v = target_rotation.T @ (pivot - target_position)
    return math.atan2(v[1], v[0])


def ring_point(target_position, target_rotation, angle, radius):
    """The point at that angle on a ring of that radius round the target, camera frame."""
    return target_position + target_rotation @ np.array(
        [radius * math.cos(angle), radius * math.sin(angle), 0.0])


def in_body(pivot, body_rotation, point):
    """A camera-frame point as (forward, left) from the bot's pivot, in its own plane."""
    v = body_rotation.T @ (point - pivot)
    return float(v[0]), float(v[1])


def wrap(angle):
    """Angle into (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def find(detections, marker_id):
    for d in detections.detections:
        if d.id == marker_id:
            return d
    return None


class Measurement:
    """Everything one camera frame says about the bot, the target and the goal."""

    def __init__(self):
        self.stamp = None           # image stamp, seconds
        self.header = None          # that frame's header, copied into the status
        self.age = NAN              # now - stamp, seconds
        self.bot_visible = False
        self.target_visible = False
        self.pivot = None
        self.body = None
        self.target = None
        self.target_rotation = None
        self.goal = None
        self.forward = NAN          # goal, ahead of the pivot
        self.left = NAN             # goal, left of the pivot
        self.distance = NAN
        self.bearing = NAN

    @property
    def both(self):
        return self.bot_visible and self.target_visible


# ---------- the node ----------

class Coordinator(Node):
    """Drives one bot to one point on a ring round a target tag."""

    def __init__(self):
        super().__init__('coordinator')

        # Declared with typed defaults on purpose: an override of the wrong
        # type (an integer for a size, say) then fails loudly at startup
        # rather than reading as 0.0.
        def param(name, default):
            return self.declare_parameter(name, default).value

        self._bot_id = param('bot_id', 0)
        self._bot_marker_id = param('bot_marker_id', 0)
        self._target_marker_id = param('target_marker_id', 3)

        # Tag sizes. detector_marker_size_m MUST match what the detector runs
        # at; every other tag size is corrected from it.
        self._detector_size = param('detector_marker_size_m', 0.136)
        self._small_size = param('small_marker_size_m', 0.095)
        self._large_ids = list(param('large_marker_ids', [0]))

        # Where the bot's tag centre sits relative to its pivot, body frame,
        # metres. From the spin test. 0 until measured.
        self._offset_forward = param('tag_offset_forward_m', 0.0)
        self._offset_left = param('tag_offset_left_m', 0.0)

        self._ring_radius = param('ring_radius_m', 0.25)
        self._speed = param('drive_speed_mps', 0.1)
        self._arrive_tolerance = param('arrive_tolerance_m', 0.05)
        self._turn_tolerance = math.radians(param('turn_tolerance_deg', 5.0))
        self._max_rounds = param('max_rounds', 5)

        self._tag_lost_s = param('tag_lost_s', 0.5)
        self._tag_wait_s = param('tag_wait_s', 3.0)
        # Act only on detections at least this old. 0 at home normally; 0.3
        # rehearses the drone's measured detection age.
        self._extra_delay = param('extra_delay_s', 0.0)
        # >= 0: ignore image stamps, take each detection as seen this long
        # before it arrived (camera on another machine, clocks not synced).
        # -1: use the image stamps (home, one machine).
        self._arrival_latency = param('arrival_latency_s', -1.0)
        # After the bot reports idle, ignore frames younger than this, so a
        # measurement never comes from a picture of the bot still moving.
        self._settle_s = param('settle_s', 0.2)
        self._state_timeout = param('state_timeout_s', 1.0)
        self._turn_timeout = param('turn_timeout_s', 7.0)
        # The firmware gives up on a turn after 5 s. A turn that long means the
        # gyro never saw the bot reach its goal - seen 15 Oct in spin_1_oct15,
        # where a frozen gyro let the bot spin to the timeout. Stop the run
        # rather than drive off on a heading nobody measured.
        self._turn_stuck_s = param('turn_stuck_s', 4.5)

        for name, value in (('detector_marker_size_m', self._detector_size),
                            ('small_marker_size_m', self._small_size),
                            ('ring_radius_m', self._ring_radius),
                            ('drive_speed_mps', self._speed),
                            ('arrive_tolerance_m', self._arrive_tolerance)):
            if not value > 0.0:
                raise ValueError('%s must be > 0, got %r' % (name, value))
        if self._max_rounds < 1:
            raise ValueError('max_rounds must be >= 1, got %r' % self._max_rounds)

        ns = '/bot%d' % self._bot_id
        self._bot_name = 'bot%d' % self._bot_id

        best_effort = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)

        # Matches the firmware: turn reliable (a lost turn is never resent),
        # drive and state best effort (replaced 100 ms later anyway).
        self._turn_pub = self.create_publisher(Float32, ns + '/turn', 10)
        self._drive_pub = self.create_publisher(Float32, ns + '/drive', best_effort)
        self.create_subscription(UInt8, ns + '/state', self._on_state, best_effort)
        self.create_subscription(
            MarkerDetectionArray, 'marker_detections', self._on_detections, 10)
        self._status_pub = self.create_publisher(CoordinatorStatus, '~/status', 10)
        self.create_service(Trigger, '~/start', self._on_start)
        self.create_service(Trigger, '~/stop', self._on_stop)

        # Recent detection messages, (image stamp, message), oldest first.
        # Kept so extra_delay_s can pick one old enough.
        self._frames = collections.deque(maxlen=100)
        self._last_frame_arrival = None

        self._bot_state = None
        self._bot_state_time = None
        self._idle_since = None      # when the bot last went idle; None while busy
        self._seen_busy = False      # a non-idle state was heard since the turn was sent

        self._phase = CoordinatorStatus.PHASE_DISABLED
        self._phase_since = self._now()
        self._reason = 'waiting for /coordinator/start'
        self._round = 0
        self._resume = False         # next decision continues the round, not a new one
        self._goal_angle = None
        self._after_stop = None      # phase to enter once a stop is confirmed
        self._stop_burst = 0
        self._stop_sent_at = None
        self._turn_sent_at = None
        self._drive_started_at = None
        self._drive_deadline = None
        self._drive_ticks = 0
        self._planned_distance = NAN  # goal distance when the current drive was planned
        self._sent_turn = NAN
        self._sent_drive = NAN
        self._drive_remaining = NAN
        self._stop_lead = NAN
        self._last_status_stamp = None
        self._last_status_time = 0.0

        self.create_timer(0.05, self._tick)

        large = ', '.join(str(i) for i in self._large_ids)
        self.get_logger().info(
            'driving %s (tag id %d) to a %.2f m ring round target id %d at %.2f m/s; '
            'arrive within %.1f cm, skip turns under %.1f deg, %d rounds max; '
            'extra_delay %.2f s'
            % (self._bot_name, self._bot_marker_id, self._ring_radius,
               self._target_marker_id, self._speed, self._arrive_tolerance * 100,
               math.degrees(self._turn_tolerance), self._max_rounds, self._extra_delay))
        self.get_logger().info(
            'TAG SIZES: the detector must be running at %.3f m. Ids [%s] are taken as '
            'that size; every other id is %.3f m, so its position is scaled by %.3f. '
            '/marker_detections, bags and the debug image show those tags %.0f%% too far.'
            % (self._detector_size, large, self._small_size,
               self._small_size / self._detector_size,
               (self._detector_size / self._small_size - 1.0) * 100))
        self.get_logger().info(
            'tag offset from pivot: %.3f m forward, %.3f m left%s'
            % (self._offset_forward, self._offset_left,
               ' (NOT MEASURED - run the spin test)'
               if self._offset_forward == 0.0 and self._offset_left == 0.0 else ''))
        if self._arrival_latency >= 0.0:
            self.get_logger().info(
                'CLOCKS: image stamps ignored; each detection taken as %.2f s old on arrival '
                '(arrival_latency_s)' % self._arrival_latency)

    # ---------- small helpers ----------

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _scale(self, marker_id):
        if marker_id in self._large_ids:
            return 1.0
        return self._small_size / self._detector_size

    def _running(self):
        return self._phase in RUNNING_PHASES

    def _set_phase(self, phase, reason):
        if phase != self._phase or reason != self._reason:
            self.get_logger().info('[%s] %s' % (PHASE_NAMES[phase], reason))
        self._phase = phase
        self._phase_since = self._now()
        self._reason = reason

    def _send_turn(self, angle):
        self._turn_pub.publish(Float32(data=float(angle)))
        self._sent_turn = float(angle)
        self._turn_sent_at = self._now()
        self._seen_busy = False

    def _send_drive(self, speed):
        self._drive_pub.publish(Float32(data=float(speed)))
        self._sent_drive = float(speed)

    def _stop(self, then_phase, reason):
        """Send the stop and wait for the bot to confirm it, then enter then_phase."""
        self._stop_burst = 3
        self._stop_sent_at = self._now()
        self._after_stop = then_phase
        self._drive_remaining = NAN
        self._stop_lead = NAN
        self._set_phase(CoordinatorStatus.PHASE_STOPPING, reason)

    def _finish(self, phase, reason):
        """End the run. Always sends the stop, whatever the bot was doing."""
        self._stop_burst = 3
        self._stop_sent_at = self._now()
        self._after_stop = None
        self._drive_remaining = NAN
        self._stop_lead = NAN
        self._set_phase(phase, reason)
        if phase == CoordinatorStatus.PHASE_GAVE_UP:
            self.get_logger().warn('run ended: ' + reason)

    def send_stop_now(self):
        """Last-gasp stop on shutdown. The bot's 0.5 s watchdog is the backup."""
        for _ in range(3):
            try:
                self._drive_pub.publish(Float32(data=0.0))
            except Exception:  # context already gone: the watchdog will stop it
                return
            time.sleep(0.03)

    def _usable_frame(self, now):
        """Newest detection message at least extra_delay_s old, or None."""
        for stamp, msg in reversed(self._frames):
            if now - stamp >= self._extra_delay:
                return stamp, msg
        return None

    def _measure(self, frame, now):
        m = Measurement()
        if frame is None:
            return m
        stamp, msg = frame
        m.stamp = stamp
        m.header = msg.header
        m.age = max(0.0, now - stamp)
        bot = find(msg, self._bot_marker_id)
        target = find(msg, self._target_marker_id)
        m.bot_visible = bot is not None
        m.target_visible = target is not None
        if bot is not None:
            position, rotation = tag_pose(bot, self._scale(self._bot_marker_id))
            m.pivot, m.body = bot_pivot(
                position, rotation, self._offset_forward, self._offset_left)
        if target is not None:
            m.target, m.target_rotation = tag_pose(
                target, self._scale(self._target_marker_id))
        if m.both and self._goal_angle is not None:
            m.goal = ring_point(m.target, m.target_rotation,
                                self._goal_angle, self._ring_radius)
            m.forward, m.left = in_body(m.pivot, m.body, m.goal)
            m.distance = math.hypot(m.forward, m.left)
            m.bearing = math.atan2(m.left, m.forward)
        return m

    def _newest_both_stamp(self, now):
        """Image stamp of the newest usable frame that held both tags, or None."""
        for stamp, msg in reversed(self._frames):
            if now - stamp < self._extra_delay:
                continue
            if (find(msg, self._bot_marker_id) is not None
                    and find(msg, self._target_marker_id) is not None):
                return stamp
        return None

    def _missing(self, m):
        names = []
        if not m.bot_visible:
            names.append('%s tag (id %d)' % (self._bot_name, self._bot_marker_id))
        if not m.target_visible:
            names.append('target (id %d)' % self._target_marker_id)
        return ' and '.join(names) if names else 'nothing'

    # ---------- callbacks ----------

    def _on_detections(self, msg):
        now = self._now()
        if self._arrival_latency >= 0.0:
            stamp = now - self._arrival_latency
        else:
            stamp = stamp_seconds(msg.header.stamp)
        self._frames.append((stamp, msg))
        self._last_frame_arrival = now

    def _on_state(self, msg):
        now = self._now()
        if msg.data != BOT_IDLE:
            self._idle_since = None
            if self._phase == CoordinatorStatus.PHASE_TURNING:
                self._seen_busy = True
        elif self._idle_since is None:
            self._idle_since = now
        self._bot_state = msg.data
        self._bot_state_time = now

    def _on_start(self, _request, response):
        now = self._now()
        response.success = False
        if self._running():
            response.message = 'already running: round %d, %s' % (
                self._round, PHASE_NAMES[self._phase])
            return response
        if self._last_frame_arrival is None or now - self._last_frame_arrival > 0.5:
            response.message = ('no detections arriving on /marker_detections'
                                ' - is the detector running?')
            return response
        if self._bot_state_time is None or now - self._bot_state_time > self._state_timeout:
            response.message = 'no /%s/state heard in the last %.1f s - is %s on the agent?' % (
                self._bot_name, self._state_timeout, self._bot_name)
            return response
        if self._bot_state != BOT_IDLE:
            response.message = '%s is busy (state %d)' % (self._bot_name, self._bot_state)
            return response
        m = self._measure(self._usable_frame(now), now)
        if not m.both:
            response.message = "can't start, not in view: " + self._missing(m)
            return response

        # The goal is fixed for the whole run as an angle round the target:
        # the point on the ring nearest where the bot starts.
        self._goal_angle = ring_angle(m.pivot, m.target, m.target_rotation)
        m = self._measure(self._usable_frame(now), now)
        self._round = 0
        self._resume = False
        self._sent_turn = NAN
        self._sent_drive = NAN
        self._set_phase(CoordinatorStatus.PHASE_MEASURING, 'run started')
        response.success = True
        response.message = 'started: goal %.2f m away, bearing %+.0f deg' % (
            m.distance, math.degrees(m.bearing))
        return response

    def _on_stop(self, _request, response):
        was = PHASE_NAMES[self._phase]
        self._finish(CoordinatorStatus.PHASE_DISABLED, 'stopped by /coordinator/stop')
        response.success = True
        response.message = 'stop sent (was %s)' % was
        return response

    # ---------- the loop, 20 Hz ----------

    def _tick(self):
        now = self._now()
        frame = self._usable_frame(now)
        m = self._measure(frame, now)

        if self._stop_burst > 0:
            self._send_drive(0.0)
            self._stop_burst -= 1

        if self._running():
            if self._bot_state_time is None or now - self._bot_state_time > self._state_timeout:
                self._finish(CoordinatorStatus.PHASE_GAVE_UP,
                             'lost /%s/state for %.1f s - %s off the network?'
                             % (self._bot_name, self._state_timeout, self._bot_name))
            else:
                handler = {
                    CoordinatorStatus.PHASE_MEASURING: self._measuring,
                    CoordinatorStatus.PHASE_TURNING: self._turning,
                    CoordinatorStatus.PHASE_DRIVING: self._driving,
                    CoordinatorStatus.PHASE_WAITING_FOR_TAGS: self._waiting_for_tags,
                    CoordinatorStatus.PHASE_STOPPING: self._stopping,
                }[self._phase]
                handler(m, now)

        if m.stamp != self._last_status_stamp or now - self._last_status_time >= 0.5:
            self._publish_status(m, now)
            self._last_status_stamp = m.stamp
            self._last_status_time = now

    def _measuring(self, m, now):
        if self._bot_state != BOT_IDLE or self._idle_since is None:
            if now - self._phase_since > self._turn_timeout:
                self._finish(CoordinatorStatus.PHASE_GAVE_UP,
                             '%s never reported idle to be measured' % self._bot_name)
            return
        settled = m.stamp is not None and m.stamp >= self._idle_since + self._settle_s
        if settled and m.both:
            self._decide(m, now)
            return
        if now - self._phase_since > self._settle_s + self._extra_delay + self._tag_wait_s:
            self._finish(CoordinatorStatus.PHASE_GAVE_UP,
                         'no settled frame with both tags for %.1f s, missing: %s'
                         % (self._tag_wait_s, self._missing(m) if m.stamp else 'all frames'))

    def _decide(self, m, now):
        if m.distance <= self._arrive_tolerance:
            self._finish(CoordinatorStatus.PHASE_ARRIVED,
                         'arrived, %.1f cm from goal after %d round(s)'
                         % (m.distance * 100, self._round))
            return
        if self._resume:
            self._resume = False
        elif self._round >= self._max_rounds:
            self._finish(CoordinatorStatus.PHASE_GAVE_UP,
                         'gave up after %d rounds, %.1f cm from goal'
                         % (self._max_rounds, m.distance * 100))
            return
        else:
            self._round += 1
        self._planned_distance = m.distance
        if abs(m.bearing) > self._turn_tolerance:
            self._send_turn(m.bearing)
            self._set_phase(CoordinatorStatus.PHASE_TURNING,
                            'round %d: turning %+.1f deg, goal %.1f cm away'
                            % (self._round, math.degrees(m.bearing), m.distance * 100))
        else:
            self._start_drive(now, 'round %d: within %.1f deg, driving %.1f cm'
                              % (self._round, math.degrees(m.bearing), m.distance * 100))

    def _turning(self, m, now):
        if now - self._turn_sent_at > self._turn_timeout:
            self._finish(CoordinatorStatus.PHASE_GAVE_UP,
                         'turn not finished after %.1f s' % self._turn_timeout)
            return
        # Done when the bot reports idle, heard after the turn went out, and it
        # either reported turning in between or a second has passed (a very
        # small turn can start and finish between two 10 Hz state messages).
        idle_after = (self._bot_state == BOT_IDLE
                      and self._bot_state_time > self._turn_sent_at + 0.3)
        if idle_after and (self._seen_busy or now - self._turn_sent_at > 1.0):
            took = self._bot_state_time - self._turn_sent_at
            if took > self._turn_stuck_s:
                self._finish(CoordinatorStatus.PHASE_GAVE_UP,
                             'turn took %.1f s, about the firmware timeout - gyro frozen or '
                             'wheels stalled? check /%s/heading against the camera'
                             % (took, self._bot_name))
                return
            self._start_drive(now, 'round %d: turn done, driving' % self._round)

    def _start_drive(self, now, reason):
        self._drive_started_at = now
        expected = self._planned_distance / self._speed
        self._drive_deadline = now + max(2.0 * expected, expected + 2.0)
        self._drive_ticks = 0
        self._set_phase(CoordinatorStatus.PHASE_DRIVING, reason)

    def _driving(self, m, now):
        # Keep the drive coming at 10 Hz: the bot's 0.5 s watchdog stops it otherwise.
        if self._drive_ticks % 2 == 0:
            self._send_drive(self._speed)
        self._drive_ticks += 1

        if now > self._drive_deadline:
            self._finish(CoordinatorStatus.PHASE_GAVE_UP,
                         'drive ran past its time limit - is %s moving?' % self._bot_name)
            return

        # Only frames taken since the drive started say where the bot is now.
        fresh = m.stamp is not None and m.stamp >= self._drive_started_at
        if fresh and m.both:
            # Each frame shows where the bot WAS, age seconds ago; it has gone
            # speed x age further since. Stop when what is left is no more.
            self._drive_remaining = m.forward
            self._stop_lead = self._speed * m.age
            if m.forward <= self._stop_lead:
                self._stop(CoordinatorStatus.PHASE_MEASURING,
                           'round %d: stopping, %.1f cm left against a %.1f cm lead'
                           % (self._round, m.forward * 100, self._stop_lead * 100))
                return

        last_both = self._newest_both_stamp(now)
        reference = max(self._drive_started_at,
                        last_both if last_both is not None else -math.inf)
        if (now - self._extra_delay) - reference > self._tag_lost_s:
            self._resume = True
            self._stop(CoordinatorStatus.PHASE_WAITING_FOR_TAGS,
                       'round %d: lost sight of %s, stopping'
                       % (self._round, self._missing(m) if m.stamp else 'all frames'))

    def _stopping(self, m, now):
        if self._bot_state == BOT_IDLE and self._bot_state_time > self._stop_sent_at + 0.05:
            nxt = self._after_stop
            self._after_stop = None
            if nxt == CoordinatorStatus.PHASE_WAITING_FOR_TAGS:
                self._set_phase(nxt, 'stopped, waiting up to %.1f s for both tags'
                                % self._tag_wait_s)
            else:
                self._set_phase(CoordinatorStatus.PHASE_MEASURING,
                                'stopped, measuring')
            return
        if now - self._stop_sent_at > 1.5:
            self._finish(CoordinatorStatus.PHASE_GAVE_UP,
                         '%s did not report idle 1.5 s after the stop' % self._bot_name)

    def _waiting_for_tags(self, m, now):
        if m.stamp is not None and m.stamp >= self._phase_since and m.both:
            self._set_phase(CoordinatorStatus.PHASE_MEASURING,
                            'both tags back, measuring (round %d continues)' % self._round)
            return
        if now - self._phase_since > self._tag_wait_s:
            self._finish(CoordinatorStatus.PHASE_GAVE_UP,
                         'not seen for %.1f s: %s'
                         % (self._tag_wait_s, self._missing(m) if m.stamp else 'all frames'))

    # ---------- the record ----------

    def _publish_status(self, m, now):
        s = CoordinatorStatus()
        if m.stamp is not None and now - self._last_frame_arrival < 0.5 + self._extra_delay:
            s.header = m.header
            s.detection_age_s = m.age
        else:
            s.header.stamp = self.get_clock().now().to_msg()
            s.header.frame_id = self._frames[-1][1].header.frame_id if self._frames else ''
            s.detection_age_s = NAN

        s.phase = self._phase
        s.round = self._round
        s.bot_marker_id = self._bot_marker_id
        s.target_marker_id = self._target_marker_id
        s.bot_visible = m.bot_visible
        s.target_visible = m.target_visible
        s.bot_tag_scale = self._scale(self._bot_marker_id)
        s.target_tag_scale = self._scale(self._target_marker_id)

        if m.pivot is not None:
            s.bot_pose.position.x, s.bot_pose.position.y, s.bot_pose.position.z = (
                float(v) for v in m.pivot)
            qx, qy, qz, qw = quaternion_from_rotation(m.body)
            s.bot_pose.orientation.x = qx
            s.bot_pose.orientation.y = qy
            s.bot_pose.orientation.z = qz
            s.bot_pose.orientation.w = qw
        else:
            p, q = s.bot_pose.position, s.bot_pose.orientation
            p.x = p.y = p.z = q.x = q.y = q.z = q.w = NAN

        if m.target is not None:
            s.target.x, s.target.y, s.target.z = (float(v) for v in m.target)
        else:
            s.target.x = s.target.y = s.target.z = NAN

        # The goal exists from a start until a /coordinator/stop; after an
        # arrival or a give-up it stays visible so the end state can be seen.
        if m.goal is not None and self._phase != CoordinatorStatus.PHASE_DISABLED:
            s.goal.x, s.goal.y, s.goal.z = (float(v) for v in m.goal)
            s.distance_to_goal_m = m.distance
            s.bearing_to_goal_rad = m.bearing
        else:
            s.goal.x = s.goal.y = s.goal.z = NAN
            s.distance_to_goal_m = NAN
            s.bearing_to_goal_rad = NAN

        driving = self._phase == CoordinatorStatus.PHASE_DRIVING
        s.drive_remaining_m = self._drive_remaining if driving else NAN
        s.stop_lead_m = self._stop_lead if driving else NAN
        s.sent_turn_rad = self._sent_turn
        s.sent_drive_mps = self._sent_drive
        s.bot_state = self._bot_state if self._bot_state is not None else 255
        s.bot_state_age_s = (now - self._bot_state_time
                             if self._bot_state_time is not None else NAN)
        s.reason = self._reason
        self._status_pub.publish(s)


def main(args=None):
    # Keep the ROS context alive through Ctrl-C so the stop below can still be
    # published. Without this rclpy shuts the context down first and the stop
    # would fail; the bot's watchdog would still stop it 0.5 s later.
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init(args=args)
    node = Coordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_stop_now()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
