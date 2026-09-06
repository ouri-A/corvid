"""Corvid's station keeper.

Holds the drone above an ArUco marker. Marker stationary first; once that is
stable the marker goes onto a ground robot and the same loop follows it.

The division of labour, because it decides everything about this file. PX4 owns
four nested loops - position, velocity, attitude, rate - and this node injects
at the second one by publishing velocity setpoints. So PX4 works out what tilt
produces the velocity asked for, holds it against wind and battery sag, keeps
the vehicle stable doing it, mixes to the motors, and runs every failsafe. None
of that is here. What is here is the strip above it: take a marker pose in the
camera's optical frame, turn it into a velocity, and hand that to PX4 at a rate
it will accept.

PX4 has no idea a marker exists. Everything about the target is ours.

TWO RATES, NOT ONE. This is the structural decision in the file. PX4 drops out
of offboard mode if OffboardControlMode stops arriving at better than 2 Hz, and
the camera runs at ~13 Hz with measured intervals between 18 and 255 ms. Driving
the setpoint publisher from the image callback would mean a single slow frame -
which has already been observed - hands the aircraft back to PX4 mid-flight. So
a fixed-rate timer publishes every tick no matter what, and the detection
callback only updates what that timer publishes. The vision rate and the
setpoint rate are decoupled and the timer is the only thing PX4 sees.

The control law runs in the timer, not the callback, which is worth being
deliberate about. It means the integrator advances on a known fixed dt instead
of on whatever interval the camera happened to deliver. ARK's tracktor-beam does
the opposite - a bare `integral += error` in the vision callback with no dt at
all - so its effective I gain rides on the frame rate. That is one of the
reasons their I gain ships at zero.

Frames. No TF anywhere: one constant rotation from the camera optical frame into
the body frame, hardcoded below, exactly as the whole PX4 vision-landing corner
of the ecosystem does it. But the command cannot be sent in the body frame,
because TrajectorySetpoint's velocity field is local NED and there is no
body-frame velocity setpoint in the uORB interface. So there is a second
rotation, by the vehicle's yaw, purely to speak PX4's language. That is what the
VehicleLocalPosition subscription is for - that, and z for altitude.

Between those two there is a third rotation, and leaving it out was a real bug.
The camera is bolted to the airframe, so optical-to-body is genuinely constant
and attitude has nothing to do with it. What is NOT constant is the step after:
rotating by yaw alone quietly asserts that roll and pitch are zero, and they are
not. A tilted camera projects a marker on the ground to a false offset of about
height x tan(tilt). Measured against Gazebo ground truth: 1.2 cm below 2 degrees
of tilt, 19.0 cm at 3-5 degrees, 26.7 cm above 5 degrees, from 2.3 m. So the
body-frame vector is levelled first - roll and pitch removed, the drone's own
heading kept - and only then does the yaw rotation touch it.

ArduPilot names this exact choice. AC_PrecLand branches on the frame a
measurement arrived in: BODY_FRD gets the full attitude, LOCAL_FRD gets yaw
only, and which is correct depends entirely on whether the sensor already
reports in a stabilised frame. Ours does not. PX4's own landing_target_estimator
does not offer the choice - it rotates the sensor ray by the full attitude
quaternion, always. It goes further up than the autopilots, too: the MAVLink
LANDING_TARGET message carries a frame field for precisely this, and PX4 accepts
only MAV_FRAME_LOCAL_NED, which is the protocol saying that resolving attitude
before you send anything is the companion computer's job.

The attitude used is the one nearest the source image's own timestamp, not the
newest one on hand. At around 0.18 s of pipeline latency and tilt rates near
10 deg/s, the newest attitude is not the attitude the picture was taken at, and
using it would put back a good part of what the correction just removed.

Height never comes from vision. Range from a marker is depth from apparent size
and is the fragile half of the pose - 3.2% at hover - so the marker sets where
to be, and PX4 sets how high.

The two axes are commanded differently, and the reason is worth stating because
the first version got it wrong and sank the aircraft. Lateral is a VELOCITY
setpoint: the marker gives an offset, that offset becomes a speed, and the
offset going to zero makes the speed go to zero. Altitude is a POSITION
setpoint: a constant handed to PX4's position loop. Commanding zero vertical
velocity does not hold height, it only declines to push, and with no altitude
feedback in that path a small bias in PX4's velocity loop integrates into a
slow sink - measured at 2-4 cm/s, which put the drone on the floor from 2.7 m
in about ninety seconds.

Altitude is measured from the takeoff point, not from the terrain. Airframe
4014 has no rangefinder - nothing publishes distance_sensor - so dist_bottom
carries no measurement and its valid flag is correctly false. It is also the
wrong reference for this task: height above whatever is underneath means the
drone bobs up as it crosses each ground robot, and Stage 3 is entirely about
flying over ground robots.

The node also publishes a StationKeeperStatus on every tick: the body-frame
offset it read, the P and I contributions separately, the command before and
after the rotation into NED, the marker's age and whether it was considered
lost. Nothing consumes it. It exists so a recorded flight can be read back as
what this node ACTUALLY decided instead of reconstructed from the marker pose,
the heading and the gains -- a reconstruction can disagree with the node and
give no sign that it has.

STILL OPEN, and marked in the code where it lands: what the drone should do when
the marker is lost. The placeholder here is to stop - zero lateral velocity, hold
height - which is the least surprising thing it could do, not a decision that
this is right.
"""

import math
from collections import deque

import rclpy
from corvid_msgs.msg import MarkerDetectionArray, StationKeeperStatus
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleAttitude,
    VehicleLocalPosition,
)
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)


# Camera optical frame to body frame (FLU: x forward, y left, z up).
#
# Optical is x right, y down, z out of the lens - and the lens points at the
# ground - so for a level drone: the marker's optical z is distance downward,
# its optical y runs along the body's fore-aft axis, and its optical x runs
# across it.
#
#     forward = -y_optical
#     left    = -x_optical
#     up      = -z_optical
#
# Sanity check that catches a sign error immediately: a marker directly beneath
# the drone is (0, 0, h) in optical, and comes out (0, 0, -h) in body - h below,
# which is where it is.
#
# This is written as three expressions rather than a matrix multiply because it
# is three sign flips and a relabelling, and a numpy import to do that would
# obscure it. The matrix form, cross-checked against the workshop's launch file
# and recorded in PROJECT_STATE.md, is [[0,-1,0],[-1,0,0],[0,0,-1]].
#
# THE TRAP, and it is a real one: this is the FLU (REP 103) version. The PX4-side
# FRD version is [[0,-1,0],[1,0,0],[0,0,1]] and the two differ by the 180-degree
# flip between conventions. Mixing them negates y and z, which reads as a badly
# tuned controller rather than as a frame bug. Everything in this file is FLU
# until the final rotation into NED, which is where FRD appears - once, named.
def optical_to_body_flu(x_opt, y_opt, z_opt):
    """Return (forward, left, up) in metres, given a point in the optical frame."""
    return -y_opt, -x_opt, -z_opt


# How far apart an image stamp and an attitude stamp may be before the pair is
# refused. Attitude arrives from PX4 at ~80 Hz, so 12.5 ms between samples; 50 ms
# allows a few dropped messages and still keeps the interpolation error small
# against 10 deg/s of tilt rate. Beyond this the honest answer is that we do not
# know how the drone was oriented when the picture was taken.
ATTITUDE_MATCH_TOLERANCE_S = 0.05

# Attitude samples kept. At ~80 Hz this is about 1.6 s of history, comfortably
# more than the 0.5 s marker timeout, so a detection can never be older than the
# buffer.
ATTITUDE_BUFFER_LEN = 128


def yaw_from_quaternion(w, x, y, z):
    """Yaw about the down axis, radians, from a body-FRD-to-NED quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def roll_pitch_from_quaternion(w, x, y, z):
    """Roll and pitch, radians, from a body-FRD-to-NED quaternion. Diagnostic only."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    # Clamped before the asin because a quaternion that is a hair off unit
    # length can push this just past 1.0 and raise ValueError, which would take
    # the node down mid-flight over a rounding error.
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return roll, math.asin(sin_pitch)


def rotate_frd_to_ned(w, x, y, z, vx, vy, vz):
    """Rotate a body-FRD vector into NED by the attitude quaternion."""
    # v' = v + w*t + (u x t), where u is the vector part and t = 2(u x v). This
    # is the standard quaternion rotation written without building a 3x3 matrix,
    # which is deliberate: a hand-written DCM is nine terms and the usual way to
    # get one wrong is to transpose it, which produces a plausible-looking
    # rotation in the wrong direction. Two cross products cannot be transposed.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def level_body_flu(forward, left, up, w, x, y, z):
    """Take roll and pitch out of a body-frame vector, keeping the drone's heading.

    In: an FLU vector in the true, tilted body frame, plus the attitude
    quaternion for the instant it was measured. Out: the same vector in the
    level body frame - still forward-and-left of the drone, but as if the
    airframe were flat, which is what the yaw rotation downstream assumes.
    """
    # Rotate all the way into NED with the full attitude, then take the yaw back
    # out. That is R_yaw' * R, which is the tilt on its own, but computed without
    # ever forming a matrix product. The yaw removed here is derived from this
    # same quaternion rather than from VehicleLocalPosition.heading, so the
    # removal is exact even if the two ever disagree.
    #
    # It looks wasteful to rotate by yaw here and by yaw again downstream. It is
    # not the same yaw: this one is the attitude sample's, from when the image
    # was taken, and the downstream one is the vehicle's now. Collapsing them
    # would silently reintroduce the latency this is built to avoid.
    n_north, n_east, n_down = rotate_frd_to_ned(w, x, y, z, forward, -left, -up)

    yaw = yaw_from_quaternion(w, x, y, z)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    level_forward = n_north * cos_yaw + n_east * sin_yaw
    level_right = -n_north * sin_yaw + n_east * cos_yaw

    # Back to FLU. Right and down both flip.
    return level_forward, -level_right, -n_down


class StationKeeper(Node):
    """Commands the velocity that keeps the drone over its marker."""

    def __init__(self):
        super().__init__('station_keeper')

        # Which marker this drone is locked to. A parameter, not a constant,
        # because in Stage 3 every ground robot carries a different marker and
        # this node only ever tracks one of them - the central bot. Every other
        # detection in the array belongs to the coordinator, and this node must
        # ignore it rather than average it in.
        self.declare_parameter('target_marker_id', 0)

        # Height to hold, in metres above the takeoff point.
        #
        # Above the TAKEOFF POINT, not above the terrain. PX4's local frame has
        # its origin where the EKF initialised, on the pad, so z is already
        # measured from there and this is a straight negation.
        #
        # It used to be terrain height from dist_bottom, and that was wrong
        # twice over. Airframe 4014 carries no rangefinder at all - nothing
        # publishes distance_sensor - so dist_bottom was the EKF's terrain
        # estimate with no range measurement behind it, which is why
        # dist_bottom_valid is false and why the height branch never once ran.
        #
        # Terrain-relative was also the wrong reference for the task. Height
        # above whatever happens to be underneath means a drone crossing a
        # 15 cm ground robot sees 15 cm of apparent climb and bobs up over
        # every robot it passes. Stage 3 is nothing but flying over robots.
        # Height above the takeoff point holds flat.
        self.declare_parameter('target_height_m', 2.5)

        # Lateral proportional gain: metres of offset in, metres per second out.
        # So the units are 1/s and the number is the reciprocal of the time
        # constant - 0.5 means the offset decays with a 2 s time constant.
        #
        # Deliberately below ARK's 1.5, which is the closest comparable
        # implementation, because our measurement is ~106 ms stale and published
        # research puts the stability collapse for vision-in-the-loop somewhere
        # between 150 and 225 ms of one-way delay. We are under that cliff but
        # not by a wide margin, and the first flight is not the place to find
        # the edge. Tune upward from here.
        self.declare_parameter('kp', 0.5)

        # Lateral integral gain. Zero, deliberately.
        #
        # The term is wired in so that turning it on is a launch argument rather
        # than an edit, but it starts off and should stay off unless a real
        # steady-state offset is observed in flight. Three reasons it is
        # unlikely to be needed. The plant is already an integrator - velocity
        # accumulated over time is position - so pure P has no steady-state
        # error to kill; a stationary marker settles to zero offset on its own.
        # PX4's own velocity loop underneath us is a PID whose integral gain
        # (MPC_XY_VEL_I_ACC) is already nonzero, so the one honest case for an
        # integrator here - PX4 quietly achieving 0.28 m/s when asked for 0.30 -
        # is a job PX4 has already taken. And stacking an integrator on an
        # integrating plant makes a double integrator, which overshoots and
        # rings in a way that looks like bad gains but is structural.
        #
        # The lag when following a moving marker is real and is NOT what this
        # term is for: the fix there is feedforward from apparent marker
        # velocity, which has no memory and so cannot wind up.
        self.declare_parameter('ki', 0.0)

        # Ceiling on the accumulated integral term's contribution, in m/s.
        # Enforced on the term itself rather than on the raw accumulator, so the
        # limit means what it says regardless of what ki is set to. ARK clamps
        # the accumulator to a velocity limit, which is a unit mismatch and part
        # of why their integrator looks bolted on.
        self.declare_parameter('integral_limit_mps', 0.3)

        # Lateral speed ceiling. Indoor lab, so well under ARK's 3.0.
        self.declare_parameter('max_velocity_mps', 1.0)

        # No vertical gain, deliberately: altitude is held with a POSITION
        # setpoint and PX4's own position loop closes it.
        #
        # The first version commanded vertical velocity, and it sank the drone
        # onto the floor from 2.7 m over about 90 seconds. Commanding zero
        # velocity is not holding altitude - it is only declining to push.
        # There is no altitude feedback anywhere in that path, so any bias in
        # PX4's vertical velocity loop integrates straight into position. A
        # position setpoint has the feedback in it by construction, and it
        # keeps holding through anything that would otherwise leave us
        # commanding zero.

        # How long to keep acting on the last detection before declaring the
        # marker lost.
        #
        # Camera intervals reach 255 ms, so anything at or below that would
        # declare the marker lost on a merely slow frame. 0.5 s is comfortably
        # above the worst interval observed and still short enough that the
        # drone is not chasing a stale position for long. ARK uses 3 s, which is
        # for a landing sequence that can afford to coast; we are station
        # keeping and should not.
        self.declare_parameter('marker_timeout_s', 0.5)

        # Setpoint rate. PX4's floor is 2 Hz; this is far above it so that a
        # scheduling hiccup cannot come near the limit, and it gives the control
        # law a fine, regular dt.
        self.declare_parameter('setpoint_rate_hz', 50.0)

        self._target_marker_id = self.get_parameter('target_marker_id').value
        self._target_height_m = self.get_parameter('target_height_m').value
        self._kp = self.get_parameter('kp').value
        self._ki = self.get_parameter('ki').value
        self._integral_limit_mps = self.get_parameter('integral_limit_mps').value
        self._max_velocity_mps = self.get_parameter('max_velocity_mps').value
        self._marker_timeout_s = self.get_parameter('marker_timeout_s').value
        setpoint_rate_hz = self.get_parameter('setpoint_rate_hz').value

        # Latest marker offset in the body frame, and the time it was measured.
        # None means nothing has been seen yet, which is a different state from
        # having seen something and lost it.
        self._offset_forward_m = None
        self._offset_left_m = None
        self._offset_stamp_s = None

        # True if the stored offset had roll and pitch taken out of it. False
        # means an attitude sample for that instant could not be found and the
        # offset is raw - it is kept and published so the bag shows what was
        # seen, but it is not flown on.
        self._offset_levelled = False

        # The attitude that was removed, and how well it was matched to the
        # image. Diagnostic; None until a levelled detection has arrived.
        self._offset_roll_rad = None
        self._offset_pitch_rad = None
        self._offset_attitude_age_s = None

        # Attitude history, oldest first, as (receive_time_s, w, x, y, z).
        #
        # Timed on the NODE'S clock at the moment the message arrived, NOT on
        # msg.timestamp. PX4's uORB timestamps are microseconds since the flight
        # controller booted, which is a different epoch from anything ROS knows
        # about - matching an image stamp against one would be meaningless
        # arithmetic that produces a number anyway. Receive time is on the same
        # clock the image header is on, and attitude comes straight off the
        # uXRCE-DDS link with none of the image pipeline's latency in front of
        # it, so the two are directly comparable.
        self._attitude_buffer = deque(maxlen=ATTITUDE_BUFFER_LEN)

        # Integral accumulators, in metre-seconds. Separate per axis.
        self._integral_forward = 0.0
        self._integral_left = 0.0

        # Latest from PX4. None until the first message; the node will not
        # command lateral motion without a yaw to rotate the command by.
        self._heading_rad = None

        # EKF z, metres, NED - so NEGATIVE above the takeoff point. z_valid was
        # true in every message of the flight that exposed the dist_bottom
        # problem, and indoors on hardware this same field is fed by the
        # rangefinder the airframe must carry to hold position at all. Same
        # code, better estimate underneath it, no sim-specific branch.
        self._z_m = None

        # Terrain estimate. NOT used for control - kept only so the log carries
        # it and a bag can be scored against it later. Do not close a loop on
        # this without checking dist_bottom_valid first.
        self._dist_bottom_m = None

        # Edge detection for the log lines, so acquiring and losing the marker
        # each print once instead of at 50 Hz.
        self._marker_available = False

        # Our own topic: short and relative, remapped at launch, so this file is
        # identical in simulation and on the real drone.
        self._detections_sub = self.create_subscription(
            MarkerDetectionArray,
            'marker_detections',
            self._on_detections,
            qos_profile_sensor_data,
        )

        # PX4's topics are absolute, unlike ours. They are not ours to name -
        # the uXRCE-DDS bridge publishes them at fixed paths - so namespacing
        # them would simply break the link.
        #
        # PX4 publishes and subscribes best effort with transient-local
        # durability. A reliable subscriber cannot receive from a best-effort
        # publisher, so matching this profile is not a preference, it is the
        # condition for the topics connecting at all.
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self._on_local_position,
            px4_qos,
        )

        # Attitude, to take the tilt out of the marker offset. Same QoS and the
        # same reason as above.
        #
        # This subscription is only as good as PX4's horizon calibration. On an
        # uncalibrated airframe PX4 reported 2.37 degrees of tilt while Gazebo
        # ground truth said 0.09, and feeding that in made the offset WORSE than
        # assuming level - 10.4 cm of standing error against 0.5 cm, which is
        # 2.3 m x tan(2.37 deg) and no coincidence. QGC's Level Horizon
        # Calibration removed it; PX4 and truth then agreed to 0.03 degrees.
        # If a flight ever shows a standing offset that scales with height,
        # check this before touching a gain.
        self._attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self._on_attitude,
            px4_qos,
        )

        self._offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos
        )
        self._setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos
        )

        # Diagnostic only, and ours, so it goes back to a short relative name
        # and default reliable QoS at depth 1. Published unconditionally rather
        # than gated on a subscriber the way the detector's debug image is -
        # this is a few dozen bytes against that topic's 48 MB/s, and the whole
        # point is that it is in the bag whether or not anyone was watching
        # live.
        self._status_pub = self.create_publisher(
            StationKeeperStatus, 'station_keeper_status', 1
        )

        self._dt_s = 1.0 / setpoint_rate_hz
        self._timer = self.create_timer(self._dt_s, self._publish_setpoint)

        # On the record every run. Gains that were guessed at and then forgotten
        # are how a flight becomes unreproducible, and the I gain in particular
        # should be visible in the log of any flight where it was switched on.
        self.get_logger().info(
            'station keeper: marker id %d, hold %.2f m above takeoff point '
            '(position setpoint on z), kp %.3f, ki %.3f, max %.2f m/s, '
            'timeout %.2f s, setpoints at %.1f Hz'
            % (
                self._target_marker_id,
                self._target_height_m,
                self._kp,
                self._ki,
                self._max_velocity_mps,
                self._marker_timeout_s,
                setpoint_rate_hz,
            )
        )

        if self._ki != 0.0:
            self.get_logger().warn(
                'integral term is ENABLED (ki %.3f). Watch for overshoot and '
                'for windup across a marker dropout.' % self._ki
            )

        self.get_logger().info(
            'this node does not arm and does not switch modes. Stream is '
            'running; put the vehicle into Offboard yourself once it is '
            'airborne and holding.'
        )

    def _on_local_position(self, msg):
        """Yaw, to rotate the command into NED, and altitude to hold."""
        self._heading_rad = msg.heading

        # z_valid, not dist_bottom_valid. This is the estimator's own vertical
        # position and it is valid whenever the EKF is running at all.
        self._z_m = msg.z if msg.z_valid else None

        # Logged, never acted on. See the note where it is stored.
        self._dist_bottom_m = msg.dist_bottom if msg.dist_bottom_valid else None

    def _on_attitude(self, msg):
        """Keep a short history of attitude, timestamped on arrival."""
        # q is (w, x, y, z), body FRD to NED. Stored rather than used because
        # the detection that needs it has not arrived yet - it is still in the
        # camera pipeline, about 0.18 s behind.
        self._attitude_buffer.append(
            (
                self.get_clock().now().nanoseconds * 1e-9,
                float(msg.q[0]),
                float(msg.q[1]),
                float(msg.q[2]),
                float(msg.q[3]),
            )
        )

    def _attitude_at(self, stamp_s):
        """The attitude sample nearest stamp_s, or None if nothing is close enough."""
        if not self._attitude_buffer:
            return None

        # Nearest, not newest, and not interpolated. Nearest is within ~6 ms of
        # the truth at 80 Hz, which is under a tenth of a degree at the tilt
        # rates seen; slerping between two samples would be more code for less
        # than the measurement noise.
        nearest = min(self._attitude_buffer, key=lambda s: abs(s[0] - stamp_s))
        if abs(nearest[0] - stamp_s) > ATTITUDE_MATCH_TOLERANCE_S:
            return None
        return nearest

    def _on_detections(self, msg):
        """Store the target marker's offset in the body frame. No commanding here."""
        # An empty array is meaningful and is NOT the same as no message: it
        # says the camera looked at this instant and saw nothing. Both leave the
        # stored offset untouched and let it go stale on its own, which is the
        # only place marker-lost is decided - in the timer, against the clock.
        for detection in msg.detections:
            if detection.id != self._target_marker_id:
                continue

            position = detection.pose.position
            forward, left, up = optical_to_body_flu(position.x, position.y, position.z)

            # The image's acquisition time, carried through from the detector -
            # not the time this callback ran. The difference is the ~106 ms of
            # pipeline staleness, and measuring age against arrival time would
            # hide exactly the quantity worth watching.
            stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            # The up component is needed from here on. It used to be discarded,
            # which was fine while the frame was assumed level: only forward and
            # left were the error signal. Levelling is a 3D rotation and the
            # vertical component feeds the horizontal ones, so throwing it away
            # is what made the level assumption unavoidable rather than chosen.
            sample = self._attitude_at(stamp_s)
            if sample is None:
                # No attitude for that instant. Store the raw offset anyway, so
                # a bag records what the camera saw, but mark it so the tick
                # refuses to command on it. Flying the raw number is what the
                # old code did and it is the thing being fixed; the alternative
                # of silently falling back to it would make the correction
                # invisible exactly when it failed.
                self._offset_levelled = False
                self._offset_roll_rad = None
                self._offset_pitch_rad = None
                self._offset_attitude_age_s = None
            else:
                attitude_t, qw, qx, qy, qz = sample
                forward, left, up = level_body_flu(forward, left, up, qw, qx, qy, qz)
                self._offset_levelled = True
                self._offset_roll_rad, self._offset_pitch_rad = (
                    roll_pitch_from_quaternion(qw, qx, qy, qz)
                )
                self._offset_attitude_age_s = stamp_s - attitude_t

            self._offset_forward_m = forward
            self._offset_left_m = left
            self._offset_stamp_s = stamp_s
            return

    def _marker_age_s(self):
        """Seconds since the target marker was last seen, or None if never."""
        if self._offset_stamp_s is None:
            return None
        return self.get_clock().now().nanoseconds * 1e-9 - self._offset_stamp_s

    def _publish_setpoint(self):
        """Publish one setpoint. Runs every tick, unconditionally."""
        # OffboardControlMode goes out first and always, before any decision
        # about what to command. It is the heartbeat: PX4 hands the aircraft
        # back if it stops arriving faster than 2 Hz, and that must not depend
        # on whether a marker was seen, whether PX4 has sent us a position yet,
        # or on anything else that can be false.
        #
        # BOTH position and velocity are true, because this setpoint is mixed:
        # velocity drives x and y, position drives z. The flags say which of
        # PX4's loops may be fed at all; the NaNs in the setpoint itself pick
        # which axis each one actually drives. Everything below - attitude,
        # rates, mixing - stays PX4's either way.
        mode = OffboardControlMode()
        mode.timestamp = self._px4_timestamp()
        mode.position = True
        mode.velocity = True
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        self._offboard_mode_pub.publish(mode)

        age_s = self._marker_age_s()
        have_marker = age_s is not None and age_s <= self._marker_timeout_s

        # Three conditions, not two. A marker in time, a heading to rotate the
        # command by, and an offset that has had the tilt taken out of it. The
        # third is new and it can only fail at startup in practice, since
        # attitude and local position arrive over the same link at similar
        # rates - but "in practice" is not a guarantee, and an offset that has
        # not been levelled carries an error that scales with height and points
        # in whatever direction the airframe happened to be leaning.
        can_command = (
            have_marker
            and self._heading_rad is not None
            and self._offset_levelled
        )

        if can_command:
            v_forward, v_left, p_fwd, p_left, i_fwd, i_left = self._lateral_command()
        else:
            p_fwd = p_left = i_fwd = i_left = 0.0
            # MARKER LOST - PLACEHOLDER BEHAVIOUR, NOT A DECISION.
            #
            # Stop laterally and hold height. It is the least surprising thing
            # the drone can do and it is safe, but "what should happen when the
            # marker is lost" is still an open design question, and this is
            # standing in for the answer. Other projects diverge sharply here:
            # ARK aborts the flight mode outright, ArduPilot exposes a whole
            # policy - retry a bounded number of times, then either land anyway
            # or hover - and PX4's estimator de-initialises rather than coast.
            # All three treat it as a policy, none as a gain.
            #
            # The setpoint stream deliberately continues through this. Going
            # quiet would drop offboard and hand PX4 the aircraft, which is a
            # far larger event than losing sight of a marker.
            #
            # Altitude genuinely IS held here, now that z is a position
            # setpoint. Laterally it is not: zero velocity stops pushing, it
            # does not hold a position, so the drone will drift with whatever
            # bias PX4's velocity loop has - the same mechanism that sank it
            # vertically, just slower and sideways. Fixing that means a
            # position setpoint on x and y too, captured at the moment the
            # marker was lost, and that is a real answer to the open question
            # rather than a placeholder, so it waits for the decision.
            v_forward, v_left = 0.0, 0.0

            # Integral reset, not freeze. Across a dropout the accumulator is a
            # wound spring pointing wherever it was last loaded, and unloading
            # it on reacquisition is the failure mode that flies a drone into a
            # wall. Freezing preserves it for a marker that comes back where it
            # left; resetting assumes nothing. With ki at zero this costs
            # nothing, and it is the behaviour worth having by default the day
            # the gain is turned on.
            self._integral_forward = 0.0
            self._integral_left = 0.0

        self._log_marker_transitions(have_marker, age_s)

        v_north, v_east = self._body_to_ned(v_forward, v_left)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp = self._px4_timestamp()

        # NaN means "not commanding this axis". x and y are driven by velocity
        # and so their position entries are NaN; z is driven by position and so
        # its velocity entry is NaN. Writing zeros instead of NaN would command
        # the origin at speed on one axis and zero vertical velocity on the
        # other - the second of which is exactly the bug this replaces.
        z_sp = self._altitude_setpoint()
        setpoint.position = [float('nan'), float('nan'), z_sp]
        setpoint.velocity = [v_north, v_east, float('nan')]

        # Yaw left uncommanded. Nothing about the task needs a heading: the
        # marker's position is read in the body frame and rotated by whatever
        # yaw the drone happens to hold, so it may sit at any heading and the
        # loop is unaffected. Commanding one would add a rotation the task never
        # asked for, and in Stage 3 the swarm frame is built on the central
        # bot's heading rather than the drone's, so the drone yawing freely is a
        # property of the design, not a gap in it.
        setpoint.yaw = float('nan')
        setpoint.yawspeed = float('nan')

        self._setpoint_pub.publish(setpoint)

        self._publish_status(
            have_marker, can_command, age_s,
            p_fwd, p_left, i_fwd, i_left,
            v_forward, v_left, v_north, v_east, z_sp,
        )

    def _publish_status(self, have_marker, can_command, age_s,
                        p_fwd, p_left, i_fwd, i_left,
                        v_forward, v_left, v_north, v_east, z_sp):
        """Record what this tick decided, so nobody has to reconstruct it."""
        nan = float('nan')
        s = StationKeeperStatus()

        # The node's own clock, not the detection's. Under use_sim_time this is
        # sim time, which is the same timebase as every PX4 and Gazebo stamp in
        # the bag, so nothing needs aligning afterwards.
        s.header.stamp = self.get_clock().now().to_msg()
        s.header.frame_id = 'base_link'

        s.target_marker_id = self._target_marker_id
        s.marker_visible = bool(have_marker)
        s.marker_lost = not have_marker
        s.commanding = bool(can_command)
        s.marker_age_s = nan if age_s is None else float(age_s)

        # NaN rather than zero when there is no marker: zero is a real offset
        # meaning "dead centre", and writing it here would put a perfect
        # station-keep in the bag at exactly the moments the marker was gone.
        s.offset_forward_m = (
            nan if self._offset_forward_m is None or not have_marker
            else float(self._offset_forward_m)
        )
        s.offset_left_m = (
            nan if self._offset_left_m is None or not have_marker
            else float(self._offset_left_m)
        )

        # These are genuinely zero when not commanding - the command really was
        # zero - so zero is the honest value and NaN would be wrong.
        s.p_forward_mps = float(p_fwd)
        s.p_left_mps = float(p_left)
        s.i_forward_mps = float(i_fwd)
        s.i_left_mps = float(i_left)
        s.cmd_forward_mps = float(v_forward)
        s.cmd_left_mps = float(v_left)
        s.cmd_north_mps = float(v_north)
        s.cmd_east_mps = float(v_east)

        # The attitude that was taken out of the offset above. NaN when there is
        # no marker, and NaN when there is one that could not be levelled -
        # which is the same condition that forced commanding to false, so the
        # two always agree in the bag.
        s.roll_deg = (
            nan if not have_marker or self._offset_roll_rad is None
            else math.degrees(self._offset_roll_rad)
        )
        s.pitch_deg = (
            nan if not have_marker or self._offset_pitch_rad is None
            else math.degrees(self._offset_pitch_rad)
        )
        s.attitude_age_s = (
            nan if not have_marker or self._offset_attitude_age_s is None
            else float(self._offset_attitude_age_s)
        )

        s.heading_rad = nan if self._heading_rad is None else float(self._heading_rad)
        s.z_m = nan if self._z_m is None else float(self._z_m)
        s.altitude_setpoint_m = float(z_sp)

        self._status_pub.publish(s)

    def _lateral_command(self):
        """Lateral command from the stored marker offset.

        Returns (v_forward, v_left, p_forward, p_left, i_forward, i_left), all
        m/s. The four extra terms are not used to fly - they exist so the
        status message can record which part of the command came from which
        term without anyone recomputing it from the gains afterwards.
        """
        # The offset IS the error. The marker sitting 0.4 m forward means the
        # drone is 0.4 m behind where it should be, so the command points
        # straight at it and no sign flip is needed. (ARK computes drone minus
        # tag and negates the result, which is the same thing said twice.)
        error_forward = self._offset_forward_m
        error_left = self._offset_left_m

        # Integrating on the timer's fixed dt, not on the camera interval. Real
        # elapsed time either way, but a regular one, so the term does not
        # breathe with the frame rate.
        self._integral_forward += error_forward * self._dt_s
        self._integral_left += error_left * self._dt_s

        i_forward = _clamp(self._ki * self._integral_forward, self._integral_limit_mps)
        i_left = _clamp(self._ki * self._integral_left, self._integral_limit_mps)

        p_forward = self._kp * error_forward
        p_left = self._kp * error_left

        v_forward = p_forward + i_forward
        v_left = p_left + i_left

        # Clamped as a vector, not per axis. Clamping each axis separately would
        # let a diagonal command reach 1.41 times the limit, and worse, would
        # bend the command away from the marker whenever one axis saturated and
        # the other did not - the drone would set off in a direction nobody
        # asked for. Scaling both together caps the speed and keeps the heading.
        speed = math.hypot(v_forward, v_left)
        if speed > self._max_velocity_mps and speed > 0.0:
            scale = self._max_velocity_mps / speed
            v_forward *= scale
            v_left *= scale

        return v_forward, v_left, p_forward, p_left, i_forward, i_left

    def _altitude_setpoint(self):
        """NED z to hold, in metres. Negative is above the takeoff point."""
        # A constant. There is no error term and no gain here because this is
        # not a controller - it is a reference handed to PX4's position loop,
        # which does the controlling.
        return -self._target_height_m

    def _body_to_ned(self, v_forward, v_left):
        """Rotate a horizontal body-frame velocity into local NED by yaw."""
        if self._heading_rad is None:
            return 0.0, 0.0

        # FLU to FRD first: PX4 thinks in forward-right-down, and this is the
        # one place in the file where the other convention appears. Left and
        # right differ by a sign, and getting it wrong sends the drone away from
        # the marker at exactly the speed it should be approaching.
        v_right = -v_left

        yaw = self._heading_rad
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        # Standard yaw rotation. heading is measured from north, so forward
        # projects onto north as cos(yaw) and onto east as sin(yaw).
        v_north = v_forward * cos_yaw - v_right * sin_yaw
        v_east = v_forward * sin_yaw + v_right * cos_yaw

        return v_north, v_east

    def _px4_timestamp(self):
        """Now, in microseconds, on the clock PX4 is using."""
        # With use_sim_time and /clock bridged from Gazebo this is sim time, and
        # PX4 in SITL takes its clock from the same place - so the two agree.
        # On hardware both are wall time. Either way the node does not branch on
        # which it is, which is the point.
        return int(self.get_clock().now().nanoseconds / 1000)

    def _log_marker_transitions(self, have_marker, age_s):
        """One line when the marker is acquired, one when it is lost."""
        if have_marker and not self._marker_available:
            self.get_logger().info(
                'marker %d acquired: %+.2f m forward, %+.2f m left'
                % (self._target_marker_id, self._offset_forward_m, self._offset_left_m)
            )
        elif not have_marker and self._marker_available:
            self.get_logger().warn(
                'marker %d lost after %.2f s - holding position'
                % (self._target_marker_id, age_s if age_s is not None else 0.0)
            )

        self._marker_available = have_marker

        if not have_marker and age_s is None:
            self.get_logger().info(
                'waiting for a detection of marker %d' % self._target_marker_id,
                throttle_duration_sec=5.0,
            )

        if self._heading_rad is None:
            self.get_logger().warn(
                'no VehicleLocalPosition yet - not commanding lateral motion. '
                'Check the uXRCE-DDS agent is running.',
                throttle_duration_sec=5.0,
            )

        if have_marker and not self._offset_levelled:
            self.get_logger().warn(
                'marker seen but no attitude within %.0f ms of the image - not '
                'commanding. Check /fmu/out/vehicle_attitude is arriving.'
                % (ATTITUDE_MATCH_TOLERANCE_S * 1000.0),
                throttle_duration_sec=5.0,
            )


def _clamp(value, limit):
    """Hold value within plus or minus limit."""
    return max(-limit, min(limit, value))


def main(args=None):
    rclpy.init(args=args)
    node = StationKeeper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
