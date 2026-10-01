#!/usr/bin/env python3
"""Drives aruco_marker_moving back and forth in a straight line.

Sim test fixture. Nothing in corvid_perception or corvid_control knows this
exists. It publishes a Twist that a ros_gz bridge carries into Gazebo's
VelocityControl plugin, which sets the model's velocity directly.

IT DOES NOT MOVE ON STARTUP. It comes up disabled, holding the marker still
with a zero twist, and waits to be told to go:

  ros2 service call /marker_mover/enable std_srvs/srv/SetBool "{data: true}"
  ros2 service call /marker_mover/enable std_srvs/srv/SetBool "{data: false}"

A service rather than a keypress because under `ros2 launch` a node has no
usable stdin -- launch multiplexes it -- so input() would hang with nothing to
type into. The service also means you can stop and restart the path as many
times as you like without restarting the node, and each enable restarts the
path from the marker's current position.

Patterns:
  line   Back and forth along one axis, reversing every
         line_length_m / speed_mps seconds.
  cross  A plus shape centred on the spawn point. Out half a leg on x, a
         full leg back the other way, half a leg home to the middle -- that
         is front and back -- then the same three moves on y for left and
         right, then repeat. Four leg-times per cycle, so 40 s at the
         defaults. The `axis` parameter does not apply and is ignored.
  stop   Zero twist always, whatever the enable state.

Only one axis is ever moving at a time, in `line` and `cross` alike. That is
deliberate: it keeps each leg readable on its own out of a bag. A diagonal --
both axes at once, which is closer to what Stage 3 actually asks of the
controller -- is a different pattern and is not written yet.

The marker never rotates -- angular velocity is always zero. Two things follow
from that, both useful:

  - The body-frame vs world-frame question for linear velocity does not matter.
    The marker is spawned at yaw 0 and never turns, so its own x axis and the
    world's stay aligned for good.
  - In-plane marker rotation stays constant, which is what Stage 3 reads for
    bot headings. A test that changes only position is the cleaner test.

Speed defaults to 0.15 m/s. Pipeline latency is 0.188 s median, so even this
slow the marker is ~2.8 cm ahead of where the controller thinks it is before
any control error at all.

line_length_m defaults to 1.5 m, which is 0.75 m either side of the spawn
point -- inside the measured ~2 m detection envelope with room to spare. The
marker-lost branch is still a placeholder, so do not go wide yet.

The legs are TIMED, not position-fed-back. A leg boundary can only land on a
publish tick, so at 10 Hz each one is up to 100 ms late and the marker creeps
roughly a centimetre off centre per boundary, accumulating. Over a few minutes
that is nothing next to the detection envelope; over twenty it wanders. The
real fix is to close the loop on the marker's ground-truth pose, which is
bridged already -- worth doing only if the drift ever actually matters.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import SetBool

PATTERNS = ('line', 'cross', 'stop')
AXES = ('x', 'y')


class MarkerMover(Node):

    def __init__(self):
        super().__init__('marker_mover')

        # VelocityControl listens on /model/<model_name>/cmd_vel and our SDF
        # sets that topic explicitly. Change the model name and this follows.
        self.declare_parameter('model_name', 'aruco_marker_moving')
        self.declare_parameter('pattern', 'line')
        self.declare_parameter('axis', 'x')
        self.declare_parameter('speed_mps', 0.15)
        self.declare_parameter('line_length_m', 1.5)
        self.declare_parameter('publish_rate_hz', 10.0)

        model = self.get_parameter('model_name').value
        self._pattern = self.get_parameter('pattern').value
        self._axis = self.get_parameter('axis').value
        self._speed = float(self.get_parameter('speed_mps').value)
        self._length = float(self.get_parameter('line_length_m').value)
        rate_hz = float(self.get_parameter('publish_rate_hz').value)

        if self._pattern not in PATTERNS:
            raise ValueError(f'pattern must be one of {PATTERNS}, got {self._pattern!r}')
        if self._axis not in AXES:
            raise ValueError(f'axis must be one of {AXES}, got {self._axis!r}')

        # Depth 1, default reliable QoS: the newest command is the only one
        # worth having. A dropped one would leave the marker coasting on
        # whatever velocity it was last given.
        self._pub = self.create_publisher(Twist, f'/model/{model}/cmd_vel', 1)

        # How long one leg takes. Half of it is spent getting from the spawn
        # point to one end, so the first leg is deliberately a half leg --
        # otherwise the path would be offset entirely to one side.
        self._leg_s = self._length / self._speed if self._speed > 0.0 else 0.0

        # Disabled until asked. _t0 is set on every enable, not here, so the
        # path always begins at the moment you say go rather than at the moment
        # the node happened to start.
        self._enabled = False
        self._t0 = self.get_clock().now()

        # Relative name, so it resolves to /marker_mover/enable.
        self._enable_srv = self.create_service(SetBool, '~/enable', self._on_enable)

        # The timer runs from the start even while disabled: it keeps
        # publishing a zero twist, which actively holds the marker still. That
        # matters because VelocityControl makes the marker COAST on whatever it
        # was last given, so 'publish nothing' is not the same as 'stay put'.
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)

        if self._pattern == 'cross':
            shape = (f'plus shape centred on spawn, {self._leg_s:.1f} s per leg -> '
                     f'{4.0 * self._leg_s:.1f} s per full cycle (axis ignored)')
        else:
            shape = f'axis={self._axis}, reverses every {self._leg_s:.1f} s sim time'
        self.get_logger().info(
            f'pattern={self._pattern} speed={self._speed} m/s '
            f'length={self._length} m -> {shape}, '
            f'publishing on /model/{model}/cmd_vel at {rate_hz} Hz'
        )
        self.get_logger().info(
            'HOLDING. The marker will not move until you say so:\n'
            '  ros2 service call /marker_mover/enable std_srvs/srv/SetBool "{data: true}"'
        )

    def _on_enable(self, request, response):
        if request.data and not self._enabled:
            # Restart the path from here, so enabling mid-session does not jump
            # into the middle of a leg.
            self._t0 = self.get_clock().now()
        self._enabled = request.data
        state = 'MOVING' if self._enabled else 'HOLDING'
        self.get_logger().info(f'{state}')
        response.success = True
        response.message = state
        return response

    def _elapsed_s(self) -> float:
        # get_clock() honours use_sim_time. Sim runs at RTF ~0.81, so timing
        # the legs off wall time would make every one of them come out short.
        return (self.get_clock().now() - self._t0).nanoseconds * 1e-9

    def _velocity(self):
        """Return (vx, vy) in m/s. (0.0, 0.0) means hold still."""
        if not self._enabled or self._pattern == 'stop' or self._leg_s <= 0.0:
            return (0.0, 0.0)

        t = self._elapsed_s()
        half = self._leg_s / 2.0

        if self._pattern == 'line':
            v = self._line_velocity(t, half)
            return (v, 0.0) if self._axis == 'x' else (0.0, v)

        return self._cross_velocity(t, half)

    def _line_velocity(self, t: float, half: float) -> float:
        # First leg is a HALF leg so the path ends up centred on the spawn
        # point: out half a length, then full lengths back and forth.
        if t < half:
            return self._speed

        # After that half leg, flip direction every full leg.
        leg = int((t - half) // self._leg_s)
        return -self._speed if leg % 2 == 0 else self._speed

    def _cross_velocity(self, t: float, half: float):
        # One cycle is four leg-times: two spent on x, two on y. Wrapping with
        # a modulo rather than counting legs means the pattern is periodic by
        # construction and cannot drift out of phase however long it runs.
        cycle = 4.0 * self._leg_s
        u = t % cycle

        # x arm: out half a leg, a full leg back, half a leg home to centre.
        if u < half:
            return (self._speed, 0.0)
        if u < half + self._leg_s:
            return (-self._speed, 0.0)
        if u < 2.0 * self._leg_s:
            return (self._speed, 0.0)

        # y arm: the same three moves, one axis over.
        u -= 2.0 * self._leg_s
        if u < half:
            return (0.0, self._speed)
        if u < half + self._leg_s:
            return (0.0, -self._speed)
        return (0.0, self._speed)

    def _tick(self):
        cmd = Twist()
        cmd.linear.x, cmd.linear.y = self._velocity()
        # angular stays zero: the marker slides, it never turns.
        self._pub.publish(cmd)

    def stop(self):
        # The marker keeps the last velocity it was given, so leaving without
        # zeroing means it drifts off forever after Ctrl+C. Sent a few times
        # because this is best-effort on the way out.
        for _ in range(3):
            self._pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = MarkerMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
