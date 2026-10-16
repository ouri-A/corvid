"""Corvid's pilot - drive one bot by hand from the keyboard, send it looking for
the target, and hand it over to the coordinator.

    ros2 run corvid_swarm pilot

Keys, one press each (a terminal cannot tell when a key is let go, so nothing
here depends on holding a key down):

    w / s     drive straight forward / back, until space or another key
    a / d     turn left / right by turn_step_deg (15)
    q / e     turn left / right by big_turn_deg (90)
    space     stop everything - the bot, a search, the coordinator
    g         go: drive straight until the target is in view, stop, then hand
              the bot to the coordinator
    c         hand the bot to the coordinator now (target already in view)
    m         take the bot back from the coordinator (any w a s d q e does too)
    h         print these keys
    x         quit (the bot is stopped)

Only two kinds of command ever reach the bot, the same two the coordinator
uses: a relative turn on /botN/turn, measured off on the bot's own gyro, and a
straight drive streamed on /botN/drive at drive_speed_mps, which the bot holds
on its gyro heading. No cmd_vel, so no arcs.

Modes, shown at the start of the status line:
    MANUAL        the keys drive the bot
    SEARCHING     driving straight, watching for the target
    HANDING OVER  stopped, waiting for the bot to settle, calling /coordinator/start
    AUTO          the coordinator has the bot; the pilot sends nothing
A run that ends (arrived or gave up) drops back to MANUAL by itself.

The status line also shows 'age': this machine's clock minus the image's stamp.
At home it reads about the camera's lag. In the lab the image is stamped on the
Orin, so it shows the difference between the two clocks as well - anything far
from ~0.3 s means they are not synced, which is why the lab runs the
coordinator with arrival_latency_s.

Safety: every drive is a stream. If this node dies, the stream stops and the
bot's own 0.5 s watchdog stops the wheels.
"""

import math
import os
import select
import sys
import time

import rclpy
from corvid_msgs.msg import CoordinatorStatus, MarkerDetectionArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, UInt8
from std_srvs.srv import Trigger

from corvid_swarm.coordinator import PHASE_NAMES, RUNNING_PHASES, find, stamp_seconds, tag_pose

MANUAL = 'MANUAL'
SEARCHING = 'SEARCHING'
HANDING_OVER = 'HANDING OVER'
AUTO = 'AUTO'

BOT_IDLE = 0
STATE_NAMES = {0: 'idle', 1: 'turning', 2: 'driving'}

HELP = """keys:
  w / s   straight forward / back (until space or another key)
  a / d   turn left / right {step:.0f} deg       q / e   turn left / right {big:.0f} deg
  space   stop everything
  g       go straight until the target is seen, then hand over to the coordinator
  c       hand over to the coordinator now     m   take the bot back
  h       these keys                           x   quit"""


class Quit(Exception):
    """Raised from a callback to leave rclpy.spin."""


class Keyboard:
    """Single key presses from the terminal, without Enter. Does nothing without a terminal."""

    def __init__(self):
        self._fd = None
        self._saved = None
        if sys.stdin.isatty():
            import termios
            import tty
            self._fd = sys.stdin.fileno()
            self._saved = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)    # keys arrive one by one; Ctrl-C still works

    @property
    def active(self):
        return self._fd is not None

    def read(self):
        if self._fd is None:
            return ''
        keys = ''
        while select.select([self._fd], [], [], 0.0)[0]:
            chunk = os.read(self._fd, 64)
            if not chunk:
                break
            keys += chunk.decode('utf-8', 'ignore')
        return keys

    def restore(self):
        if self._fd is not None and self._saved is not None:
            import termios
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)
            self._saved = None


class Pilot(Node):
    """Keyboard driving for one bot, a straight-line search, and the hand-over."""

    def __init__(self, keyboard=None):
        super().__init__('pilot')
        self._keyboard = keyboard

        def param(name, default):
            return self.declare_parameter(name, default).value

        self._bot_id = param('bot_id', 0)
        self._bot_marker_id = param('bot_marker_id', 0)
        self._target_marker_id = param('target_marker_id', 3)
        # Same tag-size correction as the coordinator, for the distance shown
        # and for stopping a search short of the target.
        self._detector_size = param('detector_marker_size_m', 0.136)
        self._small_size = param('small_marker_size_m', 0.095)
        self._large_ids = list(param('large_marker_ids', [0]))

        # The coordinator's speed; the firmware's 45% floor makes it ~0.1 m/s.
        self._speed = param('drive_speed_mps', 0.1)
        self._turn_step = math.radians(param('turn_step_deg', 15.0))
        self._big_turn = math.radians(param('big_turn_deg', 90.0))

        # Search: the target must be in this many frames in a row (with the
        # bot's own tag) before it counts, then the bot keeps going for
        # search_extra_s - driving at the target brings it nearer the middle
        # of the drone's view - unless it is already within search_stop_distance_m.
        self._confirm_frames = param('search_confirm_frames', 3)
        self._extra_s = param('search_extra_s', 1.0)
        self._stop_distance = param('search_stop_distance_m', 0.30)
        self._search_timeout = param('search_timeout_s', 30.0)
        self._tag_lost_s = param('tag_lost_s', 0.5)
        self._state_timeout = param('state_timeout_s', 1.0)

        # Hand-over: after the stop, wait for the bot to report idle and this
        # long more, so the coordinator's first picture is of a still bot.
        self._settle_s = param('handover_settle_s', 0.5)
        self._handover_timeout = param('handover_timeout_s', 3.0)
        coordinator = param('coordinator_name', 'coordinator')

        for name, value in (('drive_speed_mps', self._speed),
                            ('detector_marker_size_m', self._detector_size),
                            ('small_marker_size_m', self._small_size)):
            if not value > 0.0:
                raise ValueError('%s must be > 0, got %r' % (name, value))
        if self._confirm_frames < 1:
            raise ValueError('search_confirm_frames must be >= 1')

        self._bot_name = 'bot%d' % self._bot_id
        ns = '/' + self._bot_name
        best_effort = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)

        # The same QoS as the coordinator and the firmware.
        self._turn_pub = self.create_publisher(Float32, ns + '/turn', 10)
        self._drive_pub = self.create_publisher(Float32, ns + '/drive', best_effort)
        self.create_subscription(UInt8, ns + '/state', self._on_state, best_effort)
        self.create_subscription(
            MarkerDetectionArray, 'marker_detections', self._on_detections, 10)
        self.create_subscription(
            CoordinatorStatus, '/%s/status' % coordinator, self._on_coordinator, 10)
        self._start_client = self.create_client(Trigger, '/%s/start' % coordinator)
        self._stop_client = self.create_client(Trigger, '/%s/stop' % coordinator)

        self._mode = MANUAL
        self._latch = 0              # +1 forward, -1 back, 0 not driving
        self._stop_burst = 0         # drive 0.0 still to send, one per tick
        self._hold_until = 0.0       # send nothing to the bot before this
        self._turns = []             # queued turns, rad, + = left
        self._turn_busy = False
        self._turn_sent_at = None
        self._ticks = 0

        self._bot_state = None
        self._bot_state_time = None
        self._seen_busy = False

        self._frame_time = None      # arrival of the last detection message
        self._stamp_age = math.nan   # this clock minus that image's stamp
        self._bot_seen_at = None
        self._target_visible = False
        self._both_run = 0           # frames in a row with both tags
        self._distance = None        # bot tag to target tag, m, last frame with both

        self._search_started = None
        self._confirmed_at = None
        self._stop_at = None
        self._start_called_at = None
        self._auto_since = None
        self._seen_running = False
        self._stop_called_at = -math.inf

        self._coord_phase = None
        self._coord_reason = ''
        self._coord_time = None

        self._last_line = 0.0
        self.create_timer(0.05, self._tick)

        self._event('info', 'pilot for %s (tag id %d), target id %d, %.2f m/s; press h for keys'
                    % (self._bot_name, self._bot_marker_id, self._target_marker_id, self._speed))
        if self._keyboard is not None and not self._keyboard.active:
            self._event('warn', 'no terminal on stdin - keys will not be read')

    # ---------- helpers ----------

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _event(self, level, text):
        """One line in the log, above the status line."""
        if self._keyboard is not None and self._keyboard.active:
            sys.stdout.write('\r\x1b[2K')
            sys.stdout.flush()
        getattr(self.get_logger(), level)(text)

    def _scale(self, marker_id):
        return 1.0 if marker_id in self._large_ids else self._small_size / self._detector_size

    def _state_fresh(self, now):
        return (self._bot_state_time is not None
                and now - self._bot_state_time <= self._state_timeout)

    def _send_stop(self):
        self._stop_burst = 3
        self._latch = 0

    def _call_coordinator_stop(self, why):
        self._stop_called_at = self._now()
        if not self._stop_client.service_is_ready():
            self._event('warn', 'coordinator stop service not found (%s) - '
                        'stopping the bot directly' % why)
            return
        self._stop_client.call_async(Trigger.Request())

    # ---------- callbacks ----------

    def _on_state(self, msg):
        now = self._now()
        if msg.data != BOT_IDLE and self._turn_busy:
            self._seen_busy = True
        self._bot_state = msg.data
        self._bot_state_time = now

    def _on_detections(self, msg):
        now = self._now()
        self._frame_time = now
        self._stamp_age = now - stamp_seconds(msg.header.stamp)
        bot = find(msg, self._bot_marker_id)
        target = find(msg, self._target_marker_id)
        self._target_visible = target is not None
        if bot is not None:
            self._bot_seen_at = now
        if bot is not None and target is not None:
            self._both_run += 1
            p_bot, _ = tag_pose(bot, self._scale(self._bot_marker_id))
            p_target, _ = tag_pose(target, self._scale(self._target_marker_id))
            self._distance = float(math.dist(p_bot, p_target))
        else:
            self._both_run = 0

    def _on_coordinator(self, msg):
        now = self._now()
        if msg.reason != self._coord_reason and self._mode == AUTO:
            self._event('info', 'coordinator [%s] %s'
                        % (PHASE_NAMES.get(msg.phase, msg.phase), msg.reason))
        self._coord_phase = msg.phase
        self._coord_reason = msg.reason
        self._coord_time = now

    def _on_start_done(self, future):
        err = 'no response'
        try:
            result = future.result()
        except Exception as e:  # service went away mid-call
            result = None
            err = str(e)
        if self._mode != HANDING_OVER:
            # Cancelled while waiting. If it started anyway, stop it.
            if result is not None and result.success:
                self._call_coordinator_stop('hand-over was cancelled')
            return
        if result is None:
            self._to_manual('warn', 'coordinator start failed: ' + err)
            return
        if result.success:
            self._mode = AUTO
            self._auto_since = self._now()
            self._seen_running = False
            self._event('info', 'AUTO - coordinator has %s: %s' % (self._bot_name, result.message))
        else:
            self._to_manual('warn', 'coordinator refused: %s' % result.message)

    # ---------- keys ----------

    def handle_key(self, key):
        now = self._now()
        key = key.lower()
        if key in 'wsadqe':
            self._manual_key(key, now)
        elif key == ' ':
            self._stop_everything('stop (space)')
        elif key == 'g':
            self._start_search(now)
        elif key == 'c':
            self._start_handover_now(now)
        elif key == 'm':
            if self._mode == MANUAL:
                self._event('info', 'already MANUAL')
            else:
                self._take_over('manual (m)')
        elif key in 'h?':
            self._event('info', HELP.format(step=math.degrees(self._turn_step),
                                            big=math.degrees(self._big_turn)))
        elif key == 'x':
            raise Quit()

    def _take_over(self, why):
        """Back to MANUAL from any other mode, with the bot stopped."""
        if self._mode in (AUTO, HANDING_OVER):
            self._call_coordinator_stop(why)
        was = self._mode
        self._mode = MANUAL
        self._send_stop()
        self._turns = []
        # The coordinator's own stop goes out over its next three ticks; keep
        # clear of it so it cannot cancel what the keys send next.
        self._hold_until = max(self._hold_until, self._now() + 0.3)
        if was != MANUAL:
            self._event('info', 'MANUAL - took %s back from %s (%s)' % (self._bot_name, was, why))

    def _to_manual(self, level, why):
        self._mode = MANUAL
        self._send_stop()
        self._turns = []
        self._event(level, 'MANUAL - ' + why)

    def _stop_everything(self, why):
        if self._mode != MANUAL:
            self._take_over(why)
        else:
            self._send_stop()
            self._turns = []
            self._event('info', 'stopped')

    def _manual_key(self, key, now):
        if self._mode != MANUAL:
            self._take_over('key %r' % key)
            now = self._now()
        if key in 'ws':
            self._latch = 1 if key == 'w' else -1
            self._turns = []
            self._event('info', 'straight %s at %.2f m/s'
                        % ('forward' if key == 'w' else 'back', self._speed))
            return
        angle = {'a': self._turn_step, 'd': -self._turn_step,
                 'q': self._big_turn, 'e': -self._big_turn}[key]
        if self._latch != 0:
            # A stop and a turn sent together can arrive either way round, and
            # a stop arriving second cancels the turn. Stop first, turn after;
            # the queued turn goes out once the bot reports idle.
            self._send_stop()
            self._hold_until = max(self._hold_until, now + 0.4)
        self._turns.append(angle)
        self._event('info', 'turn %+.0f deg%s' % (
            math.degrees(angle), ' (queued)' if len(self._turns) > 1 or self._turn_busy else ''))

    def _start_search(self, now):
        if self._mode != MANUAL:
            self._event('warn', 'already %s - press space first' % self._mode)
            return
        problem = self._cannot_drive(now)
        if problem:
            self._event('warn', "can't search: " + problem)
            return
        self._turns = []
        self._latch = 0
        if self._both_run >= self._confirm_frames:
            self._event('info', 'target already in view - handing over')
            self._begin_handover(now)
            return
        self._mode = SEARCHING
        self._search_started = now
        self._confirmed_at = None
        self._event('info', 'SEARCHING - straight ahead until target id %d is seen (up to %.0f s)'
                    % (self._target_marker_id, self._search_timeout))

    def _start_handover_now(self, now):
        if self._mode != MANUAL:
            self._event('warn', 'already %s - press space first' % self._mode)
            return
        problem = self._cannot_drive(now)
        if problem:
            self._event('warn', "can't hand over: " + problem)
            return
        self._turns = []
        self._begin_handover(now)

    def _cannot_drive(self, now):
        if self._frame_time is None or now - self._frame_time > self._tag_lost_s:
            return 'no detections arriving on /marker_detections - is the drone stack up?'
        if self._bot_seen_at is None or now - self._bot_seen_at > self._tag_lost_s:
            return '%s tag (id %d) not in view' % (self._bot_name, self._bot_marker_id)
        if not self._state_fresh(now):
            return 'no /%s/state - is %s on the agent?' % (self._bot_name, self._bot_name)
        if not self._start_client.service_is_ready():
            return 'coordinator not running (no /coordinator/start)'
        return None

    def _begin_handover(self, now):
        self._mode = HANDING_OVER
        self._send_stop()
        self._stop_at = now
        self._start_called_at = None

    # ---------- the loop, 20 Hz ----------

    def _tick(self):
        now = self._now()
        if self._keyboard is not None:
            for key in self._keyboard.read():
                self.handle_key(key)
                now = self._now()
        self._ticks += 1

        if self._stop_burst > 0:
            self._drive_pub.publish(Float32(data=0.0))
            self._stop_burst -= 1

        self._track_turn(now)
        self._watch_coordinator(now)

        if self._mode == MANUAL:
            self._manual_tick(now)
        elif self._mode == SEARCHING:
            self._search_tick(now)
        elif self._mode == HANDING_OVER:
            self._handover_tick(now)
        elif self._mode == AUTO:
            self._auto_tick(now)

        if now - self._last_line >= 0.25:
            self._status_line(now)
            self._last_line = now

    def _stream(self, now, direction):
        """Keep a drive coming at 10 Hz; the bot's watchdog stops it otherwise."""
        if now >= self._hold_until and self._stop_burst == 0 and self._ticks % 2 == 0:
            self._drive_pub.publish(Float32(data=float(direction * self._speed)))

    def _track_turn(self, now):
        """Same test as the coordinator for a finished turn."""
        if not self._turn_busy:
            return
        idle_after = (self._bot_state == BOT_IDLE and self._bot_state_time is not None
                      and self._bot_state_time > self._turn_sent_at + 0.3)
        if idle_after and (self._seen_busy or now - self._turn_sent_at > 1.0):
            self._turn_busy = False
        elif now - self._turn_sent_at > 7.0:
            self._turn_busy = False
            self._event('warn', 'turn not reported finished after 7 s - check %s'
                        % self._bot_name)

    def _watch_coordinator(self, now):
        """A coordinator run started from anywhere means hands off."""
        running = self._coord_phase in RUNNING_PHASES
        recently_stopped = now - self._stop_called_at < 1.0
        if running and self._mode in (MANUAL, SEARCHING) and not recently_stopped:
            self._mode = AUTO
            self._latch = 0
            self._turns = []
            self._auto_since = now
            self._seen_running = True
            self._event('warn', 'AUTO - the coordinator started a run, keys are off the bot '
                        '(press m to take it back)')

    def _manual_tick(self, now):
        if self._latch != 0:
            self._stream(now, self._latch)
            return
        if (self._turns and not self._turn_busy and now >= self._hold_until
                and self._stop_burst == 0 and self._bot_state == BOT_IDLE):
            angle = self._turns.pop(0)
            if self._turn_pub.get_subscription_count() == 0:
                self._event('warn', '/%s/turn has no subscriber - the turn may be lost'
                            % self._bot_name)
            self._turn_pub.publish(Float32(data=float(angle)))
            self._turn_busy = True
            self._turn_sent_at = now
            self._seen_busy = False

    def _search_tick(self, now):
        self._stream(now, 1)
        why = None
        if self._frame_time is None or now - self._frame_time > self._tag_lost_s:
            why = 'detections stopped arriving'
        elif self._bot_seen_at is None or now - self._bot_seen_at > self._tag_lost_s:
            why = 'lost %s tag (id %d) for %.1f s' % (
                self._bot_name, self._bot_marker_id, self._tag_lost_s)
        elif not self._state_fresh(now):
            why = 'lost /%s/state' % self._bot_name
        elif now - self._search_started > self._search_timeout:
            why = 'target not found in %.0f s' % self._search_timeout
        if why:
            self._to_manual('warn', 'search stopped: ' + why)
            return

        if self._confirmed_at is None and self._both_run >= self._confirm_frames:
            self._confirmed_at = now
            self._event('info', 'target in view, %.2f m away - %.1f s more, then hand over'
                        % (self._distance, self._extra_s))
        close = (self._target_visible and self._distance is not None
                 and self._both_run > 0 and self._distance <= self._stop_distance)
        if close or (self._confirmed_at is not None and now - self._confirmed_at >= self._extra_s):
            self._event('info', 'stopping %.2f m from the target, handing over' % self._distance)
            self._begin_handover(now)

    def _handover_tick(self, now):
        if self._start_called_at is None:
            idle = (self._bot_state == BOT_IDLE and self._bot_state_time is not None
                    and self._bot_state_time > self._stop_at + 0.05)
            if idle and now - self._stop_at >= self._settle_s:
                if not self._start_client.service_is_ready():
                    self._to_manual('warn', 'coordinator not running (no /coordinator/start)')
                    return
                self._start_called_at = now
                future = self._start_client.call_async(Trigger.Request())
                future.add_done_callback(self._on_start_done)
            elif now - self._stop_at > self._handover_timeout:
                self._to_manual('warn', '%s did not report idle after the stop'
                                % self._bot_name)
        elif now - self._start_called_at > self._handover_timeout:
            self._to_manual('warn', 'no reply from /coordinator/start')

    def _auto_tick(self, now):
        if self._coord_time is None or now - self._coord_time > 2.0:
            if now - self._auto_since > 2.0:
                self._to_manual('warn', 'coordinator went silent - bot stopped')
            return
        if self._coord_phase in RUNNING_PHASES:
            self._seen_running = True
            return
        if self._seen_running or now - self._auto_since > 2.0:
            self._to_manual('info', 'coordinator finished: [%s] %s'
                            % (PHASE_NAMES.get(self._coord_phase, self._coord_phase),
                               self._coord_reason))

    # ---------- the status line ----------

    def _status_line(self, now):
        if self._keyboard is None or not self._keyboard.active:
            return
        mode = self._mode
        if mode == MANUAL:
            if self._latch:
                mode += ' fwd' if self._latch > 0 else ' back'
            elif self._turn_busy or self._turns:
                mode += ' turn' + (' (%d queued)' % len(self._turns) if self._turns else '')
        if self._state_fresh(now):
            bot = STATE_NAMES.get(self._bot_state, str(self._bot_state))
        else:
            bot = 'SILENT'
        if self._frame_time is None or now - self._frame_time > 0.5:
            tags = 'NO DETECTIONS'
        else:
            seen_bot = self._bot_seen_at is not None and now - self._bot_seen_at <= 0.5
            tags = 'tag %s, target %s' % ('ok' if seen_bot else '--',
                                          'ok' if self._target_visible else '--')
            if self._target_visible and self._distance is not None:
                tags += ' %.2f m' % self._distance
            tags += ' | age %.2f s' % self._stamp_age
        if self._coord_time is not None and now - self._coord_time < 2.0:
            coord = PHASE_NAMES.get(self._coord_phase, '?')
        else:
            coord = 'NOT RUNNING'
        line = '[%s] %s %s | %s | coord %s' % (mode, self._bot_name, bot, tags, coord)
        width = 200
        try:
            width = os.get_terminal_size().columns - 1
        except OSError:
            pass
        sys.stdout.write('\r\x1b[2K' + line[:width])
        sys.stdout.flush()

    # ---------- shutdown ----------

    def stop_on_exit(self):
        """Stop the bot, and the coordinator if it has the bot. The watchdog is the backup."""
        for _ in range(3):
            try:
                self._drive_pub.publish(Float32(data=0.0))
            except Exception:
                return
            time.sleep(0.03)
        if self._mode in (AUTO, HANDING_OVER) and self._stop_client.service_is_ready():
            try:
                future = self._stop_client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            except Exception:
                pass


def main(args=None):
    # As in the coordinator: keep the ROS context alive through Ctrl-C so the
    # stop can still be published on the way out.
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init(args=args)
    keyboard = Keyboard()
    node = None
    try:
        node = Pilot(keyboard)
        rclpy.spin(node)
    except (KeyboardInterrupt, Quit):
        pass
    finally:
        keyboard.restore()
        if node is not None:
            sys.stdout.write('\n')
            node.stop_on_exit()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
