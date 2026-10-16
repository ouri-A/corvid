#!/usr/bin/env python3
"""Spin sequence for the spin test: checks every bot is talking, then turns the
chosen bots in place one at a time while a bag records.

Runs inside the vx500 container, from the workspace root:

    source install/setup.bash && python3 src/corvid/scripts/spin_sequence.py
    python3 src/corvid/scripts/spin_sequence.py --bots 1 --turns 6

1. Waits up to 30 s to hear /botN/state from every bot in --expect, printing each
   one as it comes up. If one never shows, it says which and stops.
2. For each bot in --bots, in order: still for --still-s, then --turns turns of
   --angle-deg to the left, each followed by --still-s of stillness. A turn counts
   as finished when /botN/state goes back to 0 (idle).

The turn publishers live for the whole run, so a bag recorder catches every
/botN/turn message (`ros2 topic pub --once` was missing from spin_1_oct15).
Then run scripts/spin_test.py on the bag, once per bot.
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, UInt8

HEAR_TIMEOUT_S = 30.0    # how long to wait for every bot to show up
MATCH_TIMEOUT_S = 10.0   # how long to wait for a bot's /turn subscriber to match
START_TIMEOUT_S = 2.0    # a turn must show state 1 (turning) within this
FINISH_TIMEOUT_S = 10.0  # ... and be back to 0 within this (firmware gives up at 5 s)
SILENT_S = 1.0           # /botN/state quiet this long: the bot dropped off or rebooted


class Spinner(Node):
    def __init__(self, expect, bots):
        super().__init__('spin_sequence')
        self.state = {}      # bot -> last state value
        self.heard = {}      # bot -> time.monotonic() of the last state message
        for b in expect:
            # The bots publish state best effort, so the subscriber must be too.
            self.create_subscription(UInt8, '/bot%d/state' % b,
                                     lambda m, b=b: self._on_state(b, m),
                                     qos_profile_sensor_data)
        # Reliable (the default), matching the bots' /turn subscriber.
        self.turn = {b: self.create_publisher(Float32, '/bot%d/turn' % b, 10) for b in bots}

    def _on_state(self, b, msg):
        if b not in self.heard:
            print('  bot%d is up (state %d)' % (b, msg.data), flush=True)
        self.state[b] = msg.data
        self.heard[b] = time.monotonic()


def spin_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def wait_for(node, cond, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if cond():
            return True
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--expect', type=int, nargs='+', default=[0, 1, 2],
                    help='bots that must be talking before anything moves')
    ap.add_argument('--bots', type=int, nargs='+', default=[1, 2],
                    help='bots to spin, one after another')
    ap.add_argument('--turns', type=int, default=4)
    ap.add_argument('--angle-deg', type=float, default=90.0,
                    help='per turn, positive = left (counter-clockwise)')
    ap.add_argument('--still-s', type=float, default=3.0)
    a, _ = ap.parse_known_args()

    rclpy.init()
    node = Spinner(sorted(set(a.expect) | set(a.bots)), a.bots)

    print('waiting for bots %s ...' % ' '.join(str(b) for b in a.expect), flush=True)
    if not wait_for(node, lambda: all(b in node.heard for b in a.expect), HEAR_TIMEOUT_S):
        missing = [b for b in a.expect if b not in node.heard]
        print('NOT HEARD: %s - check its power, wifi, and the agent log'
              % ' '.join('bot%d' % b for b in missing))
        sys.exit(1)
    print('all %d bots talking' % len(a.expect), flush=True)

    def silent(b):
        return time.monotonic() - node.heard.get(b, 0.0) > SILENT_S

    for b in a.bots:
        pub = node.turn[b]
        if not wait_for(node, lambda: pub.get_subscription_count() > 0, MATCH_TIMEOUT_S):
            print('bot%d: /bot%d/turn never matched a subscriber - skipping it' % (b, b))
            continue
        print('\nbot%d: still %.0f s' % (b, a.still_s), flush=True)
        spin_for(node, a.still_s)
        for i in range(1, a.turns + 1):
            t0 = time.monotonic()
            pub.publish(Float32(data=math.radians(a.angle_deg)))
            started = wait_for(node, lambda: node.state.get(b) == 1, START_TIMEOUT_S)
            if not started:
                print('bot%d turn %d/%d: never started turning (state %s)%s'
                      % (b, i, a.turns, node.state.get(b),
                         ', and the bot has gone quiet' if silent(b) else ''))
                break
            done = wait_for(node, lambda: node.state.get(b) == 0 and not silent(b),
                            FINISH_TIMEOUT_S)
            if not done:
                print('bot%d turn %d/%d: not idle after %.0f s%s' % (
                    b, i, a.turns, FINISH_TIMEOUT_S,
                    ' - the bot has gone quiet (rebooted?)' if silent(b) else ''))
                break
            print('bot%d turn %d/%d: done in %.1f s, still %.0f s'
                  % (b, i, a.turns, time.monotonic() - t0, a.still_s), flush=True)
            spin_for(node, a.still_s)

    print('\nsequence finished - Ctrl-C the bag now, then run the analyse window')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
