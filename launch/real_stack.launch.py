#!/usr/bin/env python3
"""Our two nodes on the REAL drone: marker_detector and station_keeper.

The sim twin is corvid_stack.launch.py. The two are kept completely separate on
purpose, and neither file has a sim/real switch.

What differs from the sim file, and why:
  - use_sim_time is False on both nodes. There is no /clock on the drone; with
    it True the station keeper's clock never moves and the marker reads as lost
    for the whole flight.
  - No remaps. camera.launch.py publishes /image_raw and /camera_info, which are
    exactly the relative names the detector subscribes to.
  - target_height_m defaults to 1.7, the provisional hold height, under the
    HereFlow ToF's ~2 m indoor limit.
  - max_velocity_mps defaults to 0.3. At 1.7 m the C200 sees about 0.5 m either
    side, so kp 0.5 can never ask for more than ~0.25 m/s from a real
    detection. The clamp only bites on a bad pose, and that is exactly when it
    should be small.

Order of operations on the drone, all inside the container on the Orin:

  1. MicroXRCEAgent on the serial link
  2. camera.launch.py
  3. colcon build --packages-select corvid_msgs corvid_perception corvid_control
  4. this file
  5. pilot takes off in Position, hovers near target_height_m, flips to Offboard

Setpoints must already be streaming before PX4 will accept Offboard, so this
file goes up on the ground, before takeoff. It does not arm and does not switch
modes. The one mode change it can make is Land, after lost_hover_s blind, and
only while PX4 is in Offboard.

  ros2 launch src/corvid/launch/real_stack.launch.py marker_size_m:=<measured>
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    # ---- perception --------------------------------------------------------
    # marker_size_m is the black square of the PRINTED tag, measured with the
    # ruler, not the plate. Range scales linearly with it.
    marker_detector = Node(
        package='corvid_perception',
        executable='marker_detector',
        name='marker_detector',
        parameters=[{
            'use_sim_time': False,
            'marker_size_m': float(arg('marker_size_m')),
        }],
        output='screen',
    )

    # ---- control -----------------------------------------------------------
    # target_height_m is metres above where the EKF set its origin, as a
    # POSITION setpoint on z. Entering Offboard far from it means PX4 climbs or
    # descends to it, so have the pilot hover near it before the switch.
    station_keeper = Node(
        package='corvid_control',
        executable='station_keeper',
        name='station_keeper',
        parameters=[{
            'use_sim_time': False,
            'target_marker_id': int(arg('target_marker_id')),
            'target_height_m': float(arg('target_height_m')),
            'kp': float(arg('kp')),
            'ki': float(arg('ki')),
            'integral_limit_mps': float(arg('integral_limit_mps')),
            'max_velocity_mps': float(arg('max_velocity_mps')),
            'marker_timeout_s': float(arg('marker_timeout_s')),
            'lost_hover_s': float(arg('lost_hover_s')),
            'setpoint_rate_hz': float(arg('setpoint_rate_hz')),
        }],
        output='screen',
    )

    return [marker_detector, station_keeper]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'marker_size_m', default_value='0.136',
            description='Black square of the PRINTED tag, in metres. 0.136 is the '
                        'A4 print measured with a ruler on 5 Oct (the PDF says 0.144; '
                        'the printer shrank it). Re-measure any new print.'),
        DeclareLaunchArgument(
            'target_marker_id', default_value='0',
            description='Which DICT_4X4_50 id to track.'),
        DeclareLaunchArgument(
            'target_height_m', default_value='1.7',
            description='Hold height above the EKF origin. ToF is unreliable above ~2 m.'),
        DeclareLaunchArgument(
            'kp', default_value='0.5',
            description='Lateral proportional gain. Same as sim.'),
        DeclareLaunchArgument(
            'ki', default_value='0.0',
            description='Integral gain. Off.'),
        DeclareLaunchArgument(
            'integral_limit_mps', default_value='0.3',
            description='Clamp on the integral term.'),
        DeclareLaunchArgument(
            'max_velocity_mps', default_value='0.3',
            description='Lateral velocity clamp. Real detections never need more '
                        'than ~0.25 at 1.7 m; this caps what a bad pose can do.'),
        DeclareLaunchArgument(
            'marker_timeout_s', default_value='0.5',
            description='Marker-lost threshold.'),
        DeclareLaunchArgument(
            'lost_hover_s', default_value='5.0',
            description='Blind hold before commanding Land. Only counts in Offboard.'),
        DeclareLaunchArgument(
            'setpoint_rate_hz', default_value='50.0',
            description='Fixed-rate setpoint timer. PX4 drops Offboard below 2 Hz.'),
        OpaqueFunction(function=_setup),
    ])
