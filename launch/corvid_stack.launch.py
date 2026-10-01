#!/usr/bin/env python3
"""Our two nodes: marker_detector and station_keeper.

Separate from sim_bringup.launch.py on purpose. colcon build copies Python into
install/, so every edit to our code needs a rebuild and a restart -- and this
way restarting our nodes does not drop the clock bridge, which would stall
everything running on use_sim_time.

Order of operations in a session:

  1. sim_bringup.launch.py            (clock, camera, ground truth, marker)
  2. commander takeoff at the pxh> prompt
  3. colcon build --packages-select corvid_msgs corvid_perception corvid_control
  4. this file
  5. switch to Offboard

Setpoints must already be streaming before PX4 will accept Offboard, so this
file goes up BEFORE you switch modes. It does not arm and does not switch
modes -- you do that.

Run it by path, from a shell that has sourced ROS and the workspace:

  ros2 launch src/corvid/launch/corvid_stack.launch.py

Every controller parameter is a launch argument, so gain sweeps need no rebuild:

  ros2 launch src/corvid/launch/corvid_stack.launch.py kp:=0.8 max_velocity_mps:=1.5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    world = arg('world')
    model = arg('model_name')
    link = arg('camera_link')
    sensor = arg('sensor')
    cam = f'/world/{world}/model/{model}/link/{link}/sensor/{sensor}'

    # ---- perception --------------------------------------------------------
    # BOTH remaps are required. The node subscribes to the short relative names
    # camera_info and image_raw; nothing publishes those, so without the remaps
    # it sits there waiting for intrinsics forever and never complains about
    # the image at all.
    #
    # use_sim_time swaps this node's clock from wall to /clock. It changes
    # nothing it publishes -- the header stamp is copied from the incoming image
    # either way -- but it is what lets the node reason about frame age. /clock
    # must already be bridged or the node runs on a zero clock and hangs.
    #
    # marker_size_m is the MARKER, not the plate: the texture is 3/4 marker and
    # 1/4 quiet zone, so the 0.2667 m plate carries 0.200 m of actual marker,
    # and 0.200 is the number pose estimation needs. Getting this wrong scales
    # range linearly and looks like a calibration problem.
    marker_detector = Node(
        package='corvid_perception',
        executable='marker_detector',
        name='marker_detector',
        remappings=[
            ('camera_info', f'{cam}/camera_info'),
            ('image_raw', f'{cam}/image'),
        ],
        parameters=[{
            'use_sim_time': True,
            'marker_size_m': float(arg('marker_size_m')),
        }],
        output='screen',
    )

    # ---- control -----------------------------------------------------------
    # No remaps. It subscribes to marker_detections relative, the same relative
    # name the detector publishes, so both resolve to /marker_detections. Do not
    # put either node in a namespace without moving the other with it.
    #
    # use_sim_time is NOT optional here, unlike on the detector. This node
    # decides marker-lost by comparing its own clock against the detection's
    # stamp, so on wall time that difference is meaningless, the marker reads as
    # permanently lost, and it streams zeros while looking broken.
    #
    # target_height_m is metres above the TAKEOFF POINT, not above terrain.
    # Matched to MIS_TAKEOFF_ALT (2.5) so entering Offboard changes nothing and
    # the first observation of a test is a clean one. Altitude is a position
    # setpoint on z -- commanding zero velocity is not a hold, and that put the
    # aircraft on the floor from 2.7 m once.
    station_keeper = Node(
        package='corvid_control',
        executable='station_keeper',
        name='station_keeper',
        parameters=[{
            'use_sim_time': True,
            'target_marker_id': int(arg('target_marker_id')),
            'target_height_m': float(arg('target_height_m')),
            'kp': float(arg('kp')),
            'ki': float(arg('ki')),
            'integral_limit_mps': float(arg('integral_limit_mps')),
            'max_velocity_mps': float(arg('max_velocity_mps')),
            'marker_timeout_s': float(arg('marker_timeout_s')),
            'setpoint_rate_hz': float(arg('setpoint_rate_hz')),
        }],
        output='screen',
    )

    return [marker_detector, station_keeper]


def generate_launch_description():
    return LaunchDescription([
        # Must match sim_bringup.launch.py. Same defaults, deliberately.
        DeclareLaunchArgument('world', default_value='default'),
        DeclareLaunchArgument('model_name', default_value='x500_mono_cam_down_0'),
        DeclareLaunchArgument('camera_link', default_value='camera_link'),
        DeclareLaunchArgument('sensor', default_value='imager'),

        DeclareLaunchArgument(
            'marker_size_m', default_value='0.200',
            description='Marker side length in metres. The marker, not the plate.'),
        DeclareLaunchArgument(
            'target_marker_id', default_value='0',
            description='Which DICT_4X4_50 id to track. Stage 3: the central bot.'),
        DeclareLaunchArgument(
            'target_height_m', default_value='2.5',
            description='Hold height above the TAKEOFF POINT. Match MIS_TAKEOFF_ALT.'),
        DeclareLaunchArgument(
            'kp', default_value='0.5',
            description='Lateral proportional gain. Settling takes ~7 s at 0.5.'),
        DeclareLaunchArgument(
            'ki', default_value='0.0',
            description='Integral gain. Wired in but off by default.'),
        DeclareLaunchArgument(
            'integral_limit_mps', default_value='0.3',
            description='Clamp on the integral term.'),
        DeclareLaunchArgument(
            'max_velocity_mps', default_value='1.0',
            description='Lateral velocity setpoint clamp.'),
        DeclareLaunchArgument(
            'marker_timeout_s', default_value='0.5',
            description='Marker-lost threshold. Camera intervals reach 255 ms, '
                        'so this must stay well above 0.25.'),
        DeclareLaunchArgument(
            'setpoint_rate_hz', default_value='50.0',
            description='Fixed-rate setpoint timer. PX4 drops Offboard below 2 Hz, '
                        'which is why this is never driven by the camera callback.'),
        OpaqueFunction(function=_setup),
    ])
