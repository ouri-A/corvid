#!/usr/bin/env python3
"""The moving-marker rig: one cmd_vel bridge plus the marker_mover node.

Separate from sim_bringup.launch.py and corvid_stack.launch.py, which are
untouched. Stop this and the stationary setup is exactly as it was.

It does NOT spawn the model -- spawning stays a hand command, same as the
stationary marker, because which marker goes where is a per-test decision.

  ros2 launch src/corvid/launch/moving_marker.launch.py
  ros2 launch src/corvid/launch/moving_marker.launch.py axis:=y
  ros2 launch src/corvid/launch/moving_marker.launch.py speed_mps:=0.3 line_length_m:=2.0
  ros2 launch src/corvid/launch/moving_marker.launch.py pattern:=stop
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    model = arg('model_name')
    topic = f'/model/{model}/cmd_vel'

    # ROS -> Gazebo, which is ']' rather than the '[' every other bridge in this
    # project uses. This is the first thing we send INTO the sim; everything
    # else so far has only read out of it. Get the bracket wrong and the bridge
    # starts happily and the marker never moves.
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='marker_cmd_vel_bridge',
        arguments=[f'{topic}@geometry_msgs/msg/Twist]gz.msgs.Twist'],
        output='screen',
    )

    # use_sim_time because the legs reverse on elapsed time, and sim runs at
    # RTF ~0.81 -- on wall time every leg would come out short.
    marker_mover = Node(
        package='corvid_sim',
        executable='marker_mover',
        name='marker_mover',
        parameters=[{
            'use_sim_time': True,
            'model_name': model,
            'pattern': arg('pattern'),
            'axis': arg('axis'),
            'speed_mps': float(arg('speed_mps')),
            'line_length_m': float(arg('line_length_m')),
            'publish_rate_hz': float(arg('publish_rate_hz')),
        }],
        output='screen',
    )

    return [cmd_vel_bridge, marker_mover]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_name', default_value='aruco_marker_moving',
            description='Gazebo model to drive. The cmd_vel topic is built from it.'),
        DeclareLaunchArgument(
            'pattern', default_value='line',
            description='line | stop.'),
        DeclareLaunchArgument(
            'axis', default_value='x',
            description='Which world axis to slide along. Gazebo is ENU: '
                        'x is east, y is north.'),
        DeclareLaunchArgument(
            'speed_mps', default_value='0.15',
            description='Path speed. Slow on purpose -- 0.188 s of pipeline '
                        'latency is ~2.8 cm of lag at this speed.'),
        DeclareLaunchArgument(
            'line_length_m', default_value='1.5',
            description='Leg length before it reverses. Centred on the spawn '
                        'point, so 0.75 m either side at the default.'),
        DeclareLaunchArgument(
            'publish_rate_hz', default_value='10.0',
            description='Twist republish rate, so the command survives the '
                        'bridge starting late.'),
        OpaqueFunction(function=_setup),
    ])
