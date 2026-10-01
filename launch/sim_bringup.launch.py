#!/usr/bin/env python3
"""Stage 1 sim plumbing: the three Gazebo->ROS bridges.

WHAT THIS DELIBERATELY DOES NOT START, and why:

  gz sim        Gazebo must be up AND RENDERING before PX4 starts, or PX4 gets
                untimed IMU data and the drone spins on takeoff. Launch cannot
                wait for "rendering" -- only for a timer, which is a guess
                dressed up as a condition.
  PX4 SITL      You fly from its `pxh>` prompt: `commander takeoff`, `listener`,
                `param show`, the parameters.bson wipe. Under launch its stdout
                is multiplexed with everything else and there is no usable
                interactive console, so you would lose the shell you work in.
  XRCE agent    Trivial to launch, but it belongs beside PX4: if PX4 restarts,
                the agent restarts. Keeping the pair together in hand-started
                terminals keeps that obvious.
  QGroundControl  A GUI you click on.

So the manual terminals stay: Gazebo, PX4, the XRCE agent, QGC. This file
replaces the three bridge terminals. The marker is spawned by hand -- see
BRINGUP.md section 5 -- because spawning it is a per-test decision (offset to
give the controller real error, or under the drone so entering Offboard changes
nothing) and because moving it means delete-and-recreate, which is a thing you
do mid-flight, not at bring-up.

Run it by path, from a shell that has already sourced ROS and the workspace
(there is no package install step, on purpose -- nothing else in the repo is
touched):

  cd /home/ubuntu/roscon-25-workshop_ws
  source /opt/ros/humble/setup.bash && source install/setup.bash
  ros2 launch src/corvid/launch/sim_bringup.launch.py

Override anything on the command line, e.g.:

  ros2 launch src/corvid/launch/sim_bringup.launch.py model_name:=x500_mono_cam_down_1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    # perform() turns each launch argument into an ordinary Python string, so the
    # long gz topic names below are plain f-strings instead of nested lists of
    # substitution objects. Same result, far easier to read and to get right.
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    world = arg('world')
    model = arg('model_name')
    link = arg('camera_link')
    sensor = arg('sensor')

    # Every topic the camera publishes hangs off this prefix. Built from the
    # arguments rather than hardcoded, because Stage 2's camera is not called
    # x500_mono_cam_down_0 and should not inherit a sim entity's name.
    cam = f'/world/{world}/model/{model}/link/{link}/sensor/{sensor}'

    # ---- camera_info -------------------------------------------------------
    # The detector waits forever without intrinsics and Gazebo does not bridge
    # camera_info by default. '[' is the direction marker: Gazebo -> ROS only.
    camera_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_info_bridge',
        arguments=[
            f'{cam}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen',
    )

    # ---- /clock ------------------------------------------------------------
    # On its OWN bridge, NOT merged onto the camera one. Merged, restarting the
    # camera bridge also drops the clock, and every node running use_sim_time
    # then stalls on a zero clock. Saving a terminal was the only reason to
    # merge them; under launch there are no terminals to save.
    #
    # Gazebo owns the clock for PX4 and ROS 2 both, and it must never be written
    # back -- hence '[' again and no remap.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # ---- the image ---------------------------------------------------------
    # image_bridge, never parameter_bridge. Through parameter_bridge the frames
    # arrive seconds old and get older, while `ros2 topic hz` looks healthy the
    # entire time it is broken. image_bridge only handles images and only
    # Gazebo -> ROS, so the topic is a bare argument with nothing to specify.
    #
    # Expect a segfault from this one on Ctrl+C. It is cosmetic, it happens on
    # shutdown, and under launch it shows up as a non-zero exit code in the log.
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='image_bridge',
        arguments=[f'{cam}/image'],
        output='screen',
    )

    # ---- Gazebo ground truth -----------------------------------------------
    # pose/info carries every entity including static ones (the marker);
    # dynamic_pose/info carries only what moves (the drone). Both ~52 Hz and
    # almost free to record. Recording this is what stopped flight analysis
    # being half reconstruction.
    #
    # TFMessage rather than PoseArray because TFMessage keeps the entity NAMES.
    # With a PoseArray the marker is identified by its array index, which is
    # exactly the kind of inference worth deleting.
    #
    # use_sim_time is set here to match the command that produced final_rec, so
    # this file changes nothing about recorded behaviour. Be clear about what it
    # does NOT do: it does not fix the stamps. Measured on final_rec, all
    # 719,126 transforms across both topics still stamp exactly 0.0 with the
    # flag on. The flag sets the node's own clock; it does not make a message
    # converter write a header. Ground truth is aligned on bag receive time
    # through /clock, and that is permanent, not a workaround.
    ground_truth_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ground_truth_bridge',
        arguments=[
            f'/world/{world}/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            f'/world/{world}/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return [
        clock_bridge,
        camera_info_bridge,
        image_bridge,
        ground_truth_bridge,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='default',
            description='Gazebo world name. Prefixes every gz topic.'),
        DeclareLaunchArgument(
            'model_name', default_value='x500_mono_cam_down_0',
            description='Gazebo model name of the drone carrying the camera.'),
        DeclareLaunchArgument(
            'camera_link', default_value='camera_link',
            description='Link the camera sensor hangs off.'),
        DeclareLaunchArgument(
            'sensor', default_value='imager',
            description='Camera sensor name inside that link.'),
        OpaqueFunction(function=_setup),
    ])
