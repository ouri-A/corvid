#!/usr/bin/env python3
"""The Rapoo C200 on usb_cam. Real hardware only -- nothing here runs in sim.

Needs the corvid:dev image (patched usb_cam) and the container started with
--camera, which puts the C200 at /dev/video0 inside.

  ros2 launch src/corvid/launch/camera.launch.py
  ros2 launch src/corvid/launch/camera.launch.py camera_info_url:=file:///path/to/c200.yaml
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler, Shutdown)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    device = arg('video_device')

    # Two controls usb_cam has no parameter for. Set them the same way usb_cam
    # sets its own -- v4l2-ctl -- before the node opens the device. Both proven
    # 1 Oct to survive usb_cam opening it.
    #   power_line_frequency=1        50 Hz mains, no banding under lab lights
    #   exposure_dynamic_framerate=0  camera may not drop fps in dim light
    v4l2 = ExecuteProcess(
        cmd=['v4l2-ctl', '-d', device,
             '-c', 'power_line_frequency=1,exposure_dynamic_framerate=0'],
        output='screen',
    )

    camera = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='camera',
        parameters=[{
            'video_device': device,
            # 720p30 exists only as MJPEG on this camera (USB 2.0)
            'pixel_format': 'mjpeg2rgb',
            'image_width': 1280,
            'image_height': 720,
            'framerate': 30.0,
            # usb_cam defaults this to 50 and pushes it onto the camera
            'brightness': 0,
            'camera_name': 'c200',
            'frame_id': arg('frame_id'),
            'camera_info_url': arg('camera_info_url'),
        }],
        output='screen',
    )

    # Start the camera only if v4l2-ctl succeeded. Otherwise stop everything,
    # so a camera running with the wrong settings never looks like a good run.
    def _after_v4l2(event, _context):
        if event.returncode == 0:
            return [camera]
        return [LogInfo(msg=f'v4l2-ctl failed on {device} -- not starting the camera'),
                Shutdown(reason='camera controls not set')]

    return [v4l2, RegisterEventHandler(OnProcessExit(target_action=v4l2,
                                                     on_exit=_after_v4l2))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/video0',
            description='Path INSIDE the container. The launcher maps the C200 here.'),
        DeclareLaunchArgument('frame_id', default_value='camera_optical',
            description='Stamped into image and camera_info headers.'),
        DeclareLaunchArgument('camera_info_url', default_value='',
            description='file:// URL of the calibration YAML. Empty until calibrated.'),
        OpaqueFunction(function=_setup),
    ])
