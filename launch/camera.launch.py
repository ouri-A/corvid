#!/usr/bin/env python3
"""The Rapoo C200 on usb_cam. Real hardware only -- nothing here runs in sim.

Needs the corvid:dev image (patched usb_cam) and the container started with
--camera, which puts the C200 at /dev/video0 inside.

  ros2 launch src/corvid/launch/camera.launch.py
  ros2 launch src/corvid/launch/camera.launch.py camera_info_url:=file:///path/to/other.yaml

Every argument defaults to the C200, so the drone's command never changes. The
Microsoft LifeCam VX-500 (stand-in for swarm work at home, 640x480 YUYV only, and
no exposure_dynamic_framerate control) is started with:

  ros2 launch src/corvid/launch/camera.launch.py image_width:=640 image_height:=480 \
      pixel_format:=yuyv2rgb camera_name:=vx500 v4l2_controls:=power_line_frequency=1 \
      camera_info_url:=file:///home/ubuntu/roscon-25-workshop_ws/src/corvid/config/vx500.yaml
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
    controls = arg('v4l2_controls')

    # Controls usb_cam has no parameter for. Set them the same way usb_cam
    # sets its own -- v4l2-ctl -- before the node opens the device. The C200
    # defaults were proven 1 Oct to survive usb_cam opening it.
    #   power_line_frequency=1        50 Hz mains, no banding under lab lights
    #   exposure_dynamic_framerate=0  camera may not drop fps in dim light
    v4l2 = ExecuteProcess(
        cmd=['v4l2-ctl', '-d', device, '-c', controls],
        output='screen',
    )

    camera = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='camera',
        parameters=[{
            'video_device': device,
            'pixel_format': arg('pixel_format'),
            'image_width': int(arg('image_width')),
            'image_height': int(arg('image_height')),
            'framerate': float(arg('framerate')),
            # usb_cam defaults this to 50 and pushes it onto the camera
            'brightness': 0,
            'camera_name': arg('camera_name'),
            'frame_id': arg('frame_id'),
            'camera_info_url': arg('camera_info_url'),
        }],
        output='screen',
    )

    # No controls asked for: nothing to set, start the camera straight away.
    if not controls:
        return [camera]

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
        DeclareLaunchArgument('camera_info_url',
            default_value='file:///home/ubuntu/roscon-25-workshop_ws/src/corvid/config/c200.yaml',
            description='file:// URL of the calibration YAML. Defaults to the C200 calibration '
                        'as seen inside the container.'),
        # 720p30 exists only as MJPEG on the C200 (USB 2.0)
        DeclareLaunchArgument('pixel_format', default_value='mjpeg2rgb',
            description='usb_cam pixel_format. yuyv2rgb for the VX-500.'),
        DeclareLaunchArgument('image_width', default_value='1280'),
        DeclareLaunchArgument('image_height', default_value='720'),
        DeclareLaunchArgument('framerate', default_value='30.0'),
        DeclareLaunchArgument('camera_name', default_value='c200',
            description='Must match camera_name in the calibration YAML.'),
        DeclareLaunchArgument('v4l2_controls',
            default_value='power_line_frequency=1,exposure_dynamic_framerate=0',
            description='Passed to v4l2-ctl -c before the camera starts. Empty to skip.'),
        OpaqueFunction(function=_setup),
    ])
