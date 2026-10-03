FROM dronecode/roscon-25-workshop
USER root

# usb_cam from apt only to pull in its runtime deps, then remove the buggy 0.8.1 binary itself
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-usb-cam git pkg-config v4l-utils \
      libavcodec-dev libavutil-dev libswscale-dev libopencv-dev \
 && apt-get remove -y ros-humble-usb-cam \
 && rm -rf /var/lib/apt/lists/*

# usb_cam 0.8.1 + upstream fix ee0a2f7 (issue #327): stamps were early by 0-999 ms, different every start
RUN mkdir -p /opt/usb_cam_ws/src && cd /opt/usb_cam_ws/src \
 && git clone --depth 1 --branch 0.8.1 https://github.com/ros-drivers/usb_cam.git \
 && sed -i 's|epoch_time.tv_usec / 1000.0|epoch_time.tv_usec|' usb_cam/include/usb_cam/utils.hpp \
 && grep -q 'epoch_time.tv_sec \* 1000000 + epoch_time.tv_usec;' usb_cam/include/usb_cam/utils.hpp \
 && cd /opt/usb_cam_ws && . /opt/ros/humble/setup.sh \
 && colcon build --packages-select usb_cam --cmake-args -DBUILD_TESTING=OFF

RUN echo 'source /opt/usb_cam_ws/install/local_setup.bash' >> /home/ubuntu/.bashrc

# ROS camera calibrator (3.0.9): ChArUco target, writes the YAML usb_cam reads via camera_info_url
RUN apt-get update && apt-get install -y --no-install-recommends ros-humble-camera-calibration \
 && rm -rf /var/lib/apt/lists/*

USER ubuntu
