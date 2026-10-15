#!/bin/bash
# One-command swarm session on the laptop: tmux session "swarm", one window per
# process, every command typed in but NOT run. Switch windows with Ctrl+b w (or
# click the tab - mouse is on), press Enter to run.
#
# Usage:  ./swarm_session.sh <bag_name>        e.g. ./swarm_session.sh coord_2_oct16
#
# Press Enter in this order:
#   agent -> container (build) -> camera -> detector -> coordinator -> bag -> control
# Bag before the start call. Ctrl-C the bag only after bot0 has stopped for good.
# Tear down:  tmux kill-session -t swarm   (the vx500 container is --rm, it goes too)

set -euo pipefail

CORVID_PATH=$(dirname "$(realpath "$0")")
WS=/home/ubuntu/roscon-25-workshop_ws
SESSION=swarm
CAM=/dev/v4l/by-id/usb-Microsoft_Microsoft_LifeCam-video-index0
DDS="export FASTRTPS_DEFAULT_PROFILES_FILE=${WS}/src/corvid/config/fastdds.xml"

# The bag name is required so a new run can never overwrite an old bag.
if [ $# -ne 1 ]; then
    echo "Usage: $0 <bag_name>    e.g. $0 coord_2_oct16"
    exit 1
fi
BAG=$1
if [ -e "${CORVID_PATH}/bags/${BAG}" ]; then
    echo "bags/${BAG} already exists - pick a new name."
    exit 1
fi

# Refuse to start over leftovers instead of fighting them.
if tmux has-session -t "${SESSION}" 2>/dev/null; then
    echo "tmux session '${SESSION}' already running. Attach: tmux attach -t ${SESSION}"
    echo "Or kill it first: tmux kill-session -t ${SESSION}"
    exit 1
fi
if docker ps -a --format '{{.Names}}' | grep -qx vx500; then
    echo "A vx500 container already exists. Stop it first: docker stop vx500"
    exit 1
fi
if [ ! -e "${CAM}" ]; then
    echo "VX-500 not found at ${CAM} - plug it in."
    exit 1
fi

# The micro-ROS agent runs in its own container on the host network.
tmux new-session -d -s "${SESSION}" -n agent -c "${CORVID_PATH}"
tmux set-option -g mouse on
tmux send-keys -t "${SESSION}:agent" \
    "docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6"

# Window "container" starts the vx500 container itself, so its errors stay visible there.
tmux new-window -t "${SESSION}" -n container -c "${CORVID_PATH}"
tmux send-keys -t "${SESSION}:container" \
    "docker run -it --rm --net=host -v /tmp/.X11-unix:/tmp/.X11-unix:ro -e DISPLAY=\$DISPLAY --device ${CAM}:/dev/video0 --group-add \$(getent group video | cut -d: -f3) -v ${CORVID_PATH}:${WS}/src/corvid -w ${WS} --name vx500 corvid:dev bash" Enter

echo "Waiting for the vx500 container..."
for _ in $(seq 60); do
    docker ps --format '{{.Names}}' | grep -qx vx500 && break
    sleep 1
done
if ! docker ps --format '{{.Names}}' | grep -qx vx500; then
    echo "Container did not start in 60 s - look at the 'container' window:"
    echo "  tmux attach -t ${SESSION}"
    exit 1
fi

for name in camera detector coordinator bag control; do
    tmux new-window -t "${SESSION}" -n "${name}"
    tmux send-keys -t "${SESSION}:${name}" "docker exec -it vx500 bash" Enter
done

# Give each shell time to reach its prompt, then load the Fast DDS profile in every one.
sleep 3
for name in container camera detector coordinator bag control; do
    tmux send-keys -t "${SESSION}:${name}" "${DDS}" Enter
done
sleep 1

# Typed, not run: no Enter at the end of these.
tmux send-keys -t "${SESSION}:container" \
    "colcon build --packages-select corvid_msgs corvid_perception corvid_swarm && source install/setup.bash"
tmux send-keys -t "${SESSION}:camera" \
    "ros2 launch src/corvid/launch/camera.launch.py image_width:=640 image_height:=480 pixel_format:=yuyv2rgb camera_name:=vx500 v4l2_controls:=power_line_frequency=1 camera_info_url:=file://${WS}/src/corvid/config/vx500.yaml"
tmux send-keys -t "${SESSION}:detector" \
    "source install/setup.bash && ros2 run corvid_perception marker_detector --ros-args -p marker_size_m:=0.136"
tmux send-keys -t "${SESSION}:coordinator" \
    "source install/setup.bash && ros2 run corvid_swarm coordinator --ros-args -p tag_offset_forward_m:=0.0065 -p tag_offset_left_m:=0.0030 -p state_timeout_s:=20.0 -p turn_timeout_s:=25.0 -p turn_stuck_s:=25.0 -p ring_radius_m:=0.10"
tmux send-keys -t "${SESSION}:bag" \
    "source install/setup.bash && ros2 bag record -a -x '^/image_raw\$|^/debug_image\$|.*theora.*|.*compressedDepth.*' -o ${WS}/src/corvid/bags/${BAG}"
tmux send-keys -t "${SESSION}:control" \
    "source install/setup.bash && ros2 service call /coordinator/start std_srvs/srv/Trigger"

tmux select-window -t "${SESSION}:agent"
exec tmux attach -t "${SESSION}"
