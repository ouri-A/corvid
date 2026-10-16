#!/bin/bash
# Spin-test session on the laptop: tmux session "spin", one window per process,
# every command typed in but NOT run. Switch windows with Ctrl+b w (or click the
# tab - mouse is on), press Enter to run.
#
# Usage:  ./spin_session.sh <bag_name>        e.g. ./spin_session.sh spin_2_bots12_oct16
#
# Press Enter in this order:
#   agent -> (switch the bots on, still until "joining wifi...") -> container (build)
#   -> camera -> detector -> bag -> spin -> (Ctrl-C the bag when spin says so) -> analyse
# Tear down:  tmux kill-session -t spin   (after the bag has exited)

set -euo pipefail

CORVID_PATH=$(dirname "$(realpath "$0")")
WS=/home/ubuntu/roscon-25-workshop_ws
SESSION=spin
CAM=/dev/v4l/by-id/usb-Microsoft_Microsoft_LifeCam-video-index0
DDS="export FASTRTPS_DEFAULT_PROFILES_FILE=${WS}/src/corvid/config/fastdds.xml"

# The bag name is required so a new run can never overwrite an old bag.
if [ $# -ne 1 ]; then
    echo "Usage: $0 <bag_name>    e.g. $0 spin_2_bots12_oct16"
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
tmux new-window -t "${SESSION}:" -n container -c "${CORVID_PATH}"
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

for name in camera detector bag spin; do
    tmux new-window -t "${SESSION}:" -n "${name}"
    tmux send-keys -t "${SESSION}:${name}" "docker exec -it vx500 bash" Enter
done

# Give each shell time to reach its prompt, then load the Fast DDS profile in every one.
sleep 3
for name in container camera detector bag spin; do
    tmux send-keys -t "${SESSION}:${name}" "${DDS}" Enter
done
sleep 1

# The analysis runs on the laptop itself, not in the container.
tmux new-window -t "${SESSION}:" -n analyse -c "${CORVID_PATH}"

# Typed, not run: no Enter at the end of these.
tmux send-keys -t "${SESSION}:container" \
    "colcon build --packages-select corvid_msgs corvid_perception && source install/setup.bash"
tmux send-keys -t "${SESSION}:camera" \
    "ros2 launch src/corvid/launch/camera.launch.py image_width:=640 image_height:=480 pixel_format:=yuyv2rgb camera_name:=vx500 v4l2_controls:=power_line_frequency=1 camera_info_url:=file://${WS}/src/corvid/config/vx500.yaml"
tmux send-keys -t "${SESSION}:detector" \
    "source install/setup.bash && ros2 run corvid_perception marker_detector --ros-args -p marker_size_m:=0.136"
tmux send-keys -t "${SESSION}:bag" \
    "source install/setup.bash && ros2 bag record -a -x '^/image_raw\$|^/debug_image\$|.*theora.*|.*compressedDepth.*' -o ${WS}/src/corvid/bags/${BAG}"
tmux send-keys -t "${SESSION}:spin" \
    "source install/setup.bash && python3 src/corvid/scripts/spin_sequence.py"
# Small tags: 0.095 m real black square / 0.136 m assumed by the detector = 0.6985.
tmux send-keys -t "${SESSION}:analyse" \
    "uv run scripts/spin_test.py bags/${BAG} --marker 1 --bot 1 --scale 0.6985; uv run scripts/spin_test.py bags/${BAG} --marker 2 --bot 2 --scale 0.6985"

tmux select-window -t "${SESSION}:agent"
exec tmux attach -t "${SESSION}"
