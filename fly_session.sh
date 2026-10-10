#!/bin/bash
# One-command flight session on the Orin: tmux session "fly" with one named
# window per process, every window already inside the corvid-dev container and
# its command typed in but NOT run. Switch with Ctrl+b w, press Enter to run.
#
# Usage:  ./fly_session.sh <bag_name>        e.g. ./fly_session.sh hover_2
#
# Press Enter in this order: container (build) first, then agent, camera,
# stack, and bag last before arming. camera/stack/bag source the build.

set -euo pipefail

CORVID_PATH=$(dirname "$(realpath "$0")")
WS=/home/ubuntu/roscon-25-workshop_ws
SESSION=fly

# The bag name is required so a new flight can never overwrite an old bag.
if [ $# -ne 1 ]; then
    echo "Usage: $0 <bag_name>    e.g. $0 hover_2"
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
if docker ps -a --format '{{.Names}}' | grep -qx corvid-dev; then
    echo "A corvid-dev container already exists. Stop it first: docker stop corvid-dev"
    exit 1
fi

# Window "container" starts the container itself, so its errors stay visible there.
tmux new-session -d -s "${SESSION}" -n container -c "${CORVID_PATH}"
tmux set-option -g mouse on
tmux send-keys -t "${SESSION}:container" "./run_container_jetson.sh" Enter

# Every other window needs the container to exist before docker exec.
echo "Waiting for the corvid-dev container..."
for _ in $(seq 60); do
    docker ps --format '{{.Names}}' | grep -qx corvid-dev && break
    sleep 1
done
if ! docker ps --format '{{.Names}}' | grep -qx corvid-dev; then
    echo "Container did not start in 60 s - look at the 'container' window:"
    echo "  tmux attach -t ${SESSION}"
    exit 1
fi

for name in agent camera stack bag; do
    tmux new-window -t "${SESSION}" -n "${name}"
    tmux send-keys -t "${SESSION}:${name}" "docker exec -it corvid-dev bash" Enter
done

# Give each shell time to reach its prompt, so the typed command lands on it.
sleep 3

# Typed, not run: no Enter at the end of these.
tmux send-keys -t "${SESSION}:container" \
    "colcon build --packages-select corvid_msgs corvid_perception corvid_control && source install/setup.bash"
tmux send-keys -t "${SESSION}:agent" \
    "MicroXRCEAgent serial --dev /dev/ttyPixhawk -b 921600"
tmux send-keys -t "${SESSION}:camera" \
    "source install/setup.bash && ros2 launch src/corvid/launch/camera.launch.py"
tmux send-keys -t "${SESSION}:stack" \
    "source install/setup.bash && ros2 launch src/corvid/launch/real_stack.launch.py target_height_m:=1.5 marker_timeout_s:=1.0"
tmux send-keys -t "${SESSION}:bag" \
    "source install/setup.bash && ros2 bag record -o ${WS}/src/corvid/bags/${BAG} /station_keeper_status /marker_detections /image_raw/compressed /camera_info /fmu/out/vehicle_local_position /fmu/out/vehicle_attitude /fmu/out/vehicle_status_v1 /fmu/in/trajectory_setpoint /fmu/in/offboard_control_mode /fmu/in/vehicle_command"

tmux select-window -t "${SESSION}:container"
exec tmux attach -t "${SESSION}"
