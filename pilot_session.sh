#!/bin/bash
# Drive bot0 by hand from the laptop with the pilot - nothing else: no camera,
# no detector, no coordinator. tmux session "pilot", commands typed in but NOT
# run. Switch windows with Ctrl+b w (or click the tab), press Enter to run.
#
# Usage:  ./pilot_session.sh
#
# Press Enter in this order:
#   agent -> (switch bot0 on, keep it still until "joining wifi...")
#   -> container (build) -> pilot
# Keys in the pilot: w / s straight forward / back, a / d turn 15 deg,
# q / e turn 90 deg, space stop, x quit. g and c need the camera and the
# coordinator, so here they only say why they can't run.
# Tear down:  tmux kill-session -t pilot   (the container is --rm, it goes too)

set -euo pipefail

CORVID_PATH=$(dirname "$(realpath "$0")")
WS=/home/ubuntu/roscon-25-workshop_ws
SESSION=pilot
CONTAINER=pilot
DDS="export FASTRTPS_DEFAULT_PROFILES_FILE=${WS}/src/corvid/config/fastdds.xml"

# Refuse to start over leftovers instead of fighting them.
if tmux has-session -t "${SESSION}" 2>/dev/null; then
    echo "tmux session '${SESSION}' already running. Attach: tmux attach -t ${SESSION}"
    echo "Or kill it first: tmux kill-session -t ${SESSION}"
    exit 1
fi
if docker ps -a --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
    echo "A ${CONTAINER} container already exists. Stop it first: docker stop ${CONTAINER}"
    exit 1
fi

# The micro-ROS agent runs in its own container on the host network.
tmux new-session -d -s "${SESSION}" -n agent -c "${CORVID_PATH}"
tmux set-option -g mouse on
tmux send-keys -t "${SESSION}:agent" \
    "docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6"

# Window "container" starts the ROS container itself, so its errors stay visible there.
tmux new-window -t "${SESSION}:" -n container -c "${CORVID_PATH}"
tmux send-keys -t "${SESSION}:container" \
    "docker run -it --rm --net=host -v ${CORVID_PATH}:${WS}/src/corvid -w ${WS} --name ${CONTAINER} corvid:dev bash" Enter

echo "Waiting for the ${CONTAINER} container..."
for _ in $(seq 60); do
    docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}" && break
    sleep 1
done
if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
    echo "Container did not start in 60 s - look at the 'container' window:"
    echo "  tmux attach -t ${SESSION}"
    exit 1
fi

tmux new-window -t "${SESSION}:" -n pilot
tmux send-keys -t "${SESSION}:pilot" "docker exec -it ${CONTAINER} bash" Enter

# Give the shells time to reach their prompts, then load the Fast DDS profile.
sleep 3
for name in container pilot; do
    tmux send-keys -t "${SESSION}:${name}" "${DDS}" Enter
done
sleep 1

# Typed, not run: no Enter at the end of these.
tmux send-keys -t "${SESSION}:container" \
    "colcon build --packages-select corvid_msgs corvid_swarm && source install/setup.bash"
tmux send-keys -t "${SESSION}:pilot" \
    "source install/setup.bash && ros2 run corvid_swarm pilot"

tmux select-window -t "${SESSION}:agent"
exec tmux attach -t "${SESSION}"
