#!/bin/bash
# Drone + swarm session on the Orin: everything fly_session.sh does (fly and
# track bot0), plus what the laptop did at home with the VX-500 - the bots'
# micro-ROS agent, the coordinator and the pilot - all fed by the drone's camera.
# tmux session "swarm", one named window per process, commands typed in but NOT
# run. Switch with Ctrl+b w (or click the tab - mouse is on), press Enter to run.
#
# Usage, on the Orin:   ./orin_swarm_session.sh <bag_name>     e.g. ./orin_swarm_session.sh lab_1
# From the laptop:      ssh -t corvid@<orin ip> 'cd ~/corvid && ./orin_swarm_session.sh lab_1'
#
# Press Enter in this order:
#   container (build) -> agent (Pixhawk) -> bots (then switch bot0 on, keep it
#   still until "joining wifi...") -> camera -> stack -> coordinator -> bag
#   -> pilot (h for keys; g = go straight until the target is seen, then the
#   coordinator takes over)
# Bag before driving. Ctrl-C the bag and let it exit before tearing down.
# Tear down:  tmux kill-session -t swarm  &&  docker stop corvid-dev
#
# fly_session.sh is unchanged and is still the script for flying without bots.

set -euo pipefail

CORVID_PATH=$(dirname "$(realpath "$0")")
WS=/home/ubuntu/roscon-25-workshop_ws
SESSION=swarm

# The bag name is required so a new run can never overwrite an old bag.
if [ $# -ne 1 ]; then
    echo "Usage: $0 <bag_name>    e.g. $0 lab_1"
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
if ! docker image inspect microros/micro-ros-agent:humble >/dev/null 2>&1; then
    echo "The bots' agent image is not on this Orin. With internet, run once:"
    echo "  docker pull microros/micro-ros-agent:humble"
    exit 1
fi
if [ ! -d "${CORVID_PATH}/corvid_swarm" ]; then
    echo "corvid_swarm/ is missing - this checkout is older than the coordinator. Pull main first."
    exit 1
fi

# Window "container" starts the container itself, so its errors stay visible there.
tmux new-session -d -s "${SESSION}" -n container -c "${CORVID_PATH}"
tmux set-option -g mouse on
tmux send-keys -t "${SESSION}:container" "./run_container_jetson.sh" Enter

# Every other ROS window needs the container to exist before docker exec.
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

tmux new-window -t "${SESSION}:" -n agent
tmux send-keys -t "${SESSION}:agent" "docker exec -it corvid-dev bash" Enter

# The bots' agent is its own container on the host network, not inside corvid-dev.
tmux new-window -t "${SESSION}:" -n bots -c "${CORVID_PATH}"

for name in camera stack coordinator bag pilot; do
    tmux new-window -t "${SESSION}:" -n "${name}"
    tmux send-keys -t "${SESSION}:${name}" "docker exec -it corvid-dev bash" Enter
done

# Give each shell time to reach its prompt, so the typed command lands on it.
sleep 3

# Typed, not run: no Enter at the end of these.
tmux send-keys -t "${SESSION}:container" \
    "colcon build --packages-select corvid_msgs corvid_perception corvid_control corvid_swarm && source install/setup.bash"
tmux send-keys -t "${SESSION}:agent" \
    "MicroXRCEAgent serial --dev /dev/ttyPixhawk -b 921600"
tmux send-keys -t "${SESSION}:bots" \
    "docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6"
tmux send-keys -t "${SESSION}:camera" \
    "source install/setup.bash && ros2 launch src/corvid/launch/camera.launch.py"
tmux send-keys -t "${SESSION}:stack" \
    "source install/setup.bash && ros2 launch src/corvid/launch/real_stack.launch.py target_height_m:=1.5 marker_timeout_s:=1.0"
tmux send-keys -t "${SESSION}:coordinator" \
    "source install/setup.bash && ros2 run corvid_swarm coordinator --ros-args -p tag_offset_forward_m:=0.0065 -p tag_offset_left_m:=0.0030 -p state_timeout_s:=20.0 -p turn_timeout_s:=25.0 -p turn_stuck_s:=25.0 -p ring_radius_m:=0.10"
tmux send-keys -t "${SESSION}:bag" \
    "source install/setup.bash && ros2 bag record -o ${WS}/src/corvid/bags/${BAG} /station_keeper_status /marker_detections /image_raw/compressed /camera_info /fmu/out/vehicle_local_position /fmu/out/vehicle_attitude /fmu/out/vehicle_status_v1 /fmu/in/trajectory_setpoint /fmu/in/offboard_control_mode /fmu/in/vehicle_command /coordinator/status /bot0/state /bot0/heading /bot0/turn /bot0/drive /bot0/cmd_vel /rosout"
tmux send-keys -t "${SESSION}:pilot" \
    "source install/setup.bash && ros2 run corvid_swarm pilot"

tmux select-window -t "${SESSION}:container"
exec tmux attach -t "${SESSION}"
