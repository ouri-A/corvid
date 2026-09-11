#!/bin/bash
# Launcher for the corvid dev container on the Jetson Orin (headless flight computer).
#
# The laptop's run_container.sh is a DIFFERENT script for a different machine: it
# mounts the workshop repo, forwards X11, uses the nvidia runtime and publishes the
# PX4 and Foxglove ports. None of that belongs on a headless board, and the workshop
# workspace is already built inside the image.
#
# Usage:  ./run_container_jetson.sh             interactive shell
#         ./run_container_jetson.sh <cmd...>    run a command and exit

set -euo pipefail

CORVID_PATH=$(dirname "$(realpath "$0")")
IMAGE=dronecode/roscon-25-workshop
WS=/home/ubuntu/roscon-25-workshop_ws

if [ ! -d "${CORVID_PATH}/corvid_msgs" ]; then
    echo "Expected to find corvid_msgs next to this script — is it still in the repo root?"
    exit 1
fi

if [ $# -eq 0 ]; then
    set -- bash
fi

docker run -it --rm \
    --network host \
    -v "${CORVID_PATH}:${WS}/src/corvid" \
    -w "${WS}" \
    --name corvid-dev \
    "${IMAGE}" "$@"
