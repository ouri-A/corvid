#!/bin/bash
# Launcher for the corvid dev container on the Jetson Orin (headless flight computer).
#
# The laptop's run_container.sh is a DIFFERENT script for a different machine: it
# mounts the workshop repo, forwards X11, uses the nvidia runtime and publishes the
# PX4 and Foxglove ports. None of that belongs on a headless board, and the workshop
# workspace is already built inside the image.
#
# On the airframe the camera and the Pixhawk link are not optional, so both are
# hard-wired and the script refuses to start if either is missing.
#   C200            -> /dev/video0 inside     (by-id name, same on every machine)
#   FT232RL serial  -> /dev/ttyPixhawk inside (agent: MicroXRCEAgent serial --dev /dev/ttyPixhawk -b 921600)
#
# Usage:  ./run_container_jetson.sh             interactive shell
#         ./run_container_jetson.sh <cmd...>    run a command and exit
# More shells into the same container:  docker exec -it corvid-dev bash

set -euo pipefail

CORVID_PATH=$(dirname "$(realpath "$0")")
IMAGE=corvid:dev
WS=/home/ubuntu/roscon-25-workshop_ws
CAMERA=/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._Rapoo_Camera_SN0001-video-index0

if [ ! -d "${CORVID_PATH}/corvid_msgs" ]; then
    echo "Expected to find corvid_msgs next to this script — is it still in the repo root?"
    exit 1
fi

if [ ! -e "${CAMERA}" ]; then
    echo "C200 not found at ${CAMERA} — is it plugged in?"
    exit 1
fi

# /dev/ttyUSB0 renumbers, so find the converter by its by-id name. Exactly one, or stop.
shopt -s nullglob
SERIALS=(/dev/serial/by-id/usb-FTDI_*)
shopt -u nullglob
if [ ${#SERIALS[@]} -ne 1 ]; then
    echo "Expected exactly one FTDI serial adapter in /dev/serial/by-id/, found ${#SERIALS[@]}:"
    ls -l /dev/serial/by-id/ 2>/dev/null || echo "  (no /dev/serial/by-id/ at all — nothing serial is plugged in)"
    exit 1
fi
SERIAL=${SERIALS[0]}

if [ $# -eq 0 ]; then
    set -- bash
fi

# The container's ubuntu user is in no video or dialout group of its own; the
# device nodes keep the HOST's group ids, so add those numerically.
docker run -it --rm \
    --network host \
    --device "${CAMERA}:/dev/video0" \
    --device "${SERIAL}:/dev/ttyPixhawk" \
    --group-add "$(getent group video | cut -d: -f3)" \
    --group-add "$(getent group dialout | cut -d: -f3)" \
    -v "${CORVID_PATH}:${WS}/src/corvid" \
    -w "${WS}" \
    --name corvid-dev \
    "${IMAGE}" "$@"
