#!/usr/bin/env bash
set -eo pipefail

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /opt/tidybot_ws/install/setup.bash

# RealSense hardware resets can renumber /dev/video* while a container is
# running. Host /dev is mounted read-only at /host-dev, and device cgroups
# still restrict which character-device classes can actually be opened.
if [[ -d /host-dev ]]; then
    for task_index in {0..63}; do
        ln -sfn "/host-dev/video${task_index}" "/dev/video${task_index}"
    done
    for task_index in {0..15}; do
        ln -sfn "/host-dev/media${task_index}" "/dev/media${task_index}"
    done
    if [[ -d /host-dev/v4l ]]; then
        ln -sfn /host-dev/v4l /dev/v4l
    fi
fi

set -u
exec "$@"
