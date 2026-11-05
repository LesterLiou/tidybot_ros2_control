#!/usr/bin/env bash
set -eo pipefail

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /opt/tidybot_ws/install/setup.bash

set -u
exec "$@"
