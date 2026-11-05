#!/usr/bin/env bash
set -euo pipefail

task_can_interface="${TIDYBOT_CAN_INTERFACE:-can0}"
task_failed=0

printf 'Ubuntu: '
. /etc/os-release
printf '%s (%s)\n' "$PRETTY_NAME" "$VERSION_CODENAME"
printf 'ROS_DISTRO: %s\n' "${ROS_DISTRO:-not sourced}"
printf 'CAN interface: %s\n' "$task_can_interface"

python3 - <<'PY'
from importlib.metadata import version

import cv2
import phoenix6
import rclpy
import ruckig

assert hasattr(cv2, 'aruco'), 'OpenCV was built without ArUco support'
for package in ('phoenix6', 'ruckig', 'numpy'):
    print(f'{package}: {version(package)}')
print('rclpy: import OK')
PY

if ip -details link show "$task_can_interface"; then
    task_state="$(cat "/sys/class/net/${task_can_interface}/operstate" 2>/dev/null || true)"
    if [[ "$task_state" != up && "$task_state" != unknown ]]; then
        task_driver="$(basename "$(readlink -f "/sys/class/net/${task_can_interface}/device/driver/module" 2>/dev/null)" 2>/dev/null || true)"
        if [[ "$task_driver" == canivore_usb ]]; then
            printf 'ERROR: %s is a CANivore but is not initialized (state=%s).\n' \
                "$task_can_interface" "${task_state:-unavailable}" >&2
            printf 'Run: docker compose run --rm can-init\n' >&2
        else
            printf 'ERROR: %s exists but is not up (state=%s). Configure it on the host.\n' \
                "$task_can_interface" "${task_state:-unavailable}" >&2
        fi
        task_failed=1
    fi
else
    printf 'ERROR: %s is not visible. Install/configure its host driver first.\n' \
        "$task_can_interface" >&2
    task_failed=1
fi

if [[ -d /dev/input ]]; then
    printf 'Input devices:\n'
    find /dev/input -maxdepth 1 -type c -printf '  %f\n' | sort
else
    printf 'WARNING: /dev/input is unavailable; gamepad control will not work.\n' >&2
fi

if [[ -d /dev/bus/usb ]]; then
    printf 'USB devices:\n'
    lsusb || true
else
    printf 'WARNING: /dev/bus/usb is unavailable; RealSense/USB CAN adapters may not work.\n' >&2
fi

if (( task_failed != 0 )); then
    exit 1
fi
printf 'Docker software and CAN visibility checks passed. No motor command was sent.\n'
