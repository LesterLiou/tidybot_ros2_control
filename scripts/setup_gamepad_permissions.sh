#!/usr/bin/env bash
set -euo pipefail

task_user="${SUDO_USER:-$(id -un)}"
if ! task_groups="$(id -nG "$task_user")"; then
  printf 'Could not query groups for user %s.\n' "$task_user" >&2
  exit 1
fi
case " $task_groups " in
  *" plugdev "*) ;;
  *)
    printf 'User %s must already belong to plugdev before using this rule.\n' "$task_user" >&2
    exit 1
    ;;
esac
task_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_rule_source="$task_script_dir/../config/99-tidybot-gamepad.rules"
task_rule_target=/etc/udev/rules.d/99-tidybot-gamepad.rules
sudo install -m 0644 "$task_rule_source" "$task_rule_target"
sudo udevadm control --reload-rules
task_devices=0
for task_event in /sys/class/input/event[0-9]*; do
  [[ -r "$task_event/device/id/vendor" && -r "$task_event/device/id/product" ]] || continue
  if [[ "$(<"$task_event/device/id/vendor")" == 045e &&
        "$(<"$task_event/device/id/product")" == 0b13 ]]; then
    task_event_name="${task_event##*/}"
    sudo udevadm trigger --action=change --subsystem-match=input --sysname-match="$task_event_name"
    task_devices=$((task_devices + 1))
  fi
done
sudo udevadm settle
printf 'Installed Xbox joystick permission rule; refreshed %s current event device(s).\n' "$task_devices"
printf 'If the controller is disconnected, the rule applies on the next connection.\n'
printf 'Verify as your normal user: ros2 run joy joy_enumerate_devices\n'
