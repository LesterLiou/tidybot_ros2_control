#!/usr/bin/env bash
set -euo pipefail
task_ros_distro="${ROS_DISTRO:-humble}"
case "$task_ros_distro" in
  humble|jazzy) ;;
  *) printf 'Expected Humble or Jazzy; ROS_DISTRO=%s\n' "$task_ros_distro" >&2; exit 1 ;;
esac
task_packages=(
  "ros-${task_ros_distro}-realsense2-camera"
  "ros-${task_ros_distro}-realsense2-description"
  "ros-${task_ros_distro}-cv-bridge"
  "ros-${task_ros_distro}-tf2-ros"
  "ros-${task_ros_distro}-std-srvs"
  "ros-${task_ros_distro}-joy"
  "ros-${task_ros_distro}-rqt-image-view"
  python3-opencv python3-yaml python3-pytest
)
task_missing=()
for task_package in "${task_packages[@]}"; do
  task_status="$(dpkg-query -W -f='${Status}' "$task_package" 2>/dev/null || true)"
  if [[ "$task_status" != 'install ok installed' ]]; then
    task_missing+=("$task_package")
  fi
done
if (( ${#task_missing[@]} == 0 )); then
  printf 'All docking dependencies are already installed; skipping apt.\n'
  printf 'Next: source /opt/ros/%s/setup.bash\n' "$task_ros_distro"
  printf 'Then run /usr/bin/python3 check_docking_environment.py --probe-camera from this package.\n'
  exit 0
fi
task_dpkg_audit="$(dpkg --audit)"
if [[ -n "$task_dpkg_audit" ]]; then
  printf 'The package manager has unfinished packages; installation was not started.\n%s\n' \
    "$task_dpkg_audit" >&2
  printf 'Missing docking dependencies: %s\n' "${task_missing[*]}" >&2
  printf 'Resolve the existing package error before installing missing dependencies.\n' >&2
  exit 2
fi
sudo apt-get update
sudo apt-get install --no-install-recommends -y "${task_missing[@]}"
