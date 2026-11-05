"""Read-only dependency checks; optional camera discovery, never start motors."""

import argparse
import importlib
import os
import sys


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--probe-camera', action='store_true', help='Also enumerate USB cameras via the SDK')
    options = parser.parse_args(args)
    failures = []
    print(f'Python: {sys.executable} ({sys.version.split()[0]})')
    print(f'ROS_DISTRO: {os.environ.get("ROS_DISTRO", "not sourced")}')
    for module in ['rclpy', 'cv_bridge', 'tf2_ros', 'numpy', 'yaml', 'cv2']:
        try:
            loaded = importlib.import_module(module)
            print(f'OK module {module}: {getattr(loaded, "__version__", "available")}')
            if module == 'cv2' and not hasattr(loaded, 'aruco'):
                raise ImportError('OpenCV does not include ArUco')
        except Exception as exc:
            failures.append(module)
            print(f'MISSING/BROKEN {module}: {exc}')
    try:
        from ament_index_python.packages import get_package_prefix
        for package in ['realsense2_camera', 'realsense2_description', 'std_srvs', 'joy']:
            try:
                print(f'OK ROS package {package}: {get_package_prefix(package)}')
            except LookupError:
                failures.append(package)
                print(f'MISSING ROS package: {package}')
    except ImportError:
        failures.append('ament_index_python')
        print('Source /opt/ros/humble/setup.bash (or your selected ROS distribution) first.')
    if options.probe_camera:
        try:
            import pyrealsense2 as rs
            devices = rs.context().query_devices()
            if not devices:
                print('CAMERA: no visible devices; check USB connection/container device access')
            for device in devices:
                print(f'CAMERA: {device.get_info(rs.camera_info.name)}')
        except Exception as exc:
            print(f'CAMERA probe unavailable: {exc}')
            print('pyrealsense2 is optional for the ROS driver; rs-enumerate-devices is another check.')
    if failures:
        print('\nInstall in the ROS environment that will run the nodes:')
        distro = os.environ.get('ROS_DISTRO', 'humble')
        print(f'sudo apt-get install ros-{distro}-realsense2-camera ros-{distro}-realsense2-description '
              f'ros-{distro}-cv-bridge ros-{distro}-tf2-ros ros-{distro}-std-srvs '
              f'ros-{distro}-joy python3-opencv python3-yaml')
        print('Use /usr/bin/python3; a Conda Python may not match Humble rclpy/cv_bridge.')
        return 1
    print('\nSoftware dependencies are ready. Camera discovery and physical docking need separate checks.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
