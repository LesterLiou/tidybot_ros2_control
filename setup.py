from glob import glob
from setuptools import setup


package_name = 'tidybot_ros2_control'

setup(
    name=package_name,
    version='0.1.0',
    py_modules=[
        'base_controller',
        'constants',
        'ros2_base_node',
        'ros2_gamepad_teleop',
        'docking_core',
        'aruco_detection',
        'ros2_aruco_detector',
        'ros2_docking_controller',
        'ros2_control_mux',
        'check_docking_environment',
        'generate_docking_marker',
        'battery_monitor_core',
        'ros2_battery_monitor',
        'utils',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/markers', glob('markers/*')),
        ('share/' + package_name + '/templates', ['templates/battery_monitor.html']),
        ('share/' + package_name, ['DOCKING.md']),
        ('share/' + package_name, ['BATTERY_MONITOR.md']),
    ],
    # Non-ROS Python dependencies are installed explicitly from
    # requirements-ros2.txt so colcon never tries to access PyPI while building.
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='TidyBot maintainer',
    maintainer_email='maintainer@example.com',
    description='ROS 2 base driver and gamepad teleoperation for TidyBot++.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'base_node = ros2_base_node:main',
            'gamepad_teleop = ros2_gamepad_teleop:main',
            'aruco_detector = ros2_aruco_detector:main',
            'docking_controller = ros2_docking_controller:main',
            'control_mux = ros2_control_mux:main',
            'check_docking_environment = check_docking_environment:main',
            'generate_docking_marker = generate_docking_marker:main',
            'battery_monitor = ros2_battery_monitor:main',
        ],
    },
)
