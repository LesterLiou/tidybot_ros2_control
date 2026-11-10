"""Launch the USB-camera battery OCR monitor and web dashboard."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("tidybot_ros2_control")
    default_config = f"{package_share}/config/battery_monitor.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("camera", default_value="auto"),
        DeclareLaunchArgument("web_port", default_value="8080"),
        Node(
            package="tidybot_ros2_control",
            executable="battery_monitor",
            name="tidybot_battery_monitor",
            output="screen",
            emulate_tty=True,
            parameters=[
                LaunchConfiguration("config"),
                {
                    "camera": LaunchConfiguration("camera"),
                    "web_port": ParameterValue(LaunchConfiguration("web_port"), value_type=int),
                },
            ],
        ),
    ])
