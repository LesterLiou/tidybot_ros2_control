"""Launch the TidyBot base, raw joystick driver, and gamepad teleop."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('tidybot_ros2_control')
    default_config = f'{package_share}/config/gamepad.yaml'
    config = LaunchConfiguration('config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Path to the TidyBot gamepad parameter file.',
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[config],
            on_exit=[Shutdown(reason='Gamepad teleop exited')],
        ),
        Node(
            package='tidybot_ros2_control',
            executable='gamepad_teleop',
            name='tidybot_gamepad_teleop',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='tidybot_ros2_control',
            executable='base_node',
            name='tidybot_base_node',
            output='screen',
            emulate_tty=True,
            parameters=[config],
        ),
    ])
