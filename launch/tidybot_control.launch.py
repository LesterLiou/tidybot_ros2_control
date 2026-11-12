"""Start the same gamepad and mux chain for local manual or integrated navigation."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def control_nodes(context):
    mode = LaunchConfiguration('control_mode').perform(context)
    if mode not in ('manual', 'navigation'):
        raise ValueError('control_mode must be manual or navigation')
    package = 'tidybot_ros2_control'
    config = str(Path(get_package_share_directory(package)) / 'config' / 'gamepad.yaml')

    def start(executable, name, source=package, remappings=(), overrides=None):
        parameters = [config, overrides] if overrides else [config]
        return Node(
            package=source, executable=executable, name=name, parameters=parameters,
            remappings=list(remappings), output='screen',
            on_exit=[Shutdown(reason=f'{name} exited')],
        )

    mux_outputs = ('/cmd_vel', '/tidybot/enable', '/tidybot/control_frame')
    return [
        start('joy_node', 'joy_node', 'joy'),
        start('gamepad_teleop', 'tidybot_gamepad_teleop',
              overrides={'control_frame': 'local', 'require_both_deadman': True}, remappings=[
            ('/cmd_vel', '/teleop/cmd_vel'),
            ('/tidybot/enable', '/teleop/enable'),
            ('/tidybot/control_frame', '/teleop/control_frame'),
            ('/tidybot/stop', '/teleop/stop'),
        ]),
        start('control_mux', 'tidybot_control_mux', remappings=[
            (topic, '/manual_control' + topic) for topic in mux_outputs
        ] if mode == 'navigation' else ()),
        start('base_node', 'tidybot_base_node', overrides={'require_enable': True}),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('control_mode', default_value='manual', choices=['manual', 'navigation']),
        OpaqueFunction(function=control_nodes),
    ])
