"""D415 docking with separate perception, control and arbitration processes."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction, ResetLaunchConfigurations, Shutdown)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    enabled = lambda name: value(name).lower() in ('true', '1', 'yes')
    config, gamepad_config = value('config'), value('gamepad_config')
    dry_run = enabled('dry_run')
    output_prefix = '/docking/preview' if dry_run else ''
    actions = []
    if enabled('start_camera'):
        try:
            camera_share = get_package_share_directory('realsense2_camera')
        except PackageNotFoundError as exc:
            raise RuntimeError(
                'RealSense ROS driver missing. Run scripts/install_docking_deps.sh, '
                'or start with start_camera:=false to use an existing image stream.') from exc
        camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(camera_share)/'launch'/'rs_launch.py')),
            launch_arguments={
                'camera_namespace': 'camera', 'camera_name': 'camera',
                'enable_color': 'true', 'enable_depth': 'false',
                'enable_infra1': 'false', 'enable_infra2': 'false',
                'rgb_camera.color_profile': value('color_profile'),
            }.items())
        # The RealSense launch validates every visible launch argument. Give it
        # its own scope so docking options are not reported as invalid parameters.
        actions.append(GroupAction(actions=[ResetLaunchConfigurations(), camera_launch]))
    if enabled('start_gamepad'):
        actions.extend([
            Node(package='joy', executable='joy_node', name='joy_node', output='screen',
                 parameters=[gamepad_config], on_exit=[Shutdown(reason='Joy driver exited')]),
            Node(package='tidybot_ros2_control', executable='gamepad_teleop',
                 name='tidybot_gamepad_teleop', output='screen',
                 parameters=[gamepad_config, {'control_frame': 'local', 'require_both_deadman': True}],
                 remappings=[('/cmd_vel', '/teleop/cmd_vel'),
                             ('/tidybot/enable', '/teleop/enable'),
                             ('/tidybot/control_frame', '/teleop/control_frame'),
                             ('/tidybot/stop', '/teleop/stop')],
                 on_exit=[Shutdown(reason='Gamepad teleop exited')]),
        ])
    for executable, name in [('aruco_detector', 'tidybot_aruco_detector'),
                             ('docking_controller', 'tidybot_docking_controller')]:
        parameters = [config]
        if executable == 'docking_controller':
            parameters.append({'teach_on_start': enabled('teach_on_start')})
        actions.append(Node(package='tidybot_ros2_control', executable=executable, name=name,
                            output='screen', parameters=parameters,
                            on_exit=[Shutdown(reason=f'{name} exited')]))
    actions.append(Node(
        package='tidybot_ros2_control', executable='control_mux', name='tidybot_control_mux',
        output='screen', parameters=[config],
        remappings=[('/cmd_vel', output_prefix+'/cmd_vel'),
                    ('/tidybot/enable', output_prefix+'/tidybot/enable'),
                    ('/tidybot/control_frame', output_prefix+'/tidybot/control_frame')],
        on_exit=[Shutdown(reason='Control mux exited')]))
    if enabled('show_image'):
        actions.append(Node(
            package='rqt_image_view', executable='rqt_image_view', name='docking_image_view',
            # The rqt script uses env python3; use ROS's system Python even in Conda.
            prefix='/usr/bin/python3', arguments=[value('image_topic')], output='screen'))
    if enabled('start_base'):
        if dry_run:
            raise RuntimeError('start_base:=true requires dry_run:=false; preview never starts motors')
        actions.append(Node(
            package='tidybot_ros2_control', executable='base_node', name='tidybot_base_node',
            output='screen', emulate_tty=True, parameters=[gamepad_config],
            on_exit=[Shutdown(reason='Base driver exited')]))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('tidybot_ros2_control'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(share/'config'/'docking.yaml')),
        DeclareLaunchArgument('gamepad_config', default_value=str(share/'config'/'gamepad.yaml')),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('color_profile', default_value='640x480x30',
                              description='D415 RGB profile; default verified over USB 2; '
                                          '1280x720x30 can be selected with a suitable USB 3 connection'),
        DeclareLaunchArgument('show_image', default_value='true',
                              description='Open the image viewer; set false when running without a desktop'),
        DeclareLaunchArgument('image_topic', default_value='/docking/debug_image',
                              description='Image topic to display automatically in rqt_image_view'),
        DeclareLaunchArgument('start_gamepad', default_value='true'),
        DeclareLaunchArgument('start_base', default_value='false'),
        DeclareLaunchArgument('teach_on_start', default_value='false',
                              description='First setup only: robot must already be at the stop pose; '
                                          'record a missing target after valid observations, without starting docking'),
        DeclareLaunchArgument('dry_run', default_value='true',
                              description='Route commands to preview topics; never start the base'),
        OpaqueFunction(function=_launch),
    ])
