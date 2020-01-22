# Copyright 2019 Apex.AI, Inc.
# All rights reserved.

import launch
import launch_ros.actions
import launch.substitutions
import launch.launch_description_sources
import ros2launch.api


def get_param(package_name, param_file):
    return ros2launch.api.get_share_file_path_from_package(
        package_name=package_name,
        file_name=param_file
    )


def generate_launch_description():
    # CLI
    joy_translator_param = launch.actions.DeclareLaunchArgument(
        'joy_translator_param',
        default_value=[
            get_param('joystick_vehicle_interface', 'logitech_f310.default.param.yaml')
        ],
        description='Path to config file for joystick translator')
    # Launch nodes
    lgsvl_interface = launch_ros.actions.Node(
        package='lgsvl_interface',
        node_executable='lgsvl_interface_exe',
        node_name='lgsvl_interface',
        parameters=[get_param("lgsvl_interface", "lgsvl.param.yaml")],
        output='screen')
    # Joystick stuff
    launch_file_path = get_param('joystick_vehicle_interface',
                                 'joystick_vehicle_interface.launch.py')
    joystick = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(launch_file_path),
        launch_arguments=[
            (
                "joy_translator_param",
                launch.substitutions.LaunchConfiguration("joy_translator_param")
            )
        ]
    )

    return launch.LaunchDescription([
      joy_translator_param,
      joystick,
      lgsvl_interface])
