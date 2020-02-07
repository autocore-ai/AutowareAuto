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
    """
    Launches a minimal joystick + LGSVL demo. Under the default configuration, the joystick
    translator outputs and the LGSVL interface expects RawControlCommand. Controlling the vehicle
    can happen via the gamepad triggers and left joystick.
    """
    # CLI
    joy_translator_param = launch.actions.DeclareLaunchArgument(
        'joy_translator_param',
        default_value=[
            get_param('joystick_vehicle_interface', 'logitech_f310.default.param.yaml')
        ],
        description='Path to config file for joystick translator')
    lgsvl_interface_param = launch.actions.DeclareLaunchArgument(
        'lgsvl_interface_param',
        default_value=[
            get_param('lgsvl_interface', 'lgsvl.param.yaml')
        ],
        description='Path to config file for lgsvl interface')
    # Joystick stuff
    joystick_launch_file_path = get_param('joystick_vehicle_interface',
                                 'joystick_vehicle_interface.launch.py')
    joystick = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(joystick_launch_file_path),
        launch_arguments=[
            (
                "joy_translator_param",
                launch.substitutions.LaunchConfiguration("joy_translator_param")
            )
        ]
    )
    # LGSVL stuff
    lgsvl_launch_file_path = get_param('lgsvl_interface',
                                 'lgsvl.launch.py')
    lgsvl = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(lgsvl_launch_file_path),
        launch_arguments=[
            (
                "lgsvl_interface_param",
                launch.substitutions.LaunchConfiguration("lgsvl_interface_param")
            )
        ]
    )

    return launch.LaunchDescription([
      joy_translator_param,
      lgsvl_interface_param,
      joystick,
      lgsvl])
