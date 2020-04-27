# Copyright 2019 Apex.AI, Inc.
# All rights reserved.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():
    """
    Launches a minimal joystick + LGSVL demo. Under the default configuration, the joystick
    translator outputs and the LGSVL interface expects RawControlCommand. Controlling the vehicle
    can happen via the gamepad triggers and left joystick.
    """

    # --------------------------------- Params -------------------------------

    # Default joystick translator params
    joy_translator_param = DeclareLaunchArgument(
        'joy_translator_param',
        default_value=[
            get_share_file('joystick_vehicle_interface', 'logitech_f310.default.param.yaml')
        ],
        description='Path to config file for joystick translator')

    # Default lgsvl_interface params
    lgsvl_interface_param = DeclareLaunchArgument(
        'lgsvl_interface_param',
        default_value=[
            get_share_file('lgsvl_interface', 'lgsvl.param.yaml')
        ],
        description='Path to config file for lgsvl interface')

    # -------------------------------- Nodes-----------------------------------

    # Include Joystick launch
    joystick_launch_file_path = get_share_file('joystick_vehicle_interface',
                                               'joystick_vehicle_interface.launch.py')
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joystick_launch_file_path),
        launch_arguments=[
            (
                "joy_translator_param",
                LaunchConfiguration("joy_translator_param")
            )
        ]
    )

    # Include LGSVL interface launch
    lgsvl_launch_file_path = get_share_file('lgsvl_interface',
                                            'lgsvl.launch.py')
    lgsvl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lgsvl_launch_file_path),
        launch_arguments=[
            (
                "lgsvl_interface_param",
                LaunchConfiguration("lgsvl_interface_param")
            )
        ]
    )

    return LaunchDescription([
      joy_translator_param,
      lgsvl_interface_param,
      joystick,
      lgsvl
    ])
