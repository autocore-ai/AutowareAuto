#!/usr/bin/env python3

# Copyright 2020 StreetScooter GmbH, Aachen, Germany
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Modules for testing mpc with motion_model_testing_simulator."""

from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackage
from pathlib import Path

import os


context = LaunchContext()


def get_package_share_directory(package_name):
    """Return the absolute path to the share directory of the given package."""
    return os.path.join(
        Path(FindPackage(package_name).perform(context)), "share", package_name
    )


def generate_launch_description():
    """Launch controller_testing_node and mpc_controller."""
    controller_testing_pkg_prefix = get_package_share_directory("controller_testing")
    controller_testing_param_file = os.path.join(
        controller_testing_pkg_prefix, "defaults.param.yaml"
    )

    mpc_controller_pkg_prefix = get_package_share_directory("mpc_controller_node")
    mpc_controller_param_file = os.path.join(mpc_controller_pkg_prefix, "defaults.yaml")

    # Arguments
    controller_testing_param = DeclareLaunchArgument(
        "controller_testing_param_file",
        default_value=controller_testing_param_file,
        description="Path to config file for Controller Testing",
    )

    mpc_controller_param = DeclareLaunchArgument(
        "mpc_controller_param_file",
        default_value=mpc_controller_param_file,
        description="Path to config file to MPC Controller",
    )

    # Nodes

    controller_testing = Node(
        package="controller_testing",
        node_executable="controller_testing_main.py",
        node_namespace="control",
        node_name="controller_testing_node",
        output="screen",
        parameters=[LaunchConfiguration("controller_testing_param_file"), {}],
        remappings=[
            ("vehicle_state", "/vehicle/vehicle_kinematic_state"),
            ("planned_trajectory", "/planning/trajectory"),
            ("control_command", "/vehicle/control_command"),
        ],
        on_exit=Shutdown(),
    )

    # mpc_controller
    mpc_controller = Node(
        package="mpc_controller_node",
        node_executable="mpc_controller_node_exe",
        # node_namespace="control",
        node_name="mpc_controller",
        output="screen",
        parameters=[LaunchConfiguration("mpc_controller_param_file"), {}],
        remappings=[
            ("vehicle_kinematic_state", "/vehicle/vehicle_kinematic_state"),
            ("trajectory", "/planning/trajectory"),
            ("ctrl_cmd", "/vehicle/control_command"),
        ],
    )

    return LaunchDescription(
        [
            controller_testing_param,
            mpc_controller_param,
            controller_testing,
            mpc_controller,
        ]
    )
