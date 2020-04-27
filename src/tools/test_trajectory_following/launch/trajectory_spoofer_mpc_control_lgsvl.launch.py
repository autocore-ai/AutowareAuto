# Copyright 2020 The Autoware Foundation.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Testing of mpc_controller in LGSVL simulation using trajectory_spoofer."""

import launch.launch_description_sources
import launch.substitutions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():
    """
    Launch nodes with params to test mpc_controller in simulation.

    mpc_controller + LGSVL + trajectory_spoofer + lexus_rx_450h_description
    """
    # --------------------------------- Params -------------------------------
    mpc_controller_param_file = get_share_file(
        package_name='test_trajectory_following', file_name='mpc_controller.param.yaml')
    lgsvl_interface_param_file = get_share_file(
        package_name='test_trajectory_following', file_name='lgsvl_interface.param.yaml')
    rviz_cfg_path = get_share_file(
        package_name='test_trajectory_following', file_name='mpc_cotrols.rviz')
    trajectory_spoofer_param_file = get_share_file(
        package_name='test_trajectory_following', file_name='trajectory_spoofer.param.yaml')
    lexus_rx_450h_urdf_path = get_share_file(
        package_name='lexus_rx_450h_description', file_name='urdf/lexus_rx_450h.urdf')

    # --------------------------------- Arguments -------------------------------

    trajectory_spoofer_param = DeclareLaunchArgument(
        'trajectory_spoofer_param_file',
        default_value=trajectory_spoofer_param_file,
        description='Path to config file for Trajectory Spoofer'
    )
    mpc_controller_param = DeclareLaunchArgument(
        'mpc_controller_param_file',
        default_value=mpc_controller_param_file,
        description='Path to config file for MPC Controller'
    )
    lgsvl_interface_param = DeclareLaunchArgument(
        'lgsvl_interface_param_file',
        default_value=lgsvl_interface_param_file,
        description='Path to config file for LGSVL Interface'
    )

    # -------------------------------- Nodes-----------------------------------

    trajectory_spoofer_node = Node(
        package="trajectory_spoofer",
        node_executable="trajectory_spoofer_exe",
        node_name="trajectory_spoofer",
        node_namespace='planning',
        parameters=[LaunchConfiguration('trajectory_spoofer_param_file')],
        remappings=[
            ("vehicle_kinematic_state", "/vehicle/vehicle_kinematic_state")
        ],
        output='screen',
    )

    mpc_controller_node = Node(
        package="mpc_controller_node",
        node_executable="mpc_controller_node_exe",
        node_name="mpc_controller",
        node_namespace='control',
        parameters=[LaunchConfiguration('mpc_controller_param_file')],
        output='screen',
    )

    lgsvl_interface_node = Node(
        package='lgsvl_interface',
        node_executable='lgsvl_interface_exe',
        node_namespace='vehicle',
        output='screen',
        parameters=[LaunchConfiguration('lgsvl_interface_param_file')],
    )

    # lexus_rx_450h_description
    lexus_rx_450h_description = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        node_name='robot_state_publisher',
        arguments=[str(lexus_rx_450h_urdf_path)]
    )

    rviz2 = Node(
        package='rviz2',
        node_executable='rviz2',
        node_name='rviz2',
        arguments=['-d', str(rviz_cfg_path)]
    )

    return launch.LaunchDescription([
        trajectory_spoofer_param,
        trajectory_spoofer_node,
        mpc_controller_param,
        mpc_controller_node,
        lgsvl_interface_param,
        lgsvl_interface_node,
        lexus_rx_450h_description,
        rviz2
    ])
