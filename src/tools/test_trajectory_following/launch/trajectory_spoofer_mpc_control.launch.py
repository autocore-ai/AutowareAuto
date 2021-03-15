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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def IfEqualsCondition(arg_name: str, value: str):
    return IfCondition(PythonExpression([
        '"', LaunchConfiguration(arg_name), '" == "', value, '"'
    ]))


def generate_launch_description():
    """
    Launch nodes with params to test mpc_controller in simulation.

    mpc_controller + LGSVL + trajectory_spoofer + lexus_rx_450h_description
    """
    # --------------------------------- Params -------------------------------
    mpc_controller_param_file = get_share_file(
        package_name='test_trajectory_following', file_name='param/mpc_controller.param.yaml')
    lgsvl_interface_param_file = get_share_file(
        package_name='test_trajectory_following', file_name='param/lgsvl_interface.param.yaml')
    controller_testing_param_file = get_share_file(
        package_name='test_trajectory_following', file_name='param/controller_testing.param.yaml')
    rviz_cfg_path = get_share_file(
        package_name='test_trajectory_following', file_name='config/default_control.rviz')
    trajectory_spoofer_param_file = get_share_file(
        package_name='test_trajectory_following', file_name='param/trajectory_spoofer.param.yaml')
    urdf_path = get_share_file(
        package_name='lexus_rx_450h_description', file_name='urdf/lexus_rx_450h.urdf')
    with open(urdf_path, 'r') as infp:
        urdf_file = infp.read()

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
    controller_testing_param = DeclareLaunchArgument(
        'controller_testing_param_file',
        default_value=controller_testing_param_file,
        description='Path to config file for dynamics simulator'
    )
    with_sim_type_param = DeclareLaunchArgument(
        'sim_type',
        default_value='dynamics',   # lgsvl/ dynamics/ kinematics
        description='Use LGSVL or (headerless simulator) either dynamics or kinemetics'
    )
    real_time_sim_param = DeclareLaunchArgument(
        'real_time_sim',
        default_value='True',
        description='Run dynamics simulator in Realtime mode or faster'
    )
    with_rviz_param = DeclareLaunchArgument(
        'with_rviz',
        default_value='True',
        description='Launch RVIZ2 in addition to other nodes'
    )

    # -------------------------------- Nodes-----------------------------------

    trajectory_spoofer_node = Node(
        package="trajectory_spoofer",
        executable="trajectory_spoofer_node_exe",
        name="trajectory_spoofer_node",
        namespace='planning',
        parameters=[LaunchConfiguration('trajectory_spoofer_param_file')],
        remappings=[
            ("vehicle_kinematic_state", "/vehicle/vehicle_kinematic_state")
        ],
        output='screen',
        # delay added to allow rviz to be ready, better to start rviz separately, beforehand
        prefix="bash -c 'sleep 1.0; $0 $@'",
    )

    mpc_controller_nodes = Node(
        package="mpc_controller_nodes",
        executable="mpc_controller_node_exe",
        name="mpc_controller",
        namespace='control',
        parameters=[LaunchConfiguration('mpc_controller_param_file')],
        output='screen',
    )

    lgsvl_interface_node = Node(
        package='lgsvl_interface',
        executable='lgsvl_interface_exe',
        namespace='vehicle',
        output='screen',
        parameters=[LaunchConfiguration('lgsvl_interface_param_file')],
        remappings=[
            ("vehicle_control_cmd", "/lgsvl/vehicle_control_cmd"),
            ("vehicle_state_cmd", "/lgsvl/vehicle_state_cmd"),
            ("state_report", "/lgsvl/state_report"),
            ("state_report_out", "state_report"),
            ("gnss_odom", "/lgsvl/gnss_odom"),
            ("vehicle_odom", "/lgsvl/vehicle_odom")
        ],
        condition=IfEqualsCondition("sim_type", "lgsvl")
    )

    controller_testing = Node(
        package="controller_testing",
        executable="controller_testing_main.py",
        namespace="vehicle",
        name="controller_testing_node",
        output="screen",
        parameters=[LaunchConfiguration("controller_testing_param_file"), {
            'real_time_sim': LaunchConfiguration('real_time_sim')
        }],
        remappings=[
            ("vehicle_state", "/vehicle/vehicle_kinematic_state"),
            ("planned_trajectory", "/planning/trajectory"),
            ("control_command", "/vehicle/vehicle_command"),
        ],
        condition=IfEqualsCondition("sim_type", "dynamics")
    )

    vehicle_kinematics_sim_node = Node(
        package='test_trajectory_following',
        executable='vehicle_kinematics_sim.py',
        namespace='vehicle',
        output='screen',
        condition=IfEqualsCondition("sim_type", "kinematics")
    )
    # lexus_rx_450h_description
    lexus_rx_450h_description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': urdf_file}],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_cfg_path)],
        condition=IfCondition(LaunchConfiguration('with_rviz'))
    )

    return launch.LaunchDescription([
        with_rviz_param,
        with_sim_type_param,
        real_time_sim_param,
        trajectory_spoofer_param,
        trajectory_spoofer_node,
        mpc_controller_param,
        mpc_controller_nodes,
        lgsvl_interface_param,
        lgsvl_interface_node,
        controller_testing_param,
        controller_testing,
        vehicle_kinematics_sim_node,
        lexus_rx_450h_description,
        rviz2
    ])
