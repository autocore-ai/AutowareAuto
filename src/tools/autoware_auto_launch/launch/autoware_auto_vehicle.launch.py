# Copyright 2021 the Autoware Foundation
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

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """
    Launch vehicle nodes.

     * velodyne_nodes
     * ssc_interface
     * robot_state_publisher
     * odom_bl_publisher (hack #626)
    """
    autoware_auto_launch_pkg_prefix = get_package_share_directory(
        'autoware_auto_launch')

    vlp16_front_param_file = os.path.join(
        autoware_auto_launch_pkg_prefix, 'param/vlp16_front_vehicle.param.yaml')
    vlp16_rear_param_file = os.path.join(
        autoware_auto_launch_pkg_prefix, 'param/vlp16_rear_vehicle.param.yaml')
    ssc_interface_param_file = os.path.join(
        autoware_auto_launch_pkg_prefix, 'param/ssc_interface.param.yaml')

    urdf_pkg_prefix = get_package_share_directory('lexus_rx_450h_description')
    urdf_path = os.path.join(urdf_pkg_prefix, 'urdf/lexus_rx_450h_vehicle.urdf')

    # Argument
    with_lidars_param = DeclareLaunchArgument(
        'with_lidars',
        default_value='True',
        description='Launch lidar drivers in addition to other nodes'
    )
    vlp16_front_param = DeclareLaunchArgument(
        'vlp16_front_param_file',
        default_value=vlp16_front_param_file,
        description='Path to config file for front Velodyne'
    )
    vlp16_rear_param = DeclareLaunchArgument(
        'vlp16_rear_param_file',
        default_value=vlp16_rear_param_file,
        description='Path to config file for rear Velodyne'
    )
    ssc_interface_param = DeclareLaunchArgument(
        'ssc_interface_param_file',
        default_value=ssc_interface_param_file,
        description='Path to config file for SSC interface'
    )

    # Nodes
    vlp16_front = Node(
        package='velodyne_nodes',
        executable='velodyne_cloud_node_exe',
        namespace='lidar_front',
        parameters=[LaunchConfiguration('vlp16_front_param_file')],
        condition=IfCondition(LaunchConfiguration('with_lidars')),
        arguments=["--model", "vlp16"]
    )
    vlp16_rear = Node(
        package='velodyne_nodes',
        executable='velodyne_cloud_node_exe',
        namespace='lidar_rear',
        parameters=[LaunchConfiguration('vlp16_rear_param_file')],
        condition=IfCondition(LaunchConfiguration('with_lidars')),
        arguments=["--model", "vlp16"]
    )
    urdf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        arguments=[str(urdf_path)]
    )
    ssc_interface = Node(
        package='ssc_interface',
        executable='ssc_interface_node_exe',
        name='ssc_interface',
        namespace='vehicle',
        parameters=[LaunchConfiguration('ssc_interface_param_file')],
        remappings=[
            ('gear_select', '/ssc/gear_select'),
            ('arbitrated_speed_commands', '/ssc/arbitrated_speed_commands'),
            ('arbitrated_steering_commands', '/ssc/arbitrated_steering_commands'),
            ('turn_signal_command', '/ssc/turn_signal_command'),
            ('dbw_enabled_feedback', '/ssc/dbw_enabled_feedback'),
            ('gear_feedback', '/ssc/gear_feedback'),
            ('velocity_accel_cov', '/ssc/velocity_accel_cov'),
            ('state_report_out', 'state_report'),
            ('steering_feedback', '/ssc/steering_feedback'),
            ('vehicle_kinematic_state_cog', 'vehicle_kinematic_state')
        ]
    )

    # TODO(nikolai.morin): Hack, to be resolved in #626
    odom_bl_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"]
    )

    return LaunchDescription([
        with_lidars_param,
        vlp16_front_param,
        vlp16_rear_param,
        ssc_interface_param,
        vlp16_front,
        vlp16_rear,
        urdf_publisher,
        ssc_interface,
        odom_bl_publisher,
    ])
