# Copyright 2020, The Autoware Foundation
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

"""Launch Modules for Milestone 2 of the AVP 2020 Demo."""

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros import actions

import os


def generate_launch_description():
    """
    Launch all nodes defined in the architecture for Milestone 2 of the AVP 2020 Demo.

    More details about what is included can
    be found at https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/milestones/24.
    """
    avp_demo_pkg_prefix = get_package_share_directory('autoware_auto_avp_demo')
    rviz_cfg_path = os.path.join(avp_demo_pkg_prefix, 'config/ms2.rviz')
    lgsvl_pkg_prefix = get_package_share_directory('lgsvl_interface')
    lgsvl_param_file = os.path.join(lgsvl_pkg_prefix, 'lgsvl.param.yaml')
    urdf_pkg_prefix = get_package_share_directory('lexus_rx_450h_description')
    urdf_path = os.path.join(urdf_pkg_prefix, 'urdf/lexus_rx_450h.urdf')
    ndt_nodes_pkg_prefix = get_package_share_directory('ndt_nodes')

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'lgsvl_interface_param',
            default_value=lgsvl_param_file,
            description='Path to config file for lgsvl interface'
        ),
        DeclareLaunchArgument(
            'with_rviz',
            default_value='True',
            description='Launch RVIZ2 in addition to other nodes'
        ),
        # LGSVL Interface
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lgsvl_pkg_prefix, '/lgsvl.launch.py']),
            launch_arguments={
                'lgsvl_interface_param': LaunchConfiguration('lgsvl_interface_param')
            }.items(),
        ),
        # URDF Publishing
        actions.Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            node_name='robot_state_publisher',
            arguments=[str(urdf_path)]
        ),
        # RVIZ2
        actions.Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', str(rviz_cfg_path)],
            condition=IfCondition(LaunchConfiguration('with_rviz'))
        ),
        # pcd map provider from ndt_nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(ndt_nodes_pkg_prefix, 'launch'),
                    '/map_provider.launch.py'
                ]
            ),
        ),
    ])
