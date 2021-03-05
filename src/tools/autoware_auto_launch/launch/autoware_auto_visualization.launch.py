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
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """
    Launch visualization nodes.

     * rviz2
     * avp_web_interface
     * rosbridge_server
    """
    avp_web_interface_pkg_prefix = get_package_share_directory(
        'avp_web_interface')
    web_files_root = os.path.join(avp_web_interface_pkg_prefix, 'web')

    # Arguments
    with_rviz_param = DeclareLaunchArgument(
        'with_rviz',
        default_value='True',
        description='Launch RVIZ2 in addition to other nodes'
    )
    rviz_cfg_path_param = DeclareLaunchArgument(
        'rviz_cfg_path_param',
        default_value='',
        description='Launch RVIZ2 with the specified config file'
    )

    # Nodes
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_cfg_path_param)],
        condition=IfCondition(LaunchConfiguration('with_rviz')),
        remappings=[("initialpose", "/localization/initialpose"),
                    ("goal_pose", "/planning/goal_pose")],
    )
    web_bridge = Node(
        package='rosbridge_server',
        name='rosbridge_server_node',
        namespace='gui',
        executable='rosbridge_websocket'
    )
    web_server = ExecuteProcess(
      cmd=["python3", "-m", "http.server", "8000"],
      cwd=web_files_root
    )

    return LaunchDescription([
        with_rviz_param,
        rviz_cfg_path_param,
        rviz2,
        web_server,
        web_bridge,
    ])
