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


import os

from ament_index_python import get_package_share_directory

import launch

from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""
    path_package = get_package_share_directory('vehicle_constants_manager_nodes')

    path_file_param = os.path.join(path_package, 'param/params_lexus_rx_hybrid_2016.yaml')

    name_launch_arg = 'param_file_vehicle_constants_manager_nodes'

    launch_arg = DeclareLaunchArgument(
        name_launch_arg,
        default_value=path_file_param,
        description='Path to params file for Vehicle Constants Manager Node'
    )

    vehicle_constants_manager_node = Node(
        package='vehicle_constants_manager_nodes',
        executable='vehicle_constants_manager_node_exe',
        namespace='',
        output='screen',
        parameters=[
            LaunchConfiguration(name_launch_arg)
        ]
    )

    return launch.LaunchDescription([launch_arg,
                                     vehicle_constants_manager_node])
