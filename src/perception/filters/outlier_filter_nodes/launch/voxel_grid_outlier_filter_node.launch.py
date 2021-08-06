# Copyright 2021 Tier IV, Inc
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

"""
Example launch file for the VoxelGridOutlierFilterNode executable.

Note: Does not work in ROS2 dashing!
"""

import os
import launch

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with a single component."""
    container = Node(
        package='outlier_filter_nodes',
        executable='voxel_grid_outlier_filter_node_exe',
        namespace='test',
        parameters=[os.path.join(
            get_package_share_directory('outlier_filter_nodes'),
            'param/voxel_grid_outlier_filter_node_test.param.yaml'
        )],
        output='screen',
    )

    return launch.LaunchDescription([container])
