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

# Co-developed by Tier IV, Inc. and Apex.AI, Inc.

"""Launch the prediction node."""

from ament_index_python import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """Generate launch description with a single component."""
    pkg_name = 'prediction_nodes'

    pkg_prefix = get_package_share_directory(pkg_name)

    prediction_nodes_param_file = os.path.join(pkg_prefix, 'param', 'test.param.yaml')

    pkg_param = DeclareLaunchArgument(
        pkg_name + '_param_file',
        default_value=prediction_nodes_param_file,
        description='Path to parameter file for prediction node'
    )

    prediction_node = Node(
        package=pkg_name,
        executable=pkg_name + '_node_exe',
        parameters=[LaunchConfiguration(pkg_name + '_param_file')],
    )

    return launch.LaunchDescription([pkg_param, prediction_node])
