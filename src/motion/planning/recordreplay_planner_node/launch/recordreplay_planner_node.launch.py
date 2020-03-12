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


import launch
import launch_ros.actions
import launch.substitutions
import ros2launch.api


def get_param(package_name, param_file):
    return ros2launch.api.get_share_file_path_from_package(
        package_name=package_name,
        file_name=param_file
    )


def generate_launch_description():
    """Launch recordreplay_planner_node with default configuration."""
    # --------------------------------- Params -------------------------------
    recordreplay_planner_param = launch.actions.DeclareLaunchArgument(
        'recordreplay_planner_param',
        default_value=[
            get_param('recordreplay_planner_node', 'defaults.param.yaml')
        ],
        description='Path to config file for recordreplay planner node')

    # -------------------------------- Nodes-----------------------------------
    recordreplay_planner_node = launch_ros.actions.Node(
        package='recordreplay_planner_node',
        node_executable='recordreplay_planner_node_exe',
        output='screen',
        parameters=[launch.substitutions.LaunchConfiguration('recordreplay_planner_param')])

    ld = launch.LaunchDescription([
        recordreplay_planner_param,
        recordreplay_planner_node])
    return ld
