# Copyright 2020 Apex.AI, Inc.
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
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.

"""Launch LaneLet2 Global Path Planner nodes."""

import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    """Launch LaneLet2 Global Path Planner nodes."""
    # execution definition.
    global_planner_node_runner = launch_ros.actions.Node(
        package='lanelet2_global_planner',
        node_executable='lanelet2_global_planner_exe',
        output='screen',
        )

    return launch.LaunchDescription([global_planner_node_runner])
