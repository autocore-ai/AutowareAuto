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


import ament_index_python
import launch
import launch_ros.actions


def generate_launch_description():
    """Launch parking_planner_node with default configuration."""
    # -------------------------------- Nodes-----------------------------------
    parking_planner_node = launch_ros.actions.Node(
        package='parking_planner_node',
        node_executable='parking_planner_node_exe',
        node_name='parking_planner',
        node_namespace='',
        output='screen',
        parameters=[
            "{}/param/defaults.param.yaml".format(
                ament_index_python.get_package_share_directory(
                    "parking_planner_node"
                )
            ),
        ],
        remappings=[
            ('vehicle_state', '/vehicle/vehicle_kinematic_state'),
            ('planned_trajectory', '/planning/trajectory'),
        ]
    )

    ld = launch.LaunchDescription([parking_planner_node])
    return ld
