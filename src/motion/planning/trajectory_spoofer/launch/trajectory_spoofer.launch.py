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
import launch.launch_description_sources
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    """Launch trajectory_spoofer node."""
    trajectory_spoofer_node = launch_ros.actions.Node(
        package="trajectory_spoofer",
        node_executable="trajectory_spoofer_exe",
        node_name="trajectory_spoofer",
        parameters=[
            {
                "speed_ramp_on": False,
                "target_speed": 10.0,
                "num_of_points": 100,
                "trajectory_type": 'straight',  # straight or circle
                "length": 10.0,  # only used for straight
                "radius": 21.0,  # only used for circle
            }
        ],
        remappings=[
            ("vehicle_kinematic_state", "/vehicle/vehicle_kinematic_state")
        ],
        output='screen',
    )

    return launch.LaunchDescription([
        trajectory_spoofer_node
    ])
