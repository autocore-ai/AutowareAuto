# Copyright 2021 The Autoware Foundation
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
#
# Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """
    Launch the system modules.

     * autoware_state_monitor
     * emergency_handler
    """
    # Packages
    autoware_state_monitor_pkg_prefix = get_package_share_directory('autoware_state_monitor')
    emergency_handler_pkg_prefix = get_package_share_directory('emergency_handler')

    # Launch
    autoware_state_monitor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [autoware_state_monitor_pkg_prefix, '/launch/autoware_state_monitor.launch.py']),
        launch_arguments={}.items()
    )

    emergency_handler_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [emergency_handler_pkg_prefix, '/launch/emergency_handler.launch.py']),
        launch_arguments={}.items()
    )

    return LaunchDescription([
        autoware_state_monitor_launch,
        emergency_handler_launch,
    ])
