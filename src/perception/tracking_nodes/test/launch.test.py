# Copyright 2021 Autoware Foundation
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
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing

import os
import pytest


@pytest.mark.launch_test
def generate_test_description():

    multi_object_tracker = Node(
        package='tracking_nodes',
        executable='multi_object_tracker_node_exe',
        name='multi_object_tracker_node',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('tracking_nodes'), 'param/test.param.yaml')
        ],
    )

    context = {'multi_object_tracker': multi_object_tracker}

    launch_description = LaunchDescription([
        multi_object_tracker,
        # Start tests right away - no need to wait for anything
        launch_testing.actions.ReadyToTest()])

    return launch_description, context
