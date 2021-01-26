# Copyright 2021 the Autoware Foundation
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

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
import launch_testing

import pytest
import unittest


@pytest.mark.launch_test
def generate_test_description(ready_fn):

    lanelet2_map_visualizer = Node(
        package='lanelet2_map_provider',
        node_executable='lanelet2_map_visualizer_exe',
        node_namespace='had_maps')

    context = {'lanelet2_map_visualizer': lanelet2_map_visualizer}

    return LaunchDescription([
        lanelet2_map_visualizer,
        # Start tests right away - no need to wait for anything
        OpaqueFunction(function=lambda context: ready_fn())]
    ), context


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_output, proc_info, lanelet2_map_visualizer):
        # Check that process exits with code -15 code: termination request, sent to the program
        launch_testing.asserts.assertExitCodes(proc_info, [-15], process=lanelet2_map_visualizer)
