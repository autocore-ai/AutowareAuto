# Copyright 2020 the Autoware Foundation
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
import ament_index_python
import unittest
import launch
import launch.actions
import launch_ros.actions
import launch_testing

import pytest


@pytest.mark.launch_test
def generate_test_description(ready_fn):
    # The node under test and the checker node that will pass/fail our tests:
    test_topic = "veloyne_cloud_node_test_topic"
    velodyne_cloud_node = launch_ros.actions.LifecycleNode(
        package="velodyne_nodes",
        node_executable="velodyne_cloud_node_exe",
        node_name="vlp16_driver_node",
        node_namespace="lidar_front",
        parameters=[
            "{}/param/vlp16_test.param.yaml".format(
                ament_index_python.get_package_share_directory("velodyne_nodes")
            ),
            {}
        ],
        remappings=[("points_raw", test_topic)],
        arguments=["--model", "vlp16"]
    )

    context = {'vel_node': velodyne_cloud_node}

    return launch.LaunchDescription([
        velodyne_cloud_node,
        # Start tests right away - no need to wait for anything
        launch.actions.OpaqueFunction(function=lambda context: ready_fn())]
    ), context


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_output, proc_info, vel_node):
        # Check that process exits with code -15 code: termination request, sent to the program
        launch_testing.asserts.assertExitCodes(proc_info, [-15], process=vel_node)
