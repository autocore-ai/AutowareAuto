# Copyright 2021 Arm Ltd.
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

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing

import os
import pytest
import unittest


@pytest.mark.launch_test
def generate_test_description():

    apollo_lidar_segmentation = Node(
        package='apollo_lidar_segmentation_nodes',
        executable='apollo_lidar_segmentation_nodes_exe',
        name='apollo_lidar_segmentation_nodes',
        namespace='benchmark',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('apollo_lidar_segmentation_nodes'),
            'param/test.param.yaml'
        )],
    )

    context = {'apollo_lidar_segmentation': apollo_lidar_segmentation}

    launch_description = LaunchDescription([
        apollo_lidar_segmentation,
        # Start tests right away - no need to wait for anything
        launch_testing.actions.ReadyToTest()])

    return launch_description, context


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_output, proc_info, apollo_lidar_segmentation):
        # Check that process exits with code -15 code: termination request, sent to the program
        launch_testing.asserts.assertExitCodes(proc_info, [-15], process=apollo_lidar_segmentation)
