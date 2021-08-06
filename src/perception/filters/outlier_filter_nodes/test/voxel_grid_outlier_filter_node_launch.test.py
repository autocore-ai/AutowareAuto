# Copyright 2021 Tier IV, Inc
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

import os

import unittest

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing

import pytest


@pytest.mark.launch_test
def generate_test_description():

    voxel_grid_outlier_filter_node = Node(
        package='outlier_filter_nodes',
        executable='voxel_grid_outlier_filter_node_exe',
        namespace='test',
        parameters=[os.path.join(
            get_package_share_directory('outlier_filter_nodes'),
            'param/voxel_grid_outlier_filter_node_test.param.yaml'
        )]
    )

    context = {'voxel_grid_outlier_filter_node': voxel_grid_outlier_filter_node}

    return LaunchDescription([
        voxel_grid_outlier_filter_node,
        # Start tests right away - no need to wait for anything
        launch_testing.actions.ReadyToTest()]
    ), context


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_output, proc_info, voxel_grid_outlier_filter_node):
        # Check that process exits with code -15 code: termination request, sent to the program
        launch_testing.asserts.assertExitCodes(
            proc_info,
            [-15, -2],
            process=voxel_grid_outlier_filter_node
        )
