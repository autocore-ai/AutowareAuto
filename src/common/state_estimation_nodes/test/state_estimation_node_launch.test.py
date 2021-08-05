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
# Co-developed by Tier IV, Inc., Robotec.ai, and Apex.AI, Inc.

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing

import os
import pytest
import unittest


@pytest.mark.launch_test
def generate_test_description():

    state_estimation_node = Node(
        package='state_estimation_nodes',
        executable='state_estimation_node_exe',
        namespace='test',
        parameters=[os.path.join(
            get_package_share_directory('state_estimation_nodes'),
            'param/test.param.yaml'
        )]
    )

    context = {'state_estimation_node': state_estimation_node}

    return LaunchDescription([
        state_estimation_node,
        # Start tests right away - no need to wait for anything
        launch_testing.actions.ReadyToTest()]
    ), context


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_output, proc_info, state_estimation_node):
        # Check that process exits with expected codes: either SIGINT or SIGTERM codes are fine
        launch_testing.asserts.assertExitCodes(proc_info, [-2, -15], process=state_estimation_node)
