# Copyright 2021 Apex.AI, Inc.
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
import unittest

import ament_index_python

import launch
import launch.actions

import launch_ros.actions

import launch_testing
import launch_testing.util


def generate_test_description(ready_fn):
    # The node under test and the checker node that will pass/fail our tests:
    state_estimation_node = launch_ros.actions.Node(
        package="state_estimation_nodes",
        node_executable="state_estimation_node_exe",
        node_name="state_estimation_node",
        node_namespace="state_estimation_namespace",
        parameters=[
            "{}/param/state_estimation_node_test.param.yaml".format(
                ament_index_python.get_package_share_directory(
                    "state_estimation_nodes")
            ),
            {"output_frequency": 30.0}
        ]
    )

    context = {'state_node': state_estimation_node}

    return launch.LaunchDescription([
        state_estimation_node,
        # Need to keep the launch alive by having an alive process
        launch_testing.util.KeepAliveProc(),
        # Start tests right away - no need to wait for anything
        launch.actions.OpaqueFunction(function=lambda context: ready_fn())]
    ), context


class TestWaitForShutdown(unittest.TestCase):

    def test_wait_for_exit(self, proc_info, state_node):
        proc_info.assertWaitForShutdown(process=state_node, timeout=20)


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_output, proc_info, state_node):
        # Check that process exits with code -6 (failure due to uncaught error
        launch_testing.asserts.assertExitCodes(proc_info, [-6],
                                               process=state_node)

        # Check that correct error message is produced, we get the same error
        # code if the configuration file is not found
        ref_stderr = "what():  Please provide either " + \
            "'output_frequency' setting or 'data_driven' one, not both."
        full_stderr = "".join(
            output.text.decode() for output in proc_output[state_node]
            if output.from_stderr
        )
        self.assertIn(ref_stderr, full_stderr)
