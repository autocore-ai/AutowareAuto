# Copyright 2019 the Autoware Foundation
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

import unittest

import ament_index_python
import launch
import launch.actions
import launch_ros.actions

import launch_testing.event_handlers


def generate_test_description():
    test_nodes = launch_ros.actions.Node(
        package="pure_pursuit_nodes",
        executable="pure_pursuit_node_exe",
        name="pure_pursuit_node",
        parameters=[
            "{}/param/pure_pursuit_test.param.yaml".format(
                ament_index_python.get_package_share_directory("pure_pursuit_nodes")
            )
        ]
    )

    # integration test
    checker = launch_ros.actions.Node(
        package="pure_pursuit_nodes",
        executable="pure_pursuit_integration_test_exe")

    ld = launch.LaunchDescription([
        test_nodes,
        checker,
        launch_testing.actions.ReadyToTest()
    ])

    # An array of all the checkers to be enumerated by the tests
    return ld, {"_checkers": [checker]}


class TestWaitForEnd(unittest.TestCase):
    def test_checker_exits(self, _checkers):
        # The checker node exits automatically after about 10 seconds (specified in the launch)
        for checker in _checkers:
            self.proc_info.assertWaitForShutdown(process=checker, timeout=60)


@launch_testing.post_shutdown_test()
class TestCheckerOutput(unittest.TestCase):

    # TODO(Takamasa Horibe) Replace this test with one in the MPC.

    # def test_checker_outputs_success(self, _checkers):
    #     for checker in _checkers:
    #         try:
    #             launch_testing.asserts.assertInStdout(self.proc_output, "success", checker)
    #         except AssertionError:
    #             # For a failure, print a little more info before we re-raise the failure
    #             # exception
    #             print("\nFailed checker output:")
    #             for line in self.proc_output[checker]:
    #                 print(line.text.decode(), end='')
    #             raise

    def test_checker_exit_code_ok(self, _checkers):
        # Also check the exit code of the checker as a belt-and-suspenders approach
        for checker in _checkers:
            try:
                self.assertEqual(0, self.proc_info[checker].returncode)
            except AssertionError:
                # For a failure, print a little more info before we re-raise the failure
                # exception
                print("\nFailed checker output:")
                for line in self.proc_output[checker]:
                    print(line.text.decode(), end='')
                raise
