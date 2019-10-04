# Copyright 2019 Apex.AI, Inc.
# All rights reserved.

import unittest

import ament_index_python
import launch
import launch.actions
import launch_ros.actions
from launch_ros.default_launch_description import ROSSpecificLaunchStartup

import launch_testing.event_handlers


def generate_test_description(ready_fn):
    test_nodes = launch_ros.actions.Node(
        package="pure_pursuit_nodes",
        node_executable="pure_pursuit_node_exe",
        node_name="pure_pursuit_node",
        parameters=[
            "{}/pure_pursuit_test.param.yaml".format(
                ament_index_python.get_package_share_directory("pure_pursuit_nodes")
            )
        ]
    )

    # integration test
    checker = launch_ros.actions.Node(
        package="pure_pursuit_nodes",
        node_executable="pure_pursuit_integration_test_exe")

    ld = launch.LaunchDescription([
        ROSSpecificLaunchStartup(),
        test_nodes,
        checker,
        launch.actions.OpaqueFunction(function=lambda context: ready_fn())
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

    def test_checker_outputs_success(self, _checkers):
        for checker in _checkers:
            try:
                launch_testing.asserts.assertInStdout(self.proc_output, "success", checker)
            except AssertionError:
                # For a failure, print a little more info before we re-raise the failure
                # exception
                print("\nFailed checker output:")
                for line in self.proc_output[checker]:
                    print(line.text.decode(), end='')
                raise

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
