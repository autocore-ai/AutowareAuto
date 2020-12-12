# Copyright 2020 Embotech AG
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

# This file contains tests for the maneuver planning behavior of the containing
# node.

import unittest

import ament_index_python
import launch
import launch.actions
import launch_ros.actions
from launch_ros.default_launch_description import ROSSpecificLaunchStartup
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
# from autoware_auto_msgs.msg import VehicleKinematicState, Trajectory

from parking_planner_actions.action import PlanParkingManeuver

import subprocess


class MockActionCaller(Node):
    def __init__(self, node_name, action_type, action_name):
        super().__init__(node_name)
        self._action_client = ActionClient(self, action_type, action_name)
        self.action_type = action_type

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("Goal successfully canceled")
        else:
            self.get_logger().info("Goal failed to cancel")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")
        self._goal_handle = goal_handle

    def feedback_callback(self, feedback):
        self.get_logger().info(
            "Received feedback: {0}".format(feedback.feedback.sequence)
        )

    def manual_cancel(self):
        self.get_logger().info("Canceling goal")
        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done)
        return cancel_future

    def send_goal(self):
        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()

        self.get_logger().info("Sending goal request...")
        goal_msg = self.action_type.Goal()
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        send_goal_future.add_done_callback(self.goal_response_callback)
        return send_goal_future


def generate_test_description(ready_fn):
    test_nodes = launch_ros.actions.Node(
        package="parking_planner_node",
        node_executable="parking_planner_node_exe",
        node_name="parking_planner",
        parameters=[
            "{}/defaults.param.yaml".format(
                ament_index_python.get_package_share_directory(
                    "parking_planner_node"
                )
            )
        ],
    )

    # integration test
    ld = launch.LaunchDescription(
        [
            ROSSpecificLaunchStartup(),
            test_nodes,
            launch.actions.OpaqueFunction(function=lambda context: ready_fn()),
        ]
    )

    # An array of all the checkers to be enumerated by the tests
    return ld


def helper_pub_topic_one(name: str, msgtype: str):
    return subprocess.run(["ros2", "topic", "pub", name, msgtype, "-1"])


# Test: Check if "happy case" of planning a maneuver successfully
class TestBasicUsage(unittest.TestCase):
    def test_happy_case_works(self):
        # - Start a recording action by sending a goal
        mock_planmaneuver_action_caller = MockActionCaller(
            "MockPlanManeuverCaller", PlanParkingManeuver,
            "planparkingmaneuver"
        )

        # - Send goal, then wait for the goal sending to complete
        send_goal_future = mock_planmaneuver_action_caller.send_goal()
        rclpy.spin_until_future_complete(mock_planmaneuver_action_caller,
                                         send_goal_future)
        rclpy.spin_once(mock_planmaneuver_action_caller, timeout_sec=2)

        # TODO(s.me) resume here
