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

# This file contains tests for the record and replay behavior of the containing node.
# I'll start by conceptually documenting the tests I want to add, then implement them.

import unittest

import ament_index_python
import launch
import launch.actions
import launch_ros.actions
from launch_ros.default_launch_description import ROSSpecificLaunchStartup
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from autoware_auto_msgs.msg import VehicleKinematicState, Trajectory

from recordreplay_planner_actions.action import RecordTrajectory, ReplayTrajectory

import subprocess
import time


# Class to publish some dummy states
class MockStatePublisher(Node):
    def __init__(self):
        super().__init__("MockSatePublisher")
        self.publisher_ = self.create_publisher(
            VehicleKinematicState, "vehicle_kinematic_state", 10
        )

    def publish_a_state(self):
        msg = VehicleKinematicState()
        self.publisher_.publish(msg)
        self.get_logger().info("Publishing ego state...")


class MockTrajectoryMonitor(Node):
    def __init__(self):
        super().__init__("MockTrajectoryMonitor")
        self.subscription_ = self.create_subscription(
            Trajectory, "trajectory", self.listener_callback, 10
        )

    def listener_callback(self, msg):
        self.get_logger().info('Received: "{}"'.format(msg))


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

        self._goal_handle = goal_handle
        self.get_logger().info("Goal accepted :)")

    def feedback_callback(self, feedback):
        self.get_logger().info(
            "Received feedback: {0}".format(feedback.feedback.sequence)
        )

    def manual_cancel(self):
        self.get_logger().info("Canceling goal")
        # Cancel the goal
        self._cancel_future = self._goal_handle.cancel_goal_async()
        self._cancel_future.add_done_callback(self.cancel_done)

    def send_goal(self):
        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()

        goal_msg = self.action_type.Goal()

        self.get_logger().info("Sending goal request...")

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)


def generate_test_description(ready_fn):
    test_nodes = launch_ros.actions.Node(
        package="recordreplay_planner_node",
        node_executable="recordreplay_planner_node_exe",
        node_name="recordreplay_planner",
        parameters=[
            "{}/defaults.param.yaml".format(
                ament_index_python.get_package_share_directory(
                    "recordreplay_planner_node"
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


def helper_start_action_goal(
    name: str, goal: str, parameters: str, redirect_stdout: bool = False
):
    """Start an action call and return the started process object, non-blocking."""
    if redirect_stdout:
        mystdout = subprocess.PIPE
    else:
        mystdout = None
    return subprocess.Popen(
        ["ros2", "action", "send_goal", name, goal, parameters], stdout=mystdout
    )


def helper_echo_topic(name: str, msgtype: str):
    return subprocess.Popen(
        ["ros2", "topic", "echo", name, msgtype],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


# TODO(s.me): Test: Check if "happy case" of recording, then replaying works
class TestBasicUsage(unittest.TestCase):
    def test_happy_case_works(self):
        # ---- Recording
        # - Start recordreplay_planner_node exe (done in test description)

        # - Start a recording action by sending a goal
        mock_record_action_caller = MockActionCaller(
            "MockRecordCaller", RecordTrajectory, "recordtrajectory"
        )
        mock_record_action_caller.send_goal()

        # - Wait for the goal sending to complete - this feels like an antipattern
        #   though because I'm accessing the internal field of the node.
        #   TODO(s.me) get a better suggestion.
        rclpy.spin_until_future_complete(
            mock_record_action_caller, mock_record_action_caller._send_goal_future
        )

        # - Publish a few VehicleKinematicState messages
        mock_publisher = MockStatePublisher()
        for k in range(3):
            time.sleep(0.1)
            mock_publisher.publish_a_state()
            rclpy.spin_once(mock_publisher, timeout_sec=0.2)  # Is this necessary?

        # - Cancel recording action
        mock_record_action_caller.manual_cancel()

        # Wait for the cancel action to complete
        rclpy.spin_until_future_complete(
            mock_record_action_caller, mock_record_action_caller._cancel_future
        )

        # ---- Replaying
        # - Listen on the specified trajectory topic, storing to memory
        mock_listener = MockTrajectoryMonitor()

        # - Send it a "start replaying" action request
        mock_replay_action_caller = MockActionCaller(
            "MockReplayCaller", ReplayTrajectory, "replaytrajectory"
        )
        mock_replay_action_caller.send_goal()
        rclpy.spin_until_future_complete(
            mock_replay_action_caller, mock_replay_action_caller._send_goal_future
        )

        # - Publish a few VehicleKinematicState messages
        for k in range(3):
            time.sleep(0.3)
            mock_publisher.publish_a_state()
            rclpy.spin_once(mock_publisher, timeout_sec=0.2)  # Is this necessary?
            time.sleep(0.1)
            rclpy.spin_once(mock_listener, timeout_sec=0.2)  # Is this necessary?

        # - Cancel replaying action
        mock_replay_action_caller.manual_cancel()

        # - Wait for the cancellation to complete
        rclpy.spin_until_future_complete(
            mock_replay_action_caller, mock_replay_action_caller._cancel_future
        )

        # - Verify that the replayed trajectories behaved as expected
        # TODO(s.me): Make the mock_listener record what it sees and verify it matches
        # expectations.


# TODO(s.me): Test: Check if an additional record action is rejected if one is already running
# - Start recordreplay_planner_node exe
# - Use ros2 commandline to send it a "start recording" action request
# - Attempt to start a second start, verify this is rejected

# TODO(s.me): Test: Check if an additional replay action is rejected if one is already running
# - Start recordreplay_planner_node exe
# - Record a bit of trajectory like in happy case test
# - Use ros2 commandline to send it a "start replaying" action request
# - Attempt to start a second start, verify this is rejected

# TODO(s.me): Test: Check if replay stops when the trajectory being put out becomes empty.
# This is not implemented in the actual code yet - maybe not stopping but just giving out
# the last one-state trajectory is a better idea.
