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

import launch_testing.event_handlers
import subprocess


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


def helper_start_action_goal(name: str, goal: str, parameters: str):
    """Start an action call and return the started process object, non-blocking"""
    return subprocess.Popen(["ros2", "action", "send_goal", name, goal, parameters,])


def helper_publish_one_default_msg(topic: str, msgtype: str):
    """Publish a single default message on a given topic, blocking"""
    subprocess.run(["ros2", "topic", "pub", "-1", topic, msgtype])


def helper_echo_topic(name: str, msgtype: str):
    return subprocess.Popen(
        ["ros2", "topic", "echo", name, msgtype], stdout=subprocess.PIPE
    )


# TODO(s.me): Test: Check if "happy case" of recording, then replaying works
class TestHappyCase(unittest.TestCase):
    def test_happy_case(self):
        # - Start recordreplay_planner_node exe (done in test description)
        # - Send it a "start recording" action request
        recording_action = helper_start_action_goal(
            "/recordtrajectory",
            "recordreplay_planner_actions/action/RecordTrajectory",
            "{}",
        )

        # - Publish a few VehicleKinematicState messages
        for k in range(2):
            helper_publish_one_default_msg(
                "/vehicle_kinematic_state",
                "autoware_auto_msgs/msg/VehicleKinematicState",
            )

        # - Cancel action by sending a signal SIGTERM to the ros2 commandline action process
        recording_action.terminate()
        recording_action.wait()

        # - Listen on the specified trajectory topic, storing to memory
        listener_process = helper_echo_topic(
            "/trajectory", "autoware_auto_msgs/msg/Trajectory",
        )

        # - Send it a "start replaying" action request
        replaying_action = helper_start_action_goal(
            "/replaytrajectory",
            "recordreplay_planner_actions/action/ReplayTrajectory",
            "{}",
        )

        # - Publish a few VehicleKinematicState messages
        for k in range(2):
            helper_publish_one_default_msg(
                "/vehicle_kinematic_state",
                "autoware_auto_msgs/msg/VehicleKinematicState",
            )

        # - Cancel action by sending a signal to the ros2 commandline process
        replaying_action.terminate()
        replaying_action.wait()

        # - Verify that the replayed trajectories behaved as expected. FIXME this does not
        #   properly capture the output, so I can't quite verify yet.
        listener_process.terminate()
        listener_process.wait()
        stdout = listener_process.communicate()[0]
        print("stdout is: {}.".format(stdout))


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
