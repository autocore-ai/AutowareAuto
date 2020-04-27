#!/usr/bin/env python3

# Copyright 2020 The Autoware Foundation.
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

from enum import IntEnum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from autoware_auto_msgs.msg import VehicleKinematicState
from std_msgs.msg import UInt8

from recordreplay_planner_actions.action import RecordTrajectory
from recordreplay_planner_actions.action import ReplayTrajectory


class InternalState(IntEnum):
    Idle = 0
    Start_record = 1
    Recording = 2
    Stopped_record = 3
    Start_replay = 4
    Replaying = 5
    Stopped_replay = 6


class RecordReplayCmd(IntEnum):
    Noop = 0
    Start_record = 1
    Start_replay = 2
    Stop = 3


# Class to publish test triggers and subscribe the results
class JoyCtrlRecordReplayTraj(Node):
    def __init__(self):
        super().__init__('joy_ctrl_record_replay_traj')

        # Local variables
        self.record_action_goal_handle_ = None
        self.replay_action_goal_handle_ = None
        self.vehicle_kinematic_state_msg_ = None
        self.recorded_states_ = 0
        self.replay_remaining_length_ = 0
        self.vehicle_kinematic_state_msg_ = None
        self.timer_period = 0.5  # seconds
        self.internal_state_ = InternalState.Idle

        # Actionclient
        self.replay_action_client_ = ActionClient(
            self, ReplayTrajectory, "/planning/replaytrajectory")
        self.record_action_client_ = ActionClient(
            self, RecordTrajectory, "/planning/recordtrajectory")

        # Publisher / Subscriber
        self.subscriber_vehicle_kinematic_state_ = self.create_subscription(
            VehicleKinematicState, "vehicle_kinematic_state",
            self.vehicle_kinematic_state_cb, 0
        )
        self.subscriber_joystick_recordreplay_cmd_ = self.create_subscription(
            UInt8, "recordreplay_cmd",
            self.joystickStateCb, 0
        )

        self.timer = self.create_timer(self.timer_period, self.set_action)

    def set_action(self):
        # no vehicle state
        if(self.vehicle_kinematic_state_msg_ is None):
            return

        # Start recording
        if (self.internal_state_ == InternalState.Start_record):
            self.get_logger().info('Start recording')
            self.recordTrajectoryAction("recorded_trajectory.data")
            self.internal_state_ == InternalState.Recording
        # Stop recording
        elif (self.internal_state_ == InternalState.Stopped_record):
            self.get_logger().info('Stop recording')
            self.record_action_goal_handle_.cancel_goal_async()
            self.internal_state_ = InternalState.Idle

        # Start replaying
        elif (self.internal_state_ == InternalState.Start_replay):
            self.get_logger().info('Start replaying')
            self.replayTrajectoryAction("recorded_trajectory.data")

        # Stop replaying
        elif (self.internal_state_ == InternalState.Stopped_replay):
            self.get_logger().info('Cancel replaying')
            self.replay_action_goal_handle_.cancel_goal_async()
            self.internal_state_ = InternalState.Idle
        return

    def vehicle_kinematic_state_cb(self, msg):
        self.get_logger().debug('vehicle_kinematic_state_cb: with frame_id "%s"' %
                                msg.header.frame_id)
        self.vehicle_kinematic_state_msg_ = msg

    def joystickStateCb(self, msg):
        self.get_logger().debug('State {}'.format(self.internal_state_))

        # Stop
        if (msg.data == RecordReplayCmd.Stop):
            self.get_logger().debug('Joystick command Stop')
            if (self.internal_state_ == InternalState.Recording):
                self.internal_state_ = InternalState.Stopped_record
            elif (self.internal_state_ == InternalState.Replaying):
                self.internal_state_ = InternalState.Stopped_replay
        # Start Record
        elif(msg.data == RecordReplayCmd.Start_record):
            self.get_logger().debug('Joystick command Start Record')
            if (self.internal_state_ == InternalState.Idle):
                self.internal_state_ = InternalState.Start_record

        # Start Replay
        elif(msg.data == RecordReplayCmd.Start_replay):
            self.get_logger().debug('Joystick command Start Replay')
            if (self.internal_state_ == InternalState.Idle):
                self.internal_state_ = InternalState.Start_replay
        return

    # ----------- Record action and callbacks --------------
    def recordTrajectoryAction(self, path):
        self.get_logger().info('recordTrajectoryAction')

        goal_msg = RecordTrajectory.Goal()
        goal_msg.record_path = path

        if self.record_action_client_.wait_for_server(timeout_sec=self.timer_period / 2.0):
            send_goal_future = self.record_action_client_.send_goal_async(
                goal_msg, feedback_callback=self.recordTrajectoryFeedback)
            send_goal_future.add_done_callback(self.recordTrajectoryResponse)
            self.internal_state_ = InternalState.Recording
        else:
            self.internal_state_ = InternalState.Idle
            self.get_logger().info('recordTrajectoryAction timed out')

        return

    def recordTrajectoryFeedback(self, msg):
        self.get_logger().debug(
            'recordTrajectoryFeedback lenght {}'.format(msg.feedback.current_length))
        current_length = msg.feedback.current_length
        self.recorded_states_ = current_length
        return

    def recordTrajectoryResponse(self, future):
        self.get_logger().debug('recordTrajectoryResponse')
        self.record_action_goal_handle_ = future.result()

        if not self.record_action_goal_handle_.accepted:
            self.get_logger().debug('recordTrajectory Goal rejected :(')
            return

        self.get_logger().debug('recordTrajectory Goal accepted :)')
        get_result_future = self.record_action_goal_handle_.get_result_async()
        get_result_future.add_done_callback(self.recordTrajectoryResult)

        return

    def recordTrajectoryResult(self, future):
        self.get_logger().info('recordTrajectoryResult')
        self.internal_state_ = InternalState.Idle

    # ----------- Replay action and callbacks --------------

    def replayTrajectoryAction(self, path):
        self.get_logger().info('replayTrajectoryAction')
        goal_msg = ReplayTrajectory.Goal()
        goal_msg.replay_path = path

        if self.replay_action_client_.wait_for_server(timeout_sec=self.timer_period / 2.0):
            send_goal_future = self.replay_action_client_.send_goal_async(
                goal_msg, feedback_callback=self.replayTrajectoryFeedback)
            send_goal_future.add_done_callback(self.replayTrajectoryResponse)
            self.internal_state_ = InternalState.Replaying
        else:
            self.internal_state_ = InternalState.Idle
            self.get_logger().info('replayTrajectoryAction timed out')

        return

    def replayTrajectoryFeedback(self, msg):
        self.get_logger().info(
            'replayTrajectoryFeedback lenght {}'.format(msg.feedback.remaining_length))
        self.replay_remaining_length_ = msg.feedback.remaining_length
        return

    def replayTrajectoryResponse(self, future):
        self.get_logger().debug('replayTrajectoryResponse')
        self.replay_action_goal_handle_ = future.result()

        if not self.replay_action_goal_handle_.accepted:
            self.get_logger().debug('replayTrajectory Goal rejected :(')
            return

        self.get_logger().debug('replayTrajectory Goal accepted :)')
        get_result_future = self.replay_action_goal_handle_.get_result_async()
        get_result_future.add_done_callback(self.replayTrajectoryResult)

    def replayTrajectoryResult(self, future):
        self.get_logger().debug('replayTrajectoryResult')
        self.internal_state_ = InternalState.Idle
        return


def main(args=None):
    rclpy.init(args=args)

    joy_ctrl_record_replay_traj = JoyCtrlRecordReplayTraj()

    rclpy.spin(joy_ctrl_record_replay_traj)
    joy_ctrl_record_replay_traj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
