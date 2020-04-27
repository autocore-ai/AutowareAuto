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

import rclpy
from rclpy.node import Node

from autoware_auto_msgs.msg import Trajectory
from autoware_auto_msgs.msg import TrajectoryPoint
from autoware_auto_msgs.msg import VehicleKinematicState


from rclpy.duration import Duration
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile


# Class to publish test triggers and subscribe the results
class SimpleTrajectory(Node):

    def __init__(self):
        super().__init__('simple_trajectory')

        # Parameters
        self.length_m = self.declare_parameter("length_m", 20.0).value
        self.min_speed_mps = self.declare_parameter("min_speed_mps", 1.0).value
        self.max_speed_mps = self.declare_parameter("max_speed_mps", 8.0).value
        self.discretization_m = self.declare_parameter("discretization_m", 0.5).value
        self.speed_increments = self.declare_parameter("speed_increments", 0.2).value
        self.offset_distnace_m = self.declare_parameter("offset_distnace_m", 0.0).value
        self.one_time = self.declare_parameter("one_time", False).value
        self.timer_period_sec = self.declare_parameter("timer_period_sec", 0.0).value
        if self.max_speed_mps < self.min_speed_mps:
            self.max_speed_mps = self.min_speed_mps

        # Publisher / Subscriber
        qos = QoSProfile(depth=10)
        if (self.one_time):
            qos.durability = QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
            qos.depth = 1

        self.publisher_trajectory_ = self.create_publisher(
            Trajectory, "trajectory", qos
        )
        self.subscriber_vehicle_kinematic_state_ = self.create_subscription(
            VehicleKinematicState, "vehicle_kinematic_state",
            self.vehicle_kinematic_state_cb, 0
        )

        # Local variables
        self.trajectory_msg_ = None
        self.vehicle_kinematic_state_msg_ = None

        if (not self.one_time):
            self.timer = self.create_timer(self.timer_period_sec, self.publish_trajectory)

    def publish_trajectory(self):
        if(self.vehicle_kinematic_state_msg_ is None):
            return
        if(not self.one_time or self.trajectory_msg_ is None):
            self.trajectory_msg_ = self.createStraightLineTrajectory(
                self.length_m,
                self.min_speed_mps,
                self.max_speed_mps,
                self.discretization_m,
                self.speed_increments,
                self.offset_distnace_m)
            self.publisher_trajectory_.publish(self.trajectory_msg_)

    def vehicle_kinematic_state_cb(self, msg):
        # self.get_logger().info('vehicle_kinematic_state_cb: "%s"' % msg.header.frame_id)
        self.vehicle_kinematic_state_msg_ = msg
        if self.timer_period_sec <= 0.0 or self.one_time:
            self.publish_trajectory()

    def createStraightLineTrajectory(self, length, min_speed, max_speed,
                                     discretization_m, speed_increments, offset_distnace_m):
        if max_speed < min_speed:
            max_speed = min_speed

        num_points = int(length / discretization_m)
        num_points_max = 100    # max. length in Trajectory.msg
        if (num_points > num_points_max):
            num_points = num_points_max
            print('Only 100 points available - discretization set to %s'
                  % float(length / num_points_max))
        discretization_distance_m = float(length / num_points)

        trajectory_msg = Trajectory()

        # start at base_link
        init_point = self.vehicle_kinematic_state_msg_.state

        # initial speed
        if init_point.longitudinal_velocity_mps < min_speed:
            init_point.longitudinal_velocity_mps = min_speed
        current_speed = init_point.longitudinal_velocity_mps

        # initial time
        current_time = 0.0
        if current_speed > 0.0:
            current_time = float(offset_distnace_m / current_speed)
        init_point.time_from_start = Duration(seconds=current_time).to_msg()
        cos_pose = (init_point.heading.real + init_point.heading.imag) \
            * (init_point.heading.real - init_point.heading.imag)
        sin_pose = 2.0 * init_point.heading.real * init_point.heading.imag

        for i in range(0, num_points - 1):
            trajectory_point = TrajectoryPoint()
            trajectory_point.time_from_start = Duration(seconds=current_time).to_msg()
            current_length = discretization_distance_m * i + offset_distnace_m
            trajectory_point.x = init_point.x + (cos_pose * current_length)
            trajectory_point.y = init_point.y + (sin_pose * current_length)
            trajectory_point.heading = init_point.heading
            trajectory_point.longitudinal_velocity_mps = float(current_speed)
            trajectory_msg.points.append(trajectory_point)

            if current_speed < max_speed:
                current_speed = current_speed + speed_increments
            current_time = current_time + float(discretization_distance_m / current_speed)

        # Fill header
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        trajectory_msg.header.frame_id = self.vehicle_kinematic_state_msg_.header.frame_id
        return trajectory_msg


def main(args=None):
    rclpy.init(args=args)

    simple_trajectory = SimpleTrajectory()

    rclpy.spin(simple_trajectory)


if __name__ == '__main__':
    main()
