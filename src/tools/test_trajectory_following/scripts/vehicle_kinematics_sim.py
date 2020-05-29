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

import math

import rclpy
from rclpy.node import Node

from autoware_auto_msgs.msg import VehicleKinematicState
from autoware_auto_msgs.msg import VehicleControlCommand

from rclpy.qos import QoSProfile
from rclpy.duration import Duration

# from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

from nav_msgs.msg import Odometry


# Class to publish test triggers and subscribe the results
class VehicleMotionSim(Node):

    def __init__(self):
        super().__init__('vehicle_motion_sim')

        # Parameters
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_link_frame = self.declare_parameter(
            "base_link_frame", "base_link").value
        self.wheel_base = self.declare_parameter("wheel_base", 2.7).value
        self.max_speed_mps = self.declare_parameter(
            "max_speed_mps", 30.0).value
        self.min_speed_mps = self.declare_parameter(
            "min_speed_mps", -10.0).value
        self.max_acceleration_mps2 = self.declare_parameter(
            "max_acceleration_mps2", 3.0).value
        self.min_acceleration_mps2 = self.declare_parameter(
            "min_acceleration_mps2", -3.0).value
        self.max_steer_angle_rad = self.declare_parameter(
            "max_steer_angle_rad", 0.331).value
        self.min_steer_angle_rad = self.declare_parameter(
            "min_steer_angle_rad", -0.331).value
        self.timer_period_sec = self.declare_parameter(
            "timer_period_sec", 0.01).value
        self.command_timeout_sec = self.declare_parameter(
            "command_timeout_sec", 0.5).value
        self.idle_deccel = self.declare_parameter("idle_deccel", 0.9).value

        if self.timer_period_sec < 0.001:
            self.timer_period_sec = 0.001

        # Local variables
        self.command_timeout_dur_ = Duration(seconds=self.command_timeout_sec)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0
        self.accel = 0.0
        self.velocity = 0.0
        self.steer = 0.0

        self.last_cmd_time_ = None
        self.vehicle_command_msg_ = None

        # Publisher / Subscriber
        qos = QoSProfile(depth=10)
        self.pub_vehicle_kinematic_state_ = self.create_publisher(
            VehicleKinematicState, "vehicle_kinematic_state", qos
        )
        self.pub_odometry_ = self.create_publisher(Odometry, 'odom', qos)

        self.pub_tf_ = self.create_publisher(TFMessage, '/tf', qos)
        # self.broadcaster_ = TransformBroadcaster(self)

        self.sub_vehicle_command_ = self.create_subscription(
            VehicleControlCommand, "vehicle_command",
            self.vehicle_command_cb, 0
        )
        # timer
        self.timer = self.create_timer(
            self.timer_period_sec, self.update_state)

    def vehicle_command_cb(self, msg):
        self.vehicle_command_msg_ = msg
        self.last_cmd_time_ = self.get_clock().now()

    def valid_command(self):
        if(self.vehicle_command_msg_ is None):
            return False
        if(self.last_cmd_time_ is None):
            return False

        # May also use self.vehicle_command_msg_.stamp
        time_diff = self.get_clock().now() - self.last_cmd_time_

        if time_diff < self.command_timeout_dur_:
            return True
        else:
            return False

    def update_state(self):
        if self.valid_command():
            self.accel = min(
                self.vehicle_command_msg_.long_accel_mps2, self.max_acceleration_mps2)
            self.accel = max(self.accel, self.min_acceleration_mps2)
            self.steer = min(
                self.vehicle_command_msg_.front_wheel_angle_rad, self.max_steer_angle_rad)
            self.steer = max(self.steer, self.min_steer_angle_rad)
        else:
            # no valid command, slowing down, rolling resistance
            if math.fabs(self.velocity) < (self.idle_deccel * self.timer_period_sec):
                self.accel = 0.0
                self.velocity = 0.0
            elif self.velocity > 0.0:
                self.accel = -self.idle_deccel
            else:
                self.accel = self.idle_deccel

        # updating velocities
        self.velocity += self.accel * self.timer_period_sec
        self.vtheta = self.velocity * math.tan(self.steer) / self.wheel_base

        # updating delta position
        self.vx = self.velocity * math.cos(self.theta)
        self.vy = self.velocity * math.sin(self.theta)

        # updating postion
        self.x += self.vx * self.timer_period_sec
        self.y += self.vy * self.timer_period_sec
        self.theta += self.vtheta * self.timer_period_sec

        # Publishing msgs
        stamp = self.get_clock().now().to_msg()
        self.pub_odometry_ .publish(self.build_odometry(stamp))
        self.pub_vehicle_kinematic_state_.publish(
            self.build_vehicle_kienmatic_state(stamp))
        self.pub_tf_.publish(
            TFMessage(transforms=[self.build_transform(stamp)]))
        # self.broadcaster_.sendTransform(self.build_transform(stamp))

    def build_vehicle_kienmatic_state(self, stamp):
        vk_state_msg = VehicleKinematicState()

        vk_state_msg.header.stamp = stamp
        vk_state_msg.header.frame_id = self.base_link_frame

        vk_state_msg.state.x = self.x
        vk_state_msg.state.y = self.y
        vk_state_msg.state.heading.imag = math.sin(self.theta / 2.0)
        vk_state_msg.state.heading.real = math.cos(self.theta / 2.0)
        vk_state_msg.state.longitudinal_velocity_mps = self.velocity
        vk_state_msg.state.lateral_velocity_mps = 0.0
        vk_state_msg.state.acceleration_mps2 = self.accel
        vk_state_msg.state.heading_rate_rps = self.vtheta
        vk_state_msg.state.front_wheel_angle_rad
        vk_state_msg.state.rear_wheel_angle_rad = 0.0

        return vk_state_msg

    def build_odometry(self, stamp):
        odom_msg = Odometry()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_link_frame
        odom_msg.header.stamp = stamp

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom_msg.twist.twist.linear .x = self.vx
        odom_msg.twist.twist.linear .y = self.vy
        odom_msg.twist.twist.linear .z = 0.0

        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.vtheta

        return odom_msg

    def build_transform(self, stamp):
        transform = TransformStamped()
        transform.header.frame_id = self.odom_frame
        transform.header.stamp = stamp
        transform.child_frame_id = self.base_link_frame

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(self.theta / 2.0)
        transform.transform.rotation.w = math.cos(self.theta / 2.0)

        return transform


def main(args=None):
    rclpy.init(args=args)

    node = VehicleMotionSim()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
