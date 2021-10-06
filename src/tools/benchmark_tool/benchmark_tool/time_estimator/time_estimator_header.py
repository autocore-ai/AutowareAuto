#! /usr/bin/env python3

# Copyright (c) 2020-2021, Arm Limited
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
from rclpy.qos import QoSProfile
from ros2topic.api import get_msg_class
from std_msgs.msg import Int64
from benchmark_tool.utility import error


class TimeEstimatorHeader(object):
    """
    The TimeEstimatorHeader class.

    It measures the time elapsed from the reception of a message using the header stamp
    information. It measures the difference between the received time and the time in the message
    header, it uses microsecond resolution.

    This class should be used when the node that produces the message, is copying the header from
    its input topic message to the output topic message.

    This is the flow:
    1) The benchmark_tool node publish a message with a timestamp
    2) The blackbox system receives the message and compute the output message
    3) The blackbox system copies the header from the input message to the output message and send
        it
    4) The benchmark_tool is subscribed to the blackbox system, so upon receiving the output
        message, we know that the time elapsed between the reception and the timestamp on the
        message is an upper bound for how long the blackbox system has taken to perform its
        operations.
    """

    def __init__(self, node):
        """
        Create a TimeEstimator object.

        @param node: ROS2 node
        @type  node: rclpy.node.Node
        """
        self._sub_output_topic = None
        self._publisher = None
        self.node = node

    def start_listener(self, output_topic, publish_topic):
        """
        Start the time measurement.

        @param output_topic: Topic to be listened to measure the time
        @type  output_topic: str
        @param publish_topic: Topic where the measurement is published
        @type  publish_topic: str
        @return: True on success, False on failure
        """
        try:
            # Create publisher to publish the speed measure
            self._publisher = self.node.create_publisher(
                Int64,
                publish_topic,
                qos_profile=QoSProfile(depth=1)
            )

            # Get type of the output topic and subscribe to it
            output_topic_type = get_msg_class(self.node, output_topic,
                                              blocking=True)
            self._sub_output_topic = self.node.create_subscription(
                output_topic_type,
                output_topic,
                self.output_topic_callback,
                qos_profile=QoSProfile(depth=1)
            )
        except Exception as e:
            error(self.node, "%s" % str(e))
            return False

        return True

    def output_topic_callback(self, msg):
        """
        Compute a time difference on message reception.

        Callback function triggered by the reception of a message from the output topic.

        When a message is received, the time is computed using the difference between the actual
        receiving time and the time of the message header.
        After the computation, the speed is published on the publish topic.

        @param msg: The topic message
        @type  msg: The type can vary depending on the listened topic
        @return: None
        """
        # Get actual time from ROS
        time_now = self.node.get_clock().now().nanoseconds

        # Compute the amount of time elapsed from receiving this message and
        # the time written in the message header
        time_message = rclpy.time.Time()
        time_message = time_message.from_msg(msg.header.stamp)
        elapsed_ns = time_now - time_message.nanoseconds
        elapsed_us = elapsed_ns / 1e3

        publish_msg = Int64()
        publish_msg.data = int(elapsed_us)

        # Publish the measurement
        self._publisher.publish(publish_msg)
