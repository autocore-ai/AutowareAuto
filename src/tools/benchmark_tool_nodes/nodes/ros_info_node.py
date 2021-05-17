#! /usr/bin/env python3
#
# Copyright (c) 2021, Arm Limited
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

import subprocess
import threading
import traceback

import rclpy
import rclpy.action
from rclpy.qos import qos_profile_sensor_data
from ros2topic.api import get_msg_class
from ros2topic.api import get_topic_names_and_types

from benchmark_tool.utility import getParameter
from benchmark_tool.rqt_graph.dotcode import RosGraphDotcodeGenerator
from benchmark_tool.rqt_graph.rosgraph2_impl import Graph
from qt_dotgraph.pydotfactory import PydotFactory

# Global data
topics_data = []
ros_topics = []
delimiter = ' '
result_path = ''
sampling_rate = 1


class ROSTopicBandwidth(object):

    def __init__(self):
        self.lock = threading.Lock()
        self.sizes = []

    def callback(self, data):
        """Execute ros sub callback."""
        with self.lock:
            try:
                self.sizes.append(len(data))  # AnyMsg instance
            except Exception:
                traceback.print_exc()


def write_stats():
    topics_data.clear()
    for rt in ros_topics:
        with rt.lock:
            n = len(rt.sizes)
            if n > 0:
                total = sum(rt.sizes)
                mean = total / n
            else:
                total = 0
                mean = 0
            bytes_per_s = total * sampling_rate
            rt.sizes.clear()
        topics_data.append([bytes_per_s, mean])

    with open(result_path + '/ros_info.topic_bw.txt', "a") as f:
        f.write(delimiter.join([str(i[0]) for i in topics_data]) + '\n')
    with open(result_path + '/ros_info.msg_sz.txt', "a") as f:
        f.write(delimiter.join([str(i[1]) for i in topics_data]) + '\n')


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("ros_info")
    node.declare_parameters(
        namespace='',
        parameters=[
            ('result_path', None),
            ('sampling_rate', None)
        ]
    )
    global result_path, sampling_rate
    result_path = getParameter(node, "result_path")
    sampling_rate = getParameter(node, "sampling_rate")
    assert(sampling_rate > 0)

    # Get topology
    graph = Graph(node)
    graph.set_node_stale(5.0)
    graph.update()
    dotcode_factory = PydotFactory()
    dotcode_generator = RosGraphDotcodeGenerator(node)
    dotcode = dotcode_generator.generate_dotcode(
        rosgraphinst=graph,
        ns_filter="",
        topic_filter="",
        graph_mode="NODE_TOPIC_ALL_GRAPH",
        cluster_namespaces_level=5,
        dotcode_factory=dotcode_factory,
        orientation="LR",
        quiet=True,
    )

    # Write topology
    with open(result_path + '/ros_info.topology.dot', "w") as f:
        f.write(dotcode)
    subprocess.run(["dot", "-Tpng", "-O", result_path + "/ros_info.topology.dot"])

    # Get topics
    topic_names_and_types = get_topic_names_and_types(node=node)
    topic_names = [x[0] for x in topic_names_and_types]

    # Write headers
    with open(result_path + '/ros_info.topic_bw.txt', "w") as f:
        f.write(delimiter.join(topic_names) + '\n')
    with open(result_path + '/ros_info.msg_sz.txt', "w") as f:
        f.write(delimiter.join(topic_names) + '\n')

    # Subsribe to active topics
    for topic in topic_names:
        msg_class = get_msg_class(node, topic, blocking=False, include_hidden_topics=True)
        if msg_class is None:
            node.destroy_node()
            return

        rt = ROSTopicBandwidth()
        ros_topics.append(rt)
        node.create_subscription(
            msg_class,
            topic,
            rt.callback,
            qos_profile_sensor_data,
            raw=True
        )

    node.create_timer(1 / sampling_rate, write_stats)
    while rclpy.ok():
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
