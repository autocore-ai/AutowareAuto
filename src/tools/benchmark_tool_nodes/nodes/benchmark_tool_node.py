#! /usr/bin/env python3
#
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

import sys
import rclpy
import inspect
from importlib import import_module
from benchmark_tool.utility import getParameter, error, info

loop_exit = False


def main_loop(node, task):
    global loop_exit
    if not task.run():
        if rclpy.ok():
            # Second stage of the pipeline, compute metrics
            task.compute_results()
        else:
            error(node, "Task interrupted!")
        loop_exit = True


def main(args=None):
    global loop_exit
    # Create a ROS2 node
    rclpy.init(args=args)
    node = rclpy.create_node("benchmark_tool")
    node.declare_parameters(
        namespace='',
        parameters=[
            ('benchmark_task', None),
            ('dataset_path', None),
            ('input_topic', None),
            ('output_topic', None),
            ('benchmarked_input_topic', None),
            ('benchmarked_output_topic', None),
            ('result_path', None),
            ('force_end_at_frame_n', None)
        ]
    )

    # Check parameter existance
    benchmark_task = getParameter(node, "benchmark_task")

    task = None

    try:
        # Dynamically load the task
        task_module_name = 'benchmark_tool.benchmark_task.' + \
            benchmark_task
        task_module = import_module(task_module_name)

        # Inspect the module to find the task class
        task_classes = []
        for found_class in inspect.getmembers(task_module, inspect.isclass):
            if found_class[1].__module__ == task_module_name:
                task_classes.append(found_class[1])

        # Check that the module has only one class
        if len(task_classes) != 1:
            error(node, "The specified task module should have only one class.")
            sys.exit(-1)

        # Instantiate the task object
        task = task_classes[0](node)

    except Exception as e:
        error(node, "Error loading the module %s:\n%s" % (benchmark_task, str(e)))
        sys.exit(-1)

    if not task.init():
        error(node, "Cannot init the benchmark task.")
        sys.exit(-1)

    info(node, "Started task: %s" % benchmark_task)

    # Loop rate at 1000 Hz
    loop_rate_s = float(1.0/1000)

    timer = node.create_timer(
        loop_rate_s,
        lambda: main_loop(node, task)
    )

    while rclpy.ok() and not loop_exit:
        rclpy.spin_once(node)

    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
