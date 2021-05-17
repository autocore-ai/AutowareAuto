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

import rclpy
from benchmark_tool.utility import getParameter, error
try:
    import psutil
except ImportError:
    rclpy.init()
    node = rclpy.create_node("sys_info")
    error(node, "psutil not availaible, please install")
    exit()

# Global data
known_processes = set([psutil.Process(), psutil.Process().parent()])
node_processes = set()
delimiter = ' '
data = {
    'cpu': [],
    'io_r': [],
    'io_w': [],
    'mem': [],
}
result_path = ''
sampling_rate = 1


def get_processes():
    for p in psutil.process_iter():
        if p not in known_processes:
            known_processes.add(p)
            try:
                cmdline = p.cmdline()
                for s in cmdline:
                    # Identify benchmark-related processes
                    if 'benchmark_tool_nodes' in s:
                        node_processes.add(p)
                        break
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass


def write_stats():
    for key in list(data):
        data[key].clear()
    for p in node_processes:
        try:
            with p.oneshot():
                data['cpu'].append(sum(p.cpu_times()[:2]))
                data['io_r'].append(p.io_counters().read_count)
                data['io_w'].append(p.io_counters().write_count)
                data['mem'].append(p.memory_info().rss)
        except psutil.NoSuchProcess:
            pass
    sections = []
    for key in list(data):
        sections.append(delimiter.join([str(x) for x in data[key]]))
    with open(result_path + '/sys_info.txt', 'a') as f:
        f.write(delimiter.join(sections) + '\n')


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("sys_info")
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

    get_processes()
    names = []
    for p in node_processes:
        try:
            with p.oneshot():
                name = p.name()
                # Build a more explicit name in case of python
                if 'python' in name and len(p.cmdline()) > 1:
                    name += '(' + p.cmdline()[1].split('/')[-1] + ')'
                names.append(name)
        except psutil.NoSuchProcess:
            pass
    sections = []
    for key in list(data):
        sections.append(delimiter.join([key + ':' + x for x in names]))

    # Write headers
    with open(result_path + '/sys_info.txt', 'w') as f:
        f.write(delimiter.join(sections) + '\n')

    node.create_timer(1 / sampling_rate, write_stats)
    while rclpy.ok():
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
