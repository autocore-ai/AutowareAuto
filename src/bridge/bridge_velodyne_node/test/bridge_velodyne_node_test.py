# Copyright 2019 Tier IV, Inc.
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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

import copy
import os
import re
import subprocess
import unittest

from ament_index_python import get_package_share_directory

import launch
import launch.action
import launch.events
import launch.substitutions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import launch_testing
import launch_testing.util

import pytest

# Location of velodyne packet pcap file.
this_package = get_package_share_directory('bridge_velodyne_node')
velodyne_packet_file_path = this_package + "/test/velodyne_pointcloud_test.pcap"

# Location of ros1 setup script.
ros1_setup_file_path = "/opt/ros/melodic/setup.sh"

# Set execution environment.
proc_env = os.environ.copy()
proc_env['PYTHONUNBUFFERED'] = '1'
# Set execution environment for ROS1.
ros1_env = copy.deepcopy(os.environ)


def source_env(__script, __target_env):
    """
    Summary: source environment and make new environment.

    Description:
    Args:
    * __script: source script file path(.sh or .bash file).
    * __target_env: target environment to be copied.
    Return:
    * None
    Processing:
    * Executes ". <script>.sh file" and "env"
    * copy env result to target_env
    """
    _pipe = subprocess.Popen(". %s; env" % __script,
                             stdout=subprocess.PIPE, shell=True)
    _output = _pipe.communicate()[0]
    for _line in _output.decode("utf-8").splitlines():
        if re.match(".*=.*", _line):
            _splitted_line = _line.split("=", 1)
            __target_env[_splitted_line[0]] = _splitted_line[1]


source_env(ros1_setup_file_path, ros1_env)


def get_process_running(__proc_name):
    """Check whether '__proc_name' process runs or not."""
    try:
        _ps_result = subprocess.run(['pgrep', __proc_name],
                                    stdout=subprocess.PIPE)
    except subprocess.SubprocessError:
        raise
    if len(_ps_result.stdout.decode("utf8")) != 0:
        return True
    return False


def kill_process(__proc_name):
    """Kill process by process name('__proc_name')."""
    try:
        subprocess.run(['killall', '-KILL', __proc_name])
    except subprocess.SubprocessError:
        raise
    while True:
        if not get_process_running(__proc_name):
            break
    print("kill {}".format(__proc_name))
    return


def launch_roscore():
    """Launch roscore function,not using launch_testing framework."""
    # Kill running roscore and rosmaster if needed.
    def _judge_executing_roscore():
        _roscore_running = get_process_running('roscore')
        _rosmaster_running = get_process_running('rosmaster')
        _launch_roscore_ready = True
        # Kill remaining roscore and rosmaster.
        if _roscore_running and _rosmaster_running:
            _launch_roscore_ready = False
            print("roscore has been already running.")
        elif not _roscore_running and _rosmaster_running:
            kill_process('rosmaster')
        elif _roscore_running and not _rosmaster_running:
            kill_process('roscore')
        else:
            pass
        return _launch_roscore_ready
    _launch_roscore_ready = _judge_executing_roscore()

    # Return if launching roscore is not needed.
    if not _launch_roscore_ready:
        return

    # Execute roscore and rosmaster with confirming process status.
    try:
        subprocess.Popen(['roscore'], env=ros1_env)
        while True:
            _rostopic_proc = subprocess.run(['rostopic', 'list'], env=ros1_env,
                                            stdout=subprocess.PIPE,
                                            stderr=subprocess.PIPE)

            _rostopic_proc_msg = _rostopic_proc.stdout.decode("utf-8")
            if re.search('rosout', _rostopic_proc_msg):
                break
    except subprocess.SubprocessError:
        raise

    print("roscore is launched")
    return


@pytest.mark.launch_test
def generate_test_description(ready_fn):
    """
    Summary: this is main test routine for launch_testing.

    Description:
    Args:
    * ready_fn:
    Return:
    * None
    """
    proc_env['OSPL_VERBOSITY'] = '8'  # 8 = OS_NONE

    # Launch roscore at first before launching other nodes and processes.
    launch_roscore()

    # This is the process under tests.
    # Execute ros1_bridge and velodyne_node
    # Wait for several seconds after starting roscore.
    this_package = get_package_share_directory('bridge_velodyne_node')
    bridge_velodyne_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([this_package,
                                       '/launch/bridge_velodyne_node.launch.py']))

    # Execute rostopic echo /lidar_front/points_raw
    # Wait for several seconds after velodyne_node starts.
    echo_topic_action = launch.actions.ExecuteProcess(
        cmd=['rostopic', 'echo', '/lidar_front/points_raw'],
        env=ros1_env)

    # Stream Velodyne UDP packet.
    # Wait for several seconds after starting ros2_launch.
    stream_velodyne_packet_action = launch.actions.ExecuteProcess(
        cmd=['udpreplay', '-r', '1000', velodyne_packet_file_path],
        env=proc_env,
        shell=True)

    # Kill roscore and rosmaster during shutdown
    roscore_proc = ['roscore', 'rosmaster', 'rosout']
    kill_proc = []
    for proc in roscore_proc:
        kill_proc.append(launch.actions.ExecuteProcess(
            cmd=['killall', '-KILL', proc],
            env=proc_env
        ))
    proc_on_shutdown = launch.actions.RegisterEventHandler(
        launch.event_handlers.OnShutdown(
            on_shutdown=kill_proc
    ))

    # Opaquefunction.
    opaquefunction = launch.actions.OpaqueFunction(function=lambda context: ready_fn())
    return launch.LaunchDescription([
        bridge_velodyne_node_launch,
        echo_topic_action,
        stream_velodyne_packet_action,
        proc_on_shutdown,
        opaquefunction, ])


class TestROSTopicEcho(unittest.TestCase):

    def test_echo_ROSTopic(self):
        # Very simple test for checking UDP packet stream.
        self.proc_output.assertWaitFor('header:', timeout=170)
        self.proc_output.assertWaitFor('fields:', timeout=170)
        self.proc_output.assertWaitFor('is_dense: False', timeout=170)


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self):
        # Shutdown process related to ROS1 master node.
        launch_testing.asserts.assertExitCodes(self.proc_info,
                                               process='velodyne_cloud_node_exe')
        launch_testing.asserts.assertExitCodes(self.proc_info,
                                               process='dynamic_bridge')
