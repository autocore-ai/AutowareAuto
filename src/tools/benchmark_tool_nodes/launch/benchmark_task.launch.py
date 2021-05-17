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

import os
import sys
from pathlib import Path
import launch
import launch_ros.actions
from launch_ros.substitutions import ExecutableInPackage
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from os.path import join as joinPath
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition


def launch_node(context, *args, **kwargs):

    # Used OpaqueFunction to resolve the following arguments
    node_output = LaunchConfiguration('node_output').perform(context)
    node_start_delay = LaunchConfiguration('node_start_delay').perform(context)
    result_path = LaunchConfiguration('result_path').perform(context)
    rosbag_record_subfolder = LaunchConfiguration('rosbag_record_subfolder').perform(context)
    input_topic = LaunchConfiguration('input_topic').perform(context)
    output_topic = LaunchConfiguration('output_topic').perform(context)

    node_env = os.environ.copy()
    node_env["PYTHONUNBUFFERED"] = "1"

    rosbag2_node = ExecuteProcess(
        cmd=['ros2',
             'bag',
             'record',
             '-o ' + result_path + '/' + rosbag_record_subfolder,
             input_topic,
             output_topic
             ],
        condition=IfCondition(LaunchConfiguration("rosbag_record"))
    )

    benchmark_tool_node = launch_ros.actions.Node(
        package='benchmark_tool_nodes',
        executable='benchmark_tool_node.py',
        namespace='benchmark_tool',
        output={
            'stdout': node_output,
            'stderr': node_output
        },
        name=[LaunchConfiguration('node_name'), 'benchmark_tool'],
        prefix=[
            'bash -c \'sleep ' + node_start_delay + '; $0 $@\' '
        ],
        parameters=[
            {'benchmark_task': LaunchConfiguration('benchmark_task')},
            {'dataset_path': LaunchConfiguration('dataset_path')},
            {'input_topic': LaunchConfiguration('input_topic')},
            {'output_topic': LaunchConfiguration('output_topic')},
            {'benchmarked_input_topic': LaunchConfiguration('benchmarked_input_topic')},
            {'benchmarked_output_topic': LaunchConfiguration('benchmarked_output_topic')},
            {'result_path': LaunchConfiguration('result_path')},
            {'force_end_at_frame_n': LaunchConfiguration('force_end_at_frame_n')}
        ],
        on_exit=[
            launch.actions.LogInfo(msg="benchmark_tool exited"),
            launch.actions.Shutdown()
        ],
        env=node_env
    )

    ros_info_node = launch_ros.actions.Node(
        package='benchmark_tool_nodes',
        executable='ros_info_node.py',
        name=[LaunchConfiguration('node_name'), 'ros_info'],
        prefix=[
            'bash -c \'sleep ' + node_start_delay + '; $0 $@\' '
        ],
        parameters=[
            {'result_path': LaunchConfiguration('result_path')},
            {'sampling_rate': 1}
        ],
        on_exit=[
            launch.actions.LogInfo(msg="ros_info exited")
        ],
        condition=IfCondition(LaunchConfiguration("ros_info_record"))
    )

    sys_info_node = launch_ros.actions.Node(
        package='benchmark_tool_nodes',
        executable='sys_info_node.py',
        name=[LaunchConfiguration('node_name'), 'sys_info'],
        prefix=[
            'bash -c \'sleep ' + node_start_delay + '; $0 $@\' '
        ],
        parameters=[
            {'result_path': LaunchConfiguration('result_path')},
            {'sampling_rate': 1}
        ],
        on_exit=[
            launch.actions.LogInfo(msg="sys_info exited")
        ],
        condition=IfCondition(LaunchConfiguration("sys_info_record"))
    )

    perftest_exe = ExecutableInPackage(
        package='benchmark_tool_nodes',
        executable='perftest'
    ).perform(context)
    if IfCondition(LaunchConfiguration("cyclone_dds_info_record")).evaluate(context):
        Path(result_path + '/dds').mkdir(parents=True, exist_ok=True)

    def cyclone_dds_info_node(mode):
        assert(mode in ['throughput', 'latency'])
        res_file = 'sub.log'
        if mode == 'latency':
            res_file = 'ping.log'
        extract_exe = ExecutableInPackage(
            package='benchmark_tool_nodes',
            executable=mode + '-test-extract'
        ).perform(context)
        # perftest arguments used here:
        #   -L local run
        #   -f remove output directory first
        #   -t run time (seconds)
        #   -a sync or async mode
        #   -m subscriber mode
        #   -O operation
        perftest_args = '-L -f -t 3 -a sync -m listener -O ' + mode
        extract_args = result_path + '/dds/' + mode + '-result/sync-listener/' + res_file
        return ExecuteProcess(
            name='cyclone_dds_' + mode,
            shell=True,
            prefix=[
                'sleep ' + node_start_delay + ' ;'
            ],
            cmd=[
                perftest_exe + ' ' + perftest_args + ' && ' +
                extract_exe + ' ' + extract_args + ' > ' + result_path + '/dds-' + mode + '.txt'
            ],
            cwd=result_path + '/dds',
            output={'stderr': 'own_log'},
            condition=IfCondition(LaunchConfiguration("cyclone_dds_info_record"))
        )

    return [
        rosbag2_node,
        benchmark_tool_node,
        ros_info_node,
        sys_info_node,
        cyclone_dds_info_node('throughput'),
        cyclone_dds_info_node('latency'),
    ]


def handle_output(output):
    text = output.text.decode("utf-8")
    if "Progress" in text:
        sys.stdout.write(text)
        sys.stdout.flush()


def generate_launch_description():
    global ROS_VERSION

    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'benchmark_task',
            default_value='benchmark_task',
            description='Task to be launched',
        ),
        launch.actions.DeclareLaunchArgument(
            'dataset_path',
            default_value=joinPath(os.environ['HOME'], 'data_source'),
            description='Path of the dataset in the system',
        ),
        launch.actions.DeclareLaunchArgument(
            'input_topic',
            default_value='/input_topic',
            description='Input topic of the blackbox system to be benchmarked',
        ),
        launch.actions.DeclareLaunchArgument(
            'output_topic',
            default_value='/output_topic',
            description='Output topic of the blackbox system to be benchmarked',
        ),
        launch.actions.DeclareLaunchArgument(
            'benchmarked_input_topic',
            default_value=[LaunchConfiguration('input_topic')],
            description='The input topic of the benchmarked node',
        ),
        launch.actions.DeclareLaunchArgument(
            'benchmarked_output_topic',
            default_value=[LaunchConfiguration('output_topic')],
            description='The output topic of the benchmarked node',
        ),
        launch.actions.DeclareLaunchArgument(
            'result_path',
            default_value=joinPath(os.environ['HOME'], 'benchmark_tool_result'),
            description='',
        ),
        launch.actions.DeclareLaunchArgument(
            'force_end_at_frame_n',
            default_value='-1',
            description='Limit the number of played frames (-1 means unlimited)',
        ),
        launch.actions.DeclareLaunchArgument(
            'node_start_delay',
            default_value='0',
            description='',
        ),
        launch.actions.DeclareLaunchArgument(
            'node_name',
            default_value='benchmark_tool_node',
            description='The name of the node',
        ),
        launch.actions.DeclareLaunchArgument(
            'node_output',
            default_value='screen',
            description='Where to display running informations (screen or log)',
        ),
        launch.actions.DeclareLaunchArgument(
            'rosbag_record',
            default_value='False',
            description='Record on rosbag the input and output topic during the benchmark',
        ),
        launch.actions.DeclareLaunchArgument(
            'rosbag_record_subfolder',
            default_value='The subfolder on filesystem where to save the rosbag record file, it '
                          'must be a subfolder of result_path folder',
            description='Record on rosbag the input and output topic during the benchmark',
        ),
        launch.actions.DeclareLaunchArgument(
            'ros_info_record',
            default_value='False',
            description='Record ROS node topology and bandwidth information during the benchmark',
        ),
        launch.actions.DeclareLaunchArgument(
            'sys_info_record',
            default_value='False',
            description='Record system metrics during the benchmark',
        ),
        launch.actions.DeclareLaunchArgument(
            'cyclone_dds_info_record',
            default_value='False',
            description='Record DDS metrics during the benchmark',
        ),
        OpaqueFunction(function=launch_node)
    ])

    ld.add_action(
        launch.actions.RegisterEventHandler(
            launch.event_handlers.OnProcessIO(on_stdout=handle_output, on_stderr=handle_output)
        )
    )

    return ld
