# Copyright 2021 the Autoware Foundation
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

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """
    Launch planning nodes.

     * behavior_planner
     * lanelet2_global_planner
     * lane_planner
     * mpc_controller
     * object_collision_estimator
     * parking_planner
    """
    autoware_auto_launch_pkg_prefix = get_package_share_directory(
        'autoware_auto_launch')
    behavior_planner_param_file = os.path.join(
        autoware_auto_launch_pkg_prefix, 'param/behavior_planner.param.yaml')
    lane_planner_param_file = os.path.join(
        autoware_auto_launch_pkg_prefix, 'param/lane_planner.param.yaml')
    mpc_param_file = os.path.join(
        autoware_auto_launch_pkg_prefix, 'param/mpc_controller.param.yaml')
    object_collision_estimator_param_file = os.path.join(
        autoware_auto_launch_pkg_prefix, 'param/object_collision_estimator.param.yaml')
    parking_planner_param_file = os.path.join(
        autoware_auto_launch_pkg_prefix, 'param/parking_planner.param.yaml')
    vehicle_characteristics_param_file = os.path.join(
        autoware_auto_launch_pkg_prefix, 'param/vehicle_characteristics.param.yaml')

    # Arguments
    with_obstacles_param = DeclareLaunchArgument(
        'with_obstacles',
        default_value='True',
        description='Enable obstacle detection'
    )
    behavior_planner_param = DeclareLaunchArgument(
        'behavior_planner_param_file',
        default_value=behavior_planner_param_file,
        description='Path to parameter file for behavior planner'
    )
    lane_planner_param = DeclareLaunchArgument(
        'lane_planner_param_file',
        default_value=lane_planner_param_file,
        description='Path to parameter file for lane planner'
    )
    mpc_param = DeclareLaunchArgument(
        'mpc_param_file',
        default_value=mpc_param_file,
        description='Path to config file for MPC'
    )
    object_collision_estimator_param = DeclareLaunchArgument(
        'object_collision_estimator_param_file',
        default_value=object_collision_estimator_param_file,
        description='Path to parameter file for object collision estimator'
    )
    parking_planner_param = DeclareLaunchArgument(
        'parking_planner_param_file',
        default_value=parking_planner_param_file,
        description='Path to parameter file for parking planner'
    )
    vehicle_characteristics_param = DeclareLaunchArgument(
        'vehicle_characteristics_param_file',
        default_value=vehicle_characteristics_param_file,
        description='Path to config file for vehicle characteristics'
    )

    # Nodes
    behavior_planner = Node(
        package='behavior_planner_nodes',
        name='behavior_planner_node',
        namespace='planning',
        executable='behavior_planner_node_exe',
        parameters=[
            LaunchConfiguration('behavior_planner_param_file'),
            {'enable_object_collision_estimator': LaunchConfiguration('with_obstacles')},
            LaunchConfiguration('vehicle_characteristics_param_file'),
        ],
        output='screen',
        remappings=[
            ('HAD_Map_Service', '/had_maps/HAD_Map_Service'),
            ('vehicle_state', '/vehicle/vehicle_kinematic_state'),
            ('route', 'global_path'),
            ('vehicle_state_report', '/vehicle/state_report'),
            ('vehicle_state_command', '/vehicle/state_command')
        ]
    )
    lanelet2_global_planner = Node(
        package='lanelet2_global_planner_nodes',
        name='lanelet2_global_planner_node',
        namespace='planning',
        executable='lanelet2_global_planner_node_exe',
        remappings=[('HAD_Map_Client', '/had_maps/HAD_Map_Service'),
                    ('vehicle_kinematic_state', '/vehicle/vehicle_kinematic_state')]
    )
    lane_planner = Node(
        package='lane_planner_nodes',
        name='lane_planner_node',
        namespace='planning',
        executable='lane_planner_node_exe',
        parameters=[
            LaunchConfiguration('lane_planner_param_file'),
            LaunchConfiguration('vehicle_characteristics_param_file'),
        ],
        remappings=[('HAD_Map_Service', '/had_maps/HAD_Map_Service')]
    )
    mpc_controller = Node(
        package='mpc_controller_nodes',
        executable='mpc_controller_node_exe',
        name='mpc_controller_node',
        namespace='control',
        parameters=[
            LaunchConfiguration('mpc_param_file'),
            LaunchConfiguration('vehicle_characteristics_param_file'),
        ],
    )
    object_collision_estimator = Node(
        package='object_collision_estimator_nodes',
        name='object_collision_estimator_node',
        namespace='planning',
        executable='object_collision_estimator_node_exe',
        condition=IfCondition(LaunchConfiguration('with_obstacles')),
        parameters=[
            LaunchConfiguration('object_collision_estimator_param_file'),
            LaunchConfiguration('vehicle_characteristics_param_file'),
        ],
        remappings=[
            ('obstacle_topic', '/perception/lidar_bounding_boxes_filtered'),
        ]
    )
    parking_planner = Node(
        package='parking_planner_nodes',
        name='parking_planner_node',
        namespace='planning',
        executable='parking_planner_node_exe',
        parameters=[
            LaunchConfiguration('parking_planner_param_file'),
            LaunchConfiguration('vehicle_characteristics_param_file'),
        ],
        remappings=[('HAD_Map_Service', '/had_maps/HAD_Map_Service')]
    )

    return LaunchDescription([
        with_obstacles_param,
        behavior_planner_param,
        lane_planner_param,
        mpc_param,
        object_collision_estimator_param,
        parking_planner_param,
        vehicle_characteristics_param,
        behavior_planner,
        lanelet2_global_planner,
        lane_planner,
        mpc_controller,
        object_collision_estimator,
        parking_planner,
    ])
