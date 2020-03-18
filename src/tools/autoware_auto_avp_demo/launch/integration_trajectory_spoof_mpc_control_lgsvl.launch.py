# Copyright 2020 Apex.AI, Inc.
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import launch
import launch_ros.actions
import launch.substitutions
import launch.launch_description_sources
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument 
from ros2launch.api import get_share_file_path_from_package


# def get_path(package_name, param_file):
#     return ros2launch.api.get_share_file_path_from_package(
#         package_name=package_name,
#         file_name=param_file
#     )


def generate_launch_description():

    # --------------------------------- Params -------------------------------

    # In combination 'raw_command', 'basic_command' and 'high_level_command' control 
    # in what mode of control comands to operate in, 
    # only one of them can be active at a time with a topics name 
    # other should be blank/null which is achieved by used ="''"
    high_level_command_param = DeclareLaunchArgument(
        'high_level_command', 
        default_value="''", # use "high_level_command" or "''" 
        description='high_level_command control mode topic name')

    basic_command_param = DeclareLaunchArgument(
        'basic_command', 
        default_value="vehicle_command", # use "vehicle_command" or "''" 
        description='basic_command control mode topic name')

    raw_command_param = DeclareLaunchArgument(
        'raw_command', 
        default_value="''",  # use "raw_command" or "''" 
        description='raw_command control mode topic name')
    
    # Default lgsvl_interface params
    lgsvl_interface_param = DeclareLaunchArgument(
        'lgsvl_interface_param',
        default_value=[ 
            get_share_file_path_from_package(package_name='lgsvl_interface',file_name='lgsvl.param.yaml')
        ],
        description='Path to config file for lgsvl interface')

    # -------------------------------- Nodes-----------------------------------

    ######################
    ### mpc_controller ###
    ######################
    mpc_controller_node = launch_ros.actions.Node(
        package="mpc_controller_node",
        node_executable="mpc_controller_node_exe",
        node_name="mpc_controller",
        parameters=[get_share_file_path_from_package(package_name='mpc_controller_node',file_name='defaults.yaml'),
            # overwrite parameters from yaml here
            {
                "command_topic" :  "vehicle_command",
                "debug_trajectory_publish_period_ms": 100, 
                #"state_topic" : "debug_state",
                "controller.interpolation": True,
                "controller.sample_tolerance_ms": 20,
                "controller.control_lookahead_ms": 100,
                "controller.limits.min_longitudinal_velocity_mps": 0.5,
                "controller.limits.max_longitudinal_velocity_mps": 35.0,
                # jerk is used as limit for longitudinal control for whatever reason, see
                # https://gitlab.com/aninnymouse/mpc/-/blob/master/control/mpc_controller/src/mpc_controller/mpc_controller.cpp#L280
                "controller.limits.min_jerk_mps3": -3.0,
                "controller.limits.max_jerk_mps3": 3.0,
                # And also steer angle RATE for lateral control
                "controller.limits.min_steer_angle_rate_rps": -0.331,
                "controller.limits.max_steer_angle_rate_rps": 0.331,
                "controller.vehicle.cg_to_front_m": 1.2,
                "controller.vehicle.cg_to_rear_m": 1.5,
                "controller.behavior.stop_rate_mps2": 3.0,
                "controller.behavior.time_step_ms": 100,
                "controller.behavior.is_temporal_reference": False,
                "controller.weights.nominal.pose": 10.0,
                "controller.weights.nominal.heading": 10.0,
                "controller.weights.nominal.longitudinal_velocity": 10.0,
                "controller.weights.terminal.pose": 1000.0,
                "controller.weights.terminal.heading": 1000.0,
                "controller.weights.terminal.longitudinal_velocity": 1000.0,
            }
        ],
        output='screen',
    )

    ###########################
    ### trajectory_spoofer ###
    ###########################
    trajectory_spoofer_node = launch_ros.actions.Node(
        package="trajectory_spoofer",
        node_executable="trajectory_spoofer_exe",
        node_name="trajectory_spoofer",
        parameters=[
            {
                "speed_ramp_on": False,
                "target_speed": 3.0,
                "num_of_points": 50,
                "trajectory_type": 'straight', #straight or circle
                "length": 10.0, # only used for straight
                "radius": 21.0, # only used for circle
            }
        ],
        output='screen',
    )

    ###########################
    ### LGSVL interface     ###
    ###########################
    lgsvl_launch_file_path = get_share_file_path_from_package(package_name='lgsvl_interface',file_name='lgsvl.launch.py')
    lgsvl = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(lgsvl_launch_file_path),
        launch_arguments={
            "lgsvl_interface_param": LaunchConfiguration("lgsvl_interface_param"),
            "high_level_command": LaunchConfiguration('high_level_command'),
            "basic_command": LaunchConfiguration('basic_command'),
            "raw_command": LaunchConfiguration('raw_command')
        }.items()
    )

 
    #######################################
    ### lexus_rx_450h_description       ###
    #######################################
    lexus_rx_450h_urdf_path = get_share_file_path_from_package(package_name='lexus_rx_450h_description',file_name='lexus_rx_450h.urdf')
    lexus_rx_450h_description_node = launch_ros.actions.Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            node_name='robot_state_publisher',
            arguments=[str(lexus_rx_450h_urdf_path)])
    

    return launch.LaunchDescription([
      high_level_command_param,
      basic_command_param,
      raw_command_param,
      mpc_controller_node,
      trajectory_spoofer_node,
      lgsvl_interface_param,
      lgsvl,
      lexus_rx_450h_description_node])
