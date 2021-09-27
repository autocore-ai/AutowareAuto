# Copyright 2021 The Autoware Foundation
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
#
# Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

"""Launch required subsystems for running scenario simulator."""

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution

import os
from enum import Enum


class MapFile(Enum):
    POINT_CLOUD_FILE = 1,
    POINT_CLOUD_PARAMS_FILE = 2,
    MAP_PUBLISHER_PARAMS = 3,
    LANELET2_MAP = 4,
    LANELET2_MAP_PROVIDER_PARAMS = 5


def find_and_validate_single_file_with_extension(base_path, file_list, extension):
    error_msg = ""
    file_path = ""

    matches = []
    for file in file_list:
        if file.endswith(extension):
            matches.append(file)

    if len(matches) > 1:
        error_msg = "ERROR: Single *.{} file expected in {}. Files found: {}".\
            format(extension, base_path, " ".join(matches))

    if len(matches) == 0:
        error_msg = "ERROR: No files fulfilling pattern {}/*.{}".format(base_path, extension)

    if not error_msg:
        file_path = os.path.join(base_path, matches[0])
    return file_path, error_msg


def launch_setup(context, *args, **kwargs):
    DeclareLaunchArgument('map_path')
    map_path = LaunchConfiguration('map_path').perform(context)

    map_filenames_extensions = {MapFile.POINT_CLOUD_FILE: 'pcd',
                                MapFile.POINT_CLOUD_PARAMS_FILE: 'pcd.yaml',
                                MapFile.MAP_PUBLISHER_PARAMS: 'map_publisher.yaml',
                                MapFile.LANELET2_MAP: 'osm',
                                MapFile.LANELET2_MAP_PROVIDER_PARAMS: 'osm.yaml'}

    files_in_map_path = os.listdir(map_path)
    map_file_paths = {}
    for key, map_filename_extension in map_filenames_extensions.items():
        found_file, error_msg = \
            find_and_validate_single_file_with_extension(map_path, files_in_map_path,
                                                         map_filename_extension)
        if error_msg:
            return [LogInfo(msg=TextSubstitution(text=error_msg))]
        else:
            map_file_paths[key] = found_file

    autoware_launch_prefix = get_package_share_directory('autoware_auto_launch')

    lanelet2_map_provider = Node(
        package='lanelet2_map_provider',
        executable='lanelet2_map_provider_exe',
        namespace='had_maps',
        name='lanelet2_map_provider_node',
        parameters=[map_file_paths[MapFile.LANELET2_MAP_PROVIDER_PARAMS],
                    {"map_osm_file": map_file_paths[MapFile.LANELET2_MAP]}]
    )

    lanelet2_map_visualizer = Node(
        package='lanelet2_map_provider',
        executable='lanelet2_map_visualizer_exe',
        name='lanelet2_map_visualizer_node',
        namespace='had_maps'
    )

    map_publisher = Node(
        package='ndt_nodes',
        executable='ndt_map_publisher_exe',
        namespace='localization',
        parameters=[map_file_paths[MapFile.MAP_PUBLISHER_PARAMS],
                    {"map_pcd_file": map_file_paths[MapFile.POINT_CLOUD_FILE],
                     "map_yaml_file": map_file_paths[MapFile.POINT_CLOUD_PARAMS_FILE]}]
    )

    planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([autoware_launch_prefix,
                                       '/launch/autoware_auto_planning.launch.py']),
        launch_arguments={}.items()
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([autoware_launch_prefix,
                                       '/launch/autoware_auto_perception.launch.py']),
        launch_arguments={}.items()
    )

    return [
        lanelet2_map_provider,
        lanelet2_map_visualizer,
        map_publisher,
        planning,
        perception
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
