import os
from ament_index_python import get_package_share_directory
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():

    # map_provider parameter file definition
    ndt_map_provider_file_path = os.path.join(
        get_package_share_directory("ndt_nodes"),
        "param",
        "map_publisher.param.yaml")
    map_provider_param_file=launch.substitutions.LaunchConfiguration(
        "params", default=[ndt_map_provider_file_path])

    # map_provide node execution definition
    map_provider_node_runner = launch_ros.actions.Node(
        package="ndt_nodes",
        node_executable="ndt_map_publisher_exe",
        parameters=[map_provider_param_file])

    # map downsampler paramter file definition
    ndt_map_downsampler_file_path = os.path.join(
        get_package_share_directory("ndt_nodes"),
        "param",
        "pcl_map_voxel_grid_downsample.param.yaml")
    map_downsampler_param_file=launch.substitutions.LaunchConfiguration(
        "params", default=[ndt_map_downsampler_file_path])

    # map downsample node execution definition
    map_downsampler_node_runner = launch_ros.actions.Node(
        package="voxel_grid_nodes",
        node_executable="voxel_grid_cloud_node_exe",
        parameters=[map_downsampler_param_file])
    
    return launch.LaunchDescription([
        map_provider_node_runner,
        map_downsampler_node_runner])
