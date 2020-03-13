"""Launch a few nodes for easy debugging and testing of the kinematic tracker"""

import launch
import launch_ros.actions


def generate_launch_description():
  driver = launch_ros.actions.Node(
      package='velodyne_node', node_executable='velodyne_block_node_exe',
      node_name='vlp16_front', node_namespace='lidar_front')
  classifier = launch_ros.actions.Node(
      package='ray_ground_classifier_nodes', node_executable='ray_ground_classifier_block_node_exe',
      node_name='ray_ground_classifier')
  return launch.LaunchDescription([driver, classifier])

