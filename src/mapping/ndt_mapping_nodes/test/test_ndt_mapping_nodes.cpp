// Copyright 2020 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

/// \copyright Copyright 2020 Apex.AI, Inc.
/// All rights reserved.

#include <gtest/gtest.h>
#include <ndt_mapping_nodes/ndt_mapping_nodes.hpp>

TEST(TestNdtMappingNode, Instantiate)
{
  // Basic test to ensure that TrajectorySpooferNode can be instantiated
  rclcpp::init(0, nullptr);

  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("file_name_prefix", "ndt_sample_map");
  node_options.append_parameter_override("publish_map_increment", true);

  node_options.append_parameter_override("map.capacity", 1000000);
  node_options.append_parameter_override("map.min_point.x", -1000.0);
  node_options.append_parameter_override("map.min_point.y", -1000.0);
  node_options.append_parameter_override("map.min_point.z", -3.0);
  node_options.append_parameter_override("map.max_point.x", 1000.0);
  node_options.append_parameter_override("map.max_point.y", 1000.0);
  node_options.append_parameter_override("map.max_point.z", 3.0);
  node_options.append_parameter_override("map.voxel_size.x", 1.0);
  node_options.append_parameter_override("map.voxel_size.y", 1.0);
  node_options.append_parameter_override("map.voxel_size.z", 1.0);
  node_options.append_parameter_override("map.frame_id", "map");

  node_options.append_parameter_override("localizer.map.capacity", 1000000);
  node_options.append_parameter_override("localizer.map.min_point.x", -1000.0);
  node_options.append_parameter_override("localizer.map.min_point.y", -1000.0);
  node_options.append_parameter_override("localizer.map.min_point.z", -3.0);
  node_options.append_parameter_override("localizer.map.max_point.x", 1000.0);
  node_options.append_parameter_override("localizer.map.max_point.y", 1000.0);
  node_options.append_parameter_override("localizer.map.max_point.z", 3.0);
  node_options.append_parameter_override("localizer.map.voxel_size.x", 3.5);
  node_options.append_parameter_override("localizer.map.voxel_size.y", 3.5);
  node_options.append_parameter_override("localizer.map.voxel_size.z", 3.5);
  node_options.append_parameter_override("localizer.scan.capacity", 1000000);
  node_options.append_parameter_override("localizer.optimization.outlier_ratio", 0.55);
  node_options.append_parameter_override("localizer.optimizer.max_iterations", 40);
  node_options.append_parameter_override("localizer.optimizer.score_tolerance", 0.1);
  node_options.append_parameter_override("localizer.optimizer.parameter_tolerance", 0.1);
  node_options.append_parameter_override("localizer.optimizer.gradient_tolerance", 0.1);
  node_options.append_parameter_override("localizer.optimizer.line_search.step_max", 0.12);
  node_options.append_parameter_override("localizer.optimizer.line_search.step_min", 0.001);
  node_options.append_parameter_override("localizer.guess_time_tolerance_ms", 750);

  node_options.append_parameter_override("map_increment_pub.history_depth", 10);
  node_options.append_parameter_override("observation_sub.history_depth", 10);
  node_options.append_parameter_override("map_sub.history_depth", 1);
  node_options.append_parameter_override("pose_pub.history_depth", 10);
  node_options.append_parameter_override("publish_tf", true);

  node_options.append_parameter_override("predict_pose_threshold.translation", 50.0);
  node_options.append_parameter_override("predict_pose_threshold.rotation", 3.15);

  node_options.append_parameter_override("init_hack.translation.x", 0.0);
  node_options.append_parameter_override("init_hack.translation.y", 0.0);
  node_options.append_parameter_override("init_hack.translation.z", 0.0);

  node_options.append_parameter_override("init_hack.quaternion.x", 0.0);
  node_options.append_parameter_override("init_hack.quaternion.y", 0.0);
  node_options.append_parameter_override("init_hack.quaternion.z", 0.0);
  node_options.append_parameter_override("init_hack.quaternion.w", 1.0);
  // TODO(yunus.caliskan): Write a proper mapping node test: #927
  ASSERT_NO_THROW(autoware::mapping::ndt_mapping_nodes::P2DNDTVoxelMapperNode<>{node_options});
}
