// Copyright 2020 Tier IV, Inc.
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

#ifndef TEST_EUCLIDEAN_CLUSTER_NODES_HPP_
#define TEST_EUCLIDEAN_CLUSTER_NODES_HPP_

#include <rclcpp/rclcpp.hpp>
#include <euclidean_cluster_nodes/euclidean_cluster_node.hpp>

#include <vector>

using autoware::perception::segmentation::euclidean_cluster_nodes::EuclideanClusterNode;

class EuclideanClusterNodesTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_FALSE(rclcpp::ok());
    rclcpp::init(0, nullptr);
    ASSERT_TRUE(rclcpp::ok());
  }

  void TearDown() override
  {
    (void)rclcpp::shutdown();
  }
};

TEST_F(EuclideanClusterNodesTest, Instantiate)
{
  rclcpp::NodeOptions node_options;

  std::vector<rclcpp::Parameter> params;

  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("use_detected_objects", false);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("use_cluster", true);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("use_box", true);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("max_cloud_size", 55000);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("downsample", false);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("use_lfit", true);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("use_z", true);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("cluster.frame_id", "base_link");
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("cluster.min_cluster_size", 10);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("cluster.max_num_clusters", 256);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("cluster.min_cluster_threshold_m", 0.5);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("cluster.max_cluster_threshold_m", 1.5);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("cluster.threshold_saturation_distance_m", 60.0);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("hash.min_x", -130.0);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("hash.max_x", 130.0);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("hash.min_y", -130.0);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("hash.max_y", 130.0);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("hash.side_length", 1.0);
  node_options.parameter_overrides(params);

  ASSERT_NO_THROW(EuclideanClusterNode{node_options});
}


TEST_F(EuclideanClusterNodesTest, InstantiateDownsample)
{
  rclcpp::NodeOptions node_options;

  std::vector<rclcpp::Parameter> params;

  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("use_detected_objects", false);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("use_cluster", true);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("use_box", true);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("max_cloud_size", 55000);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("downsample", true);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("use_lfit", true);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("use_z", true);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("cluster.frame_id", "base_link");
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("cluster.min_cluster_size", 10);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("cluster.max_num_clusters", 256);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("cluster.min_cluster_threshold_m", 0.5);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("cluster.max_cluster_threshold_m", 1.5);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("cluster.threshold_saturation_distance_m", 60.0);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("hash.min_x", -130.0);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("hash.max_x", 130.0);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("hash.min_y", -130.0);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("hash.max_y", 130.0);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("hash.side_length", 1.0);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("voxel.min_point.x", -130.0);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("voxel.min_point.y", -130.0);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("voxel.min_point.z", -130.0);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("voxel.max_point.x", 130.0);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("voxel.max_point.y", 130.0);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("voxel.max_point.z", 130.0);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("voxel.voxel_size.x", 0.2);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("voxel.voxel_size.y", 0.2);
  node_options.parameter_overrides(params);
  ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("voxel.voxel_size.z", 0.2);
  node_options.parameter_overrides(params);

  ASSERT_NO_THROW(EuclideanClusterNode{node_options});
}

#endif  // TEST_EUCLIDEAN_CLUSTER_NODES_HPP_
