// Copyright 2019 the Autoware Foundation
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

#ifndef TEST_EUCLIDEAN_CLUSTER_HPP_
#define TEST_EUCLIDEAN_CLUSTER_HPP_

#include <common/types.hpp>

#include <vector>
#include <utility>

#include "euclidean_cluster/euclidean_cluster.hpp"

#include "test_euclidean_cluster_aux.hpp"

using autoware::common::types::float32_t;

/////////////////////////////////////////////
/// test for minimal functionality
TEST(EuclideanCluster, SimpleBar)
{
  /// setup
  Config cfg{"foo", 10U, 100U, 1.0F, 1.0F, 10.0F};
  HashConfig hcfg{-130.0F, 130.0F, -130.0F, 130.0F, 1.0F, 10000U};
  EuclideanCluster cls{cfg, hcfg};
  Clusters clusters;
  std::vector<std::pair<float, float>> output;
  // insert points
  insert_line(output, cls, -10.0F, -15.0F, -10.0F, 5.0F, 0.9F);
  // check clusters
  cls.cluster(clusters);
  EXPECT_EQ(clusters.cluster_boundary.size(), 1U);
  // TODO(c.ho) frame
  EXPECT_TRUE(check_cluster(clusters, 0, output));
  EXPECT_EQ(cls.get_error(), EuclideanCluster::Error::NONE);

  // push another point to force a reset
  insert_point(cls, 0.0F, 0.0F);
  cls.cluster(clusters);
  EXPECT_EQ(clusters.cluster_boundary.size(), 0U);
  EXPECT_EQ(cls.get_error(), EuclideanCluster::Error::NONE);
}

/// another cluster type + noise
TEST(EuclideanCluster, NoisyL)
{
  // setup
  builtin_interfaces::msg::Time t;
  Config cfg{"foo", 10U, 100U, 1.0F, 1.0F, 10.0F};
  HashConfig hcfg{-130.0F, 130.0F, -130.0F, 130.0F, 1.0F, 10000U};
  EuclideanCluster cls{cfg, hcfg};
  Clusters res1;
  std::vector<std::pair<float, float>> output;
  std::vector<std::pair<float, float>> empty;
  // insert points
  insert_line(output, cls, 10.0F, 15.0F, 15.0F, 20.0F, 0.9F);
  insert_line(output, cls, 5.0F, 20.0F, 10.0F, 15.0F, 0.9F);
  // insert noise
  insert_mesh(empty, cls, -10.0F, -10.0F, -20.0F, -20.0F, 2.0F, 2.0F);
  cls.cluster(res1);
  // check clusters
  EXPECT_EQ(res1.cluster_boundary.size(), 1U);
  EXPECT_TRUE(check_cluster(res1, 0, output));
  EXPECT_EQ(cls.get_error(), EuclideanCluster::Error::NONE);

  // insert_point function is already tested for memory
  // push another point to force a reset
  insert_point(cls, 0.0F, 0.0F);
  cls.cluster(res1);
  EXPECT_EQ(res1.cluster_boundary.size(), 0U);
  EXPECT_EQ(cls.get_error(), EuclideanCluster::Error::NONE);
}

/// everything plus noise
TEST(EuclideanCluster, MultiObject)
{
  // setup
  builtin_interfaces::msg::Time t;
  Config cfg{"bar", 10U, 100U, 1.0F, 1.0F, 10.0F};
  HashConfig hcfg{-130.0F, 130.0F, -130.0F, 130.0F, 1.0F, 10000U};
  EuclideanCluster cls{cfg, hcfg};
  Clusters res1;
  std::vector<std::pair<float, float>> output1;
  std::vector<std::pair<float, float>> output2;
  std::vector<std::pair<float, float>> output3;
  std::vector<std::pair<float, float>> output4;
  std::vector<std::pair<float, float>> empty;
  // L
  insert_line(output1, cls, 11.0F, 16.0F, 16.0F, 21.0F, 0.9F);
  insert_ring(cls, 70.0F, 30U);  // noise ring
  insert_line(output1, cls, 5.0F, 20.0F, 11.0F, 16.0F, 0.9F);

  // bar
  insert_line(output2, cls, 5.0F, 10.0F, 0.0F, 5.0F, 0.3F);

  // mesh
  insert_mesh(output3, cls, -10.0F, -10.0F, -20.0F, -20.0F, 0.5F, 0.5F);

  // insert noise ring
  insert_ring(cls, 50.0F, 30U);

  // bracket
  insert_line(output4, cls, -10.0F, 5.0F, -5.0F, 5.0F, 0.9F);
  insert_line(output4, cls, -5.0F, 5.0F, -5.0F, 15.0F, 0.9F);
  insert_ring(cls, 90.0F, 30U);  // noise ring
  insert_line(output4, cls, -15.0F, 17.0F, -4.0F, 14.0F, 0.9F);

  // cluster
  cls.cluster(res1);
  // check clusters
  EXPECT_EQ(res1.cluster_boundary.size(), 4U);

  std::vector<std::vector<std::pair<float, float>> *> outputs =
  {&output1, &output2, &output3, &output4};
  check_clusters(res1, outputs);
  EXPECT_EQ(cls.get_error(), EuclideanCluster::Error::NONE);

  // push another point to force a reset
  insert_point(cls, 0.0F, 0.0F);
  cls.cluster(res1);
  EXPECT_EQ(res1.cluster_boundary.size(), 0U);
  EXPECT_EQ(cls.get_error(), EuclideanCluster::Error::NONE);
}

/// simple negative test case
TEST(EuclideanCluster, NoCluster)
{
  // setup
  builtin_interfaces::msg::Time t;
  Config cfg{"bar", 10U, 100U, 1.0F, 1.0F, 10.0F};
  HashConfig hcfg{-130.0F, 130.0F, -130.0F, 130.0F, 1.0F, 10000U};
  EuclideanCluster cls{cfg, hcfg};
  Clusters res;
  std::vector<std::pair<float, float>> output;
  // insert points
  insert_line(output, cls, 0.0F, 0.0F, 8.0F, 0.0F, 0.9F);
  insert_line(output, cls, 16.0F, 1.1F, 8.0F, 1.1F, 0.9F);

  // cluster and check
  cls.cluster(res);
  EXPECT_EQ(res.cluster_boundary.size(), 0U);
  EXPECT_EQ(cls.get_error(), EuclideanCluster::Error::NONE);
}
#endif  // TEST_EUCLIDEAN_CLUSTER_HPP_
