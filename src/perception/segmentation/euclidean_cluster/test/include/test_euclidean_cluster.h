// Copyright 2019 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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

#include <vector>
#include <utility>

#include "euclidean_cluster/euclidean_cluster.hpp"

#include "test_euclidean_cluster_aux.h"

/////////////////////////////////////////////
/// test for minimal functionality
TEST(euclidean_cluster, simple_bar)
{
  // setup
  builtin_interfaces::msg::Time t;
  Config cfg{"foo", 10U, 100U};
  HashConfig hcfg{-130.0F, 130.0F, -130.0F, 130.0F, 1.0F, 10000U};
  EuclideanCluster cls{cfg, hcfg};
  std::vector<std::pair<float, float>> output;
  // insert points
  insert_line(output, cls, -10.0F, -15.0F, -10.0F, 5.0F, 0.9F);
  // check clusters
  const auto & clusters = cls.cluster(t);
  EXPECT_EQ(clusters.clusters.size(), 1);
  // TODO(c.ho) frame
  EXPECT_TRUE(check_cluster(clusters.clusters[0], output));
  EXPECT_EQ(clusters.clusters[0].header.frame_id, "foo");
  EXPECT_EQ(cls.get_error(), EuclideanCluster::Error::NONE);

  // push another point to force a reset
  insert_point(cls, 0.0F, 0.0F);
  const auto & clusters2 = cls.cluster(t);
  EXPECT_EQ(clusters2.clusters.size(), 0);
  EXPECT_EQ(cls.get_error(), EuclideanCluster::Error::NONE);
}

/// another cluster type + noise
TEST(euclidean_cluster, noisy_L)
{
  // setup
  builtin_interfaces::msg::Time t;
  Config cfg{"foo", 10U, 100U};
  HashConfig hcfg{-130.0F, 130.0F, -130.0F, 130.0F, 1.0F, 10000U};
  EuclideanCluster cls{cfg, hcfg};
  std::vector<std::pair<float, float>> output;
  std::vector<std::pair<float, float>> empty;
  // insert points
  insert_line(output, cls, 10.0F, 15.0F, 15.0F, 20.0F, 0.9F);
  insert_line(output, cls, 5.0F, 20.0F, 10.0F, 15.0F, 0.9F);
  // insert noise
  insert_mesh(empty, cls, -10.0F, -10.0F, -20.0F, -20.0F, 2.0F, 2.0F);
  const auto & res1 = cls.cluster(t);
  // check clusters
  EXPECT_EQ(res1.clusters.size(), 1);
  EXPECT_TRUE(check_cluster(res1.clusters[0], output));
  EXPECT_EQ(res1.clusters[0].header.frame_id, "foo");
  EXPECT_EQ(cls.get_error(), EuclideanCluster::Error::NONE);

  // insert_point function is already tested for memory
  // push another point to force a reset
  insert_point(cls, 0.0F, 0.0F);
  const auto & res2 = cls.cluster(t);
  EXPECT_EQ(res2.clusters.size(), 0);
  EXPECT_EQ(cls.get_error(), EuclideanCluster::Error::NONE);
}

/// everything plus noise
TEST(euclidean_cluster, multi_object)
{
  // setup
  builtin_interfaces::msg::Time t;
  Config cfg{"bar", 10U, 100U};
  HashConfig hcfg{-130.0F, 130.0F, -130.0F, 130.0F, 1.0F, 10000U};
  EuclideanCluster cls{cfg, hcfg};
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
  const auto & res1 = cls.cluster(t);
  // check clusters
  EXPECT_EQ(res1.clusters.size(), 4);

  std::vector<std::vector<std::pair<float, float>> *> outputs =
  {&output1, &output2, &output3, &output4};
  check_clusters(res1, outputs, "bar");
  EXPECT_EQ(cls.get_error(), EuclideanCluster::Error::NONE);

  // push another point to force a reset
  insert_point(cls, 0.0F, 0.0F);
  const auto & res2 = cls.cluster(t);
  EXPECT_EQ(res2.clusters.size(), 0);
  EXPECT_EQ(cls.get_error(), EuclideanCluster::Error::NONE);
}

/// simple negative test case
TEST(euclidean_cluster, no_cluster)
{
  // setup
  builtin_interfaces::msg::Time t;
  Config cfg{"bar", 10U, 100U};
  HashConfig hcfg{-130.0F, 130.0F, -130.0F, 130.0F, 1.0F, 10000U};
  EuclideanCluster cls{cfg, hcfg};
  std::vector<std::pair<float, float>> output;
  // insert points
  insert_line(output, cls, 0.0F, 0.0F, 8.0F, 0.0F, 0.9F);
  insert_line(output, cls, 16.0F, 1.1F, 8.0F, 1.1F, 0.9F);

  // cluster and check
  const auto & res = cls.cluster(t);
  EXPECT_EQ(res.clusters.size(), 0);
  EXPECT_EQ(cls.get_error(), EuclideanCluster::Error::NONE);
}
