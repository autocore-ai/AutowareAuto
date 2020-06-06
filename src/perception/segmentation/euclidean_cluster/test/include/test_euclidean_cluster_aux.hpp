// Copyright 2019 Apex.AI, Inc.
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

#ifndef TEST_EUCLIDEAN_CLUSTER_AUX_HPP_
#define TEST_EUCLIDEAN_CLUSTER_AUX_HPP_

#include <common/types.hpp>

#include <thread>
#include <chrono>
#include <vector>
#include <utility>
#include <algorithm>
#include <string>

#include "euclidean_cluster/euclidean_cluster.hpp"

using autoware::perception::segmentation::euclidean_cluster::PointXYZII;
using autoware::perception::segmentation::euclidean_cluster::PointXYZI;
using autoware::perception::segmentation::euclidean_cluster::HashConfig;
using autoware::perception::segmentation::euclidean_cluster::Config;
using autoware::perception::segmentation::euclidean_cluster::EuclideanCluster;
using autoware::perception::segmentation::euclidean_cluster::Clusters;
using autoware::perception::segmentation::euclidean_cluster::Cluster;
using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

///
void insert_point(EuclideanCluster & cls, const float32_t x, const float32_t y)
{
  cls.insert(PointXYZI{x, y, 0.0F, 0.0F});
}


///
void insert_ring(
  EuclideanCluster & cls,
  const float32_t r,
  const uint32_t num_pts,
  const float32_t dx = 0.0F,
  const float32_t dy = 0.0F)
{
  const float32_t dth = 2.0F * 3.14159F / num_pts;
  float32_t th = 0.0F;
  for (uint32_t idx = 0U; idx < num_pts; ++idx) {
    const float32_t x = r * cosf(th) + dx;
    const float32_t y = r * sinf(th) + dy;
    insert_point(cls, x, y);
    th += dth;
  }
}
///
void insert_line(
  std::vector<std::pair<float32_t, float32_t>> & output,
  EuclideanCluster & cls,
  const float32_t x1, const float32_t y1,
  const float32_t x2, const float32_t y2,
  const float32_t ds)
{
  const float32_t dx = x2 - x1;
  const float32_t dy = y2 - y1;
  const float32_t len = sqrtf(dx * dx + dy * dy);
  const uint32_t num_pts = static_cast<uint32_t>(std::ceil(fabsf((len / ds))));
  for (uint32_t idx = 0U; idx < num_pts; ++idx) {
    const float32_t x = x1 + (idx * dx / num_pts);
    const float32_t y = y1 + (idx * dy / num_pts);
    insert_point(cls, x, y);
    output.push_back({x, y});
  }
}

///
void insert_mesh(
  std::vector<std::pair<float32_t, float32_t>> & output,
  EuclideanCluster & cls,
  const float32_t x1, const float32_t y1,
  const float32_t x2, const float32_t y2,
  const float32_t dx, const float32_t dy)
{
  const float32_t len_y = y2 - y1;
  const uint32_t num_pts = static_cast<uint32_t>(std::ceil(fabsf(len_y / dy)));
  for (uint32_t idx = 0U; idx < num_pts; ++idx) {
    const float32_t y = y1 + idx * dy;
    insert_line(output, cls, x1, y, x2, y, dx);
  }
}

///
bool8_t check_point(
  const std::vector<std::pair<float32_t, float32_t>> & expected, const PointXYZI & pt)
{
  bool8_t found = false;
  const float32_t TOL = 0.00001F;
  for (auto & val : expected) {
    if (fabsf(pt.x - val.first) < TOL && fabsf(pt.y - val.second) < TOL) {
      found = true;
      break;
    }
  }
  return found;
}

///
bool8_t check_cluster(
  const Cluster & cls, const std::vector<std::pair<float32_t, float32_t>> & expected)
{
  bool8_t found = true;

  for (uint32_t idx = 0U; idx < cls.width; ++idx) {
    const auto jdx = cls.point_step * idx;
    PointXYZI pt;
    (void)memcpy(&pt, &cls.data[jdx], std::min(cls.point_step, static_cast<uint32_t>(sizeof(pt))));
    found &= check_point(expected, pt);
  }
  // std::cout << "Expected num points: " << expected.size();
  // std::cout << " num in cluster: " << num_points << "\n";
  return found;
}

///
void check_clusters(
  const Clusters & cls,
  const std::vector<std::vector<std::pair<float32_t, float32_t>> *> & expected,
  const std::string & frame)
{
  for (uint32_t idx = 0U; idx < cls.clusters.size(); ++idx) {
    bool8_t found = false;
    for (auto & vals : expected) {
      found |= check_cluster(cls.clusters[idx], *vals);
      EXPECT_EQ(cls.clusters[idx].header.frame_id, frame);
      if (found) {
        break;
      }
    }
    ASSERT_TRUE(found);
  }
}

#endif  // TEST_EUCLIDEAN_CLUSTER_AUX_HPP_
