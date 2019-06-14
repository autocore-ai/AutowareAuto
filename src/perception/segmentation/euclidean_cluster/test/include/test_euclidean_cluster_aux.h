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

#ifndef TEST_EUCLIDEAN_CLUSTER_AUX_H_
#define TEST_EUCLIDEAN_CLUSTER_AUX_H_

#include <thread>
#include <chrono>
#include <vector>
#include <utility>

#include "euclidean_cluster/euclidean_cluster.hpp"

using autoware::perception::segmentation::euclidean_cluster::PointXYZII;
using autoware::perception::segmentation::euclidean_cluster::PointXYZI;
using autoware::perception::segmentation::euclidean_cluster::HashConfig;
using autoware::perception::segmentation::euclidean_cluster::Config;
using autoware::perception::segmentation::euclidean_cluster::EuclideanCluster;
using autoware::perception::segmentation::euclidean_cluster::Clusters;
using autoware::perception::segmentation::euclidean_cluster::Cluster;

///
void insert_point(EuclideanCluster & cls, const float x, const float y)
{
  cls.insert(PointXYZI{x, y, 0.0F, 0.0F});
}


///
void insert_ring(
  EuclideanCluster & cls,
  const float r,
  const uint32_t num_pts,
  const float dx = 0.0F,
  const float dy = 0.0F)
{
  const float dth = 2.0F * 3.14159F / num_pts;
  float th = 0.0F;
  for (uint32_t idx = 0U; idx < num_pts; ++idx) {
    const float x = r * cosf(th) + dx;
    const float y = r * sinf(th) + dy;
    insert_point(cls, x, y);
    th += dth;
  }
}
///
void insert_line(
  std::vector<std::pair<float, float>> & output,
  EuclideanCluster & cls,
  const float x1, const float y1,
  const float x2, const float y2,
  const float ds)
{
  const float dx = x2 - x1;
  const float dy = y2 - y1;
  const float len = sqrtf(dx * dx + dy * dy);
  const uint32_t num_pts = static_cast<uint32_t>(std::ceil(fabsf((len / ds))));
  for (uint32_t idx = 0U; idx < num_pts; ++idx) {
    const float x = x1 + (idx * dx / num_pts);
    const float y = y1 + (idx * dy / num_pts);
    insert_point(cls, x, y);
    output.push_back({x, y});
  }
}

///
void insert_mesh(
  std::vector<std::pair<float, float>> & output,
  EuclideanCluster & cls,
  const float x1, const float y1,
  const float x2, const float y2,
  const float dx, const float dy)
{
  const float len_y = y2 - y1;
  const uint32_t num_pts = static_cast<uint32_t>(std::ceil(fabsf(len_y / dy)));
  for (uint32_t idx = 0U; idx < num_pts; ++idx) {
    const float y = y1 + idx * dy;
    insert_line(output, cls, x1, y, x2, y, dx);
  }
}

///
bool check_point(const std::vector<std::pair<float, float>> & expected, const PointXYZI & pt)
{
  bool found = false;
  const float TOL = 0.00001F;
  for (auto & val : expected) {
    if (fabsf(pt.x - val.first) < TOL && fabsf(pt.y - val.second) < TOL) {
      found = true;
      break;
    }
  }
  return found;
}

///
bool check_cluster(const Cluster & cls, const std::vector<std::pair<float, float>> & expected)
{
  bool found = true;

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
  const std::vector<std::vector<std::pair<float, float>> *> & expected, const std::string & frame)
{
  for (uint32_t idx = 0U; idx < cls.clusters.size(); ++idx) {
    bool found = false;
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

#endif  // TEST_EUCLIDEAN_CLUSTER_AUX_H_
