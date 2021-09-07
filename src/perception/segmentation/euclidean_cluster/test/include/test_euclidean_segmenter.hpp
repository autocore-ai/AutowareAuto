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

#ifndef TEST_EUCLIDEAN_SEGMENTER_HPP_
#define TEST_EUCLIDEAN_SEGMENTER_HPP_

#include <geometry/bounding_box_2d.hpp>
#include <common/types.hpp>
#include <test_euclidean_cluster_aux.hpp>

#include <vector>
#include <utility>

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

void check_box(
  const std::vector<autoware_auto_msgs::msg::BoundingBox> & expect,
  const autoware_auto_msgs::msg::BoundingBox & box,
  const float32_t TOL = 5.0E-1F)
{
  bool8_t found = false;
  for (auto & ebox : expect) {
    bool8_t close = true;
    close &= fabsf(box.centroid.x - ebox.centroid.x) < TOL;
    close &= fabsf(box.centroid.y - ebox.centroid.y) < TOL;
    close &= fabsf(box.size.x - ebox.size.x) < TOL;
    close &= fabsf(box.size.y - ebox.size.y) < TOL;
    // test corners
    for (uint32_t idx = 0U; idx < 4U; ++idx) {
      bool8_t found = false;
      for (uint32_t jdx = 0U; jdx < 4U; ++jdx) {
        if (fabsf(box.corners[idx].x - ebox.corners[jdx].x) < TOL &&
          fabsf(box.corners[idx].y - ebox.corners[jdx].y) < TOL)
        {
          found = true;
          break;
        }
      }
      close &= found;
    }
    // TODO(c.ho)
    // close &= fabsf(box.orientation.x = 1.0F;
    if (close) {
      found = true;
      break;
    }
  }
  if (!found) {
    std::cerr << "Box fail\n";
    std::cout << box.centroid.x << ", " << box.centroid.y << "\n";
    std::cout << box.size.x << ", " << box.size.y << "\n";
    std::cout << box.corners[0].x << ", " << box.corners[0].y << "\n";
    std::cout << box.corners[1].x << ", " << box.corners[1].y << "\n";
    std::cout << box.corners[2].x << ", " << box.corners[2].y << "\n";
    std::cout << box.corners[3].x << ", " << box.corners[3].y << "\n";
  }
  EXPECT_TRUE(found);
}

autoware_auto_msgs::msg::BoundingBox compute_box(Cluster & cls)
{
  EXPECT_EQ(sizeof(PointXYZI), cls.point_step);
  const auto begin = reinterpret_cast<PointXYZI *>(&cls.data[0U]);
  const auto end = reinterpret_cast<PointXYZI *>(&cls.data[cls.row_step]);
  auto q = begin;
  std::vector<PointXYZI> v;
  // Ensure that memcpy is the same as reinterpret cast
  for (uint32_t idx = 0U; idx < cls.width; ++idx) {
    // memcpy
    PointXYZI p;
    (void)memcpy(&p, &(cls.data[idx * cls.point_step]), cls.point_step);
    // check
    EXPECT_FLOAT_EQ(p.x, q->x);
    EXPECT_FLOAT_EQ(p.y, q->y);
    EXPECT_FLOAT_EQ(p.z, q->z);
    EXPECT_FLOAT_EQ(p.intensity, q->intensity);
    // increment
    ++q;
    v.push_back(p);
  }
  EXPECT_EQ(q, end);
  EXPECT_EQ(std::distance(begin, end), cls.width);
  EXPECT_EQ(cls.width, v.size());
  // Check that using raw pointers is the same as doing the memcpy aliasing
  const auto box_v =
    autoware::common::geometry::bounding_box::lfit_bounding_box_2d(v.begin(), v.end());
  const auto box_p =
    autoware::common::geometry::bounding_box::lfit_bounding_box_2d(begin, end);
  EXPECT_FLOAT_EQ(box_v.centroid.x, box_p.centroid.x);
  EXPECT_FLOAT_EQ(box_v.centroid.y, box_p.centroid.y);
  EXPECT_FLOAT_EQ(box_v.size.x, box_p.size.x);
  EXPECT_FLOAT_EQ(box_v.size.y, box_p.size.y);
  EXPECT_FLOAT_EQ(box_v.orientation.w, box_p.orientation.w);
  EXPECT_FLOAT_EQ(box_v.orientation.z, box_p.orientation.z);
  for (uint32_t idx = 0U; idx < 4U; ++idx) {
    EXPECT_FLOAT_EQ(box_v.corners[idx].x, box_p.corners[idx].x);
    EXPECT_FLOAT_EQ(box_v.corners[idx].y, box_p.corners[idx].y);
  }
  return box_p;
}

///////////////////////////////////////////////////////////////////////////////////////
// test that for a big point cloud, you can subdivide into distinct clusters, and turn into bounding
// boxes
// This tests the "cluster object lives in another scope" use case and direct bounding box formation
TEST(EuclideanSegmenter, Combined)
{
  Config cfg{"razzledazz", 10U, 100U, 1.5, 3.5, 60.0F};
  HashConfig hcfg{-130.0F, 130.0F, -130.0F, 130.0F, 1.0F, 10000U};
  EuclideanCluster cls{cfg, hcfg};
  Clusters res;
  res.clusters.reserve(cfg.max_num_clusters());
  std::vector<autoware_auto_msgs::msg::BoundingBox> expect;
  autoware_auto_msgs::msg::BoundingBox box;
  std::vector<std::pair<float32_t, float32_t>> dummy;
  const auto make = [](const float32_t x, const float32_t y) -> geometry_msgs::msg::Point32
    {
      geometry_msgs::msg::Point32 p;
      p.x = x;
      p.y = y;
      return p;
    };

  const auto box_fuzz = [](
    EuclideanCluster & cls,
    std::vector<std::pair<float32_t, float32_t>> & dummy,
    const autoware_auto_msgs::msg::BoundingBox & box) -> void
    {
      insert_point(cls, box.corners[0U].x, box.corners[0U].y);
      insert_point(cls, box.corners[1U].x, box.corners[1U].y);
      insert_point(cls, box.corners[2U].x, box.corners[2U].y);
      insert_point(cls, box.corners[3U].x, box.corners[3U].y);
      insert_line(
        dummy, cls, box.corners[0U].x, box.corners[0U].y,
        box.corners[1U].x, box.corners[1U].y, 0.1F);
      insert_line(
        dummy, cls, box.corners[0U].x, box.corners[0U].y,
        box.corners[3U].x, box.corners[3U].y, 0.1F);
    };

  // box 1
  box.centroid = make(15.0, 15.0);
  box.size = make(6.0, 6.0);
  box.orientation.x = 1.0F;
  box.corners = {make(12.0, 12.0), make(18, 12), make(18, 18), make(12, 18)};
  expect.push_back(box);
  box_fuzz(cls, dummy, box);

  // box 2
  box.centroid = make(-15.0, 15.0);
  box.size = make(6.0, 6.0);
  box.orientation.x = 1.0F;
  box.corners = {make(-12.0, 12.0), make(-18, 12), make(-18, 18), make(-12, 18)};
  expect.push_back(box);
  box_fuzz(cls, dummy, box);

  // box 3
  box.centroid = make(-15.0, -15.0);
  box.size = make(6.0, 6.0);
  box.orientation.x = 1.0F;
  box.corners = {make(-12.0, -12.0), make(-18, -12), make(-18, -18), make(-12, -18)};
  expect.push_back(box);
  box_fuzz(cls, dummy, box);

  // box 4
  box.centroid = make(15.0, -15.0);
  box.size = make(6.0, 6.0);
  box.orientation.x = 1.0F;
  box.corners = {make(12.0, -12.0), make(18, -12), make(18, -18), make(12, -18)};
  expect.push_back(box);
  box_fuzz(cls, dummy, box);

  // box 5
  box.centroid = make(0, 0);
  box.size = make(6.0, 6.0);
  box.orientation.x = 1.0F;
  box.corners = {make(3, -3), make(-3, -3), make(-3, 3), make(3, 3)};
  expect.push_back(box);
  box_fuzz(cls, dummy, box);

  // cluster
  cls.cluster(res);
  EXPECT_EQ(res.clusters.size(), 5U);
  box = compute_box(res.clusters[0]);
  check_box(expect, box, 0.1F);

  box = compute_box(res.clusters[1]);
  check_box(expect, box, 0.1F);

  box = compute_box(res.clusters[2]);
  check_box(expect, box, 0.1F);

  box = compute_box(res.clusters[3]);
  check_box(expect, box, 0.1F);

  box = compute_box(res.clusters[4]);
  check_box(expect, box, 0.1F);

  for (auto & cluster : res.clusters) {
    cluster.header.frame_id = "foo";
  }
  EXPECT_NO_THROW(cls.cleanup(res));
}
#endif  // TEST_EUCLIDEAN_SEGMENTER_HPP_
