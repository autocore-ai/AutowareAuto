// Copyright 2017-2019 Apex.AI, Inc.
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

#ifndef TEST_VOXEL_GRID_NODES_HPP_
#define TEST_VOXEL_GRID_NODES_HPP_

#include <voxel_grid_nodes/algorithm/voxel_cloud_approximate.hpp>
#include <voxel_grid_nodes/algorithm/voxel_cloud_centroid.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <memory>

using autoware::perception::filters::voxel_grid::PointXYZ;
using autoware::perception::filters::voxel_grid::Config;

using autoware::perception::filters::voxel_grid_nodes::algorithm::VoxelCloudBase;
using autoware::perception::filters::voxel_grid_nodes::algorithm::VoxelCloudApproximate;
using autoware::perception::filters::voxel_grid_nodes::algorithm::VoxelCloudCentroid;
using autoware::perception::filters::voxel_grid::PointXYZIF;

class VoxelAlgorithm : public ::testing::Test
{
protected:
  PointXYZIF make(const float x, const float y, const float z)
  {
    PointXYZIF ret;
    ret.x = x;
    ret.y = y;
    ret.z = z;
    return ret;
  }
  std::unique_ptr<Config> cfg_ptr;
  std::array<PointXYZIF, 16U> obs_points1;
  std::array<PointXYZIF, 8U> ref_points1;

public:
  VoxelAlgorithm()
  {
    // Initialize config
    PointXYZ min_point;
    min_point.x = -1.0F;
    min_point.y = -1.0F;
    min_point.z = -1.0F;
    PointXYZ max_point;
    max_point.x = 1.0F;
    max_point.y = 1.0F;
    max_point.z = 1.0F;
    PointXYZ voxel_size;
    voxel_size.x = 1.0F;
    voxel_size.y = 1.0F;
    voxel_size.z = 1.0F;
    const std::size_t capacity = 10U;
    cfg_ptr = std::make_unique<Config>(min_point, max_point, voxel_size, capacity);
    // List of points
    obs_points1 = {
      make(-1.0F, -1.0F, -1.0F),  // voxel 0
      make(-0.5F, -0.5F, -0.5F),
      make(1.0F, -1.0F, -1.0F),  // voxel 1
      make(0.5F, -0.5F, -0.5F),
      make(-1.0F, 1.0F, -1.0F),  // voxel 2
      make(-0.5F, 0.5F, -0.5F),
      make(1.0F, 1.0F, -1.0F),  // voxel 3
      make(0.5F, 0.5F, -0.5F),
      make(-1.0F, -1.0F, 1.0F),  // voxel 4
      make(-0.5F, -0.5F, 0.5F),
      make(1.0F, -1.0F, 1.0F),  // voxel 5
      make(0.5F, -0.5F, 0.5F),
      make(-1.0F, 1.0F, 1.0F),  // voxel 6
      make(-0.5F, 0.5F, 0.5F),
      make(1.0F, 1.0F, 1.0F),  // voxel 7
      make(0.5F, 0.5F, 0.5F)
    };
  }
};
////////////////////////////////////////////////////////////////////////////////
class CloudAlgorithm : public VoxelAlgorithm
{
protected:
  using VoxelAlgorithm::make;
  void make(sensor_msgs::msg::PointCloud2 & cloud, std::size_t N)
  {
    sensor_msgs::PointCloud2Modifier mod{cloud};
    mod.resize(N);
    mod.setPointCloud2Fields(4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    sensor_msgs::PointCloud2Iterator<float> it_x{cloud, "x"};
    sensor_msgs::PointCloud2Iterator<float> it_y{cloud, "y"};
    sensor_msgs::PointCloud2Iterator<float> it_z{cloud, "z"};
    for (std::size_t idx = 0U; idx < N; ++idx) {
      *it_x = obs_points1[idx].x;
      *it_y = obs_points1[idx].y;
      *it_z = obs_points1[idx].z;
      ++it_x;
      ++it_y;
      ++it_z;
    }
  }
  bool check(sensor_msgs::msg::PointCloud2 cloud, std::size_t N)
  {
    bool ret = true;
    constexpr float TOL = 1.0E-6F;
    sensor_msgs::PointCloud2Iterator<float> it_x{cloud, "x"};
    sensor_msgs::PointCloud2Iterator<float> it_y{cloud, "y"};
    sensor_msgs::PointCloud2Iterator<float> it_z{cloud, "z"};
    for (std::size_t idx = 0U; idx < cloud.width; ++idx) {
      bool found = false;
      for (std::size_t jdx = 0U; jdx < N; ++jdx) {
        const auto & ref = ref_points1[jdx];
        if ((fabsf(*it_x - ref.x) < TOL) &&
          (fabsf(*it_y - ref.y) < TOL) &&
          (fabsf(*it_z - ref.z) < TOL))
        {
          found = true;
          break;
        }
      }
      if (!found) {
        ret = false;
        break;
      }
      ++it_x;
      ++it_y;
      ++it_z;
    }
    return ret;
  }
  sensor_msgs::msg::PointCloud2 cloud1;
  sensor_msgs::msg::PointCloud2 cloud2;
  std::unique_ptr<VoxelCloudBase> alg_ptr;

public:
  CloudAlgorithm()
  {
    // Make clouds
    make(cloud1, 8U);
    make(cloud2, obs_points1.size());
  }
};

TEST_F(CloudAlgorithm, approximate)
{
  this->ref_points1[0U] = this->make(-0.5F, -0.5F, -0.5F);
  this->ref_points1[1U] = this->make(0.5F, -0.5F, -0.5F);
  this->ref_points1[2U] = this->make(-0.5F, 0.5F, -0.5F);
  this->ref_points1[3U] = this->make(0.5F, 0.5F, -0.5F);
  this->ref_points1[4U] = this->make(-0.5F, -0.5F, 0.5F);
  this->ref_points1[5U] = this->make(0.5F, -0.5F, 0.5F);
  this->ref_points1[6U] = this->make(-0.5F, 0.5F, 0.5F);
  this->ref_points1[7U] = this->make(0.5F, 0.5F, 0.5F);
  // initialize
  alg_ptr = std::make_unique<VoxelCloudApproximate>(*cfg_ptr);
  // check empty
  EXPECT_EQ(alg_ptr->get().width, 0U);
  // add points
  alg_ptr->insert(cloud1);
  // get
  EXPECT_TRUE(check(alg_ptr->get(), 4U));
  // check empty
  EXPECT_EQ(alg_ptr->get().width, 0U);
  // add more points
  alg_ptr->insert(cloud1);
  alg_ptr->insert(cloud2);
  // get again
  EXPECT_TRUE(check(alg_ptr->get(), ref_points1.size()));
  // check empty
  EXPECT_EQ(alg_ptr->get().width, 0U);
}

TEST_F(CloudAlgorithm, centroid)
{
  this->ref_points1[0U] = this->make(-0.75F, -0.75F, -0.75F);
  this->ref_points1[1U] = this->make(0.75F, -0.75F, -0.75F);
  this->ref_points1[2U] = this->make(-0.75F, 0.75F, -0.75F);
  this->ref_points1[3U] = this->make(0.75F, 0.75F, -0.75F);
  this->ref_points1[4U] = this->make(-0.75F, -0.75F, 0.75F);
  this->ref_points1[5U] = this->make(0.75F, -0.75F, 0.75F);
  this->ref_points1[6U] = this->make(-0.75F, 0.75F, 0.75F);
  this->ref_points1[7U] = this->make(0.75F, 0.75F, 0.75F);
  // initialize
  alg_ptr = std::make_unique<VoxelCloudCentroid>(*cfg_ptr);
  // check empty
  EXPECT_EQ(alg_ptr->get().width, 0U);
  // add points
  alg_ptr->insert(cloud1);
  // get
  EXPECT_TRUE(check(alg_ptr->get(), 4U));
  // check empty
  EXPECT_EQ(alg_ptr->get().width, 0U);
  // add more points
  alg_ptr->insert(cloud1);
  alg_ptr->insert(cloud2);
  // get again
  EXPECT_TRUE(check(alg_ptr->get(), ref_points1.size()));
  // check empty
  EXPECT_EQ(alg_ptr->get().width, 0U);
}

#endif  // TEST_VOXEL_GRID_NODES_HPP_
