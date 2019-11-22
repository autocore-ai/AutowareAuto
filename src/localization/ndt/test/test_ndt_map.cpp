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

#include <gtest/gtest.h>
#include "test_ndt_map.hpp"
#include <vector>
#include <set>
namespace autoware
{
namespace localization
{
namespace ndt
{
constexpr uint32_t TestMapValid::point_step;
constexpr uint32_t TestMapValid::num_points;
constexpr uint32_t TestMapValid::data_size;
constexpr uint32_t TestMapValid::width;
constexpr uint32_t TestMapValid::row_step;

TEST_F(TestMapValid, map_pcl_meta_validation) {
  using PointField = sensor_msgs::msg::PointField;

  // Some invalid fields:
  const auto pf10 = make_pf("x", 8U, PointField::FLOAT32, 1U);
  const auto pf11 = make_pf("x", 4U, PointField::FLOAT64, 1U);
  const auto pf12 = make_pf("x", 8U, PointField::FLOAT64, 2U);
  const auto pf13 = make_pf("xyz", 8U, PointField::FLOAT64, 1U);

  const std::vector<PointField> correct_field_set{pf1, pf2, pf3, pf4, pf5, pf6, pf7, pf8, pf9};

  const std::vector<std::vector<PointField>> incorrect_field_sets{
    {pf1, pf2, pf3, pf4, pf5, pf6, pf7, pf8, pf9, pf10},  // Extra field
    {pf2, pf1, pf3, pf4, pf5, pf6, pf7, pf8, pf9},  // Misordered
    {pf10, pf1, pf2, pf3, pf4, pf5, pf6, pf7, pf8, pf9},  // Extra field in start
    {pf1, pf2, pf10, pf3, pf4, pf5, pf6, pf7, pf8, pf9},  // Extra field in middle
    {pf10, pf2, pf3, pf4, pf5, pf6, pf7, pf8, pf9},
    {pf11, pf2, pf3, pf4, pf5, pf6, pf7, pf8, pf9},
    {pf12, pf2, pf3, pf4, pf5, pf6, pf7, pf8, pf9},
    {pf13, pf2, pf3, pf4, pf5, pf6, pf7, pf8, pf9},
    {pf13, pf2, pf3, pf4, pf5, pf6, pf7, pf8, pf9},
    {pf1, pf2, pf3, pf4, pf5, pf6, pf7, pf8},  // missing field
    {}
  };

  auto correct_pcl = make_pcl(correct_field_set, 1U, data_size, row_step, width, point_step);

  EXPECT_GT(validate_pcl_map(correct_pcl), 0U);

  for (const auto & incorrect_field_set : incorrect_field_sets) {
    auto incorrect_pcl = make_pcl(incorrect_field_set, 1U, data_size, row_step, width, point_step);
    EXPECT_EQ(validate_pcl_map(incorrect_pcl), 0U);
  }

  // Check with invalid height
  EXPECT_EQ(validate_pcl_map(make_pcl(correct_field_set, 0U, data_size, row_step, width,
    point_step)), 0U);
  // Check with invalid point step
  EXPECT_EQ(validate_pcl_map(make_pcl(correct_field_set, 1U, data_size, row_step, width,
    point_step - 1U)), 0U);
}

TEST_F(TestMapValid, map_pcl_size_validation) {
  const std::vector<PointField> field_set{pf1, pf2, pf3, pf4, pf5, pf6, pf7, pf8, pf9};

  EXPECT_EQ(validate_pcl_map(make_pcl(field_set, 1U, data_size, row_step, width,
    point_step)), num_points);

  // Data larger than expected means that the return value will exclude the excess data
  // data.size(), row_step and width should be consistent, otherwise the minimum of 3 will
  // determine the returned size
  EXPECT_EQ(validate_pcl_map(
      make_pcl(field_set, 1U, data_size + point_step, row_step, width, point_step)), num_points);
  EXPECT_EQ(validate_pcl_map(
      make_pcl(field_set, 1U, data_size, row_step + point_step, width, point_step)), num_points);
  EXPECT_EQ(validate_pcl_map(
      make_pcl(field_set, 1U, data_size, row_step, width + point_step, point_step)), num_points);

  // Even if the metadata and the data size is consistent, if the size is not divisible to
  // point_step, the excess data will be ignored.
  ASSERT_NE((data_size + 1U) % point_step, 0U);  // Required for the test to work.
  EXPECT_EQ(validate_pcl_map(
      make_pcl(field_set, 1U, data_size + 1U, row_step + 1U, width + 1U, point_step)), num_points);

  // Data shorter than expected: return the greatest number of points that can be read
  // given the point_step
  EXPECT_EQ(validate_pcl_map(make_pcl(field_set, 1U, data_size - 1U, row_step, width,
    point_step)), num_points - 1U);
  EXPECT_EQ(validate_pcl_map(make_pcl(field_set, 1U, data_size, row_step - 1U, width,
    point_step)), num_points - 1U);
  EXPECT_EQ(validate_pcl_map(make_pcl(field_set, 1U, data_size, row_step, width - 1U,
    point_step)), num_points - 1U);
  EXPECT_EQ(validate_pcl_map(
      make_pcl(field_set, 1U, data_size - 2 * point_step, row_step, width,
      point_step)), num_points - 2U);
  EXPECT_EQ(validate_pcl_map(
      make_pcl(field_set, 1U, data_size - 2 * point_step - 1U, row_step, width,
      point_step)), num_points - 3U);
}

TEST(NDTVoxelTest, ndt_voxel_basic_io) {
  constexpr auto eps = 1e-6;

  NDTVoxel voxel;

  EXPECT_THROW(voxel.centroid(), std::out_of_range);
// Empty state corresponds to multiple failure states for the covariance function so `any_throw`
// is used to reflect that.
  EXPECT_ANY_THROW(voxel.covariance());
  EXPECT_EQ(voxel.count(), 0U);
  Eigen::Vector3d point({5, 5, 5});

  voxel.add_observation(point);

// voxel is considered unoccupied when it has less point than its point threshold
  if (NDTVoxel::NUM_POINT_THRESHOLD > 1U) {
    EXPECT_THROW(voxel.centroid(), std::out_of_range);
    EXPECT_ANY_THROW(voxel.covariance());
  }

// Add the same point until the voxel has sufficient number of points
  for (uint64_t i = 1U; i < NDTVoxel::NUM_POINT_THRESHOLD; i++) {
    voxel.add_observation(point);
    EXPECT_ANY_THROW(voxel.covariance());
  }

  EXPECT_EQ(voxel.count(), NDTVoxel::NUM_POINT_THRESHOLD);
// Centroid should equal to the point as we added the same point multiple times.
  EXPECT_TRUE(point.isApprox(voxel.centroid(), eps));
// Covariance cannot be computed until all points in the voxel are fed back to the voxel
// for covariance update
  EXPECT_THROW(voxel.covariance(), std::length_error);

// Update the covariance using all the points in the voxel but one.
  for (uint64_t i = 0U; i < NDTVoxel::NUM_POINT_THRESHOLD - 1; i++) {
    voxel.add_point_for_covariance(point);
  }

// covariance can't be accessed because one point is not included in the covariance yet.
  EXPECT_FALSE(voxel.cov_computed());
  EXPECT_THROW(voxel.covariance(), std::length_error);

// add the last point to the covariance calculation
  voxel.add_point_for_covariance(point);

// Covariance can now be computed. Its values are zero since all points are the same
// and there's no variance.
  EXPECT_TRUE(voxel.cov_computed());
  EXPECT_NO_THROW(
    EXPECT_LT(voxel.covariance().norm(), eps);
  );

// adding another point after the covariance computation is done which resets the covariance
// computation.
  voxel.add_observation(point);
  voxel.add_point_for_covariance(point);
// Covariance computation needs all the other points to compute the covariance from scratch
// since the computation is not incremental
  EXPECT_FALSE(voxel.cov_computed());
  EXPECT_THROW(voxel.covariance(), std::length_error);
}

///////////////////////////////////

TEST(NDTVoxelTest, ndt_voxel_basic) {
  constexpr auto eps = 1e-6;
  NDTVoxel voxel;
  auto num_points = 5U;
  EXPECT_GE(num_points, NDTVoxel::NUM_POINT_THRESHOLD);

  std::vector<Eigen::Vector3d> points;
  // Add points to the voxel ([0,0,0]... to [4,4,4])
  for (auto i = 0U; i < num_points; i++) {
    auto point =
      Eigen::Vector3d{static_cast<double>(i), static_cast<double>(i), static_cast<double>(i)};
    points.push_back(point);
    voxel.add_observation(point);
  }

  // Update voxel covariance with all the points
  for (const auto & point : points) {
    voxel.add_point_for_covariance(point);
  }

  // validate the mean in numpy
  // np.mean(np.array([[0,1,2,3,4],[0,1,2,3,4],[0,1,2,3,4]]),1)
  Eigen::Vector3d expected_centroid{2.0, 2.0, 2.0};
  // Validate covariance in numpy: np.cov(np.array([[0,1,2,3,4],[0,1,2,3,4],[0,1,2,3,4]]))
  Eigen::Matrix3d expected_covariance;
  expected_covariance.setConstant(2.5);

  EXPECT_NO_THROW(
    EXPECT_TRUE(voxel.centroid().isApprox(expected_centroid, eps));
  );
  EXPECT_NO_THROW(
    EXPECT_TRUE(voxel.covariance().isApprox(expected_covariance, eps));
  );
}


TEST_F(NDTMapTest, map_lookup) {
  constexpr auto eps = 1e-5;
  // The idea is to have a 5x5x5 grid with cell edge length of 1
  auto grid_config = perception::filters::voxel_grid::Config(m_min_point, m_max_point, m_voxel_size,
      m_capacity);

  NDTVoxelMap ndt_map(grid_config);

  EXPECT_EQ(ndt_map.cell(1, 1, 1).count(), 0U);

  // build a pointcloud map.
  // It contains 5*5*5*7 points where each cell would have a center (ranging from (1,1,1) to (5,5,5))
  // and 6 surrounding points with a 0.3 distance from the center
  build_pc(grid_config);

  // The center points are added to a map with their voxel indices for easy lookup
  EXPECT_EQ(m_voxel_centers.size(), 125U);

  // Insert the pointcloud into the ndt map
  EXPECT_NO_THROW(ndt_map.insert(m_pc));
  // ndt map has 125 voxels now: a 5x5x5 grid
  EXPECT_EQ(ndt_map.size(), 125U);

  // Al cells have the same variance. The value can be validated via numpy:
  // >>> dev = 0.3
  // >>> np.cov(np.array([ [1, 1+dev, 1-dev,1,1,1,1], [1,1,1,1+dev,1-dev,1,1], [1,1,1,1,1,1+dev,
  // 1-dev]  ]))
  Eigen::Matrix3d expected_cov;
  expected_cov << 0.03, 0.0, 0.0,
    0.0, 0.03, 0.0,
    0.0, 0.0, 0.03;

  for (auto & voxel_it : ndt_map) {
    Eigen::Vector3d center;
    // Each voxel has 7 points
    EXPECT_EQ(voxel_it.second.count(), 7U);
    EXPECT_NO_THROW(center = voxel_it.second.centroid());
    // Check if the voxel centroid is the same as the intended centroid
    auto voxel_idx = grid_config.index(center);
    EXPECT_TRUE(m_voxel_centers[voxel_idx].isApprox(center, eps));
    // Check if covariance matches the pre-computed value
    EXPECT_NO_THROW(
      EXPECT_TRUE(voxel_it.second.covariance().isApprox(expected_cov, eps))
    );
  }

  // Iterate the grid and do lookups:
  for (auto x = 1; x <= POINTS_PER_DIM; x++) {
    for (auto y = 1; y <= POINTS_PER_DIM; y++) {
      for (auto z = 1; z <= POINTS_PER_DIM; z++) {
        // Query the idx of the expected centroid via the config object:
        auto expected_idx = grid_config.index(Eigen::Vector3d(x, y, z));
        // Get the cell index ndt map estimated:
        auto cell = ndt_map.cell(x, y, z);
        auto map_idx = grid_config.index(cell.centroid());
        EXPECT_EQ(expected_idx, map_idx);
      }
    }
  }
}


}  // namespace ndt
}  // namespace localization
}  // namespace autoware
