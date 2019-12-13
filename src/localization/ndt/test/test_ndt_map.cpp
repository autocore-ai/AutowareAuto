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
#include <gtest/gtest.h>
#include "test_ndt_map.hpp"
#include <vector>
#include <set>
#include <fstream>

namespace autoware
{
namespace localization
{
namespace ndt
{
constexpr uint32_t MapValidationContext::point_step;
constexpr uint32_t MapValidationContext::num_points;
constexpr uint32_t MapValidationContext::data_size;
constexpr uint32_t MapValidationContext::width;
constexpr uint32_t MapValidationContext::row_step;

TEST_F(MapValidationTest, map_pcl_meta_validation) {
  using PointField = sensor_msgs::msg::PointField;

  // Some invalid fields:
  const auto pf11 = make_pf("x", 4U, PointField::FLOAT64, 1U);
  const auto pf12 = make_pf("x", 8U, PointField::FLOAT64, 2U);
  const auto pf13 = make_pf("xyz", 8U, PointField::FLOAT64, 1U);
  const auto pf14 = make_pf("x", 8U, PointField::FLOAT32, 1U);

  const std::vector<PointField> correct_field_set{pf1, pf2, pf3, pf4, pf5, pf6, pf7, pf8, pf9,
    pf10};

  const std::vector<std::vector<PointField>> incorrect_field_sets{
    {pf1, pf2, pf3, pf4, pf5, pf6, pf7, pf8, pf9, pf10, pf11},  // Extra field
    {pf2, pf1, pf3, pf4, pf5, pf6, pf7, pf8, pf9, pf10},  // Misordered
    {pf14, pf1, pf2, pf3, pf4, pf5, pf6, pf7, pf8, pf9, pf10},  // Extra field in start
    {pf1, pf2, pf14, pf3, pf4, pf5, pf6, pf7, pf8, pf9, pf10},  // Extra field in middle
    {pf14, pf2, pf3, pf4, pf5, pf6, pf7, pf8, pf9, pf10},
    {pf11, pf2, pf3, pf4, pf5, pf6, pf7, pf8, pf9, pf10},
    {pf12, pf2, pf3, pf4, pf5, pf6, pf7, pf8, pf9, pf10},
    {pf13, pf2, pf3, pf4, pf5, pf6, pf7, pf8, pf9, pf10},
    {pf13, pf2, pf3, pf4, pf5, pf6, pf7, pf8, pf9, pf10},
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

TEST_F(MapValidationTest, map_pcl_size_validation) {
  const std::vector<PointField> field_set{pf1, pf2, pf3, pf4, pf5, pf6, pf7, pf8, pf9, pf10};

  EXPECT_EQ(validate_pcl_map(make_pcl(field_set, 1U, data_size, row_step, width,
    point_step)), num_points);

  // Data larger than expected means that the return value will exclude the excess data
  // data.size(), row_step and width * point_step should be consistent, otherwise the
  // minimum of 3 will determine the returned size
  EXPECT_EQ(validate_pcl_map(
      make_pcl(field_set, 1U, data_size + point_step, row_step, width, point_step)), num_points);
  EXPECT_EQ(validate_pcl_map(
      make_pcl(field_set, 1U, data_size, row_step + point_step, width, point_step)), num_points);
  EXPECT_EQ(validate_pcl_map(
      make_pcl(field_set, 1U, data_size, row_step, width + 1U, point_step)), num_points);

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

////////////////////////////////////

TEST(DynamicNDTVoxelTest, ndt_dense_voxel_basic_io) {
  constexpr auto eps = 1e-6;

  DynamicNDTVoxel voxel;

  EXPECT_THROW(voxel.centroid(), std::out_of_range);
  EXPECT_THROW(voxel.covariance(), std::out_of_range);
  EXPECT_EQ(voxel.count(), 0U);
  Eigen::Vector3d point({5, 5, 5});

  voxel.add_observation(point);

  // voxel is considered unoccupied when it has less point than its point threshold
  if (DynamicNDTVoxel::NUM_POINT_THRESHOLD > 1U) {
    EXPECT_THROW(voxel.centroid(), std::out_of_range);
    EXPECT_THROW(voxel.covariance(), std::out_of_range);
  }

  // Add the same point until the voxel has sufficient number of points
  for (uint64_t i = 1U; i < DynamicNDTVoxel::NUM_POINT_THRESHOLD; i++) {
    EXPECT_THROW(voxel.covariance(), std::out_of_range);
    voxel.add_observation(point);
  }

  EXPECT_EQ(voxel.count(), DynamicNDTVoxel::NUM_POINT_THRESHOLD);
  // Centroid should equal to the point as we added the same point multiple times.
  EXPECT_TRUE(point.isApprox(voxel.centroid(), eps));

  // Covariance values are zero since all points are the same
  // and there's no variance.
  EXPECT_NO_THROW(
    EXPECT_LT(voxel.covariance().norm(), eps);
  );
}

///////////////////////////////////

TEST(DynamicNDTVoxelTest, ndt_dense_voxel_basic) {
  constexpr auto eps = 1e-6;
  DynamicNDTVoxel voxel;
  auto num_points = 5U;
  EXPECT_GE(num_points, DynamicNDTVoxel::NUM_POINT_THRESHOLD);

  std::vector<Eigen::Vector3d> points;
  // Add points to the voxel ([0,0,0]... to [4,4,4])
  for (auto i = 0U; i < num_points; i++) {
    auto point =
      Eigen::Vector3d{static_cast<double>(i), static_cast<double>(i), static_cast<double>(i)};
    points.push_back(point);
    voxel.add_observation(point);
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


TEST_F(DenseNDTMapTest, map_lookup) {
  constexpr auto eps = 1e-5;
  // The idea is to have a 5x5x5 grid with cell edge length of 1
  auto grid_config = perception::filters::voxel_grid::Config(m_min_point, m_max_point, m_voxel_size,
      m_capacity);

  DynamicNDTMap ndt_map(grid_config);

  // No map is added, so a lookup should return an empty vector.
  EXPECT_TRUE(ndt_map.cell(0.0, 0.0, 0.0).empty());

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

  for (const auto & voxel_it : ndt_map) {
    Eigen::Vector3d center;
    // Each voxel has 7 points
    EXPECT_EQ(voxel_it.second.count(), 7U);
    EXPECT_NO_THROW(center = voxel_it.second.centroid());
    // Check if the voxel centroid is the same as the intended centroid
    auto voxel_idx = grid_config.index(center);
    EXPECT_TRUE(m_voxel_centers[voxel_idx].isApprox(center, eps));
    // Check if covariance matches the pre-computed value
    EXPECT_NO_THROW(
      EXPECT_TRUE(voxel_it.second.covariance().isApprox(expected_cov, eps));
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
        auto map_idx = grid_config.index(cell[0].centroid());
        EXPECT_EQ(expected_idx, map_idx);
      }
    }
  }
}


///////////////////////////////////////

TEST(StaticNDTVoxelTest, ndt_map_voxel_basics) {
  StaticNDTVoxel vx;
  Eigen::Vector3d pt{5.0, 5.0, 5.0};
  Eigen::Matrix3d cov;
  cov(0, 0) = 7.0; // just making it nonzero

  // default constructor zero-initializes and doesn't set the voxel as occupied
  EXPECT_FALSE(vx.usable());
  EXPECT_THROW(vx.centroid(), std::out_of_range);
  EXPECT_THROW(vx.covariance(), std::out_of_range);

  StaticNDTVoxel vx2{pt, cov};
  EXPECT_TRUE(vx2.usable());
  EXPECT_TRUE(vx2.centroid().isApprox(pt, std::numeric_limits<Real>::epsilon()));
  EXPECT_TRUE(vx2.covariance().isApprox(cov, std::numeric_limits<Real>::epsilon()));

  // Copying is legal as we replace the voxels in the map via copy constructors.
  vx = vx2;
  EXPECT_TRUE(vx.usable());
  EXPECT_TRUE(vx.centroid().isApprox(pt, std::numeric_limits<Real>::epsilon()));
  EXPECT_TRUE(vx.covariance().isApprox(cov, std::numeric_limits<Real>::epsilon()));
}

TEST_F(NDTMapTest, map_representation_bad_input) {
  const std::vector<PointField> field_set{pf1, pf2, pf3, pf4, pf5, pf6, pf7, pf8, pf9, pf10};

  auto grid_config =
    perception::filters::voxel_grid::Config(m_min_point, m_max_point, m_voxel_size, m_capacity);

  StaticNDTMap map_grid(grid_config);

  // Map with invalid field set
  auto invalid_pc1 = make_pcl({pf1, pf2, pf3, pf4, pf5, pf6, pf7, pf8, pf9},
      1U, data_size, row_step, width, point_step);
  // Empty map
  auto invalid_pc2 = make_pcl(field_set, 1U, 0U, 0U, 0U, point_step);
  // Single point map but with invalid cell ID
  auto invalid_pc3 = make_pcl(field_set, 1U, point_step, point_step, point_step, point_step);
  sensor_msgs::PointCloud2Iterator<uint32_t> invalid_cell_it(invalid_pc3, "cell_id");
  const auto actual_idx = grid_config.index(Eigen::Vector3d{});
  *invalid_cell_it = actual_idx + 1U;
  ASSERT_NE(actual_idx, *invalid_cell_it);

  EXPECT_THROW(map_grid.insert(invalid_pc1), std::runtime_error);
  EXPECT_THROW(map_grid.insert(invalid_pc2), std::runtime_error);
  EXPECT_THROW(map_grid.insert(invalid_pc3), std::domain_error);
}

TEST_F(NDTMapTest, map_representation_basics) {

  auto add_pt = [](sensor_msgs::msg::PointCloud2 & pc,
      std::vector<sensor_msgs::PointCloud2Iterator<Real>> & pc_its,
      Real value) {
      for (auto & it : pc_its) {
        *it = value;
        ++it;
      }
    };

  const std::vector<PointField> field_set{pf1, pf2, pf3, pf4, pf5, pf6, pf7, pf8, pf9, pf10};
  auto msg = make_pcl(field_set, 1U, data_size, row_step, width, point_step);

  auto grid_config =
    perception::filters::voxel_grid::Config(m_min_point, m_max_point, m_voxel_size, m_capacity);

  // This grid will be used to generate
  DynamicNDTMap generator_grid(grid_config);
  StaticNDTMap map_grid(grid_config);

  // No map is added, so a lookup should return an empty vector.
  EXPECT_TRUE(map_grid.cell(0.0, 0.0, 0.0).empty());

  std::vector<sensor_msgs::PointCloud2Iterator<Real>> pc_its{
    sensor_msgs::PointCloud2Iterator<Real>(msg, "x"),
    sensor_msgs::PointCloud2Iterator<Real>(msg, "y"),
    sensor_msgs::PointCloud2Iterator<Real>(msg, "z"),
    sensor_msgs::PointCloud2Iterator<Real>(msg, "cov_xx"),
    sensor_msgs::PointCloud2Iterator<Real>(msg, "cov_xy"),
    sensor_msgs::PointCloud2Iterator<Real>(msg, "cov_xz"),
    sensor_msgs::PointCloud2Iterator<Real>(msg, "cov_yy"),
    sensor_msgs::PointCloud2Iterator<Real>(msg, "cov_yz"),
    sensor_msgs::PointCloud2Iterator<Real>(msg, "cov_zz")
  };
  sensor_msgs::PointCloud2Iterator<uint32_t> cell_id_it(msg, "cell_id");

  auto value = Real{1.0};
  while (std::all_of(pc_its.begin(), pc_its.end(), [](auto & it) {return it != it.end();}) &&
    cell_id_it != cell_id_it.end())
  {

    const Eigen::Vector3d added_pt{value, value, value};
    // Turn the point into a pointcloud to insert. The point should be inserted enough to make
    // the voxel usable. (equal to NUM_POINT_THRESHOLD)
    generator_grid.insert(make_pcl(std::vector<Eigen::Vector3d>{
            DynamicNDTVoxel::NUM_POINT_THRESHOLD, added_pt}));

    // For simplicity, the inserted points already correspond to the centroids and there's point per voxel.
    const auto & generating_voxel = generator_grid.cell(added_pt)[0U];
    ASSERT_TRUE(generating_voxel.centroid().isApprox(added_pt,
      std::numeric_limits<Real>::epsilon()));
    ASSERT_EQ(generating_voxel.count(), DynamicNDTVoxel::NUM_POINT_THRESHOLD);

    add_pt(msg, pc_its, value);
    *cell_id_it = static_cast<uint32_t>(grid_config.index(added_pt));
    ++cell_id_it;

    ++value;
  }

  // Each point should correspond to a single voxel for this test case
  ASSERT_EQ(generator_grid.size(), num_points);

  // All points should be able to be inserted since
  EXPECT_NO_THROW(map_grid.insert(msg));
  EXPECT_EQ(map_grid.size(), generator_grid.size());

  // dif to be used for grid lookup.
  auto diff = 0.1;
  // Ensure, when added, the diff doesn't drift a centroid to another voxel.
  ASSERT_LT(diff, grid_config.get_voxel_size().x);

  // Check if every voxel in the generator grid is passed to the map representation.
  for (auto & vx : generator_grid) {
    EXPECT_NE(std::find_if(map_grid.begin(), map_grid.end(), [&vx](const auto & map_elem) {
        return map_elem.first == vx.first;
      }), map_grid.end());

    auto pt = vx.second.centroid();
    // slightly move the point before looking up to check if it gets the correct voxel.
    pt(0) += diff;
    EXPECT_TRUE(map_grid.cell(pt(0), pt(1), pt(2))[0].centroid().isApprox(vx.second.centroid(),
      std::numeric_limits<Real>::epsilon()));
  }

  map_grid.clear();
  EXPECT_EQ(map_grid.size(), 0U);
}


}  // namespace ndt
}  // namespace localization
}  // namespace autoware
