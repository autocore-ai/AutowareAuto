// Copyright 2021 Apex.AI, Inc.
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

/// \copyright Copyright 2021 Apex.AI, Inc.
/// All rights reserved.


#include <measurement_conversion/eigen_utils.hpp>

#include <common/types.hpp>

#include <gtest/gtest.h>

using autoware::common::state_estimation::downscale_isometry;
using autoware::common::state_estimation::array_to_matrix;
using autoware::common::state_estimation::slice;
using autoware::common::state_estimation::DataStorageOrder;
using autoware::common::types::float64_t;

/// \test Check that we can downscale a trivial isometry.
TEST(MeasurementConversion, TrivialIsometryDownscaling) {
  const auto isometry = downscale_isometry<2>(Eigen::Isometry3f::Identity());
  EXPECT_TRUE(isometry.isApprox(Eigen::Isometry2f::Identity()));
}

/// \test Check that we can downscale an isometry.
TEST(MeasurementConversion, IsometryDownscaling) {
  Eigen::Isometry3f initial_isometry;
  initial_isometry.linear() =
    Eigen::AngleAxisf{autoware::common::types::PI, Eigen::Vector3f::UnitZ()} *Eigen::Scaling(1.0F);
  const auto isometry = downscale_isometry<2>(initial_isometry);
  Eigen::Matrix2f expected_rotation = Eigen::Rotation2Df(autoware::common::types::PI) *
    Eigen::Scaling(1.0F);
  EXPECT_TRUE(isometry.linear().isApprox(expected_rotation));
}

/// \test Check that an array can be represented as a matrix.
TEST(MeasurementConversion, WholeArrayToMatrix) {
  std::array<float64_t, 4UL> data{0, 1, 2, 3};
  const auto mat_as_row_major = array_to_matrix<2, 2>(data, 0, 2, DataStorageOrder::kRowMajor);
  EXPECT_DOUBLE_EQ(mat_as_row_major(0, 0), data[0]);
  EXPECT_DOUBLE_EQ(mat_as_row_major(0, 1), data[1]);
  EXPECT_DOUBLE_EQ(mat_as_row_major(1, 0), data[2]);
  EXPECT_DOUBLE_EQ(mat_as_row_major(1, 1), data[3]);
  const auto mat_as_col_major = array_to_matrix<2, 2>(data, 0, 2, DataStorageOrder::kColumnMajor);
  EXPECT_DOUBLE_EQ(mat_as_col_major(0, 0), data[0]);
  EXPECT_DOUBLE_EQ(mat_as_col_major(1, 0), data[1]);
  EXPECT_DOUBLE_EQ(mat_as_col_major(0, 1), data[2]);
  EXPECT_DOUBLE_EQ(mat_as_col_major(1, 1), data[3]);
}

/// \test Check that an array can be represented as a matrix.
TEST(MeasurementConversion, PartialArrayToMatrix) {
  std::array<float64_t, 9UL> data{0, 1, 2, 3, 4, 5, 6, 7, 8};
  const auto stride = 3;
  const auto start_index = 1;
  const auto mat_as_row_major =
    array_to_matrix<2, 2>(data, start_index, stride, DataStorageOrder::kRowMajor);
  EXPECT_DOUBLE_EQ(mat_as_row_major(0, 0), data[1]);
  EXPECT_DOUBLE_EQ(mat_as_row_major(0, 1), data[2]);
  EXPECT_DOUBLE_EQ(mat_as_row_major(1, 0), data[4]);
  EXPECT_DOUBLE_EQ(mat_as_row_major(1, 1), data[5]);
  const auto mat_as_col_major =
    array_to_matrix<2, 2>(data, start_index, stride, DataStorageOrder::kColumnMajor);
  EXPECT_DOUBLE_EQ(mat_as_col_major(0, 0), data[1]);
  EXPECT_DOUBLE_EQ(mat_as_col_major(1, 0), data[2]);
  EXPECT_DOUBLE_EQ(mat_as_col_major(0, 1), data[4]);
  EXPECT_DOUBLE_EQ(mat_as_col_major(1, 1), data[5]);
  const auto too_big_start = 6;
  EXPECT_THROW(
    (array_to_matrix<2, 2>(data, too_big_start, stride, DataStorageOrder::kColumnMajor)),
    std::runtime_error);
  EXPECT_THROW(
    (array_to_matrix<2, 2>(data, too_big_start, stride, DataStorageOrder::kRowMajor)),
    std::runtime_error);
  const auto too_big_stride = 10;
  EXPECT_THROW(
    (array_to_matrix<2, 2>(data, start_index, too_big_stride, DataStorageOrder::kColumnMajor)),
    std::runtime_error);
  EXPECT_THROW(
    (array_to_matrix<2, 2>(data, start_index, too_big_stride, DataStorageOrder::kRowMajor)),
    std::runtime_error);
}

/// \test Check that a matrix can be represented as an array.
TEST(MeasurementConversion, WholeMatrixToArray) {
  std::array<float64_t, 4UL> data_row_major{};
  const Eigen::Matrix2d matrix{(Eigen::Matrix2d{} <<
      1.0, 2.0,
      3.0, 4.0).finished()};
  set_from_matrix(data_row_major, matrix, 0, 2, DataStorageOrder::kRowMajor);
  EXPECT_DOUBLE_EQ(matrix(0, 0), data_row_major[0]);
  EXPECT_DOUBLE_EQ(matrix(0, 1), data_row_major[1]);
  EXPECT_DOUBLE_EQ(matrix(1, 0), data_row_major[2]);
  EXPECT_DOUBLE_EQ(matrix(1, 1), data_row_major[3]);
  std::array<float64_t, 4UL> data_col_major{};
  set_from_matrix(data_col_major, matrix, 0, 2, DataStorageOrder::kColumnMajor);
  EXPECT_DOUBLE_EQ(matrix(0, 0), data_col_major[0]);
  EXPECT_DOUBLE_EQ(matrix(1, 0), data_col_major[1]);
  EXPECT_DOUBLE_EQ(matrix(0, 1), data_col_major[2]);
  EXPECT_DOUBLE_EQ(matrix(1, 1), data_col_major[3]);
}

/// \test Check that a matrix can be set to a part of an array.
TEST(MeasurementConversion, PartialMatrixToArray) {
  const Eigen::Matrix2d matrix{(Eigen::Matrix2d{} <<
      1.0, 2.0,
      3.0, 4.0).finished()};
  std::array<float64_t, 9UL> data_row_major{};
  const auto stride = 3;
  const auto start_index = 1;
  set_from_matrix(data_row_major, matrix, start_index, stride, DataStorageOrder::kRowMajor);
  EXPECT_DOUBLE_EQ(0.0, data_row_major[0]);
  EXPECT_DOUBLE_EQ(matrix(0, 0), data_row_major[1]);
  EXPECT_DOUBLE_EQ(matrix(0, 1), data_row_major[2]);
  EXPECT_DOUBLE_EQ(0.0, data_row_major[3]);
  EXPECT_DOUBLE_EQ(matrix(1, 0), data_row_major[4]);
  EXPECT_DOUBLE_EQ(matrix(1, 1), data_row_major[5]);
  EXPECT_DOUBLE_EQ(0.0, data_row_major[6]);
  EXPECT_DOUBLE_EQ(0.0, data_row_major[7]);
  EXPECT_DOUBLE_EQ(0.0, data_row_major[8]);
  std::array<float64_t, 9UL> data_col_major{};
  set_from_matrix(data_col_major, matrix, start_index, stride, DataStorageOrder::kColumnMajor);
  EXPECT_DOUBLE_EQ(0.0, data_col_major[0]);
  EXPECT_DOUBLE_EQ(matrix(0, 0), data_col_major[1]);
  EXPECT_DOUBLE_EQ(matrix(1, 0), data_col_major[2]);
  EXPECT_DOUBLE_EQ(0.0, data_col_major[3]);
  EXPECT_DOUBLE_EQ(matrix(0, 1), data_col_major[4]);
  EXPECT_DOUBLE_EQ(matrix(1, 1), data_col_major[5]);
  EXPECT_DOUBLE_EQ(0.0, data_col_major[6]);
  EXPECT_DOUBLE_EQ(0.0, data_col_major[7]);
  EXPECT_DOUBLE_EQ(0.0, data_col_major[8]);

  std::array<float64_t, 9UL> data{};
  const auto too_big_start = 6;
  EXPECT_THROW(
    (set_from_matrix(data, matrix, too_big_start, stride, DataStorageOrder::kColumnMajor)),
    std::runtime_error);
  EXPECT_THROW(
    (set_from_matrix(data, matrix, too_big_start, stride, DataStorageOrder::kColumnMajor)),
    std::runtime_error);
  const auto too_big_stride = 10;
  EXPECT_THROW(
    (set_from_matrix(data, matrix, start_index, too_big_stride, DataStorageOrder::kColumnMajor)),
    std::runtime_error);
  EXPECT_THROW(
    (set_from_matrix(data, matrix, start_index, too_big_stride, DataStorageOrder::kColumnMajor)),
    std::runtime_error);
}


/// \test Check that matrix slicing works as expected.
TEST(MeasurementConversion, MatrixSlicing) {
  const Eigen::Matrix3d matrix = Eigen::Matrix3d::Random();
  std::array<Eigen::Index, 2UL> indices_top{0, 1};
  EXPECT_TRUE(slice(matrix, indices_top, indices_top).isApprox(matrix.topLeftCorner<2, 2>()));
  std::array<Eigen::Index, 2UL> indices_bottom{1, 2};
  EXPECT_TRUE(
    slice(matrix, indices_bottom, indices_bottom).isApprox(matrix.bottomRightCorner<2, 2>()));
  std::array<Eigen::Index, 2UL> indices_skip{0, 2};
  const Eigen::Matrix2d expected_slice{(Eigen::Matrix2d{} <<
      matrix(0, 0), matrix(0, 2),
      matrix(2, 0), matrix(2, 2)).finished()};
  EXPECT_TRUE(slice(matrix, indices_skip, indices_skip).isApprox(expected_slice));
  std::array<Eigen::Index, 1UL> indices_line{1};
  EXPECT_TRUE(slice(matrix, indices_top, indices_line).isApprox(matrix.col(1).head<2>()));
}
