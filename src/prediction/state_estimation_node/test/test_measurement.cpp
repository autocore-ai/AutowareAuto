// Copyright 2020 Apex.AI, Inc.
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

/// \copyright Copyright 2020 Apex.AI, Inc.
/// All rights reserved.

#include <gtest/gtest.h>

#include <state_estimation_node/measurement.hpp>

using autoware::prediction::Measurement;
using autoware::prediction::MeasurementBasedTime;

/// A utility enum to name possible measurement modalities.
struct MeasurementModality
{
  static constexpr std::uint32_t kX = 0U;
  static constexpr std::uint32_t kY = 1U;
  static constexpr std::uint32_t kSpeedX = 2U;
  static constexpr std::uint32_t kSpeedY = 3U;
  static constexpr std::uint32_t kAccelerationX = 4U;
  static constexpr std::uint32_t kAccelerationY = 5U;
};

/// \test Create a measurement of X and Y with variance values.
TEST(MeasurementTest, x_y) {
  using MeasurementType = Measurement<float, MeasurementModality::kX, MeasurementModality::kY>;
  const auto now = MeasurementBasedTime{std::chrono::system_clock::now()};
  MeasurementType measurement{now, {42.0F, 42.42F}, {23.23F, 23.0F}};
  ASSERT_EQ(now, measurement.get_acquisition_time());
  const auto values = measurement.get_values();
  ASSERT_EQ(2U, values.size());
  EXPECT_FLOAT_EQ(42.0F, values[0]);
  EXPECT_FLOAT_EQ(42.42F, values[1]);
  const auto variances = measurement.get_variances();
  ASSERT_EQ(2U, variances.size());
  EXPECT_FLOAT_EQ(23.23F, variances[0]);
  EXPECT_FLOAT_EQ(23.0F, variances[1]);
  const auto state = measurement.get_values_in_full_state<6U>();
  ASSERT_EQ(6U, state.size());
  EXPECT_FLOAT_EQ(42.0F, state[0]);
  EXPECT_FLOAT_EQ(42.42F, state[1]);
  for (auto i = 2U; i < state.size(); ++i) {
    EXPECT_FLOAT_EQ(0.0F, state[i]);
  }
  using MappingMatrixT = Eigen::Matrix<float, 2U, 6U>;
  const MappingMatrixT expected_mapping{MappingMatrixT::Identity()};
  const auto mapping_matrix = MeasurementType::get_observation_to_state_mapping<6U>();
  ASSERT_EQ(expected_mapping.size(), mapping_matrix.size());
  EXPECT_TRUE(expected_mapping.isApprox(mapping_matrix)) <<
    "Expected:\n" << expected_mapping << "\n" <<
    "Got:\n" << mapping_matrix;
}

/// \test Create a measurement of only Speed X and Speed Y with variance values.
///
/// In this case the values will be out of order in the matrix as the speed variables
/// are not at the beginning of the state.
TEST(MeasurementTest, speed_x_speed_y) {
  using MeasurementType = Measurement<float,
      MeasurementModality::kSpeedX,
      MeasurementModality::kSpeedY>;
  const Eigen::Vector2f speed{1.0F, 2.0F};
  const Eigen::Vector2f var{3.0F, 4.0F};
  const auto now = MeasurementBasedTime{std::chrono::system_clock::now()};
  MeasurementType measurement{now, speed, var};
  ASSERT_EQ(now, measurement.get_acquisition_time());
  const auto values = measurement.get_values();
  ASSERT_EQ(speed.size(), values.size());
  for (auto i = 0U; i < speed.size(); ++i) {
    EXPECT_FLOAT_EQ(speed[i], values[i]);
  }
  const auto variances = measurement.get_variances();
  ASSERT_EQ(var.size(), variances.size());
  for (auto i = 0U; i < var.size(); ++i) {
    EXPECT_FLOAT_EQ(var[i], variances[i]);
  }
  auto state = measurement.get_values_in_full_state<6U>();
  using VectorT = Eigen::Matrix<float, 6U, 1U>;
  VectorT expected_state{
    (VectorT{} << 0.0F, 0.0F, speed[0], speed[1], 0.0F, 0.0F).finished()};
  ASSERT_EQ(6U, state.size());
  EXPECT_TRUE(expected_state.isApprox(state));
  auto donor_state =
    (VectorT{} << 42.0F, 42.0F, 42.0F, 42.0F, 42.0F, 42.0F).finished();
  state = measurement.get_values_in_full_state(donor_state);
  ASSERT_EQ(6U, state.size());
  expected_state = donor_state;
  expected_state[2] = speed[0];
  expected_state[3] = speed[1];
  EXPECT_TRUE(expected_state.isApprox(state));

  using MappingMatrixT = Eigen::Matrix<float, 2U, 6U>;
  const MappingMatrixT expected_mapping{(MappingMatrixT{} <<
      0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F
    ).finished()};
  const auto mapping_matrix = MeasurementType::get_observation_to_state_mapping<6U>();
  ASSERT_EQ(expected_mapping.size(), mapping_matrix.size());
  EXPECT_TRUE(expected_mapping.isApprox(mapping_matrix)) <<
    "Expected:\n" << expected_mapping << "\n" <<
    "Got:\n" << mapping_matrix;
}

/// \test Create a measurement of X, Y, Speed X and Speed Y with variance values.
TEST(MeasurementTest, x_y_speed_x_speed_y) {
  using MeasurementType = Measurement<float,
      MeasurementModality::kX,
      MeasurementModality::kY,
      MeasurementModality::kSpeedX,
      MeasurementModality::kSpeedY>;
  const Eigen::Vector4f pos{1.0F, 2.0F, 3.0F, 4.0F};
  const Eigen::Vector4f var{5.0F, 6.0F, 7.0F, 8.0F};
  const auto now = MeasurementBasedTime{std::chrono::system_clock::now()};
  MeasurementType measurement{now, pos, var};
  ASSERT_EQ(now, measurement.get_acquisition_time());
  const auto values = measurement.get_values();
  ASSERT_EQ(pos.size(), values.size());
  for (auto i = 0U; i < pos.size(); ++i) {
    EXPECT_FLOAT_EQ(pos[i], values[i]);
  }
  const auto variances = measurement.get_variances();
  ASSERT_EQ(var.size(), variances.size());
  for (auto i = 0U; i < var.size(); ++i) {
    EXPECT_FLOAT_EQ(var[i], variances[i]);
  }
  const auto state = measurement.get_values_in_full_state<6U>();
  ASSERT_EQ(6U, state.size());
  for (auto i = 0U; i < pos.size(); ++i) {
    EXPECT_FLOAT_EQ(pos[i], state[i]);
  }
  for (auto i = 4U; i < state.size(); ++i) {
    EXPECT_FLOAT_EQ(0.0F, state[i]);
  }
  using MappingMatrixT = Eigen::Matrix<float, 4U, 6U>;
  const MappingMatrixT expected_mapping{MappingMatrixT::Identity()};
  const auto mapping_matrix = MeasurementType::get_observation_to_state_mapping<6U>();
  ASSERT_EQ(expected_mapping.size(), mapping_matrix.size());
  EXPECT_TRUE(expected_mapping.isApprox(mapping_matrix)) <<
    "Expected:\n" << expected_mapping << "\n" <<
    "Got:\n" << mapping_matrix;
}

/// \test Create a measurement of X, Y, Speed X and Speed Y with variance values and mixed order.
///
/// The values in this test are in mixed order, i.e., SpeedX, SpeedY, X, Y.
/// This has an influence on how the resulting matrix looks.
TEST(MeasurementTest, speed_x_speed_y_x_y_mixed_order) {
  using MeasurementType = Measurement<float,
      MeasurementModality::kSpeedX,
      MeasurementModality::kSpeedY,
      MeasurementModality::kX,
      MeasurementModality::kY>;
  const Eigen::Vector4f pos{1.0F, 2.0F, 3.0F, 4.0F};
  const Eigen::Vector4f var{5.0F, 6.0F, 7.0F, 8.0F};
  const auto now = MeasurementBasedTime{std::chrono::system_clock::now()};
  MeasurementType measurement{now, pos, var};
  ASSERT_EQ(now, measurement.get_acquisition_time());
  const auto values = measurement.get_values();
  ASSERT_EQ(pos.size(), values.size());
  for (auto i = 0U; i < pos.size(); ++i) {
    EXPECT_FLOAT_EQ(pos[i], values[i]);
  }
  const auto variances = measurement.get_variances();
  ASSERT_EQ(var.size(), variances.size());
  for (auto i = 0U; i < var.size(); ++i) {
    EXPECT_FLOAT_EQ(var[i], variances[i]);
  }
  const auto state = measurement.get_values_in_full_state<4U>();
  ASSERT_EQ(4U, state.size());
  // We assume that this state always has the ordering x, y, speed_x, speed_y, acc_x, acc_y, etc.
  EXPECT_FLOAT_EQ(pos[2], state[0]);
  EXPECT_FLOAT_EQ(pos[3], state[1]);
  EXPECT_FLOAT_EQ(pos[0], state[2]);
  EXPECT_FLOAT_EQ(pos[1], state[3]);
  using MappingMatrixT = Eigen::Matrix<float, 4U, 4U>;
  const MappingMatrixT expected_mapping{(MappingMatrixT{} <<
      0.0F, 0.0F, 1.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 1.0F,
      1.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 1.0F, 0.0F, 0.0F
    ).finished()};
  const auto mapping_matrix = MeasurementType::get_observation_to_state_mapping<4U>();
  ASSERT_EQ(expected_mapping.size(), mapping_matrix.size());
  EXPECT_TRUE(expected_mapping.isApprox(mapping_matrix)) <<
    "Expected:\n" << expected_mapping << "\n" <<
    "Got:\n" << mapping_matrix;
}
