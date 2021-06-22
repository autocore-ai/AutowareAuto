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

#include <state_estimation/measurement/linear_measurement.hpp>
#include <state_vector/generic_state.hpp>

#include <gtest/gtest.h>

#include <tuple>

using autoware::common::state_vector::variable::X;
using autoware::common::state_vector::variable::Y;
using autoware::common::state_vector::variable::X_VELOCITY;
using autoware::common::state_vector::variable::Y_VELOCITY;
using autoware::common::state_estimation::LinearMeasurement;
using autoware::common::state_vector::FloatState;

/// @test Test that a measurement is correctly created and queried.
TEST(TestLinearMeasurement, Create) {
  using MeasurementState = FloatState<X, Y>;
  // When the standard deviation constructor is used, the covariance matrix is diagonal
  auto measurement = LinearMeasurement<MeasurementState>::create_with_stddev(
    {1.0F, 2.0F},
    {3.0F, 4.0F});
  EXPECT_EQ((MeasurementState{{1.0F, 2.0F}}), measurement.state());
  auto expected_covariance = (MeasurementState::Matrix{} <<
    9.0F, 0.0F,
    0.0F, 16.0F).finished();
  EXPECT_TRUE(expected_covariance.isApprox(measurement.covariance())) << measurement.covariance();
  // Covariance is preserved when the covariance constructor is used, including off-diagonal
  // elements.
  expected_covariance(1, 0) = 1.0F;
  expected_covariance(0, 1) = 1.0F;
  measurement = LinearMeasurement<MeasurementState>{{1.0F, 2.0F}, expected_covariance};
  EXPECT_TRUE(expected_covariance.isApprox(measurement.covariance())) << measurement.covariance();
}

/// @test Test that a measurement is correctly mapped to another state.
TEST(TestLinearMeasurement, CreateAndMapFromOtherState) {
  using State = FloatState<X, X_VELOCITY, Y, Y_VELOCITY>;
  using MeasurementState = FloatState<X, Y>;
  auto measurement =
    LinearMeasurement<MeasurementState>::create_with_stddev({42.0F, 23.0F}, {23.0F, 42.0F});
  EXPECT_EQ((MeasurementState{{42.0F, 23.0F}}), measurement.state());
  const auto mapping_matrix = measurement.mapping_matrix_from(State{});
  ASSERT_EQ(mapping_matrix.rows(), 2);
  ASSERT_EQ(mapping_matrix.cols(), 4);
  const auto expected_matrix = (Eigen::Matrix<float, 2, 4>{} <<
    1.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 1.0F, 0.0F).finished();
  EXPECT_TRUE(mapping_matrix.isApprox(expected_matrix));
  State state{{1.0F, 2.0F, 3.0F, 4.0F}};
  const auto measurement_mapped_state = measurement.create_new_instance_from(state);
  ASSERT_EQ(measurement_mapped_state.size(), 2);
  EXPECT_FLOAT_EQ(measurement_mapped_state[0], 1.0F);
  EXPECT_FLOAT_EQ(measurement_mapped_state[1], 3.0F);
}

/// @test Test that a measurement is correctly mapped to another state.
TEST(TestLinearMeasurement, CreateAndMapToOtherState) {
  using State = FloatState<X, X_VELOCITY, Y, Y_VELOCITY>;
  using SmallerState = FloatState<X, X_VELOCITY>;
  using MeasurementState = FloatState<X, Y>;
  auto measurement =
    LinearMeasurement<MeasurementState>::create_with_stddev({42.0F, 23.0F}, {23.0F, 42.0F});
  EXPECT_EQ((MeasurementState{{42.0F, 23.0F}}), measurement.state());
  State state{{1.0F, 2.0F, 3.0F, 4.0F}};
  const auto filled_state = measurement.map_into(state);
  ASSERT_EQ(filled_state.size(), state.size());
  EXPECT_FLOAT_EQ(filled_state[1], state[1]);
  EXPECT_FLOAT_EQ(filled_state[3], state[3]);
  EXPECT_FLOAT_EQ(filled_state.at<X>(), measurement.state().at<X>());
  EXPECT_FLOAT_EQ(filled_state.at<Y>(), measurement.state().at<Y>());
  MeasurementState measurement_state{};
  SmallerState smaller_state_temp{};
  const auto temp = measurement_state.copy_into(smaller_state_temp);
  ASSERT_EQ(temp.size(), SmallerState::size());
  const auto smaller_state = measurement.map_into(SmallerState{});
  ASSERT_EQ(smaller_state.size(), SmallerState::size());
  EXPECT_FLOAT_EQ(smaller_state.at<X>(), measurement.state().at<X>());
  EXPECT_FLOAT_EQ(smaller_state.at<X_VELOCITY>(), 0.0F);
}

/// @test Test equality operator.
TEST(TestLinearMeasurement, Equality) {
  using MeasurementState = FloatState<X, Y>;
  auto measurement_1 = LinearMeasurement<MeasurementState>::create_with_stddev(
    {42.42F, 23.23F},
    {23.42F, 42.23F});
  auto measurement_2 = LinearMeasurement<MeasurementState>::create_with_stddev(
    {42.42F, 23.23F},
    {23.42F, 42.23F});
  EXPECT_EQ(measurement_1, measurement_2);
}
