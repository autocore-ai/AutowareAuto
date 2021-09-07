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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <common/types.hpp>
#include <state_estimation/measurement/linear_measurement.hpp>
#include <state_vector/common_variables.hpp>
#include <state_vector/generic_state.hpp>

namespace autoware
{
namespace common
{
namespace state_estimation
{
namespace detail
{
// Needed to allow for usage with our simplistic tests that use int instead of vectors and matrices.
// Must be above the history include.
bool passes_mahalanobis_gate(
  const std::int32_t, const std::int32_t, const std::int32_t,
  const common::types::float32_t)
{
  return true;
}
}  // namespace detail
}  // namespace state_estimation
}  // namespace common
}  // namespace autoware

#include <state_estimation_nodes/history.hpp>

#include <memory>

namespace
{

using MeasurementState =
  autoware::common::state_vector::FloatState<autoware::common::state_vector::variable::X>;
using FilterState = MeasurementState;

using Measurement = autoware::common::state_estimation::LinearMeasurement<MeasurementState>;

class MockFilter
{
public:
  using State = FilterState;

  MOCK_METHOD(State, state, (), (const));
  MOCK_METHOD(State::Matrix, covariance, (), (const));
  MOCK_METHOD(void, reset, (State, State::Matrix), (const));
  MOCK_METHOD(void, predict, (std::chrono::system_clock::duration));
  MOCK_METHOD(void, correct, (Measurement));
};

using ::testing::_;
using ::testing::Return;

}  // namespace


using autoware::common::state_estimation::History;
using autoware::common::state_estimation::PredictionEvent;
using autoware::common::state_estimation::ResetEvent;

/// @test Test that empty history can be created.
TEST(HistoryTest, CreateEmpty) {
  using HistoryT = History<MockFilter, PredictionEvent, ResetEvent<MockFilter>, Measurement>;
  auto filter = std::make_unique<MockFilter>();
  HistoryT history{*filter, 10, 100};
  ASSERT_TRUE(history.empty());
}

/// @test A basic test that history can be initialized with a ResetEvent.
TEST(HistoryTest, AddResetEvent) {
  using HistoryT = History<MockFilter, PredictionEvent, ResetEvent<MockFilter>, Measurement>;
  auto filter = std::make_unique<MockFilter>();
  HistoryT history{*filter, 10, 100};
  ASSERT_TRUE(history.empty());

  const std::chrono::system_clock::time_point timestamp{std::chrono::system_clock::now()};
  EXPECT_THROW(history.emplace_event(timestamp, PredictionEvent{}), std::runtime_error);
  EXPECT_THROW(history.emplace_event(timestamp, Measurement{}), std::runtime_error);
  ASSERT_TRUE(history.empty());

  const FilterState expected_state{FilterState::Vector{23.0F}};
  const FilterState::Matrix expected_covariance{FilterState::Matrix::Identity()};

  EXPECT_CALL(history.get_filter(), reset(expected_state, expected_covariance));
  EXPECT_CALL(history.get_filter(), state()).WillOnce(Return(expected_state));
  EXPECT_CALL(history.get_filter(), covariance()).WillOnce(Return(expected_covariance));

  EXPECT_NO_THROW(
    history.emplace_event(timestamp, ResetEvent<MockFilter>{expected_state, expected_covariance}));
  EXPECT_EQ(history.get_last_event().stored_state(), expected_state);
  EXPECT_EQ(history.get_last_event().stored_covariance(), expected_covariance);
}

/// @test Test that measurements can be added to the history correctly.
TEST(HistoryTest, AddMeasurementEvents) {
  using HistoryT = History<MockFilter, PredictionEvent, ResetEvent<MockFilter>, Measurement>;

  const FilterState reset_state{FilterState::Vector{23.0F}};
  const FilterState::Matrix reset_covariance{23.0F * FilterState::Matrix::Identity()};

  auto filter = std::make_unique<MockFilter>();
  HistoryT history{*filter, 10, 100};
  ASSERT_TRUE(history.empty());

  EXPECT_CALL(history.get_filter(), reset(reset_state, reset_covariance));
  EXPECT_CALL(history.get_filter(), state()).WillOnce(Return(reset_state));
  EXPECT_CALL(history.get_filter(), covariance()).WillOnce(Return(reset_covariance));
  const std::chrono::system_clock::time_point timestamp{};
  history.emplace_event(timestamp, ResetEvent<MockFilter>{reset_state, reset_covariance});
  EXPECT_EQ(history.get_last_event().stored_state(), reset_state);
  EXPECT_EQ(history.get_last_event().stored_covariance(), reset_covariance);

  const MeasurementState latest_observed_state{MeasurementState::Vector{42.0F}};
  const auto latest_observed_covariance = 42.0F * MeasurementState::Matrix::Identity();

  const std::chrono::system_clock::duration dt{std::chrono::milliseconds{10}};

  EXPECT_CALL(history.get_filter(), reset(reset_state, reset_covariance));
  EXPECT_CALL(history.get_filter(), predict(dt));
  EXPECT_CALL(
    history.get_filter(), correct(
      Measurement{latest_observed_state.vector(), latest_observed_covariance}));
  EXPECT_CALL(history.get_filter(), state()).WillRepeatedly(Return(latest_observed_state));
  EXPECT_CALL(
    history.get_filter(),
    covariance()).WillRepeatedly(Return(latest_observed_covariance));

  // Add a measurement later than the initial reset message. This must succeed.
  EXPECT_NO_THROW(
    history.emplace_event(
      timestamp + dt,
      Measurement{latest_observed_state.vector(), latest_observed_covariance}));
  EXPECT_EQ(history.get_last_event().stored_state(), latest_observed_state);
  EXPECT_EQ(history.get_last_event().stored_covariance(), latest_observed_covariance);

  // Check that we cannot insert a non-reset event to the beginning of history.
  EXPECT_THROW(history.emplace_event(timestamp - dt, PredictionEvent{}), std::runtime_error);
  EXPECT_THROW(history.emplace_event(timestamp - dt, Measurement{}), std::runtime_error);

  // Add a measurement in the past with respect to the latest one in history. This must succeed and
  // the final state must be updated.
  const MeasurementState older_observed_state{MeasurementState::Vector{42.42F}};
  const auto older_observed_covariance = 42.42F * MeasurementState::Matrix::Identity();
  EXPECT_CALL(history.get_filter(), reset(reset_state, reset_covariance));
  EXPECT_CALL(history.get_filter(), predict(dt / 2)).Times(2);
  EXPECT_CALL(
    history.get_filter(),
    correct(Measurement{older_observed_state.vector(), older_observed_covariance}));
  EXPECT_CALL(
    history.get_filter(),
    correct(Measurement{latest_observed_state.vector(), latest_observed_covariance}));
  EXPECT_CALL(history.get_filter(), state())
  .WillOnce(Return(reset_state))                  // mahalanobis distance
  .WillOnce(Return(reset_state))                  // mahalanobis distance
  .WillOnce(Return(older_observed_state))         // remember updated inserted state
  .WillOnce(Return(older_observed_state))         // mahalanobis distance
  .WillOnce(Return(older_observed_state))         // mahalanobis distance
  .WillOnce(Return(latest_observed_state));       // remember latest inserted state
  EXPECT_CALL(history.get_filter(), covariance())
  .WillOnce(Return(reset_covariance))             // mahalanobis distance
  .WillOnce(Return(older_observed_covariance))    // remember updated inserted covariance
  .WillOnce(Return(older_observed_covariance))    // mahalanobis distance
  .WillOnce(Return(latest_observed_covariance));  // remember latest inserted covariance

  EXPECT_NO_THROW(
    history.emplace_event(
      timestamp + dt / 2, Measurement{older_observed_state.vector(), older_observed_covariance}));
}

/// @test Test that history can be overwritten when more events come than it can hold.
TEST(HistoryTest, ExhaustHistory) {
  using HistoryT = History<MockFilter, PredictionEvent, ResetEvent<MockFilter>, Measurement>;

  const auto history_size = 3U;
  const std::int32_t number_of_measurements = 10;
  const std::chrono::system_clock::time_point timestamp{std::chrono::system_clock::now()};
  const std::chrono::system_clock::duration dt{std::chrono::milliseconds{10}};
  const FilterState state{FilterState::Vector{23.0F}};
  const FilterState::Matrix covariance{23.0F * FilterState::Matrix::Identity()};
  auto filter = std::make_unique<MockFilter>();
  HistoryT history{*filter, history_size, 100};
  // A reset is called for every measurement + once for the initial reset.
  EXPECT_CALL(history.get_filter(), reset(state, covariance)).Times(number_of_measurements + 1);
  EXPECT_CALL(history.get_filter(), state()).WillRepeatedly(Return(state));
  EXPECT_CALL(history.get_filter(), covariance()).WillRepeatedly(Return(covariance));
  EXPECT_CALL(history.get_filter(), predict(dt)).Times(number_of_measurements);
  EXPECT_CALL(history.get_filter(), correct(_)).Times(number_of_measurements);

  history.emplace_event(timestamp - dt, ResetEvent<MockFilter>{state, covariance});
  for (std::int32_t i = 0; i < number_of_measurements; ++i) {
    EXPECT_NO_THROW(
      history.emplace_event(timestamp + i * dt, Measurement{state.vector(), covariance}));
  }
  ASSERT_EQ(history_size, history.size());
}
