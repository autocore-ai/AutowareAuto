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

/// \copyright Copyright 2021 Apex.AI, Inc.
/// All rights reserved.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <common/types.hpp>

namespace autoware
{
namespace prediction
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
}  // namespace prediction
}  // namespace autoware

#include <state_estimation_nodes/history.hpp>

#include <memory>

namespace
{

struct FakeMeasurement
{
  std::int32_t get_values() const noexcept {return values;}
  std::int32_t get_variances() const noexcept {return variances;}

  template<std::size_t kNumStates>
  static std::int32_t get_observation_to_state_mapping()
  {
    return kNumStates;
  }

  std::int32_t get_values_in_full_state(std::int32_t state) const noexcept {return state;}

  std::int32_t values;
  std::int32_t variances;
};

class FakeState
{
public:
  FakeState() = default;
  // cppcheck-suppress noExplicitConstructor
  FakeState(std::int32_t n)  // NOLINT(runtime/explicit) we allow conversion to ease notation.
  : m_state{n} {}
  operator std::int32_t() const {return m_state;}
  static constexpr std::int32_t Zero() {return 0;}

private:
  std::int32_t m_state{};
};

class MockFilter
{
public:
  using state_vec_t = FakeState;
  using square_mat_t = FakeState;

  MOCK_METHOD(state_vec_t, get_state, (), (const));
  MOCK_METHOD(square_mat_t, get_covariance, (), (const));
  MOCK_METHOD(void, reset, (std::int32_t, std::int32_t), (const));
  MOCK_METHOD(void, temporal_update, (std::chrono::system_clock::duration));
  MOCK_METHOD(void, observation_update, (std::int32_t, std::int32_t, std::int32_t));
};

using ::testing::_;
using ::testing::Return;

}  // namespace


using autoware::prediction::History;
using autoware::prediction::PredictionEvent;
using autoware::prediction::ResetEvent;

/// @test Test that empty history can be created.
TEST(HistoryTest, create_empty) {
  using HistoryT = History<MockFilter, 1, PredictionEvent, ResetEvent<MockFilter>, FakeMeasurement>;
  auto filter = std::make_unique<MockFilter>();
  HistoryT history{*filter, 10, 100};
  ASSERT_TRUE(history.empty());
}

/// @test A basic test that history can be initialized with a ResetEvent.
TEST(HistoryTest, add_reset_event) {
  using HistoryT = History<MockFilter, 1, PredictionEvent, ResetEvent<MockFilter>, FakeMeasurement>;
  auto filter = std::make_unique<MockFilter>();
  HistoryT history{*filter, 10, 100};
  ASSERT_TRUE(history.empty());

  const std::chrono::system_clock::time_point timestamp{std::chrono::system_clock::now()};
  EXPECT_THROW(history.emplace_event(timestamp, PredictionEvent{}), std::runtime_error);
  EXPECT_THROW(history.emplace_event(timestamp, FakeMeasurement{}), std::runtime_error);
  ASSERT_TRUE(history.empty());

  const std::int32_t expected_state = 23;
  const std::int32_t expected_covariance = 42;

  EXPECT_CALL(history.get_filter(), reset(expected_state, expected_covariance));
  EXPECT_CALL(history.get_filter(), get_state()).WillOnce(Return(expected_state));
  EXPECT_CALL(history.get_filter(), get_covariance()).WillOnce(Return(expected_covariance));

  EXPECT_NO_THROW(
    history.emplace_event(timestamp, ResetEvent<MockFilter>{expected_state, expected_covariance}));
  EXPECT_EQ(history.get_last_event().stored_state(), expected_state);
  EXPECT_EQ(history.get_last_event().stored_covariance_factor(), expected_covariance);
}

/// @test Test that measurements can be added to the history correctly.
TEST(HistoryTest, add_measurement_events) {
  using HistoryT = History<MockFilter, 1, PredictionEvent, ResetEvent<MockFilter>, FakeMeasurement>;

  const std::int32_t reset_state = 23;
  const std::int32_t reset_covariance = 42;

  auto filter = std::make_unique<MockFilter>();
  HistoryT history{*filter, 10, 100};
  ASSERT_TRUE(history.empty());

  EXPECT_CALL(history.get_filter(), reset(reset_state, reset_covariance));
  EXPECT_CALL(history.get_filter(), get_state()).WillOnce(Return(reset_state));
  EXPECT_CALL(history.get_filter(), get_covariance()).WillOnce(Return(reset_covariance));
  const std::chrono::system_clock::time_point timestamp{std::chrono::system_clock::now()};
  history.emplace_event(timestamp, ResetEvent<MockFilter>{reset_state, reset_covariance});
  EXPECT_EQ(history.get_last_event().stored_state(), reset_state);
  EXPECT_EQ(history.get_last_event().stored_covariance_factor(), reset_covariance);

  const std::int32_t latest_observed_state = 2323;
  const std::int32_t latest_observed_covariance = 4242;

  const std::chrono::system_clock::duration dt{std::chrono::milliseconds{10}};

  EXPECT_CALL(history.get_filter(), reset(reset_state, reset_covariance));
  EXPECT_CALL(history.get_filter(), temporal_update(dt));
  EXPECT_CALL(
    history.get_filter(),
    observation_update(latest_observed_state, 1, latest_observed_covariance));
  EXPECT_CALL(history.get_filter(), get_state()).WillRepeatedly(Return(latest_observed_state));
  EXPECT_CALL(
    history.get_filter(),
    get_covariance()).WillRepeatedly(Return(latest_observed_covariance));

  // Add a measurement later than the initial reset message. This must succeed.
  EXPECT_NO_THROW(
    history.emplace_event(
      timestamp + dt, FakeMeasurement{latest_observed_state, latest_observed_covariance}));
  EXPECT_EQ(history.get_last_event().stored_state(), latest_observed_state);
  EXPECT_EQ(history.get_last_event().stored_covariance_factor(), latest_observed_covariance);

  // Check that we cannot insert a non-reset event to the beginning of history.
  EXPECT_THROW(history.emplace_event(timestamp - dt, PredictionEvent{}), std::runtime_error);
  EXPECT_THROW(history.emplace_event(timestamp - dt, FakeMeasurement{}), std::runtime_error);

  // Add a measurement in the past with respect to the latest one in history. This must succeed and
  // the final state must be updated.
  const std::int32_t older_observed_state = 232323;
  const std::int32_t older_observed_covariance = 424242;
  EXPECT_CALL(history.get_filter(), reset(reset_state, reset_covariance));
  EXPECT_CALL(history.get_filter(), temporal_update(dt / 2)).Times(2);
  EXPECT_CALL(
    history.get_filter(),
    observation_update(older_observed_state, 1, older_observed_covariance));
  EXPECT_CALL(
    history.get_filter(),
    observation_update(latest_observed_state, 1, latest_observed_covariance));
  EXPECT_CALL(history.get_filter(), get_state())
  .WillOnce(Return(reset_state))                  // mahalanobis distance
  .WillOnce(Return(reset_state))                  // mahalanobis distance
  .WillOnce(Return(older_observed_state))         // remember updated inserted state
  .WillOnce(Return(older_observed_state))         // mahalanobis distance
  .WillOnce(Return(older_observed_state))         // mahalanobis distance
  .WillOnce(Return(latest_observed_covariance));  // remember latest inserted state
  EXPECT_CALL(history.get_filter(), get_covariance())
  .WillOnce(Return(reset_covariance))             // mahalanobis distance
  .WillOnce(Return(older_observed_covariance))    // remember updated inserted covariance
  .WillOnce(Return(older_observed_covariance))    // mahalanobis distance
  .WillOnce(Return(latest_observed_covariance));  // remember latest inserted covariance

  EXPECT_NO_THROW(
    history.emplace_event(
      timestamp + dt / 2,
      FakeMeasurement{older_observed_state, older_observed_covariance}));
}

/// @test Test that history can be overwritten when more events come than it can hold.
TEST(HistoryTest, exhaust_history) {
  using HistoryT = History<MockFilter, 1, PredictionEvent, ResetEvent<MockFilter>, FakeMeasurement>;

  const auto history_size = 3U;
  const std::int32_t number_of_measurements = 10;
  const std::chrono::system_clock::time_point timestamp{std::chrono::system_clock::now()};
  const std::chrono::system_clock::duration dt{std::chrono::milliseconds{10}};
  const std::int32_t state = 23;
  const std::int32_t covariance = 42;
  auto filter = std::make_unique<MockFilter>();
  HistoryT history{*filter, history_size, 100};
  // A reset is called for every measurement + once for the initial reset.
  EXPECT_CALL(history.get_filter(), reset(state, covariance)).Times(number_of_measurements + 1);
  EXPECT_CALL(history.get_filter(), get_state()).WillRepeatedly(Return(state));
  EXPECT_CALL(history.get_filter(), get_covariance()).WillRepeatedly(Return(covariance));
  EXPECT_CALL(history.get_filter(), temporal_update(dt)).Times(number_of_measurements);
  EXPECT_CALL(history.get_filter(), observation_update(_, _, _)).Times(number_of_measurements);

  history.emplace_event(timestamp - dt, ResetEvent<MockFilter>{state, covariance});
  for (std::int32_t i = 0; i < number_of_measurements; ++i) {
    EXPECT_NO_THROW(history.emplace_event(timestamp + i * dt, FakeMeasurement{state, covariance}));
  }
  ASSERT_EQ(history_size, history.size());
}
