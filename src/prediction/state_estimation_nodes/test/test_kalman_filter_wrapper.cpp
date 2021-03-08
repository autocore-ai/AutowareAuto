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

#include <gtest/gtest.h>

#include <state_estimation_nodes/kalman_filter_wrapper.hpp>
#include <state_estimation_nodes/measurement_typedefs.hpp>

#include <common/types.hpp>

#include <limits>

using autoware::common::types::float64_t;
using autoware::common::types::float32_t;

using autoware::motion::motion_model::ConstantAcceleration;
using autoware::prediction::ConstantAccelerationFilter;
using autoware::prediction::Measurement;
using autoware::prediction::MeasurementPose;
using autoware::prediction::MeasurementSpeed;
using autoware::prediction::MeasurementPoseAndSpeed;

using Eigen::Matrix;
template<std::int64_t kSize>
using Vector = Eigen::Matrix<float32_t, kSize, 1>;
using Vector2f = Vector<2>;

namespace
{

// TODO(igor): move these to a better place.
template<typename T, std::uint64_t kRows, std::uint64_t kCols>
void assert_matrices_eq(const Matrix<T, kRows, kCols> & m1, const Matrix<T, kRows, kCols> & m2)
{
  ASSERT_EQ(m1.height(), m2.height());
  ASSERT_EQ(m1.width(), m2.width());
  for (auto r = 0U; r < m1.height(); ++r) {
    for (auto c = 0U; c < m1.width(); ++c) {
      EXPECT_FLOAT_EQ(m1.at(r, c), m2.at(r, c));
    }
  }
}

constexpr float64_t kEpsilon = std::numeric_limits<float64_t>::epsilon();
const auto kCovarianceIdentity = Eigen::Matrix<float32_t, 6, 6>::Identity();
const auto kNoiseIdentity{(Eigen::Matrix<float32_t, 6, 2>{} <<
    1.0F, 0.0F,
    0.0F, 1.0F,
    1.0F, 0.0F,
    0.0F, 1.0F,
    1.0F, 0.0F,
    0.0F, 1.0F).finished()};

}  // namespace

/// \test Creating an empty filter and checking that everything is zero.
TEST(KalmanFilterWrapperTest, create_empty) {
  ConstantAccelerationFilter filter{
    kCovarianceIdentity, kNoiseIdentity, std::chrono::milliseconds{100LL}, "map"};
  EXPECT_FALSE(filter.is_initialized());
  EXPECT_THROW(filter.get_state(), std::runtime_error);
}

/// \test Before the filter is initialized we don't want to accept any updates.
TEST(KalmanFilterWrapperTest, ignore_everything_before_initialization) {
  ConstantAccelerationFilter filter{
    kCovarianceIdentity, kNoiseIdentity, std::chrono::milliseconds{100LL}, "map"};
  const auto now = std::chrono::system_clock::time_point{std::chrono::system_clock::now()};
  MeasurementPoseAndSpeed measurement{now, Vector<4U>{42.0F, 42.0F, 42.0F, 42.0F}};
  EXPECT_FALSE(filter.is_initialized());
  EXPECT_FALSE(filter.add_next_temporal_update_to_history());
  EXPECT_FALSE(filter.add_observation_to_history(measurement));
}

/// \test Initialize filter with a measurement.
TEST(KalmanFilterWrapperTest, initialize) {
  ConstantAccelerationFilter filter{
    kCovarianceIdentity, kNoiseIdentity, std::chrono::milliseconds{100LL}, "map"};
  const auto now_measurement =
    std::chrono::system_clock::time_point{std::chrono::system_clock::now()};
  EXPECT_FALSE(filter.is_initialized());
  MeasurementPoseAndSpeed measurement{now_measurement, Vector<4U>{42.0F, 42.0F, 42.0F, 42.0F}};
  EXPECT_FALSE(filter.add_observation_to_history(measurement));
  filter.add_reset_event_to_history(measurement);
  ASSERT_TRUE(filter.is_initialized());
  const auto odom_msg = filter.get_state();
  EXPECT_NEAR(odom_msg.pose.pose.position.x, 42.0, kEpsilon);
  EXPECT_NEAR(odom_msg.pose.pose.position.y, 42.0, kEpsilon);
  EXPECT_NEAR(odom_msg.twist.twist.linear.x, 42.0, kEpsilon);
  EXPECT_NEAR(odom_msg.twist.twist.linear.y, 42.0, kEpsilon);
  EXPECT_NEAR(odom_msg.pose.covariance[0], 1.0, kEpsilon);
  EXPECT_NEAR(odom_msg.pose.covariance[7], 1.0, kEpsilon);
  EXPECT_NEAR(odom_msg.twist.covariance[0], 1.0, kEpsilon);
  EXPECT_NEAR(odom_msg.twist.covariance[7], 1.0, kEpsilon);
}

/// \test Initialize filter with a measurement and ignore the ones coming from the past.
TEST(KalmanFilterWrapperTest, ignore_measurements_from_the_past) {
  ConstantAccelerationFilter filter{
    kCovarianceIdentity, kNoiseIdentity, std::chrono::milliseconds{100LL}, "map"};
  const auto now_measurement_time =
    std::chrono::system_clock::time_point{std::chrono::system_clock::now()};
  const auto dt = std::chrono::milliseconds{2LL};
  const auto state = Vector<4U>{42.0F, 42.0F, 0.0F, 0.0F};
  const auto variance = Vector<4U>{1.0F, 1.0F, 1.0F, 1.0F};
  EXPECT_FALSE(filter.is_initialized());
  MeasurementPoseAndSpeed measurement_now{now_measurement_time, state, variance};
  MeasurementPoseAndSpeed measurement_later{now_measurement_time + dt, state, variance};
  filter.add_reset_event_to_history(measurement_now);
  ASSERT_TRUE(filter.is_initialized());
  EXPECT_TRUE(filter.add_observation_to_history(measurement_later));
  const auto mid_measurement_time{now_measurement_time + dt / 2};
  MeasurementPoseAndSpeed mid_measurement{mid_measurement_time, state, variance};
  // Update that comes in the future bt carries a measurement after the first one, but before the
  // last one, so it should land in the middle of history.
  EXPECT_TRUE(filter.add_observation_to_history(mid_measurement));
  const auto odom_msg = filter.get_state();
  EXPECT_NEAR(odom_msg.pose.pose.position.x, state(0), kEpsilon);
  EXPECT_NEAR(odom_msg.pose.pose.position.y, state(1), kEpsilon);
  EXPECT_NEAR(odom_msg.twist.twist.linear.x, state(2), kEpsilon);
  EXPECT_NEAR(odom_msg.twist.twist.linear.y, state(3), kEpsilon);
}

/// \test Ignore measurements that don't pass the Mahalanobis threshold.
TEST(KalmanFilterWrapperTest, ignore_far_away_measurements) {
  using namespace std::chrono_literals;
  const float32_t mahalanobis_threshold = 1.0F;
  ConstantAccelerationFilter filter{
    kCovarianceIdentity,
    kNoiseIdentity,
    std::chrono::milliseconds{100LL},
    "map",
    std::chrono::milliseconds{5000LL},
    mahalanobis_threshold};
  EXPECT_FALSE(filter.is_initialized());
  std::chrono::system_clock::time_point timestamp{std::chrono::system_clock::now()};
  filter.add_reset_event_to_history(
    Vector<6>::Zero(),
    kCovarianceIdentity,
    timestamp);
  ASSERT_TRUE(filter.is_initialized());
  // Check that nothing else forbids us from updating the state.
  timestamp += 10ms;
  filter.add_observation_to_history(
    MeasurementPose{timestamp, Vector2f{0.0F, 0.0F}, Vector2f{1.0F, 1.0F}});
  const auto odom_msg_before = filter.get_state();
  timestamp += 10ms;
  // Try to add a very precise measurement that should be ignored due to the mahalanobis gate.
  filter.add_observation_to_history(
    MeasurementPose{timestamp, Vector2f{10.0F, 0.0F}, Vector2f{0.1F, 0.1F}});
  const auto odom_msg_after = filter.get_state();
  EXPECT_NEAR(odom_msg_after.pose.pose.position.x, 0.0, kEpsilon);
  EXPECT_NEAR(odom_msg_after.pose.pose.position.y, 0.0, kEpsilon);
  // The covariance has grown because we ignore that measurement as it does not pass the mahalanobis
  // gate.
  EXPECT_GT(odom_msg_after.pose.covariance[0], odom_msg_before.pose.covariance[0]);
  EXPECT_GT(odom_msg_after.pose.covariance[7], odom_msg_before.pose.covariance[7]);
}

/// \test Covariance of a static object grows without new observations.
TEST(KalmanFilterWrapperTest, covariance_grows_with_time) {
  ConstantAccelerationFilter filter{
    kCovarianceIdentity, kNoiseIdentity, std::chrono::milliseconds{100LL}, "map"};
  EXPECT_FALSE(filter.is_initialized());
  const auto timestamp = std::chrono::system_clock::time_point{std::chrono::system_clock::now()};
  MeasurementPoseAndSpeed measurement{timestamp, Vector<4U>{42.0F, 42.0F, 0.0F, 0.0F}};
  filter.add_reset_event_to_history(measurement);
  ASSERT_TRUE(filter.is_initialized());
  const auto odom_msg = filter.get_state();
  EXPECT_NEAR(odom_msg.pose.pose.position.x, 42.0, kEpsilon);
  EXPECT_NEAR(odom_msg.pose.pose.position.y, 42.0, kEpsilon);
  EXPECT_NEAR(odom_msg.twist.twist.linear.x, 0.0, kEpsilon);
  EXPECT_NEAR(odom_msg.twist.twist.linear.y, 0.0, kEpsilon);
  EXPECT_NEAR(odom_msg.pose.covariance[0], 1.0, kEpsilon);
  EXPECT_NEAR(odom_msg.pose.covariance[7], 1.0, kEpsilon);
  EXPECT_NEAR(odom_msg.twist.covariance[0], 1.0, kEpsilon);
  EXPECT_NEAR(odom_msg.twist.covariance[7], 1.0, kEpsilon);
  using namespace std::chrono_literals;
  filter.add_next_temporal_update_to_history();
  const auto odom_msg_later = filter.get_state();
  EXPECT_NEAR(odom_msg_later.pose.pose.position.x, 42.0, kEpsilon);
  EXPECT_NEAR(odom_msg_later.pose.pose.position.y, 42.0, kEpsilon);
  EXPECT_NEAR(odom_msg_later.twist.twist.linear.x, 0.0, kEpsilon);
  EXPECT_NEAR(odom_msg_later.twist.twist.linear.y, 0.0, kEpsilon);
  EXPECT_GT(odom_msg_later.pose.covariance[0], 1.0 + kEpsilon);
  EXPECT_GT(odom_msg_later.pose.covariance[7], 1.0 + kEpsilon);
  EXPECT_GT(odom_msg_later.twist.covariance[0], 1.0 + kEpsilon);
  EXPECT_GT(odom_msg_later.twist.covariance[7], 1.0 + kEpsilon);
}


/// \test Track a static object.
TEST(KalmanFilterWrapperTest, track_static_object) {
  using namespace std::chrono_literals;
  ConstantAccelerationFilter filter{
    kCovarianceIdentity, kNoiseIdentity, std::chrono::milliseconds{100LL}, "map"};
  EXPECT_FALSE(filter.is_initialized());
  std::chrono::system_clock::time_point timestamp{std::chrono::system_clock::now()};
  filter.add_reset_event_to_history(
    MeasurementPose{timestamp, Vector2f{42.0F, 42.0F}, Vector2f{1.0F, 1.0F}});
  ASSERT_TRUE(filter.is_initialized());
  const auto initial_state = filter.get_state();
  for (int i = 0; i < 10; ++i) {
    timestamp += 100ms;
    EXPECT_TRUE(
      filter.add_observation_to_history(
        MeasurementPose{timestamp, Vector2f{42.0F, 42.0F}, Vector2f{1.0F, 1.0F}}));
    auto odom_msg = filter.get_state();
    EXPECT_NEAR(odom_msg.pose.pose.position.x, initial_state.pose.pose.position.x, kEpsilon);
    EXPECT_NEAR(odom_msg.pose.pose.position.y, initial_state.pose.pose.position.y, kEpsilon);
    // Check that the covariance of observed state is dropping.
    EXPECT_LT(odom_msg.pose.covariance[0], 1.0 - kEpsilon);
    EXPECT_LT(odom_msg.pose.covariance[7], 1.0 - kEpsilon);
    // Check that the covariance of unobserved state is growing.
    EXPECT_GT(odom_msg.twist.covariance[0], 1.0 + kEpsilon);
    EXPECT_GT(odom_msg.twist.covariance[7], 1.0 + kEpsilon);
  }
}

/// \test Track a ball thrown at 45 deg angle. We perfectly observe positions of the ball.
///
/// The ball moves at a parabola starting at (0, 0):
///  ^     ___
///  |   _/   \_
///  | _/       \_
///  |/           \_
///  └-------------->
/// start           end
///
TEST(KalmanFilterWrapperTest, track_thrown_ball) {
  using namespace std::chrono_literals;
  using FloatSeconds = std::chrono::duration<float32_t>;
  using autoware::motion::motion_model::ConstantAcceleration;

  const float32_t g = -9.80665F;  // m/s^2.
  const float32_t initial_speed = 9.80665F;  // m/s
  auto state{(Vector<6U>{} << 0.0F, 0.0F, initial_speed, initial_speed, 0.0F, g).finished()};

  ConstantAccelerationFilter filter{
    kCovarianceIdentity, kNoiseIdentity, std::chrono::milliseconds{100LL}, "map"};
  EXPECT_FALSE(filter.is_initialized());
  const std::chrono::system_clock::time_point start_time{std::chrono::system_clock::now()};
  filter.add_reset_event_to_history(state, kCovarianceIdentity, start_time);
  ASSERT_TRUE(filter.is_initialized());

  // In the way we model the ball it is going to reach the ground at this time.
  const auto expected_end_time = start_time + 2000ms;

  const auto increment = 10ms;
  const auto observation_interval = 100ms;
  const float32_t seconds_increment{FloatSeconds{increment}.count()};
  auto current_cycle_milliseconds = 0ms;
  for (auto timestamp = start_time; timestamp <= expected_end_time; timestamp += increment) {
    state(0U) += seconds_increment * state[ConstantAcceleration::States::VELOCITY_X];
    state(1U) += seconds_increment * state[ConstantAcceleration::States::VELOCITY_Y];
    state[ConstantAcceleration::States::VELOCITY_X] +=
      seconds_increment * state[ConstantAcceleration::States::ACCELERATION_X];
    state[ConstantAcceleration::States::VELOCITY_Y] +=
      seconds_increment * state[ConstantAcceleration::States::ACCELERATION_Y];

    current_cycle_milliseconds += increment;
    if (current_cycle_milliseconds >= observation_interval) {
      EXPECT_TRUE(
        filter.add_observation_to_history(
          MeasurementPose{timestamp, Vector2f{state(0U), state(1U)}, Vector2f{1.0F, 1.0F}}));
      current_cycle_milliseconds = 0ms;
    }
  }
  // Quickly check our "simulation" of the ball.
  const auto kRelaxedEpsilon = 0.2F;  // Allow up to 20 cm error.
  EXPECT_NEAR(state(0U), 9.8F * 2.0F, kRelaxedEpsilon);
  EXPECT_NEAR(state(1U), 0.0F, kRelaxedEpsilon);

  // Check that the filter did not get lost
  const auto odom_msg = filter.get_state();
  EXPECT_NEAR(odom_msg.pose.pose.position.x, state(0U), kRelaxedEpsilon);
  EXPECT_NEAR(odom_msg.pose.pose.position.y, state(1U), kRelaxedEpsilon);
  EXPECT_NEAR(
    odom_msg.twist.twist.linear.x,
    state[ConstantAcceleration::States::VELOCITY_X],
    kRelaxedEpsilon);
  EXPECT_NEAR(
    odom_msg.twist.twist.linear.y,
    state[ConstantAcceleration::States::VELOCITY_Y],
    kRelaxedEpsilon);
}
