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

#include <state_estimation_nodes/kalman_filter_wrapper.hpp>

#include <limits>

using autoware::common::types::float64_t;
using autoware::common::types::float32_t;

namespace
{
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

using autoware::motion::motion_model::ConstantAcceleration;
using autoware::prediction::ConstantAccelerationFilter;
using autoware::prediction::Measurement;
using autoware::prediction::MeasurementBasedTime;
using autoware::prediction::GlobalTime;

using MeasurementPose = Measurement<float32_t,
    ConstantAcceleration::States::POSE_X,
    ConstantAcceleration::States::POSE_Y>;

using MeasurementPoseAndSpeed = Measurement<float32_t,
    ConstantAcceleration::States::POSE_X,
    ConstantAcceleration::States::POSE_Y,
    ConstantAcceleration::States::VELOCITY_X,
    ConstantAcceleration::States::VELOCITY_Y>;

using MeasurementSpeed = Measurement<float32_t,
    ConstantAcceleration::States::VELOCITY_X,
    ConstantAcceleration::States::VELOCITY_Y>;

/// \test Creating an empty filter and checking that everything is zero.
TEST(KalmanFilterWrapperTest, create_empty) {
  ConstantAccelerationFilter filter{
    kCovarianceIdentity, kNoiseIdentity, std::chrono::milliseconds{100LL}, "map"};
  EXPECT_FALSE(filter.is_initialized());
  EXPECT_THROW(filter.get_state(), std::runtime_error);
}

/// \test Before the filter is initialized we don't want to accept some updates.
TEST(KalmanFilterWrapperTest, ignore_everything_before_initialization) {
  ConstantAccelerationFilter filter{
    kCovarianceIdentity, kNoiseIdentity, std::chrono::milliseconds{100LL}, "map"};
  const auto now = GlobalTime{std::chrono::system_clock::now()};
  EXPECT_FALSE(filter.is_initialized());
  EXPECT_FALSE(filter.temporal_update(now));
}

/// \test Initialize filter with a measurement.
TEST(KalmanFilterWrapperTest, initialize) {
  ConstantAccelerationFilter filter{
    kCovarianceIdentity, kNoiseIdentity, std::chrono::milliseconds{100LL}, "map"};
  const auto now_measurement = MeasurementBasedTime{std::chrono::system_clock::now()};
  const auto now_global = GlobalTime{std::chrono::system_clock::now()};
  EXPECT_FALSE(filter.is_initialized());
  MeasurementPoseAndSpeed measurement{now_measurement, {42.0F, 42.0F, 42.0F, 42.0F}};
  EXPECT_TRUE(filter.observation_update(now_global, measurement));
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
  const auto now_measurement = MeasurementBasedTime{std::chrono::system_clock::now()};
  const auto now_global = GlobalTime{std::chrono::system_clock::now()};
  EXPECT_FALSE(filter.is_initialized());
  MeasurementPoseAndSpeed measurement{now_measurement, {42.0F, 42.0F, 42.0F, 42.0F}};
  EXPECT_TRUE(filter.observation_update(now_global, measurement));
  const auto past{now_measurement - std::chrono::milliseconds{2LL}};
  const auto future{now_global + std::chrono::milliseconds{2LL}};
  MeasurementPoseAndSpeed past_measurement{past, {0.0F, 0.0F, 0.0F, 0.0F}};
  // Update that comes later than now, but carries a measurement from the past.
  EXPECT_FALSE(filter.observation_update(future, past_measurement));
}

/// \test Ignore measurements that don't pass the Mahalanobis threshold.
TEST(KalmanFilterWrapperTest, ignore_far_away_measurements) {
  using namespace std::chrono_literals;
  const float32_t mahalanobis_threshold = 1.0F;
  ConstantAccelerationFilter filter{
    kCovarianceIdentity, kNoiseIdentity, std::chrono::milliseconds{100LL}, "map",
    mahalanobis_threshold};
  EXPECT_FALSE(filter.is_initialized());
  MeasurementBasedTime timestamp{std::chrono::system_clock::now()};
  GlobalTime time_measurement_received{std::chrono::system_clock::now()};
  filter.reset(
    Eigen::Matrix<float32_t, 6, 1>::Zero(),
    kCovarianceIdentity,
    timestamp,
    time_measurement_received);
  ASSERT_TRUE(filter.is_initialized());
  // Check that nothing else forbids us from updating the state.
  timestamp += 10ms;
  time_measurement_received += 10ms;
  EXPECT_TRUE(
    filter.observation_update(
      time_measurement_received,
      MeasurementPose{timestamp, {0.0F, 0.0F}, {1.0F, 1.0F}}));
  // Check that measurements don't pass the Mahalanobis threshold.
  EXPECT_FALSE(
    filter.observation_update(
      time_measurement_received,
      MeasurementPose{timestamp, {10.0F, 0.0F}, {1.0F, 1.0F}}));
  EXPECT_FALSE(
    filter.observation_update(
      time_measurement_received,
      MeasurementPose{timestamp, {0.0F, 10.0F}, {1.0F, 1.0F}}));
  EXPECT_FALSE(
    filter.observation_update(
      time_measurement_received,
      MeasurementSpeed{timestamp, {10.0F, 0.0F}, {1.0F, 1.0F}}));
  EXPECT_FALSE(
    filter.observation_update(
      time_measurement_received,
      MeasurementSpeed{timestamp, {0.0F, 10.0F}, {1.0F, 1.0F}}));
}

/// \test Covariance of a static object grows without new observations.
TEST(KalmanFilterWrapperTest, covariance_grows_with_time) {
  ConstantAccelerationFilter filter{
    kCovarianceIdentity, kNoiseIdentity, std::chrono::milliseconds{100LL}, "map"};
  EXPECT_FALSE(filter.is_initialized());
  const auto timestamp = MeasurementBasedTime{std::chrono::system_clock::now()};
  const auto received_message_time = GlobalTime{std::chrono::system_clock::now()};
  MeasurementPoseAndSpeed measurement{timestamp, {42.0F, 42.0F, 0.0F, 0.0F}};
  EXPECT_TRUE(filter.observation_update(received_message_time, measurement));
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
  filter.temporal_update(received_message_time + 100ms);
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
  MeasurementBasedTime timestamp{std::chrono::system_clock::now()};
  GlobalTime time_measurement_received{std::chrono::system_clock::now()};
  EXPECT_TRUE(
    filter.observation_update(
      time_measurement_received, MeasurementPose{timestamp, {42.0F, 42.0F}}));
  const auto initial_state = filter.get_state();
  for (int i = 0; i < 10; ++i) {
    timestamp += 100ms;
    time_measurement_received += 100ms;
    EXPECT_TRUE(
      filter.observation_update(
        time_measurement_received, MeasurementPose{timestamp, {42.0F, 42.0F}, {1.0F, 1.0F}}));
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
/// The ball moves at a prabola starting at (0, 0):
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

  const float32_t g = -9.8F;  // m/s^2.
  Eigen::Matrix<float32_t, 6, 1> state;
  const float32_t initial_speed = 9.8F;  // m/s
  state << 0.0F, 0.0F, initial_speed, initial_speed, 0.0F, g;

  ConstantAccelerationFilter filter{
    kCovarianceIdentity, kNoiseIdentity, std::chrono::milliseconds{100LL}, "map"};
  EXPECT_FALSE(filter.is_initialized());
  const MeasurementBasedTime start_time{std::chrono::system_clock::now()};
  const GlobalTime start_time_global{std::chrono::system_clock::now()};
  filter.reset(state, kCovarianceIdentity, start_time, start_time_global);
  EXPECT_TRUE(filter.is_initialized());

  // In the way we model the ball it is going to reach the ground at this time.
  const auto expected_end_time = start_time + 2000ms;

  const auto increment = 10ms;
  const auto observation_interval = 100ms;
  const float32_t seconds_increment{FloatSeconds{increment}.count()};
  auto current_cycle_milliseconds = 0ms;
  for (auto timestamp = start_time; timestamp <= expected_end_time; timestamp += increment) {
    state.x() += seconds_increment * state[ConstantAcceleration::States::VELOCITY_X];
    state.y() += seconds_increment * state[ConstantAcceleration::States::VELOCITY_Y];
    state[ConstantAcceleration::States::VELOCITY_X] +=
      seconds_increment * state[ConstantAcceleration::States::ACCELERATION_X];
    state[ConstantAcceleration::States::VELOCITY_Y] +=
      seconds_increment * state[ConstantAcceleration::States::ACCELERATION_Y];

    current_cycle_milliseconds += increment;
    if (current_cycle_milliseconds >= observation_interval) {
      filter.observation_update(
        GlobalTime{timestamp},
        MeasurementPose{timestamp, {state.x(), state.y()}, {1.0F, 1.0F}});
      current_cycle_milliseconds = 0ms;
    }
  }
  // Quickly check our "simulation" of the ball.
  const auto kRelaxedEpsilon = 0.2F;  // Allow up to 20 cm error.
  EXPECT_NEAR(state.x(), 9.8F * 2.0F, kRelaxedEpsilon);
  EXPECT_NEAR(state.y(), 0.0F, kRelaxedEpsilon);

  // Check that the filter did not get lost
  const auto odom_msg = filter.get_state();
  EXPECT_NEAR(odom_msg.pose.pose.position.x, state.x(), kRelaxedEpsilon);
  EXPECT_NEAR(odom_msg.pose.pose.position.y, state.y(), kRelaxedEpsilon);
  EXPECT_NEAR(
    odom_msg.twist.twist.linear.x,
    state[ConstantAcceleration::States::VELOCITY_X],
    kRelaxedEpsilon);
  EXPECT_NEAR(
    odom_msg.twist.twist.linear.y,
    state[ConstantAcceleration::States::VELOCITY_Y],
    kRelaxedEpsilon);
}
