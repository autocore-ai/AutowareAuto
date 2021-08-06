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

#include <common/types.hpp>
#include <motion_model/linear_motion_model.hpp>
#include <motion_model/stationary_motion_model.hpp>
#include <state_estimation/kalman_filter/kalman_filter.hpp>
#include <state_estimation/measurement/linear_measurement.hpp>
#include <state_estimation/noise_model/wiener_noise.hpp>

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <tuple>

using autoware::common::state_vector::variable::X;
using autoware::common::state_vector::variable::Y;
using autoware::common::state_vector::variable::YAW;
using autoware::common::state_vector::variable::X_VELOCITY;
using autoware::common::state_vector::variable::Y_VELOCITY;
using autoware::common::state_vector::variable::YAW_CHANGE_RATE;
using autoware::common::state_vector::variable::X_ACCELERATION;
using autoware::common::state_vector::variable::Y_ACCELERATION;
using autoware::common::state_vector::variable::YAW_CHANGE_ACCELERATION;
using autoware::common::state_vector::Variable;
using autoware::common::state_vector::FloatState;
using autoware::common::state_estimation::LinearMeasurement;
using autoware::common::state_estimation::KalmanFilter;
using autoware::common::state_estimation::NoiseInterface;
using autoware::common::state_estimation::WienerNoise;
using autoware::common::state_estimation::make_kalman_filter;
using autoware::common::state_estimation::make_correction_only_kalman_filter;
using autoware::common::motion_model::LinearMotionModel;
using autoware::common::motion_model::StationaryMotionModel;
using autoware::common::state_vector::ConstAccelerationXY32;
using autoware::common::state_vector::ConstAccelerationXYYaw32;
using autoware::common::types::float32_t;

/// @test Test that a filter can be created and reset and is in a valid state throughout this.
TEST(TestKalmanFilter, CreateAndReset) {
  using State = LinearMotionModel<ConstAccelerationXY32>::State;
  using Matrix = State::Matrix;
  LinearMotionModel<ConstAccelerationXY32> motion_model{};
  WienerNoise<ConstAccelerationXY32> noise_model{{1.0F, 1.0F}};
  auto kf = make_kalman_filter(
    motion_model, noise_model, State{}, {{1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F}});
  EXPECT_TRUE(kf.state().vector().isApproxToConstant(0.0F));
  EXPECT_TRUE(kf.covariance().isApprox(Matrix::Identity()));
  kf.reset(State{State::Vector::Ones()}, Matrix::Ones());
  EXPECT_TRUE(kf.state().vector().isApproxToConstant(1.0F));
  EXPECT_TRUE(kf.covariance().isApproxToConstant(1.0F));
}

/// @test Test that predictions without measurements always increase uncertainty.
TEST(TestKalmanFilter, PredictionsIncreaseUncertainty) {
  using State = LinearMotionModel<ConstAccelerationXY32>::State;
  using Matrix = State::Matrix;
  LinearMotionModel<ConstAccelerationXY32> motion_model{};
  WienerNoise<ConstAccelerationXY32> noise_model{{1.0F, 1.0F}};
  auto kf = make_kalman_filter(motion_model, noise_model, State{}, Matrix::Identity());
  EXPECT_TRUE(kf.state().vector().isApproxToConstant(0.0F));
  auto covariance = kf.covariance();
  for (int i = 0; i < 20; ++i) {
    kf.predict(std::chrono::milliseconds{100LL});
    const auto diff = kf.covariance() - covariance;
    EXPECT_TRUE((diff.diagonal().array() > 0.0F).all()) <<
      "New covariance: \n" << kf.covariance() <<
      "\nis not bigger than old one:\n" << covariance;
    covariance = kf.covariance();
  }
}

/// @test Test that we can track a static object measuring its full state.
TEST(TestKalmanFilter, TrackStaticObjectWithDirectMeasurements) {
  using State = LinearMotionModel<ConstAccelerationXY32>::State;
  using Matrix = State::Matrix;
  using MeasurementState = FloatState<
    X, X_VELOCITY, X_ACCELERATION,
    Y, Y_VELOCITY, Y_ACCELERATION>;
  LinearMotionModel<ConstAccelerationXY32> motion_model{};
  WienerNoise<ConstAccelerationXY32> noise_model{{1.0F, 1.0F}};
  auto kf = make_kalman_filter(motion_model, noise_model, State{}, Matrix::Identity());
  EXPECT_TRUE(kf.state().vector().isApproxToConstant(0.0F));
  auto covariance = kf.covariance();
  const MeasurementState::Vector stddev = MeasurementState::Vector::Constant(0.1F);
  for (int i = 0; i < 10; ++i) {
    kf.predict(std::chrono::milliseconds{100LL});
    kf.correct(
      LinearMeasurement<MeasurementState>::create_with_stddev(
        MeasurementState::Vector::Zero(),
        stddev));
    EXPECT_TRUE(kf.state().vector().isApproxToConstant(0.0F)) <<
      "Vector " << kf.state().vector().transpose() << " is not a zero vector.";
    const auto covariance_difference = kf.covariance() - covariance;
    EXPECT_TRUE((covariance_difference.diagonal().array() < 0.0F).all()) <<
      "New covariance: \n" << kf.covariance() <<
      "\nis not smaller than old one:\n" << covariance;
  }
}

/// @test Test that we can track a static object measuring only its partial state.
TEST(TestKalmanFilter, TrackStaticObjectHiddenState) {
  using State = LinearMotionModel<ConstAccelerationXY32>::State;
  using Matrix = State::Matrix;
  using MeasurementState = FloatState<X, Y>;
  LinearMotionModel<ConstAccelerationXY32> motion_model{};
  WienerNoise<ConstAccelerationXY32> noise_model{{1.0F, 1.0F}};
  auto kf = make_kalman_filter(motion_model, noise_model, State{}, 10.0F * Matrix::Identity());
  EXPECT_TRUE(kf.state().vector().isApproxToConstant(0.0F));
  auto covariance = kf.covariance();
  const MeasurementState::Vector stddev = MeasurementState::Vector::Constant(0.1F);
  for (int i = 0; i < 10; ++i) {
    kf.predict(std::chrono::milliseconds{100LL});
    kf.correct(
      LinearMeasurement<MeasurementState>::create_with_stddev(
        MeasurementState::Vector::Zero(),
        stddev));
    EXPECT_TRUE(kf.state().vector().isApproxToConstant(0.0F)) <<
      "Vector " << kf.state().vector().transpose() << " is not a zero vector.";
  }
  // Perform this check only in the end as the covariance of the unobserved variables _can_ grow
  // initially but will eventually fall below the original values.
  const auto covariance_difference = kf.covariance() - covariance;
  EXPECT_TRUE((covariance_difference.diagonal().array() < 0.0F).all()) <<
    "New covariance: \n" << kf.covariance() <<
    "\nis not smaller than old one:\n" << covariance;
}


/// @test Test that we can track a moving object measuring part of its state.
///
/// @details The object is assumed to move at a straight line, changing its orientation with
///     constant angular velocity. All the variables, X, Y, YAW are changing independently.
TEST(TestKalmanFilter, TrackMovingObject) {
  using State = LinearMotionModel<ConstAccelerationXYYaw32>::State;
  using Matrix = State::Matrix;
  using MeasurementState = FloatState<X, Y, YAW>;
  LinearMotionModel<ConstAccelerationXYYaw32> motion_model{};
  WienerNoise<ConstAccelerationXYYaw32> noise_model{{1.0F, 1.0F, 1.0F}};
  const auto initial_covariance = Matrix::Identity();
  auto kf = make_kalman_filter(motion_model, noise_model, State{}, initial_covariance);
  EXPECT_TRUE(kf.state().vector().isApproxToConstant(0.0F));
  const auto speed = 10.0F;  // m/s
  const std::chrono::milliseconds dt{100LL};
  const std::chrono::seconds total_time{5};
  const MeasurementState::Vector stddev = MeasurementState::Vector::Constant(0.1F);
  for (auto t = dt; t <= total_time; t += dt) {
    const auto float_seconds = std::chrono::duration<float32_t>{t}.count();
    const auto travelled_distance = float_seconds * speed;
    const auto observation = travelled_distance * MeasurementState::Vector::Ones();
    kf.predict(std::chrono::milliseconds{100LL});
    kf.correct(
      LinearMeasurement<MeasurementState>::create_with_stddev(observation, stddev));
  }
  const auto total_float_seconds = std::chrono::duration<float32_t>{total_time}.count();
  const float32_t eps = 0.001F;
  EXPECT_NEAR(kf.state().at<X>(), total_float_seconds * speed, eps);
  EXPECT_NEAR(kf.state().at<Y>(), total_float_seconds * speed, eps);
  EXPECT_NEAR(
    kf.state().at<YAW>(),
    autoware::common::helper_functions::wrap_angle(total_float_seconds * speed), eps);
  EXPECT_NEAR(kf.state().at<X_VELOCITY>(), speed, eps);
  EXPECT_NEAR(kf.state().at<Y_VELOCITY>(), speed, eps);
  EXPECT_NEAR(kf.state().at<YAW_CHANGE_RATE>(), speed, eps);
  EXPECT_NEAR(kf.state().at<X_ACCELERATION>(), 0.0F, eps);
  EXPECT_NEAR(kf.state().at<Y_ACCELERATION>(), 0.0F, eps);
  EXPECT_NEAR(kf.state().at<YAW_CHANGE_ACCELERATION>(), 0.0F, eps);
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
TEST(TestKalmanFilter, TrackThrownBall) {
  using namespace std::chrono_literals;
  using FloatSeconds = std::chrono::duration<float32_t>;
  using State = LinearMotionModel<ConstAccelerationXY32>::State;
  using Matrix = State::Matrix;
  using MeasurementState = FloatState<X, Y>;
  LinearMotionModel<ConstAccelerationXY32> motion_model{};
  WienerNoise<ConstAccelerationXY32> noise_model{{1.0F, 1.0F}};

  const float32_t g = -9.80665F;  // m/s^2.
  const float32_t initial_speed = 9.80665F;  // m/s
  State initial_state{};
  initial_state.at<X_VELOCITY>() = initial_speed;
  initial_state.at<Y_VELOCITY>() = initial_speed;
  initial_state.at<Y_ACCELERATION>() = g;
  const auto initial_covariance = Matrix::Identity();
  auto kf = make_kalman_filter(
    motion_model,
    noise_model,
    initial_state,
    initial_covariance);
  // In the way we model the ball it is going to reach the ground at this time.
  const std::chrono::system_clock::time_point start_time{std::chrono::system_clock::now()};
  const auto duration = 2000ms;
  const auto expected_end_time = start_time + duration;

  const auto increment = 10ms;
  const float32_t seconds_increment{FloatSeconds{increment}.count()};
  State expected_state{initial_state};
  const MeasurementState::Vector stddev = MeasurementState::Vector::Constant(0.1F);
  for (auto timestamp = start_time; timestamp <= expected_end_time; timestamp += increment) {
    expected_state.at<X>() += seconds_increment * expected_state.at<X_VELOCITY>();
    expected_state.at<Y>() += seconds_increment * expected_state.at<Y_VELOCITY>();
    expected_state.at<X_VELOCITY>() += seconds_increment * expected_state.at<X_ACCELERATION>();
    expected_state.at<Y_VELOCITY>() += seconds_increment * expected_state.at<Y_ACCELERATION>();

    kf.predict(increment);
    kf.correct(
      LinearMeasurement<MeasurementState>::create_with_stddev(
        MeasurementState::Vector{expected_state.at<X>(), expected_state.at<Y>()},
        stddev));
  }
  // Quickly check our "simulation" of the ball.
  const auto duration_seconds = std::chrono::duration<float32_t>{duration}.count();
  const auto kRelaxedEpsilon = 0.2F;  // Allow up to 20 cm error.
  EXPECT_NEAR(expected_state.at<X>(), initial_speed * duration_seconds, kRelaxedEpsilon);
  EXPECT_NEAR(expected_state.at<Y>(), 0.0F, kRelaxedEpsilon);

  EXPECT_NEAR(expected_state.at<X>(), kf.state().at<X>(), 0.001F);
  EXPECT_NEAR(expected_state.at<Y>(), kf.state().at<Y>(), 0.001F);
  EXPECT_NEAR(expected_state.at<X_VELOCITY>(), kf.state().at<X_VELOCITY>(), kRelaxedEpsilon);
  EXPECT_NEAR(expected_state.at<Y_VELOCITY>(), kf.state().at<Y_VELOCITY>(), kRelaxedEpsilon);
  EXPECT_NEAR(expected_state.at<X_ACCELERATION>(), 0.0F, kRelaxedEpsilon);
  EXPECT_NEAR(expected_state.at<Y_ACCELERATION>(), g, kRelaxedEpsilon);
}

/// \test Check that Kalman Filter can be used for classification. In this example of a traffic
/// light state.
TEST(KalmanFilterWrapperTest, TrafficLightState) {
  struct RED : public Variable {};
  struct GREEN : public Variable {};
  struct ORANGE : public Variable {};

  using TrafficLightState = FloatState<RED, GREEN, ORANGE>;
  const auto uniform_probability = 1.0F / TrafficLightState::size();
  const auto uniform_vector = TrafficLightState::Vector::Constant(uniform_probability);
  TrafficLightState traffic_light_state{uniform_vector};
  EXPECT_FLOAT_EQ(traffic_light_state.at<RED>(), uniform_probability);
  EXPECT_FLOAT_EQ(traffic_light_state.at<GREEN>(), uniform_probability);
  EXPECT_FLOAT_EQ(traffic_light_state.at<ORANGE>(), uniform_probability);

  auto filter = make_correction_only_kalman_filter(
    traffic_light_state, TrafficLightState::Matrix::Identity());

  // Observe a red light.
  const auto red_light_measurement = LinearMeasurement<TrafficLightState>::create_with_stddev(
    {1.0F, 0.0F, 0.0F},
    {0.1F, 0.1F, 0.1F});
  EXPECT_FLOAT_EQ(filter.state().at<RED>(), uniform_probability);
  EXPECT_FLOAT_EQ(filter.state().at<GREEN>(), uniform_probability);
  EXPECT_FLOAT_EQ(filter.state().at<ORANGE>(), uniform_probability);
  filter.correct(red_light_measurement);
  const auto epsilon = 0.1F;  // An epsilon chosen to match the observation covariance.
  EXPECT_GT(filter.state().at<RED>(), uniform_probability + epsilon);
  EXPECT_LT(filter.state().at<GREEN>(), uniform_probability - epsilon);
  EXPECT_LT(filter.state().at<ORANGE>(), uniform_probability - epsilon);

  // Observe a green light.
  const auto green_light_measurement = LinearMeasurement<TrafficLightState>::create_with_stddev(
    {0.0F, 1.0F, 0.0F},
    {0.1F, 0.1F, 0.1F});
  filter.correct(green_light_measurement);
  // We have now seen both green and red once, so it should be roughly 50/50 probability for either
  // of these states of the traffic light, but definitely not orange.
  EXPECT_NEAR(filter.state().at<RED>(), 0.5F, epsilon);
  EXPECT_NEAR(filter.state().at<GREEN>(), 0.5F, epsilon);
  EXPECT_NEAR(filter.state().at<ORANGE>(), 0.0F, epsilon);
}
