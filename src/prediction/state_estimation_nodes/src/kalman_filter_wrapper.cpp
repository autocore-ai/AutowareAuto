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

#include <state_estimation_nodes/kalman_filter_wrapper.hpp>

#include <common/types.hpp>
#include <kalman_filter/esrcf.hpp>
#include <motion_model/constant_acceleration.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>
#include <state_estimation_nodes/measurement.hpp>
#include <state_estimation_nodes/measurement_typedefs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cstdint>
#include <limits>


namespace
{
constexpr auto kCovarianceMatrixRows = 6U;
constexpr auto kIndexX = 0U;
constexpr auto kIndexY = kCovarianceMatrixRows + 1U;
constexpr auto kCovarianceMatrixRowsSquared = kCovarianceMatrixRows * kCovarianceMatrixRows;
static_assert(
  std::tuple_size<
    geometry_msgs::msg::PoseWithCovariance::_covariance_type>::value ==
  kCovarianceMatrixRowsSquared, "We expect the covariance matrix to have 36 entries.");

/// Convert a chrono timepoint to ros time.
rclcpp::Time to_ros_time(const std::chrono::system_clock::time_point & time_point)
{
  using std::chrono::duration_cast;
  using std::chrono::nanoseconds;
  return rclcpp::Time{duration_cast<nanoseconds>(time_point.time_since_epoch()).count()};
}

/// Get the speed magnitude.
autoware::common::types::float64_t get_speed(
  const autoware::common::types::float32_t x_speed,
  const autoware::common::types::float32_t y_speed) noexcept
{
  return static_cast<autoware::common::types::float64_t>(
    std::sqrt(x_speed * x_speed + y_speed * y_speed));
}

}  // namespace

namespace autoware
{
namespace prediction
{

using motion::motion_model::ConstantAcceleration;
using common::types::float64_t;
using common::types::bool8_t;

template<typename MotionModelT, std::int32_t kNumOfStates, int32_t kProcessNoiseDim>
void KalmanFilterWrapper<MotionModelT, kNumOfStates, kProcessNoiseDim>::add_reset_event_to_history(
  const VectorT<kNumOfStates> & state,
  const SquareMatrixT<kNumOfStates> & initial_covariance_chol,
  const std::chrono::system_clock::time_point & event_timestamp)
{
  m_history->emplace_event(event_timestamp, ResetEvent<FilterT>{state, initial_covariance_chol});
  m_time_grid = SteadyTimeGrid{event_timestamp, m_expected_prediction_period};
}

template<typename MotionModelT, std::int32_t kNumOfStates, int32_t kProcessNoiseDim>
// cppcheck-suppress syntaxError
template<typename MeasurementT>
void KalmanFilterWrapper<MotionModelT, kNumOfStates, kProcessNoiseDim>::add_reset_event_to_history(
  const MeasurementT & measurement)
{
  add_reset_event_to_history(
    measurement.get_values_in_full_state(m_motion_model.get_state()),
    m_initial_covariance_factor,
    measurement.get_acquisition_time());
}

template<typename MotionModelT, std::int32_t kNumOfStates, int32_t kProcessNoiseDim>
common::types::bool8_t  // We cannot use an alias here as Doxygen thinks its a different signature.
KalmanFilterWrapper<MotionModelT, kNumOfStates,
  kProcessNoiseDim>::add_next_temporal_update_to_history()
{
  if (!is_initialized()) {return false;}
  const auto next_prediction_timestamp =
    m_time_grid.get_next_timestamp_after(m_history->get_last_timestamp());
  m_history->emplace_event(next_prediction_timestamp, PredictionEvent{});
  return true;
}

template<typename MotionModelT, std::int32_t kNumOfStates, int32_t kProcessNoiseDim>
// cppcheck-suppress syntaxError
template<typename MeasurementT>
common::types::bool8_t  // We cannot use an alias here as Doxygen thinks its a different signature.
KalmanFilterWrapper<MotionModelT, kNumOfStates, kProcessNoiseDim>::add_observation_to_history(
  const MeasurementT & measurement)
{
  if (!is_initialized()) {return false;}
  m_history->emplace_event(measurement.get_acquisition_time(), measurement);
  return true;
}

template<typename MotionModelT, std::int32_t kNumOfStates, int32_t kProcessNoiseDim>
nav_msgs::msg::Odometry KalmanFilterWrapper<MotionModelT, kNumOfStates,
  kProcessNoiseDim>::get_state() const
{
  static_assert(
    sizeof(MotionModelT) == -1,
    "You have to have a specialization for get_state() function!");
  // We only throw here because otherwise the linter complaints there is no return value.
  throw std::runtime_error("You have to have a specialization for get_state() function!");
}

template<>
nav_msgs::msg::Odometry ConstantAccelerationFilter::get_state() const
{
  using std::chrono::duration_cast;
  using std::chrono::nanoseconds;
  nav_msgs::msg::Odometry msg{};
  if (!is_initialized()) {
    throw std::runtime_error("Filter not is_initialized, cannot get state.");
  }
  const auto last_event = m_history->get_last_event();
  msg.header.stamp = rclcpp::Time{to_ros_time(m_history->get_last_timestamp())};
  msg.header.frame_id = m_frame_id;
  // Fill state.
  const auto state = last_event.stored_state();
  msg.pose.pose.position.x =
    static_cast<float64_t>(state[ConstantAcceleration::States::POSE_X]);
  msg.pose.pose.position.y =
    static_cast<float64_t>(state[ConstantAcceleration::States::POSE_Y]);
  // Odometry message has velocity stored in the local frame of the message. In this case, the car
  // always moves along its orientation vector with the speed along it's "forward" direction being
  // the magnitude of the velocity vector estimated by the filter.
  msg.twist.twist.linear.x = get_speed(
    state[ConstantAcceleration::States::VELOCITY_X],
    state[ConstantAcceleration::States::VELOCITY_Y]);
  msg.twist.twist.linear.y = 0.0;
  msg.twist.twist.linear.z = 0.0;
  msg.twist.twist.angular.set__x(0.0).set__y(0.0).set__z(0.0);

  const auto rotation_around_z = std::atan2(
    static_cast<float64_t>(state[ConstantAcceleration::States::VELOCITY_Y]),
    static_cast<float64_t>(state[ConstantAcceleration::States::VELOCITY_X]));
  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 0.0, rotation_around_z);
  msg.pose.pose.orientation = tf2::toMsg(rotation);

  // Fill covariances.
  const auto & covariance_factor = last_event.stored_covariance_factor();
  const auto covariance = covariance_factor * covariance_factor.transpose();
  msg.pose.covariance[kIndexX] = static_cast<double>(covariance(
      ConstantAcceleration::States::POSE_X, ConstantAcceleration::States::POSE_X));
  msg.pose.covariance[kIndexY] = static_cast<double>(covariance(
      ConstantAcceleration::States::POSE_Y, ConstantAcceleration::States::POSE_Y));
  msg.twist.covariance[kIndexX] = static_cast<double>(covariance(
      ConstantAcceleration::States::VELOCITY_X, ConstantAcceleration::States::VELOCITY_X));
  msg.twist.covariance[kIndexY] = static_cast<double>(covariance(
      ConstantAcceleration::States::VELOCITY_Y, ConstantAcceleration::States::VELOCITY_Y));
  // TODO(igor): do we need elements off the diagonal?
  return msg;
}

/// Explicit class instantiation.
template class KalmanFilterWrapper<ConstantAcceleration, 6, 2>;

template bool8_t KalmanFilterWrapper<ConstantAcceleration, 6, 2>::add_observation_to_history<>(
  const MeasurementPose &);
template bool8_t KalmanFilterWrapper<ConstantAcceleration, 6, 2>::add_observation_to_history<>(
  const MeasurementSpeed &);
template bool8_t KalmanFilterWrapper<ConstantAcceleration, 6, 2>::add_observation_to_history<>(
  const MeasurementPoseAndSpeed &);

//! @cond Doxygen_Suppress Doxygen is confused about explicit template instantiation.
template void KalmanFilterWrapper<ConstantAcceleration, 6, 2>::add_reset_event_to_history<>(
  const MeasurementPose &);
template void KalmanFilterWrapper<ConstantAcceleration, 6, 2>::add_reset_event_to_history<>(
  const MeasurementSpeed &);
template void KalmanFilterWrapper<ConstantAcceleration, 6, 2>::add_reset_event_to_history<>(
  const MeasurementPoseAndSpeed &);
//! @endcond

}  // namespace prediction
}  // namespace autoware
