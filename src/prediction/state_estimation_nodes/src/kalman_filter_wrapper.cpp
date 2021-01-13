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

#include <state_estimation_nodes/kalman_filter_wrapper.hpp>

#include <common/types.hpp>
#include <kalman_filter/esrcf.hpp>
#include <motion_model/constant_acceleration.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>
#include <state_estimation_nodes/measurement.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <limits>
#include <cstdint>
#include <algorithm>


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

}  // namespace

namespace autoware
{
namespace prediction
{

using motion::motion_model::ConstantAcceleration;
using common::types::float64_t;
using common::types::bool8_t;

template<typename MotionModelT, std::int32_t kNumOfStates, int32_t kProcessNoiseDim>
void KalmanFilterWrapper<MotionModelT, kNumOfStates, kProcessNoiseDim>::reset(
  const VectorT<kNumOfStates> & state,
  const SquareMatrixT<kNumOfStates> & initial_covariance_chol,
  const MeasurementBasedTime & event_timestamp,
  const GlobalTime & time_of_event_occurance)
{
  m_ekf->reset(state, initial_covariance_chol);
  m_time_keeper = MeasurementBasedTimeKeeper{time_of_event_occurance, event_timestamp};
  m_ekf_initialized = true;
}

template<typename MotionModelT, std::int32_t kNumOfStates, int32_t kProcessNoiseDim>
// cppcheck-suppress syntaxError
template<typename MeasurementT>
void KalmanFilterWrapper<MotionModelT, kNumOfStates, kProcessNoiseDim>::reset(
  const MeasurementT & measurement,
  const GlobalTime & time_of_event_occurance)
{
  reset(
    measurement.get_values_in_full_state(m_motion_model.get_state()),
    m_initial_covariance_factor,
    measurement.get_acquisition_time(),
    time_of_event_occurance);
}

template<typename MotionModelT, std::int32_t kNumOfStates, int32_t kProcessNoiseDim>
// cppcheck-suppress syntaxError
template<TimeReferenceFrame kTimeReferenceFrame>
common::types::bool8_t  // We cannot use an alias here as Doxygen thinks its a different signature.
KalmanFilterWrapper<MotionModelT, kNumOfStates, kProcessNoiseDim>::temporal_update(
  const Time<kTimeReferenceFrame> & time_of_update)
{
  if (!is_initialized()) {return false;}
  const auto dt = m_time_keeper.time_since_last_temporal_update(time_of_update);
  if (dt <= std::chrono::nanoseconds{0LL}) {return false;}
  m_ekf->temporal_update(dt);
  m_time_keeper.increment_last_temporal_update_time(dt);
  return true;
}

template<typename MotionModelT, std::int32_t kNumOfStates, int32_t kProcessNoiseDim>
// cppcheck-suppress syntaxError
template<typename MeasurementT>
common::types::bool8_t  // We cannot use an alias here as Doxygen thinks its a different signature.
KalmanFilterWrapper<MotionModelT, kNumOfStates, kProcessNoiseDim>::observation_update(
  const GlobalTime & global_time_of_message_received,
  const MeasurementT & measurement)
{
  if (!is_initialized()) {
    // TODO(igor): this is not strictly correct, but should be good enough. If we get an observation
    // and the filter is not set to any state, we reset it. In this case we assume that this
    // measurement actually is statefull (not purely differential) and we ignore the variance of
    // this measurement.
    reset(measurement, global_time_of_message_received);
    return true;
  }
  // TODO(igor): I am not sure this should be calling latest_timestamp() as if we had a prediction
  // step with a later timestamp than our measurement here we will discard the measurement. Should
  // be just compare to the latest measurement time stored in the time keeper?
  if (m_time_keeper.latest_timestamp() > measurement.get_acquisition_time()) {return false;}
  if (!temporal_update(measurement.get_acquisition_time())) {return false;}
  // TODO(igor): I see a couple of ways to check mahalanobis distance in case the measurement does
  // not cover the full state. Here I upscale it to the full state, copying the values of the
  // current state for ones missing in the observation. We can alternatively apply the H matrix and
  // only compute the distance in the measurement world. Don't really know which one is best here.
  if (!passes_mahalanobis_gate(
      measurement.get_values_in_full_state(m_motion_model.get_state()),
      m_motion_model.get_state(),
      m_ekf->get_covariance())) {return false;}
  m_ekf->observation_update(
    measurement.get_values(),
    MeasurementT::template get_observation_to_state_mapping<kNumOfStates>(),
    measurement.get_variances());
  m_time_keeper.update_with_measurement(global_time_of_message_received, measurement);
  return true;
}

template<typename MotionModelT, std::int32_t kNumOfStates, int32_t kProcessNoiseDim>
bool8_t KalmanFilterWrapper<MotionModelT, kNumOfStates,
  kProcessNoiseDim>::passes_mahalanobis_gate(
  const VectorT<kNumOfStates> & sample,
  const VectorT<kNumOfStates> & mean,
  const SquareMatrixT<kNumOfStates> & covariance_factor) const
{
  // This is equivalent to the squared Mahalanobis distance of the form: diff.T * C.inv() * diff
  // Instead of the covariance matrix C we have its lower-triangular factor L, such that C = L * L.T
  // squared_mahalanobis_distance = diff.T * C.inv() * diff
  // = diff.T * (L * L.T).inv() * diff
  // = diff.T * L.T.inv() * L.inv() * diff
  // = (L.inv() * diff).T * (L.inv() * diff)
  // this allows us to efficiently find the squared Mahalanobis distance using (L.inv() * diff),
  // which can be found as a solution to: L * x = diff.
  const auto diff = sample - mean;
  const auto squared_threshold = m_mahalanobis_threshold * m_mahalanobis_threshold;
  const auto x = covariance_factor.ldlt().solve(diff);
  return x.transpose() * x < squared_threshold;
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
  msg.header.stamp = rclcpp::Time{to_ros_time(m_time_keeper.latest_timestamp())};
  msg.header.frame_id = m_frame_id;
  // Fill state.
  msg.pose.pose.position.x =
    static_cast<float64_t>(m_motion_model[ConstantAcceleration::States::POSE_X]);
  msg.pose.pose.position.y =
    static_cast<float64_t>(m_motion_model[ConstantAcceleration::States::POSE_Y]);
  msg.twist.twist.linear.x =
    static_cast<float64_t>(m_motion_model[ConstantAcceleration::States::VELOCITY_X]);
  msg.twist.twist.linear.y =
    static_cast<float64_t>(m_motion_model[ConstantAcceleration::States::VELOCITY_Y]);

  const auto rotation_around_z = std::atan2(msg.twist.twist.linear.y, msg.twist.twist.linear.x);
  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 0.0, rotation_around_z);
  msg.pose.pose.orientation = tf2::toMsg(rotation);

  // Fill covariances.
  const auto & covariance_factor = m_ekf->get_covariance();
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

/// Excplicit class instantiation.
template class KalmanFilterWrapper<ConstantAcceleration, 6, 2>;

using MeasurementPose = Measurement<common::types::float32_t,
    ConstantAcceleration::States::POSE_X,
    ConstantAcceleration::States::POSE_Y>;

using MeasurementPoseAndSpeed = Measurement<common::types::float32_t,
    ConstantAcceleration::States::POSE_X,
    ConstantAcceleration::States::POSE_Y,
    ConstantAcceleration::States::VELOCITY_X,
    ConstantAcceleration::States::VELOCITY_Y>;

using MeasurementSpeed = Measurement<common::types::float32_t,
    ConstantAcceleration::States::VELOCITY_X,
    ConstantAcceleration::States::VELOCITY_Y>;

template bool8_t KalmanFilterWrapper<ConstantAcceleration, 6, 2>::observation_update<>(
  const GlobalTime &, const MeasurementPose &);
template bool8_t KalmanFilterWrapper<ConstantAcceleration, 6, 2>::observation_update<>(
  const GlobalTime &, const MeasurementSpeed &);
template bool8_t KalmanFilterWrapper<ConstantAcceleration, 6, 2>::observation_update<>(
  const GlobalTime &, const MeasurementPoseAndSpeed &);

template bool8_t KalmanFilterWrapper<ConstantAcceleration, 6, 2>::temporal_update<>(
  const GlobalTime &);
template bool8_t KalmanFilterWrapper<ConstantAcceleration, 6, 2>::temporal_update<>(
  const MeasurementBasedTime &);

}  // namespace prediction
}  // namespace autoware
