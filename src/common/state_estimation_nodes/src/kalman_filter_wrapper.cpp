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

#include <state_estimation_nodes/kalman_filter_wrapper.hpp>

#include <common/types.hpp>
#include <measurement_conversion/eigen_utils.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cstdint>
#include <limits>


namespace
{
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::common::types::bool8_t;
using autoware::common::state_vector::variable::X;
using autoware::common::state_vector::variable::Y;
using autoware::common::state_vector::variable::Z;
using autoware::common::state_vector::variable::ROLL;
using autoware::common::state_vector::variable::PITCH;
using autoware::common::state_vector::variable::YAW;
using autoware::common::state_vector::variable::X_VELOCITY;
using autoware::common::state_vector::variable::Y_VELOCITY;
using autoware::common::state_vector::variable::Z_VELOCITY;
using autoware::common::state_vector::variable::ROLL_CHANGE_RATE;
using autoware::common::state_vector::variable::PITCH_CHANGE_RATE;
using autoware::common::state_vector::variable::YAW_CHANGE_RATE;
using autoware::common::state_vector::ConstAccelerationXY32;
using autoware::common::state_vector::ConstAccelerationXYZRPY32;
using autoware::common::state_estimation::DataStorageOrder;
using autoware::common::state_estimation::set_from_matrix;
using autoware::common::state_estimation::slice;

constexpr auto kCovarianceMatrixRows = 6U;
constexpr auto kIndexX = 0U;
constexpr auto kIndexY = kCovarianceMatrixRows + 1U;
constexpr auto kIndexZ = 2U * kCovarianceMatrixRows + 2U;
constexpr auto kDefaultChildFrameId = "base_link";
constexpr auto kCovarianceMatrixRowsSquared = kCovarianceMatrixRows * kCovarianceMatrixRows;
static_assert(
  std::tuple_size<
    geometry_msgs::msg::PoseWithCovariance::_covariance_type>::value ==
  kCovarianceMatrixRowsSquared, "We expect the covariance matrix to have 36 entries.");

/// Convert a chrono timepoint to ROS time.
rclcpp::Time to_ros_time(const std::chrono::system_clock::time_point & time_point)
{
  using std::chrono::duration_cast;
  using std::chrono::nanoseconds;
  return rclcpp::Time{duration_cast<nanoseconds>(time_point.time_since_epoch()).count()};
}

template<typename StateT>
struct OdometryFiller
{
  static_assert(sizeof(StateT) == 0UL, "This struct must be specialized.");
};


template<>
struct OdometryFiller<ConstAccelerationXY32>
{
  template<typename InputStateT>
  static nav_msgs::msg::Odometry fill_odom_msg(
    const InputStateT & state,
    const Eigen::Matrix<float32_t, 6, 6> & covariance_float)
  {
    const Eigen::Matrix<float64_t, 6, 6> covariance = covariance_float.template cast<float64_t>();
    using std::chrono::duration_cast;
    using std::chrono::nanoseconds;
    nav_msgs::msg::Odometry msg{};
    // Fill state.
    msg.pose.pose.position.x = static_cast<float64_t>(state.template at<X>());
    msg.pose.pose.position.y = static_cast<float64_t>(state.template at<Y>());
    msg.pose.pose.position.z = 0.0;
    const auto rotation_around_z = std::atan2(
      static_cast<float64_t>(state.template at<Y_VELOCITY>()),
      static_cast<float64_t>(state.template at<X_VELOCITY>()));
    tf2::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, rotation_around_z);
    msg.pose.pose.orientation = tf2::toMsg(rotation);

    Eigen::Isometry3d transform_to_global_from_ego;
    tf2::fromMsg(msg.pose.pose, transform_to_global_from_ego);
    const Eigen::Vector3d speed_global{
      static_cast<float64_t>(state.template at<X_VELOCITY>()),
      static_cast<float64_t>(state.template at<Y_VELOCITY>()),
      0.0};
    const auto rotate_to_ego_from_global = transform_to_global_from_ego.linear().transpose();
    const auto speed_ego = rotate_to_ego_from_global * speed_global;

    msg.twist.twist.linear.x = speed_ego.x();
    msg.twist.twist.linear.y = speed_ego.y();
    msg.twist.twist.linear.z = speed_ego.z();
    msg.twist.twist.angular.set__x(0.0).set__y(0.0).set__z(0.0);

    const std::array<Eigen::Index, 2> pose_indices{
      state.template index_of<X>(), state.template index_of<Y>()};
    const Eigen::Matrix2d pose_covariance{slice(covariance, pose_indices, pose_indices)};
    set_from_matrix(
      msg.pose.covariance, pose_covariance,
      0, kCovarianceMatrixRows, DataStorageOrder::kRowMajor);

    const std::array<Eigen::Index, 2> speed_indices{
      state.template index_of<X_VELOCITY>(), state.template index_of<Y_VELOCITY>()};
    Eigen::Matrix3d speed_covariance = Eigen::Matrix3d::Zero();
    speed_covariance.topLeftCorner<2, 2>() = slice(covariance, speed_indices, speed_indices);
    speed_covariance =
      rotate_to_ego_from_global * speed_covariance * rotate_to_ego_from_global.transpose();
    set_from_matrix(
      msg.twist.covariance, speed_covariance,
      0, kCovarianceMatrixRows, DataStorageOrder::kRowMajor);

    return msg;
  }
};

template<>
struct OdometryFiller<ConstAccelerationXYZRPY32>
{
  template<typename InputStateT>
  static nav_msgs::msg::Odometry fill_odom_msg(
    const InputStateT & state,
    const Eigen::Matrix<float32_t, 18, 18> & covariance_float)
  {
    const Eigen::Matrix<float32_t, 6, 6> covariance_2d = covariance_float.topLeftCorner<6, 6>();
    auto msg = OdometryFiller<ConstAccelerationXY32>::fill_odom_msg(state, covariance_2d);
    const Eigen::Matrix<float64_t, 18, 18> covariance = covariance_float.cast<float64_t>();
    msg.pose.pose.position.z = static_cast<float64_t>(state.template at<Z>());
    tf2::Quaternion rotation;
    rotation.setRPY(
      static_cast<float64_t>(state.template at<ROLL>()),
      static_cast<float64_t>(state.template at<PITCH>()),
      static_cast<float64_t>(state.template at<YAW>()));
    msg.pose.pose.orientation = tf2::toMsg(rotation);

    Eigen::Isometry3d transform_to_global_from_ego;
    tf2::fromMsg(msg.pose.pose, transform_to_global_from_ego);
    const Eigen::Vector3d speed_global{
      static_cast<float64_t>(state.template at<X_VELOCITY>()),
      static_cast<float64_t>(state.template at<Y_VELOCITY>()),
      static_cast<float64_t>(state.template at<Z_VELOCITY>())};
    const auto rotate_to_ego_from_global = transform_to_global_from_ego.linear().transpose();
    const auto speed_ego = rotate_to_ego_from_global * speed_global;

    msg.twist.twist.linear.x = speed_ego.x();
    msg.twist.twist.linear.y = speed_ego.y();
    msg.twist.twist.linear.z = speed_ego.z();
    msg.twist.twist.angular.set__x(0.0).set__y(0.0).set__z(0.0);

    const std::array<Eigen::Index, 3> pose_indices{
      state.template index_of<X>(), state.template index_of<Y>(), state.template index_of<Z>()};
    const Eigen::Matrix3d pose_covariance{slice(covariance, pose_indices, pose_indices)};
    // For now we are only setting the covariance for the position but not for the rotation.
    set_from_matrix(
      msg.pose.covariance, pose_covariance,
      0, kCovarianceMatrixRows, DataStorageOrder::kRowMajor);

    const std::array<Eigen::Index, 3> speed_indices{
      state.template index_of<X_VELOCITY>(),
      state.template index_of<Y_VELOCITY>(),
      state.template index_of<Z_VELOCITY>()};
    Eigen::Matrix3d speed_covariance = slice(covariance, speed_indices, speed_indices);
    speed_covariance =
      rotate_to_ego_from_global * speed_covariance * rotate_to_ego_from_global.transpose();
    set_from_matrix(
      msg.twist.covariance, speed_covariance,
      0, kCovarianceMatrixRows, DataStorageOrder::kRowMajor);
    return msg;
  }
};

}  // namespace

namespace autoware
{
namespace common
{
namespace state_estimation
{

template<typename FilterT>
nav_msgs::msg::Odometry KalmanFilterWrapper<FilterT>::get_state() const
{
  if (!is_initialized()) {
    throw std::runtime_error("Filter not is_initialized, cannot get state.");
  }
  const auto last_event = m_history.get_last_event();
  const auto state = last_event.stored_state();
  const auto & covariance = last_event.stored_covariance();
  auto msg = OdometryFiller<typename FilterT::State>::fill_odom_msg(state, covariance);
  msg.header.stamp = rclcpp::Time{to_ros_time(m_history.get_last_timestamp())};
  msg.header.frame_id = m_frame_id;
  msg.child_frame_id = kDefaultChildFrameId;
  return msg;
}

template class STATE_ESTIMATION_NODES_PUBLIC
    KalmanFilterWrapper<ConstAccelerationKalmanFilterXY>;
template class STATE_ESTIMATION_NODES_PUBLIC
    KalmanFilterWrapper<ConstAccelerationKalmanFilterXYZRPY>;

}  // namespace state_estimation
}  // namespace common
}  // namespace autoware
