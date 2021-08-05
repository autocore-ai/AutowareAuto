// Copyright 2021 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
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

#include <measurement_conversion/measurement_conversion.hpp>

#include <common/types.hpp>
#include <measurement_conversion/eigen_utils.hpp>
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace
{
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;


constexpr auto kCovarianceMatrixRows = 6U;
constexpr auto kCovarianceMatrixRowsRelativePos = 3U;
constexpr auto kAngleOffset = 3U * kCovarianceMatrixRows + kCovarianceMatrixRows / 2U;
constexpr auto kCovarianceMatrixRowsSquared = kCovarianceMatrixRows * kCovarianceMatrixRows;
constexpr auto kCovarianceMatrixRowsRelativePosSquared =
  kCovarianceMatrixRowsRelativePos * kCovarianceMatrixRowsRelativePos;
static_assert(
  std::tuple_size<
    geometry_msgs::msg::PoseWithCovariance::_covariance_type>::value ==
  kCovarianceMatrixRowsSquared,
  "We expect the PoseWithCovariance covariance matrix to have 36 entries.");
static_assert(
  std::tuple_size<
    autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped::_covariance_type>::value ==
  kCovarianceMatrixRowsRelativePosSquared,
  "We expect the RelativePositionWithCovarianceStamped covariance matrix to have 9 entries.");
}  // namespace

namespace autoware
{
namespace common
{
namespace state_estimation
{

PoseMeasurementXYZRPY64 convert_to<PoseMeasurementXYZRPY64>::from(
  const geometry_msgs::msg::PoseWithCovariance & msg)
{
  using Vector6d = Eigen::Matrix<float64_t, 6, 1>;
  using Matrix6d = Eigen::Matrix<float64_t, 6, 6>;
  float64_t roll{}, pitch{}, yaw{};
  tf2::Quaternion quaternion;
  tf2::fromMsg(msg.pose.orientation, quaternion);
  tf2::Matrix3x3{quaternion}.getRPY(roll, pitch, yaw);
  const Vector6d mean = (Vector6d{} <<
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, roll, pitch, yaw).finished();
  Matrix6d covariance = Matrix6d::Zero();
  const auto & cov = msg.covariance;
  const auto stride = kCovarianceMatrixRows;
  const auto position_start_idx = 0;
  covariance.topLeftCorner<3, 3>() =
    array_to_matrix<3, 3>(cov, position_start_idx, stride, DataStorageOrder::kRowMajor);
  const auto rotation_start_idx{kAngleOffset};
  covariance.bottomRightCorner<3, 3>() =
    array_to_matrix<3, 3>(cov, rotation_start_idx, stride, DataStorageOrder::kRowMajor);
  return PoseMeasurementXYZRPY64{mean, covariance};
}

PoseMeasurementXYZ64 convert_to<PoseMeasurementXYZ64>::from(
  const autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped & msg)
{
  const Eigen::Vector3d mean{msg.position.x, msg.position.y, msg.position.z};
  const auto & cov = msg.covariance;
  const auto start_idx = 0;
  const auto stride = kCovarianceMatrixRowsRelativePos;
  const Eigen::Matrix3d covariance =
    array_to_matrix<3, 3>(cov, start_idx, stride, DataStorageOrder::kRowMajor);
  return PoseMeasurementXYZ64{mean, covariance};
}

}  // namespace state_estimation
}  // namespace common
}  // namespace autoware
