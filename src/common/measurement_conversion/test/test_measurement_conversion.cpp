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

#include <gtest/gtest.h>

#include <common/types.hpp>
#include <measurement_conversion/measurement_conversion.hpp>

using autoware::common::state_estimation::Stamped;
using autoware::common::state_estimation::PoseMeasurementXYZ64;
using autoware::common::state_estimation::PoseMeasurementXYZRPY64;
using autoware::common::state_estimation::convert_to;
using autoware::common::state_vector::variable::X;
using autoware::common::state_vector::variable::Y;
using autoware::common::state_vector::variable::Z;
using autoware::common::state_vector::variable::ROLL;
using autoware::common::state_vector::variable::PITCH;
using autoware::common::state_vector::variable::YAW;
using autoware::common::types::float32_t;

namespace
{

std_msgs::msg::Header create_header() noexcept
{
  std_msgs::msg::Header msg{};
  msg.frame_id = "map";
  msg.stamp.sec = 42;
  msg.stamp.nanosec = 0;
  return msg;
}

geometry_msgs::msg::PoseWithCovarianceStamped create_pose_msg() noexcept
{
  geometry_msgs::msg::PoseWithCovarianceStamped msg{};
  msg.header = create_header();
  msg.pose.pose.position.x = 42.0;
  msg.pose.pose.position.y = 23.0;
  msg.pose.pose.position.z = 1.0;
  // Rotation around z axis by 90 degrees.
  msg.pose.pose.orientation.x = 0.0;
  msg.pose.pose.orientation.y = 0.0;
  msg.pose.pose.orientation.z = 0.7071068;
  msg.pose.pose.orientation.w = 0.7071068;
  msg.pose.covariance[0] = 1.0;
  msg.pose.covariance[7] = 2.0;
  msg.pose.covariance[14] = 3.0;
  msg.pose.covariance[21] = 4.0;
  msg.pose.covariance[28] = 5.0;
  msg.pose.covariance[35] = 6.0;
  return msg;
}

autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped create_relative_pos_msg() noexcept
{
  const auto pose_msg = create_pose_msg();
  autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped msg{};
  msg.child_frame_id = "base_link";
  msg.header = create_header();
  msg.position.x = pose_msg.pose.pose.position.x;
  msg.position.y = pose_msg.pose.pose.position.y;
  msg.position.z = pose_msg.pose.pose.position.z;
  msg.covariance[0] = 1.0;
  msg.covariance[4] = 2.0;
  msg.covariance[8] = 3.0;
  return msg;
}

}  // namespace

/// \test Create a measurement from a relative pose.
TEST(MeasurementConversionTest, RelativePose) {
  const auto msg = create_relative_pos_msg();
  const auto measurement = convert_to<Stamped<PoseMeasurementXYZ64>>::from(msg);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().at<X>(), msg.position.x);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().at<Y>(), msg.position.y);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().at<Z>(), msg.position.z);
  EXPECT_DOUBLE_EQ(measurement.measurement.covariance()(0, 0), msg.covariance[0]);
  EXPECT_DOUBLE_EQ(measurement.measurement.covariance()(1, 1), msg.covariance[4]);
  EXPECT_DOUBLE_EQ(measurement.measurement.covariance()(2, 2), msg.covariance[8]);
  EXPECT_EQ(
    measurement.timestamp.time_since_epoch(),
    std::chrono::seconds{42LL});
}

/// \test Create a measurement from pose.
TEST(MeasurementConversionTest, Pose) {
  const auto msg = create_pose_msg();
  const auto measurement = convert_to<Stamped<PoseMeasurementXYZRPY64>>::from(msg);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().at<X>(), msg.pose.pose.position.x);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().at<Y>(), msg.pose.pose.position.y);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().at<Z>(), msg.pose.pose.position.z);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().at<ROLL>(), 0.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().at<PITCH>(), 0.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().at<YAW>(), M_PI_2);
  const auto x_idx = measurement.measurement.state().index_of<X>();
  const auto y_idx = measurement.measurement.state().index_of<Y>();
  const auto z_idx = measurement.measurement.state().index_of<Z>();
  const auto roll_idx = measurement.measurement.state().index_of<ROLL>();
  const auto pitch_idx = measurement.measurement.state().index_of<PITCH>();
  const auto yaw_idx = measurement.measurement.state().index_of<YAW>();
  EXPECT_DOUBLE_EQ(measurement.measurement.covariance()(x_idx, x_idx), msg.pose.covariance[0]);
  EXPECT_DOUBLE_EQ(measurement.measurement.covariance()(y_idx, y_idx), msg.pose.covariance[7]);
  EXPECT_DOUBLE_EQ(measurement.measurement.covariance()(z_idx, z_idx), msg.pose.covariance[14]);
  EXPECT_DOUBLE_EQ(
    measurement.measurement.covariance()(roll_idx, roll_idx),
    msg.pose.covariance[21]);
  EXPECT_DOUBLE_EQ(
    measurement.measurement.covariance()(pitch_idx, pitch_idx),
    msg.pose.covariance[28]);
  EXPECT_DOUBLE_EQ(
    measurement.measurement.covariance()(yaw_idx, yaw_idx),
    msg.pose.covariance[35]);
  EXPECT_EQ(
    measurement.timestamp.time_since_epoch(),
    std::chrono::seconds{42LL});
}
