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

#include <state_estimation_nodes/measurement_conversion.hpp>

using autoware::prediction::Measurement;
using autoware::prediction::MeasurementPose;
using autoware::prediction::MeasurementSpeed;
using autoware::prediction::MeasurementPoseAndSpeed;
using autoware::prediction::message_to_measurement;
using autoware::prediction::downscale_isometry;

/// \test Create a measurement from odometry.
TEST(MeasurementConversionTest, odom) {
  nav_msgs::msg::Odometry msg{};
  msg.header.frame_id = "map";
  msg.header.stamp.sec = 42;
  msg.header.stamp.nanosec = 0;
  msg.pose.pose.position.x = 42.0;
  msg.pose.pose.position.y = 23.0;
  msg.pose.covariance[0] = 1.0;
  msg.pose.covariance[7] = 2.0;
  msg.twist.twist.linear.x = 23.0;
  msg.twist.twist.linear.y = 42.0;
  msg.twist.covariance[0] = 3.0;
  msg.twist.covariance[7] = 4.0;
  const auto measurement = message_to_measurement<MeasurementPoseAndSpeed>(
    msg, Eigen::Isometry3f::Identity(), Eigen::Isometry3f::Identity());
  EXPECT_FLOAT_EQ(measurement.get_values().x(), 42.0F);
  EXPECT_FLOAT_EQ(measurement.get_values().y(), 23.0F);
  EXPECT_FLOAT_EQ(measurement.get_variances().x(), 1.0F);
  EXPECT_FLOAT_EQ(measurement.get_variances().y(), 2.0F);

  EXPECT_FLOAT_EQ(measurement.get_values()[2], 23.0F);
  EXPECT_FLOAT_EQ(measurement.get_values()[3], 42.0F);
  EXPECT_FLOAT_EQ(measurement.get_variances()[2], 3.0F);
  EXPECT_FLOAT_EQ(measurement.get_variances()[3], 4.0F);

  EXPECT_EQ(
    measurement.get_acquisition_time().time_since_epoch(),
    std::chrono::seconds{42LL});
}

/// \test Create a measurement from pose.
TEST(MeasurementConversionTest, pose) {
  geometry_msgs::msg::PoseWithCovarianceStamped msg{};
  msg.header.frame_id = "map";
  msg.header.stamp.sec = 42;
  msg.header.stamp.nanosec = 0;
  msg.pose.pose.position.x = 42.0;
  msg.pose.pose.position.y = 23.0;
  msg.pose.covariance[0] = 1.0;
  msg.pose.covariance[7] = 2.0;
  const auto measurement = message_to_measurement<MeasurementPose>(
    msg, Eigen::Isometry3f::Identity());
  EXPECT_FLOAT_EQ(measurement.get_values().x(), 42.0F);
  EXPECT_FLOAT_EQ(measurement.get_values().y(), 23.0F);
  EXPECT_FLOAT_EQ(measurement.get_variances().x(), 1.0F);
  EXPECT_FLOAT_EQ(measurement.get_variances().y(), 2.0F);
  EXPECT_EQ(
    measurement.get_acquisition_time().time_since_epoch(),
    std::chrono::seconds{42LL});
}

/// \test Create a measurement from tist.
TEST(MeasurementConversionTest, twist) {
  geometry_msgs::msg::TwistWithCovarianceStamped msg{};
  msg.header.frame_id = "map";
  msg.header.stamp.sec = 42;
  msg.header.stamp.nanosec = 0;
  msg.twist.twist.linear.x = 23.0;
  msg.twist.twist.linear.y = 42.0;
  msg.twist.covariance[0] = 3.0;
  msg.twist.covariance[7] = 4.0;
  const auto measurement = message_to_measurement<MeasurementSpeed>(
    msg, Eigen::Isometry3f::Identity());
  EXPECT_FLOAT_EQ(measurement.get_values()[0], 23.0F);
  EXPECT_FLOAT_EQ(measurement.get_values()[1], 42.0F);
  EXPECT_FLOAT_EQ(measurement.get_variances()[0], 3.0F);
  EXPECT_FLOAT_EQ(measurement.get_variances()[1], 4.0F);

  EXPECT_EQ(
    measurement.get_acquisition_time().time_since_epoch(),
    std::chrono::seconds{42LL});
}

/// \test Check that we can downscale a trivial isometry.
TEST(MeasurementConversionTest, trivial_isometry_downscaling) {
  const auto isometry = downscale_isometry<2>(Eigen::Isometry3f::Identity());
  EXPECT_TRUE(isometry.isApprox(Eigen::Isometry2f::Identity()));
}

/// \test Check that we can downscale an isometry.
TEST(MeasurementConversionTest, isometry_downscaling) {
  Eigen::Isometry3f initial_isometry;
  initial_isometry.linear() =
    Eigen::AngleAxisf{M_PI_2, Eigen::Vector3f::UnitZ()} *Eigen::Scaling(1.0F);
  const auto isometry = downscale_isometry<2>(initial_isometry);
  Eigen::Matrix2f expected_rotation = Eigen::Rotation2Df(M_PI_2) * Eigen::Scaling(1.0F);
  EXPECT_TRUE(isometry.linear().isApprox(expected_rotation));
}
