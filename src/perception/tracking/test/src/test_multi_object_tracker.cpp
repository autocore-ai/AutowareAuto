// Copyright 2021 The Autoware Foundation
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

#include "gtest/gtest.h"
#include "autoware_auto_msgs/msg/detected_objects.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tracking/multi_object_tracker.hpp"

using Tracker = autoware::perception::tracking::MultiObjectTracker;
using CreationPolicies = autoware::perception::tracking::TrackCreationPolicy;
using Options = autoware::perception::tracking::MultiObjectTrackerOptions;
using Status = autoware::perception::tracking::TrackerUpdateStatus;
using DetectedObjects = autoware_auto_msgs::msg::DetectedObjects;
using Odometry = nav_msgs::msg::Odometry;

class MultiObjectTrackerTest : public ::testing::Test
{
public:
  MultiObjectTrackerTest()
  : m_tracker{Options{{2.0F, 2.5F, true}, {}, {CreationPolicies::LidarClusterOnly, 1.0F, 1.0F}},
      m_tf_buffer}
  {
    m_detections.header.frame_id = "base_link";
    m_detections.header.stamp.sec = 1000;
    m_odom.header.frame_id = "map";
    m_odom.child_frame_id = "base_link";
    m_odom.header.stamp.sec = 1000;
    m_odom.pose.pose.orientation.w = 1.0;
  }

  void SetUp() {}

  void TearDown() {}

  tf2::BufferCore m_tf_buffer;
  Tracker m_tracker;
  DetectedObjects m_detections;
  Odometry m_odom;
};

TEST_F(MultiObjectTrackerTest, TestHappyPath) {
  EXPECT_NO_THROW(m_tracker.update(m_detections, m_odom));
}

TEST_F(MultiObjectTrackerTest, TestTimestamps) {
  m_tracker.update(m_detections, m_odom);
  m_detections.header.stamp.sec = 999;
  const auto result = m_tracker.update(m_detections, m_odom);
  EXPECT_EQ(result.status, Status::WentBackInTime);
}

TEST_F(MultiObjectTrackerTest, TestFrameOrientationValidation) {
  // This rotates the object to be on its side, which does not make sense
  m_odom.pose.pose.orientation.w = 0.0;
  m_odom.pose.pose.orientation.x = 1.0;
  const auto result = m_tracker.update(m_detections, m_odom);
  EXPECT_EQ(result.status, Status::FrameNotGravityAligned);
}
