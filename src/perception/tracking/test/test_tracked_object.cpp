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

#include <chrono>
#include <vector>

#include "gtest/gtest.h"
#include "autoware_auto_msgs/msg/detected_object.hpp"
#include "tracking/tracked_object.hpp"

using TrackedObject = autoware::perception::tracking::TrackedObject;
using DetectedObjectMsg = autoware_auto_msgs::msg::DetectedObject;
using TrackedObjectMsg = autoware_auto_msgs::msg::TrackedObject;

// Test that creating a tracked object without pose is not allowed.
TEST(test_tracked_object, test_pose_required) {
  DetectedObjectMsg msg;
  EXPECT_THROW((TrackedObject{msg, 1.0F, 1.0F}), std::invalid_argument);
  msg.kinematics.has_pose = true;
  EXPECT_NO_THROW((TrackedObject{msg, 1.0F, 1.0F}));
}

// Test that the has_twist field is respected when initializing the tracked object.
TEST(test_tracked_object, test_optional_twist) {
  DetectedObjectMsg msg;
  msg.kinematics.has_pose = true;
  msg.kinematics.twist.twist.linear.x = 3.0;
  TrackedObject object{msg, 1.0F, 30.0F};
  // The twist is ignored and set to 0 when has_twist == false
  EXPECT_EQ(object.msg().kinematics.twist.twist.linear.x, 0.0);
  msg.kinematics.has_twist = true;
  object = TrackedObject {msg, 1.0F, 30.0F};
  EXPECT_EQ(object.msg().kinematics.twist.twist.linear.x, 3.0);
}

// Test that pose and twist covariances are handled correctly, i.e. they are read only when
// has_pose_covariance/has_twist_covariance are set, and otherwise the default is used.
TEST(test_tracked_object, test_optional_covariance) {
  const float kDefaultVarianceF = 1.0F;
  const double kDefaultVarianceD = static_cast<double>(kDefaultVarianceF);
  DetectedObjectMsg msg;
  msg.kinematics.has_pose = true;
  msg.kinematics.pose.covariance[0] = 3.0;  // linear x
  msg.kinematics.pose.covariance[1] = 2.0;  // linear xy
  msg.kinematics.pose.covariance[6] = 2.0;  // linear xy
  msg.kinematics.pose.covariance[7] = 3.0;  // linear y
  msg.kinematics.pose.covariance[14] = 3.0;  // linear z
  msg.kinematics.pose.covariance[35] = 3.0;  // angular z
  TrackedObject object{msg, kDefaultVarianceF, 30.0F};
  EXPECT_EQ(object.msg().kinematics.pose.covariance[0], kDefaultVarianceD);  // equal to the default
  EXPECT_EQ(object.msg().kinematics.pose.covariance[1], 0.0);  // off-diagonals are 0 by default
  EXPECT_EQ(object.msg().kinematics.pose.covariance[6], 0.0);
  EXPECT_EQ(object.msg().kinematics.pose.covariance[7], kDefaultVarianceD);
  EXPECT_EQ(object.msg().kinematics.pose.covariance[14], 0.0);  // This is a 2D model, z will be 0
  EXPECT_EQ(object.msg().kinematics.pose.covariance[35], 0.0);  // This is a CA model, θ will be 0
  msg.kinematics.has_pose_covariance = true;
  object = TrackedObject {msg, kDefaultVarianceF, 30.0F};
  EXPECT_EQ(object.msg().kinematics.pose.covariance[0], 3.0);  // equal to input value
  EXPECT_EQ(object.msg().kinematics.pose.covariance[1], 2.0);
  EXPECT_EQ(object.msg().kinematics.pose.covariance[6], 2.0);
  EXPECT_EQ(object.msg().kinematics.pose.covariance[7], 3.0);
  EXPECT_EQ(object.msg().kinematics.pose.covariance[14], 0.0);  // z will still be 0
  EXPECT_EQ(object.msg().kinematics.pose.covariance[14], 0.0);  // θ will still be 0
  // =========== same thing for twist ===========
  msg.kinematics.twist.covariance[0] = 5.0;
  msg.kinematics.twist.covariance[1] = 4.0;
  msg.kinematics.twist.covariance[6] = 4.0;
  msg.kinematics.twist.covariance[7] = 5.0;
  msg.kinematics.twist.covariance[14] = 5.0;
  msg.kinematics.twist.covariance[35] = 5.0;
  object = TrackedObject {msg, kDefaultVarianceF, 30.0F};
  EXPECT_EQ(object.msg().kinematics.twist.covariance[0], kDefaultVarianceD);
  EXPECT_EQ(object.msg().kinematics.twist.covariance[1], 0.0);
  EXPECT_EQ(object.msg().kinematics.twist.covariance[6], 0.0);
  EXPECT_EQ(object.msg().kinematics.twist.covariance[7], kDefaultVarianceD);
  EXPECT_EQ(object.msg().kinematics.twist.covariance[14], 0.0);
  EXPECT_EQ(object.msg().kinematics.twist.covariance[35], 0.0);
  msg.kinematics.has_twist_covariance = true;
  object = TrackedObject {msg, kDefaultVarianceF, 30.0F};
  EXPECT_EQ(object.msg().kinematics.twist.covariance[0], 5.0);
  EXPECT_EQ(object.msg().kinematics.twist.covariance[1], 4.0);
  EXPECT_EQ(object.msg().kinematics.twist.covariance[6], 4.0);
  EXPECT_EQ(object.msg().kinematics.twist.covariance[7], 5.0);
  EXPECT_EQ(object.msg().kinematics.twist.covariance[14], 0.0);
  EXPECT_EQ(object.msg().kinematics.twist.covariance[35], 0.0);
}

// Test that calling predict() indeed changes the coordinates
TEST(test_tracked_object, test_predict) {
  DetectedObjectMsg msg;
  msg.kinematics.has_pose = true;
  msg.kinematics.has_twist = true;
  msg.kinematics.twist.twist.linear.x = 3.0;
  TrackedObject object{msg, 1.0F, 30.0F};
  EXPECT_EQ(object.msg().kinematics.pose.pose.position.x, 0.0);
  object.predict(std::chrono::milliseconds(500));
  EXPECT_NE(object.msg().kinematics.pose.pose.position.x, 0.0);
}

// Test that the update() method can handle detected objects with or without pose and twist.
TEST(test_tracked_object, test_update) {
  const float kDefaultVarianceF = 1.0F;
  DetectedObjectMsg msg;
  msg.kinematics.has_pose = true;
  std::vector<TrackedObjectMsg> distinct_results;
  // Iterate over possible true/false combinations
  for (size_t i = 1; i < 4; ++i) {
    // Create an object at (0, 0) with velocity 0.
    TrackedObject track{msg, kDefaultVarianceF, 30.0F};

    DetectedObjectMsg obs;
    obs.kinematics.has_pose = static_cast<bool>(i & 1);
    obs.kinematics.has_twist = static_cast<bool>(i & 2);
    obs.kinematics.twist.twist.linear.x = 3.0;
    obs.kinematics.pose.pose.position.y = 2.0;
    EXPECT_NO_THROW(track.update(obs));
    distinct_results.push_back(track.msg());
  }

  for (size_t i = 0; i < distinct_results.size(); ++i) {
    for (size_t j = i + 1; j < distinct_results.size(); ++j) {
      EXPECT_NE(distinct_results[i], distinct_results[j]);
    }
  }
}

// Test that a newly created track isn't pruned right away, but an old track is
TEST(test_tracked_object, test_should_be_removed) {
  DetectedObjectMsg msg;
  msg.kinematics.has_pose = true;
  TrackedObject object{msg, 1.0F, 30.0F};
  EXPECT_EQ(object.should_be_removed(std::chrono::milliseconds(100), 5), false);
  object.predict(std::chrono::milliseconds(500));
  object.no_update();
  EXPECT_EQ(object.should_be_removed(std::chrono::milliseconds(100), 5), true);
  EXPECT_EQ(object.should_be_removed(std::chrono::milliseconds(1000), 5), false);
  object.no_update();
  object.no_update();
  object.no_update();
  object.no_update();
  EXPECT_EQ(object.should_be_removed(std::chrono::milliseconds(1000), 5), true);
}
