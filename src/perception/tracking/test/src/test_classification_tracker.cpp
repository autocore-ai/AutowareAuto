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


#include <autoware_auto_msgs/msg/detected_object.hpp>
#include <tracking/classification_tracker.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <vector>

using autoware::perception::tracking::ClassificationTracker;
using autoware_auto_msgs::msg::DetectedObject;
using autoware_auto_msgs::msg::ObjectClassification;
using autoware::common::types::float32_t;

namespace
{
struct ClassificationWithProbability
{
  std::uint8_t object_class;
  float32_t probability;
};

}  // namespace


DetectedObject create_object(
  const std::vector<ClassificationWithProbability> & classifications)
{
  DetectedObject object;
  for (const auto classification : classifications) {
    ObjectClassification object_classification;
    object_classification.classification = classification.object_class;
    object_classification.probability = classification.probability;
    object.classification.emplace_back(object_classification);
  }
  return object;
}

TEST(ClassificaitonTrackerTest, TestInit) {
  ClassificationTracker tracker;
  EXPECT_EQ(
    autoware_auto_msgs::msg::ObjectClassification::UNKNOWN, tracker.most_likely_class());
}

TEST(ClassificaitonTrackerTest, TestSingleUpdate) {
  const DetectedObject object = create_object(
    {{autoware_auto_msgs::msg::ObjectClassification::CAR, 0.8F}});
  ClassificationTracker tracker;
  tracker.update(object.classification);
  EXPECT_EQ(autoware_auto_msgs::msg::ObjectClassification::CAR, tracker.most_likely_class());
  const auto resulting_classification = tracker.object_classification_vector();
  EXPECT_EQ(
    resulting_classification[autoware_auto_msgs::msg::ObjectClassification::UNKNOWN].classification,
    autoware_auto_msgs::msg::ObjectClassification::UNKNOWN);
  EXPECT_FLOAT_EQ(
    resulting_classification[autoware_auto_msgs::msg::ObjectClassification::UNKNOWN].probability,
    tracker.state()[autoware_auto_msgs::msg::ObjectClassification::UNKNOWN]);
  EXPECT_EQ(
    resulting_classification[autoware_auto_msgs::msg::ObjectClassification::CAR].classification,
    autoware_auto_msgs::msg::ObjectClassification::CAR);
  EXPECT_FLOAT_EQ(
    resulting_classification[autoware_auto_msgs::msg::ObjectClassification::CAR].probability,
    tracker.state()[autoware_auto_msgs::msg::ObjectClassification::CAR]);
  EXPECT_GT(
    resulting_classification[autoware_auto_msgs::msg::ObjectClassification::CAR].probability, 0.7F);
}

TEST(ClassificaitonTrackerTest, TestMultipleUpdates) {
  const DetectedObject car = create_object(
    {{autoware_auto_msgs::msg::ObjectClassification::CAR, 1.0F}});
  const DetectedObject truck = create_object(
    {{autoware_auto_msgs::msg::ObjectClassification::TRUCK, 1.0F}});
  const DetectedObject unknown_object = create_object(
    {{autoware_auto_msgs::msg::ObjectClassification::UNKNOWN, 1.0F}});
  ClassificationTracker tracker;
  tracker.update(car.classification);
  EXPECT_EQ(autoware_auto_msgs::msg::ObjectClassification::CAR, tracker.most_likely_class());
  tracker.update(truck.classification);
  tracker.update(truck.classification);
  EXPECT_EQ(autoware_auto_msgs::msg::ObjectClassification::TRUCK, tracker.most_likely_class());
  // Perform updates with a very high covariance. These should play near to no role.
  tracker.update(car.classification, 100.0F);
  tracker.update(car.classification, 100.0F);
  tracker.update(car.classification, 100.0F);
  EXPECT_EQ(autoware_auto_msgs::msg::ObjectClassification::TRUCK, tracker.most_likely_class());
  // Perform updates with a normal covariance. These should change the most likely class.
  tracker.update(car.classification);
  tracker.update(car.classification);
  EXPECT_EQ(autoware_auto_msgs::msg::ObjectClassification::CAR, tracker.most_likely_class());
  tracker.update(unknown_object.classification);
  tracker.update(unknown_object.classification);
  tracker.update(unknown_object.classification);
  EXPECT_EQ(autoware_auto_msgs::msg::ObjectClassification::UNKNOWN, tracker.most_likely_class());
}

TEST(ClassificaitonTrackerTest, TestMultipleUpdatesInOneObject) {
  const DetectedObject more_car = create_object(
  {
    {autoware_auto_msgs::msg::ObjectClassification::CAR, 0.6F},
    {autoware_auto_msgs::msg::ObjectClassification::PEDESTRIAN, 0.4F}
  });
  const DetectedObject more_pedestrian = create_object(
  {
    {autoware_auto_msgs::msg::ObjectClassification::CAR, 0.3F},
    {autoware_auto_msgs::msg::ObjectClassification::PEDESTRIAN, 0.7F}
  });
  ClassificationTracker tracker;
  tracker.update(more_car.classification);
  EXPECT_EQ(autoware_auto_msgs::msg::ObjectClassification::CAR, tracker.most_likely_class());
  tracker.update(more_pedestrian.classification);
  EXPECT_EQ(
    autoware_auto_msgs::msg::ObjectClassification::PEDESTRIAN, tracker.most_likely_class());
  EXPECT_LT(
    tracker.state()[autoware_auto_msgs::msg::ObjectClassification::UNKNOWN],
    tracker.default_observation_covariance());
}

TEST(ClassificaitonTrackerTest, TestUncartainObject) {
  const DetectedObject certain_car = create_object(
    {{autoware_auto_msgs::msg::ObjectClassification::CAR, 0.9F}});
  const DetectedObject uncertain_car = create_object(
    {{autoware_auto_msgs::msg::ObjectClassification::CAR, 0.1F}});
  ClassificationTracker tracker;
  tracker.update(certain_car.classification);
  EXPECT_EQ(autoware_auto_msgs::msg::ObjectClassification::CAR, tracker.most_likely_class());
  tracker.update(uncertain_car.classification);
  EXPECT_EQ(
    autoware_auto_msgs::msg::ObjectClassification::UNKNOWN, tracker.most_likely_class());
}

TEST(ClassificaitonTrackerTest, OverconfidentObject) {
  const DetectedObject overconfident_object = create_object(
  {
    {autoware_auto_msgs::msg::ObjectClassification::CAR, 0.9F},
    {autoware_auto_msgs::msg::ObjectClassification::PEDESTRIAN, 0.9F}
  });
  ClassificationTracker tracker;
  EXPECT_THROW(tracker.update(overconfident_object.classification), std::domain_error);
}
