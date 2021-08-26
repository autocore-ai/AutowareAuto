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
#include "tracking/track_creator.hpp"

using AssociatorResult = autoware::perception::tracking::AssociatorResult;
using CreationPolicies = autoware::perception::tracking::TrackCreationPolicy;
using DetectedObject = autoware_auto_msgs::msg::DetectedObject;
using DetectedObjects = autoware_auto_msgs::msg::DetectedObjects;
using TrackCreator = autoware::perception::tracking::TrackCreator;

TEST(TrackCreatorTest, test_basic)
{
  TrackCreator creator{{CreationPolicies::LidarClusterOnly, 1.0F, 1.0F}};
  DetectedObject obj;
  DetectedObjects objs;
  const int num_objects = 10;
  for (int i = 0; i < num_objects; ++i) {
    objs.objects.push_back(obj);
  }

  AssociatorResult result;
  result.unassigned_detection_indices = {0, 2, 4};

  creator.add_unassigned_lidar_clusters(objs, result);
  EXPECT_EQ(creator.create_tracks().size(), 3U);
}
