// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

#include "autoware_state_monitor/odometry_updater.hpp"

#include <memory>

#include "gtest/gtest.h"

#include "autoware_state_monitor/odometry_buffer.hpp"
#include "test_utils.hpp"

using autoware::state_monitor::OdometryBuffer;
using autoware::state_monitor::OdometryUpdater;

class OdometryUpdaterTest : public ::testing::Test
{
public:
  OdometryUpdaterTest()
  {
    updater.reset(new OdometryUpdater(buffer, buffer_length_sec));
  }
  ~OdometryUpdaterTest() = default;

  double buffer_length_sec = 1.0;
  OdometryBuffer buffer;
  std::shared_ptr<OdometryUpdater> updater;
};

TEST_F(OdometryUpdaterTest, create_destroy)
{
}

TEST_F(OdometryUpdaterTest, add_new_samples)
{
  EXPECT_EQ(buffer.size(), 0);
  updater->update(prepareVehicleOdometryMsg(1.0));
  EXPECT_EQ(buffer.size(), 1);
  EXPECT_EQ(buffer.front()->velocity_mps, 1.0);
  EXPECT_EQ(buffer.back()->velocity_mps, 1.0);

  updater->update(prepareVehicleOdometryMsg(-1.0, 0.0));
  EXPECT_EQ(buffer.size(), 2);
  EXPECT_EQ(buffer.front()->velocity_mps, 1.0);
  EXPECT_EQ(buffer.back()->velocity_mps, -1.0);

  updater->update(prepareVehicleOdometryMsg(0.5, 0.0));
  EXPECT_EQ(buffer.size(), 3);
  EXPECT_EQ(buffer.front()->velocity_mps, 1.0);
  EXPECT_EQ(buffer.back()->velocity_mps, 0.5);
}

TEST_F(OdometryUpdaterTest, null_message_do_nothing)
{
  updater->update(nullptr);
}

TEST_F(OdometryUpdaterTest, old_messages_removal_to_preserve_length)
{
  const double buffer_length_sec = 1.0;
  updater.reset(new OdometryUpdater(buffer, buffer_length_sec));

  auto stamp = toTime(0.0);
  updater->update(prepareVehicleOdometryMsg(1.0, 0, 0, stamp));
  EXPECT_EQ(buffer.size(), 1);

  // time_diff = 0.5 < buffer_length_sec --> sample should stay
  stamp = toTime(0.5);
  updater->update(prepareVehicleOdometryMsg(2.0, 0, 0, stamp));
  EXPECT_EQ(buffer.size(), 2);

  // time_diff = 1.0 == buffer_length_sec --> sample should stay
  stamp = toTime(1.0);
  updater->update(prepareVehicleOdometryMsg(3.0, 0, 0, stamp));
  EXPECT_EQ(buffer.size(), 3);
  EXPECT_EQ(buffer.front()->velocity_mps, 1.0);

  // time_diff = 1.1 > buffer_length_sec --> first sample should be removed
  stamp = toTime(1.1);
  updater->update(prepareVehicleOdometryMsg(4.0, 0, 0, stamp));
  EXPECT_EQ(buffer.size(), 3);
  EXPECT_EQ(buffer.front()->velocity_mps, 2.0);
  EXPECT_EQ(buffer.back()->velocity_mps, 4.0);

  // time_diff = 1.5 > buffer_length_sec --> first sample should be removed
  stamp = toTime(2.0);
  updater->update(prepareVehicleOdometryMsg(5.0, 0, 0, stamp));
  EXPECT_EQ(buffer.size(), 3);
  EXPECT_EQ(buffer.front()->velocity_mps, 3.0);
  EXPECT_EQ(buffer.back()->velocity_mps, 5.0);
}
