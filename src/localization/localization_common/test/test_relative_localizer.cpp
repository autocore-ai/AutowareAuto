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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <gtest/gtest.h>
#include <localization_common/localizer_base.hpp>
#include "test_relative_localizer.hpp"

class TestRelativeLocalizerBase : public ::testing::Test
{
protected:
  void SetUp()
  {
    set_id(m_init, std::to_string(m_init_id));

    ASSERT_NE(m_pose_id, ERR_CODE);
    ASSERT_NE(m_init_id, ERR_CODE);
    ASSERT_NE(m_map_id, ERR_CODE);

    ASSERT_NE(m_pose_id, INVALID_ID);
    ASSERT_NE(m_init_id, INVALID_ID);
    ASSERT_NE(m_map_id, INVALID_ID);
  }

  Transform m_init;
  const int m_pose_id{3};
  const int m_init_id{5};
  const int m_map_id{7};
};

TEST_F(TestRelativeLocalizerBase, basic_io) {
  TestLocalizer localizer;
  TestLocalizer::PoseWithCovarianceStamped dummy_pose;
  ASSERT_EQ(localizer.map_frame_id(), "");

  // no map is set yet.
  EXPECT_FALSE(localizer.map_valid());
  // Can't register without a map set.
  EXPECT_THROW(localizer.register_measurement(0, m_init, dummy_pose), std::logic_error);

  EXPECT_NO_THROW(localizer.set_map(m_map_id));
  EXPECT_TRUE(localizer.map_valid());
  EXPECT_EQ(localizer.map_frame_id(), std::to_string(m_map_id));

  PoseWithCovarianceStamped pose_out;
  EXPECT_NO_THROW(localizer.register_measurement(m_pose_id, m_init, pose_out));

  EXPECT_EQ(get_id(pose_out), merge_ids(m_pose_id, m_init_id, m_map_id));
}

TEST_F(TestRelativeLocalizerBase, bad_map) {
  TestLocalizer localizer;
  ASSERT_EQ(localizer.map_frame_id(), "");
  TestLocalizer::PoseWithCovarianceStamped dummy_pose;

  // no map is set yet.
  EXPECT_FALSE(localizer.map_valid());
  EXPECT_THROW(localizer.register_measurement(0, m_init, dummy_pose),
    std::logic_error);

  // Emulate setting a bad map.
  ASSERT_THROW(localizer.set_map(ERR_CODE), std::runtime_error);
  EXPECT_FALSE(localizer.map_valid());
  EXPECT_THROW(localizer.register_measurement(0, m_init, dummy_pose), std::logic_error);

  // now set a valid map.
  EXPECT_NO_THROW(localizer.set_map(m_map_id));
  EXPECT_TRUE(localizer.map_valid());
  EXPECT_NO_THROW(localizer.register_measurement(0, m_init, dummy_pose));

  // Set an invalid map again.
  ASSERT_THROW(localizer.set_map(ERR_CODE), std::runtime_error);
  EXPECT_FALSE(localizer.map_valid());
  EXPECT_THROW(localizer.register_measurement(0, m_init, dummy_pose), std::logic_error);
}
