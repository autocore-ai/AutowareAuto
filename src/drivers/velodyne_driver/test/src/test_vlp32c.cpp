// Copyright 2020 the Autoware Foundation
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

#include <velodyne_driver/vlp32c_data.hpp>
#include <gtest/gtest.h>
#include <algorithm>


using autoware::drivers::velodyne_driver::VLP32CData;
using autoware::drivers::velodyne_driver::DEG2IDX;
using autoware::drivers::velodyne_driver::AZIMUTH_ROTATION_RESOLUTION;
using autoware::drivers::velodyne_driver::NUM_POINTS_PER_BLOCK;
using autoware::common::types::float32_t;
using autoware::common::types::bool8_t;

// The logic in the test is quite similar to the logic of azimuth table initialization logic
// however indexing is not identical.
TEST(VLP32CDataTest, AngleLookupTest) {
  constexpr auto rpm{300U};
  VLP32CData vlp32c_data{rpm};

  std::array<float32_t, VLP32CData::NUM_LASERS> azimuth_offsets{
    1.4F, -4.2F, 1.4F, -1.4F, 1.4F, -1.4F, 4.2F, -1.4F,
    1.4F, -4.2F, 1.4F, -1.4F, 4.2F, -1.4F, 4.2F, -1.4F,
    1.4F, -4.2F, 1.4F, -4.2F, 4.2F, -1.4F, 1.4F, -1.4F,
    1.4F, -1.4F, 1.4F, -4.2F, 4.2F, -1.4F, 1.4F, -1.4F
  };

  std::transform(
    azimuth_offsets.begin(),
    azimuth_offsets.end(), azimuth_offsets.begin(),
    [](const auto & elem) {return elem < 0.0F ? elem + 360.0F : elem;});

  const float32_t US_TO_DEG = (rpm * 360.0F) / (60.0F * 1.0E6F);

  for (auto firing_id = 0U; firing_id < 16U; ++firing_id) {
    const auto time_passed = static_cast<float32_t>(firing_id) * VLP32CData::FIRE_DURATION_US;
    const auto degree_indices_passed = US_TO_DEG * time_passed;

    for (auto laser_id_in_pair = 0U; laser_id_in_pair < 2U; ++laser_id_in_pair) {
      const auto laser_id = firing_id * 2U + laser_id_in_pair;
      const auto azimuth =
        static_cast<uint32_t>(std::roundf(
          DEG2IDX *
          (degree_indices_passed + azimuth_offsets[laser_id])));
      EXPECT_EQ(azimuth, vlp32c_data.azimuth_offset(0U, 0U, laser_id));
    }
  }
}

TEST(VLP32CDataTest, SeqIdTest) {
  constexpr auto rpm{300U};
  VLP32CData vlp32c_data{rpm};
  EXPECT_EQ(vlp32c_data.seq_id(0U, 12U), 0U);
  EXPECT_EQ(vlp32c_data.seq_id(5U, 15U), 5U);
  EXPECT_EQ(vlp32c_data.seq_id(7U, 31U), 7U);
  EXPECT_EQ(vlp32c_data.seq_id(8U, 0U), 8U);
}

TEST(VLP32CDataTest, FlagTest) {
  constexpr auto rpm{300U};
  VLP32CData vlp32c_data{rpm};

  const auto check_flag = [&vlp32c_data](const VLP32CData::BlockFlag & flag,
      bool8_t valid, uint16_t expected_bank_num, bool8_t check_bank) {
      const auto res = vlp32c_data.check_flag(flag);
      EXPECT_EQ(res.first, valid);
      if (check_bank) {
        EXPECT_EQ(res.second, expected_bank_num);
      }
    };

  check_flag({static_cast<uint8_t>(0xFF), static_cast<uint8_t>(0xEE)}, true, 0U, true);
  check_flag({static_cast<uint8_t>(0xFF), static_cast<uint8_t>(0xDD)}, false, 0U, false);
  check_flag({static_cast<uint8_t>(0xFF), static_cast<uint8_t>(0xCC)}, false, 0U, false);
  check_flag({static_cast<uint8_t>(0xFF), static_cast<uint8_t>(0xBB)}, false, 0U, false);
  check_flag({static_cast<uint8_t>(0xFF), static_cast<uint8_t>(0xAA)}, false, 0U, false);
  check_flag({static_cast<uint8_t>(0x00), static_cast<uint8_t>(0xDD)}, false, 0U, false);
}
