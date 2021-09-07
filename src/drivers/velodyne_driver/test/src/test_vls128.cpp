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

#include <velodyne_driver/vls128_data.hpp>
#include <gtest/gtest.h>
#include <algorithm>


using autoware::drivers::velodyne_driver::VLS128Data;
using autoware::drivers::velodyne_driver::DEG2IDX;
using autoware::drivers::velodyne_driver::AZIMUTH_ROTATION_RESOLUTION;
using autoware::drivers::velodyne_driver::NUM_POINTS_PER_BLOCK;
using autoware::common::types::float32_t;
using autoware::common::types::bool8_t;

// The logic in the test is quite similar to the logic of azimuth table initialization logic however
// indexing is not identical.
TEST(VLS128DataTest, AngleLookupTest) {
  constexpr auto rpm{300U};
  VLS128Data vls128_data{rpm};
  constexpr uint32_t num_blocks_in_sequence =
    static_cast<uint32_t>(VLS128Data::NUM_LASERS / NUM_POINTS_PER_BLOCK);
  constexpr auto num_groups_in_block =
    static_cast<uint32_t>(NUM_POINTS_PER_BLOCK / VLS128Data::GROUP_SIZE);

  EXPECT_FLOAT_EQ(vls128_data.distance_resolution(), 0.004F);

  // See the spec sheet (Figure 9-8)
  // (The signs might be falsely flipped depending on the version of the manual.)
  std::array<float32_t, VLS128Data::GROUP_SIZE> group_azimuth_offsets{
    6.354F, 4.548F, 2.732F, 0.911F, -0.911F, -2.732F, -4.548F, -6.354F
  };

  std::transform(
    group_azimuth_offsets.begin(),
    group_azimuth_offsets.end(), group_azimuth_offsets.begin(),
    [](const auto & elem) {return elem < 0.0F ? elem + 360.0F : elem;});

  const float32_t US_TO_IDX = ((rpm * 360.0F) / (60.0F * 1.0E6F)) * DEG2IDX;

  // The computation is nearly identical to the code in VLS128Data::init_azimuth_table,
  // The idea is that the future refactors should not affect the functionality
  for (auto block_id = 0U; block_id < num_blocks_in_sequence; ++block_id) {
    for (auto group_id_in_block = 0U; group_id_in_block < num_groups_in_block;
      ++group_id_in_block)
    {
      const auto firing_id = block_id * num_groups_in_block + group_id_in_block;

      const auto maintaining_offset = firing_id < 8U ? 0U :
        VLS128Data::MAINTENANCE_DURATION1_US;

      const auto current_time_offset = static_cast<float32_t>(firing_id) *
        VLS128Data::FIRE_DURATION_US;
      const auto firing_total_azimuth_offset =
        (maintaining_offset + current_time_offset) * US_TO_IDX;

      const auto num_banked_pts = block_id * NUM_POINTS_PER_BLOCK;

      for (auto pt_id = 0U; pt_id < VLS128Data::GROUP_SIZE; ++pt_id) {
        const auto pt_id_in_block = (group_id_in_block * VLS128Data::GROUP_SIZE) +pt_id;
        const auto raw_azimuth_lookup = vls128_data.azimuth_offset(
          static_cast<uint16_t>(num_banked_pts), block_id,
          pt_id_in_block);

        const auto expected_azimuth =
          static_cast<uint32_t>(std::roundf(
            firing_total_azimuth_offset +
            group_azimuth_offsets[pt_id] * DEG2IDX));
        const auto altitude = vls128_data.altitude(
          static_cast<uint16_t>(num_banked_pts), block_id,
          pt_id_in_block);
        EXPECT_EQ(raw_azimuth_lookup, expected_azimuth);
        EXPECT_LE(altitude, AZIMUTH_ROTATION_RESOLUTION);
        EXPECT_GE(altitude, 0U);
        EXPECT_LE(raw_azimuth_lookup, AZIMUTH_ROTATION_RESOLUTION);
        EXPECT_GE(raw_azimuth_lookup, 0U);
        EXPECT_EQ(vls128_data.seq_id(static_cast<uint16_t>(block_id), pt_id_in_block), 0U);
      }
    }
  }
  EXPECT_EQ(vls128_data.seq_id(5U, 15U), 1U);
  EXPECT_EQ(vls128_data.seq_id(7U, 31U), 1U);
  EXPECT_EQ(vls128_data.seq_id(8U, 0U), 2U);
}

TEST(VLS128DataTest, SeqIdTest) {
  constexpr auto rpm{300U};
  VLS128Data vls128_data{rpm};
  EXPECT_EQ(vls128_data.seq_id(0U, 12U), 0U);
  EXPECT_EQ(vls128_data.seq_id(5U, 15U), 1U);
  EXPECT_EQ(vls128_data.seq_id(7U, 31U), 1U);
  EXPECT_EQ(vls128_data.seq_id(8U, 0U), 2U);
}

TEST(VLS128DataTest, FlagTest) {
  constexpr auto rpm{300U};
  VLS128Data vls128_data{rpm};
  const auto check_flag = [&vls128_data](const VLS128Data::BlockFlag & flag,
      bool8_t valid, uint16_t expected_bank_num, bool8_t check_bank) {
      const auto res = vls128_data.check_flag(flag);
      EXPECT_EQ(res.first, valid);
      if (check_bank) {
        EXPECT_EQ(res.second, expected_bank_num);
      }
    };

  check_flag({static_cast<uint8_t>(0xFF), static_cast<uint8_t>(0xEE)}, true, 0U, true);
  check_flag({static_cast<uint8_t>(0xFF), static_cast<uint8_t>(0xDD)}, true, 32U, true);
  check_flag({static_cast<uint8_t>(0xFF), static_cast<uint8_t>(0xCC)}, true, 64U, true);
  check_flag({static_cast<uint8_t>(0xFF), static_cast<uint8_t>(0xBB)}, true, 96U, true);
  check_flag({static_cast<uint8_t>(0xFF), static_cast<uint8_t>(0xAA)}, false, 0U, false);
  check_flag({static_cast<uint8_t>(0x00), static_cast<uint8_t>(0xDD)}, false, 0U, false);
}
