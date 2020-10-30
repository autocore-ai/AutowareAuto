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
#include <utility>
#include <limits>

namespace autoware
{
namespace drivers
{
namespace velodyne_driver
{

VLS128Data::VLS128Data(const float32_t rpm)
{
  init_azimuth_table(rpm);
  init_altitude_table();
  // (60E6 us/min * x min/rev = us/rev) * (seq/us * block/seq) = block / rev
  m_num_blocks_per_revolution =
    static_cast<uint16_t>(std::roundf(60.0E6F /
    (rpm * FIRE_SEQ_OFFSET_US * NUM_SEQUENCES_PER_BLOCK)));
}

uint32_t VLS128Data::azimuth_offset(uint16_t num_banked_pts, uint32_t, uint32_t pt_id) const
{
  return m_azimuth_ind[num_banked_pts + pt_id];
}

uint32_t VLS128Data::altitude(uint16_t num_banked_pts, uint32_t, uint32_t pt_id) const
{
  return m_altitude_ind[num_banked_pts + pt_id];
}

uint16_t VLS128Data::seq_id(uint16_t num_blocks, uint32_t) const noexcept
{
  return static_cast<uint16_t>(std::floor(NUM_SEQUENCES_PER_BLOCK * num_blocks));
}

void VLS128Data::init_azimuth_table(const float32_t rpm)
{
  // Azimuth offsets in a group from the spec sheet(Figure 9-8).
  // A group is 8 lasers horizontally aligned. 128 lasers are
  // aligned in a 16 x 8 grid. So there are only 8 different azimuth offsets.
  //---------
  // IMPORTANT NOTE: In the reference manual, these values have the exact opposite signs,
  // however experiments has shown that the actual offse values should be like the following:
  constexpr std::array<float32_t, GROUP_SIZE> group_azimuth_offsets{
    6.354F, 4.548F, 2.732F, 0.911F, -0.911F, -2.732F, -4.548F, -6.354F
  };
  constexpr auto eps = std::numeric_limits<float32_t>::epsilon();

  const float32_t US_TO_DEG = (rpm * 360.0F) / (60.0F * 1.0E6F);
  for (uint32_t pt_id = 0U; pt_id < NUM_LASERS; ++pt_id) {
    // Each block has 32 points and 4 firings(groups). There's a constant offset between each
    // firing. All 8 lasers inside a group are assumed to be fired at the same time.
    // Thus the time offset for a point in a block is FIRE_DURATION_US * number_of_groups_fired
    const auto num_groups_fired = static_cast<uint16_t>(std::floor(pt_id / GROUP_SIZE));
    const auto maintenance_offset_us = num_groups_fired < 8U ? 0U : MAINTENANCE_DURATION1_US;
    const auto total_firing_offset_us = num_groups_fired * FIRE_DURATION_US + maintenance_offset_us;
    const auto pt_id_in_group = pt_id % GROUP_SIZE;

    const auto raw_idx = std::roundf(DEG2IDX * (US_TO_DEG * (total_firing_offset_us) +
        group_azimuth_offsets[pt_id_in_group]));
    const auto normalized_idx = raw_idx < -eps ?
      (raw_idx + AZIMUTH_ROTATION_RESOLUTION) : raw_idx;
    m_azimuth_ind[pt_id] = static_cast<uint32_t>(normalized_idx);
  }
}

uint16_t VLS128Data::num_blocks_per_revolution() const noexcept
{
  return m_num_blocks_per_revolution;
}

std::pair<bool8_t, uint16_t> VLS128Data::check_flag(const BlockFlag & flag)
{
  auto valid = (flag[0U] == static_cast<uint8_t>(0xFF));
  auto banked_points = 0U;
  switch (flag[1U]) {
    case static_cast<uint8_t>(0xEE):
      banked_points = 0U;
      break;
    case static_cast<uint8_t>(0xDD):
      banked_points = NUM_POINTS_PER_BLOCK;
      break;
    case static_cast<uint8_t>(0xCC):
      banked_points = 2U * NUM_POINTS_PER_BLOCK;
      break;
    case static_cast<uint8_t>(0xBB):
      banked_points = 3U * NUM_POINTS_PER_BLOCK;
      break;
    default:
      valid = false;
  }
  return std::make_pair(valid, banked_points);
}

void VLS128Data::init_altitude_table()
{
  // altitude angles in degrees: from spec sheet(Figure 9-8)
  constexpr std::array<float32_t, NUM_LASERS> altitude_deg{
    -11.742F, -1.99F, 3.4F, -5.29F, -0.78F, 4.61F, -4.08F, 1.31F,   // 0-7
    -6.5F, -1.11F, 4.28F, -4.41F, 0.1F, 6.48F, -3.2F, 2.19F,        // 8-15
    -3.86F, 1.53F, -9.244F, -1.77F, 2.74F, -5.95F, -0.56F, 4.83F,    // 16-23
    -2.98F, 2.41F, -6.28F, -0.89F, 3.62F, -5.07F, 0.32F, 7.58F,     // 24-31
    ////////////////////////////////////////////////////////////
    -0.34F, 5.18F, -3.64F, 1.75F, -25.0F, -2.43F, 2.96F, -5.73F,     // 32-39
    0.54F, 9.7F, -2.76F, 2.63F, -7.65F, -1.55F, 3.84F, -4.85F,      // 40-47
    3.18F, -5.51F, -0.12F, 5.73F, -4.3F, 1.09F, -16.042F, -2.21F,   // 48-55
    4.06F, -4.63F, 0.76F, 15.0F, -3.42F, 1.97F, -6.85F, -1.33F,     // 56-63
    ////////////////////////////////////////////////////////////
    -5.62F, -0.23F, 5.43F, -3.53F, 0.98F, -19.582F, -2.32F, 3.07F,  // 64-71
    -4.74F, 0.65F, 11.75F, -2.65F, 1.86F, -7.15F, -1.44F, 3.95F,    // 72-79
    -2.1F, 3.29F, -5.4F, -0.01F, 4.5F, -4.19F, 1.2F, -13.565F,     // 80-87
    -1.22F, 4.17F, -4.52F, 0.87F, 6.08F, -3.31F, 2.08F, -6.65F,      // 88-95
    ////////////////////////////////////////////////////////////
    1.42F, -10.346F, -1.88F, 3.51F, -6.06F, -0.67F, 4.72F, -3.97F,  // 96-103
    2.3F, -6.39F, -1.0F, 4.39F, -5.18F, 0.21F, 6.98F, -3.09F,       // 104-111
    4.98F, -3.75F, 1.64F, -8.352F, -2.54F, 2.85F, -5.84F, -0.45F,   // 112-119
    8.43F, -2.87F, 2.52F, -6.17F, -1.66F, 3.73F, -4.96F, 0.43F      // 120-128
  };

  // convert to idx in lookup table
  for (uint16_t idx = 0U; idx < NUM_LASERS; ++idx) {
    int32_t deg_idx = static_cast<int32_t>(altitude_deg[idx] * DEG2IDX);
    if (deg_idx < 0) {
      deg_idx += static_cast<int32_t>(AZIMUTH_ROTATION_RESOLUTION);
    }
    m_altitude_ind[idx] = static_cast<uint32_t>(deg_idx);
  }
}
}  // namespace velodyne_driver
}  // namespace drivers
}  // namespace autoware
