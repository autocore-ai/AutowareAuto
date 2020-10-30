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
#include <utility>
#include <limits>

namespace autoware
{
namespace drivers
{
namespace velodyne_driver
{

VLP32CData::VLP32CData(const float32_t rpm)
{
  init_azimuth_table(rpm);
  init_altitude_table();
  // (60E6 us/min * x min/rev = us/rev) * (seq/us * block/seq) = block / rev
  m_num_blocks_per_revolution =
    static_cast<uint16_t>(std::floor(60.0E6F /
    (rpm * FIRE_SEQ_OFFSET_US * NUM_SEQUENCES_PER_BLOCK)));
}

uint32_t VLP32CData::azimuth_offset(uint16_t, uint32_t, uint32_t pt_id) const
{
  return m_azimuth_ind[pt_id];
}

uint32_t VLP32CData::altitude(uint16_t, uint32_t, uint32_t pt_id) const
{
  return m_altitude_ind[pt_id];
}

uint16_t VLP32CData::seq_id(uint16_t num_blocks, uint32_t) const noexcept
{
  return num_blocks;
}

void VLP32CData::init_azimuth_table(const float32_t rpm)
{
  // Table 9-2
  constexpr std::array<float32_t, NUM_LASERS> azimuth_offsets{
    1.4F, -4.2F, 1.4F, -1.4F, 1.4F, -1.4F, 4.2F, -1.4F,
    1.4F, -4.2F, 1.4F, -1.4F, 4.2F, -1.4F, 4.2F, -1.4F,
    1.4F, -4.2F, 1.4F, -4.2F, 4.2F, -1.4F, 1.4F, -1.4F,
    1.4F, -1.4F, 1.4F, -4.2F, 4.2F, -1.4F, 1.4F, -1.4F
  };
  constexpr auto eps = std::numeric_limits<float32_t>::epsilon();

  const float32_t US_TO_DEG = (rpm * 360.0F) / (60.0F * 1.0E6F);
  for (uint32_t pt_id = 0U; pt_id < NUM_LASERS; ++pt_id) {
    const uint32_t group_id = pt_id / GROUP_SIZE;

    const auto raw_idx = std::roundf(DEG2IDX * (US_TO_DEG *
        (static_cast<float32_t>(group_id) * FIRE_DURATION_US) + azimuth_offsets[pt_id]));
    const auto normalized_idx = raw_idx < -eps ?
      (raw_idx + AZIMUTH_ROTATION_RESOLUTION) : raw_idx;
    m_azimuth_ind[pt_id] = static_cast<uint32_t>(normalized_idx);
  }
}

uint16_t VLP32CData::num_blocks_per_revolution() const noexcept
{
  return m_num_blocks_per_revolution;
}

std::pair<bool8_t, uint16_t> VLP32CData::check_flag(const BlockFlag & flag)
{
  const auto valid = (flag[0U] == static_cast<uint8_t>(0xFF)) &&
    (flag[1U] == static_cast<uint8_t>(0xEE));
  return std::make_pair(valid, 0U);
}

void VLP32CData::init_altitude_table()
{
  // altitude angles in degrees: from spec sheet(Table 9-2)
  constexpr std::array<float32_t, NUM_LASERS> altitude_deg{
    25.0F, -1.0F, -1.667F, -15.639F, -11.31F, 0.0F, -0.667F, -8.843F,
    -7.254F, 0.333F, -0.333F, -6.148F, -5.333F, 1.333F, 0.667F, -4.0F,
    -4.667F, 1.667F, 1.0F, -3.667F, -3.333F, 3.333F, 2.333F, -2.667F,
    -3.0F, 7.0F, 4.667F, -2.333F, -2.0F, 15.0F, 10.333F, -1.333F
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
