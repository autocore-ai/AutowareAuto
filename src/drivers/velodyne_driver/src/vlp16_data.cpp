// Copyright 2018-2020 the Autoware Foundation
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

#include <velodyne_driver/vlp16_data.hpp>
#include <utility>

namespace autoware
{
namespace drivers
{
namespace velodyne_driver
{
VLP16Data::VLP16Data(const float32_t rpm)
{
  init_azimuth_table(rpm);
  init_altitude_table();
  // (60E6 us/min * x min/rev = us/rev) * (seq/us * block/seq) = block / rev
  m_num_blocks_per_revolution =
    static_cast<uint16_t>(std::ceil(60.0E6F / (rpm * FIRE_SEQ_OFFSET_US * 2.0F)));
}

uint32_t VLP16Data::azimuth_offset(uint16_t, uint32_t, uint32_t pt_id) const
{
  return m_azimuth_ind[pt_id];
}

uint32_t VLP16Data::altitude(uint16_t, uint32_t, uint32_t pt_id) const
{
  return m_altitude_ind[pt_id % NUM_LASERS];
}

uint16_t VLP16Data::seq_id(uint16_t num_blocks, uint32_t pt_id) const noexcept
{
  const auto num_seqs = static_cast<uint16_t>(NUM_SEQUENCES_PER_BLOCK * num_blocks);
  return (pt_id < NUM_LASERS) ? num_seqs : static_cast<uint16_t>(num_seqs + uint16_t{1U});
}

void VLP16Data::init_azimuth_table(const float32_t rpm)
{
  const float32_t US_TO_DEG = (rpm * 360.0F) / (60.0F * 1.0E6F);
  // This is how much the vlp16 turns after a single firing sequence. Unit in index (deg * 100)
  const auto azimuth_per_sequence{
    static_cast<uint32_t>(std::roundf(US_TO_DEG * DEG2IDX * FIRE_SEQ_OFFSET_US))};

  // us = 55.296*fire_seq + 2.304*(point idx % 16)
  for (uint32_t pt_id = 0U; pt_id < NUM_POINTS_PER_BLOCK; ++pt_id) {
    // If the pt_id is from the second firing sequence, we need to add the azimuth offset
    // of a single sequence.
    auto offset = pt_id >= NUM_LASERS ? azimuth_per_sequence : 0U;
    m_azimuth_ind[pt_id] = static_cast<uint32_t>(
      roundf(DEG2IDX * US_TO_DEG * (FIRE_DURATION_US * static_cast<float32_t>(pt_id)))) + offset;
  }
}

std::pair<bool8_t, uint16_t> VLP16Data::check_flag(const BlockFlag & flag)
{
  const auto valid = (flag[0U] == static_cast<uint8_t>(0xFF)) &&
    (flag[1U] == static_cast<uint8_t>(0xEE));

  return std::make_pair(valid, 0U);
}


uint16_t VLP16Data::num_blocks_per_revolution() const noexcept
{
  return m_num_blocks_per_revolution;
}

void VLP16Data::init_altitude_table()
{
  // TODO(christopher.ho) from calibration file if available
  // altitude angles in degrees: from spec sheet
  float32_t altitude_deg[16U];
  altitude_deg[0U] = -15.0F;
  altitude_deg[1U] = 1.0F;
  altitude_deg[2U] = -13.0F;
  altitude_deg[3U] = 3.0F;
  altitude_deg[4U] = -11.0F;
  altitude_deg[5U] = 5.0F;
  altitude_deg[6U] = -9.0F;
  altitude_deg[7U] = 7.0F;
  altitude_deg[8U] = -7.0F;
  altitude_deg[9U] = 9.0F;
  altitude_deg[10U] = -5.0F;
  altitude_deg[11U] = 11.0F;
  altitude_deg[12U] = -3.0F;
  altitude_deg[13U] = 13.0F;
  altitude_deg[14U] = -1.0F;
  altitude_deg[15U] = 15.0F;
  // convert to idx in lookup table
  for (uint16_t idx = 0U; idx < NUM_LASERS; ++idx) {
    // multiply by 2/3 to get VLP16-HiRes 20 deg FoV (instead of 30 deg)
    const float32_t deg_raw = roundf(((altitude_deg[idx] * 2.0F) / 3.0F) * DEG2IDX);
    int32_t deg_idx = static_cast<int32_t>(deg_raw);
    if (deg_idx < 0) {
      deg_idx += static_cast<int32_t>(AZIMUTH_ROTATION_RESOLUTION);
    }
    m_altitude_ind[idx] = static_cast<uint32_t>(deg_idx);
  }
}

}  // namespace velodyne_driver
}  // namespace drivers
}  // namespace autoware
