// Copyright 2018 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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

#include <common/types.hpp>
#include <cstring>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>
#include "velodyne_driver/vlp16_translator.hpp"

namespace autoware
{
namespace drivers
{
namespace velodyne_driver
{

using autoware::common::types::float32_t;
using autoware::common::types::PointXYZIF;

//lint -e9103 not reusing, just using decl NOLINT
static const uint16_t POINTS_PER_FIRE_SEQ = 16U;

////////////////////////////////////////////////////////////////////////////////
Vlp16Translator::Vlp16Translator(const Config & config)
: m_fire_id(0U),
  m_num_firing_per_scan(0U)
{
  // technically don't need these, but pclint would yell at you
  (void)memset(static_cast<void *>(m_altitude_ind), 0, sizeof(m_altitude_ind));
  // do precomputation
  init_tables(config);
}

////////////////////////////////////////////////////////////////////////////////
void Vlp16Translator::convert(const Packet & pkt, std::vector<PointXYZIF> & output)
{
  uint32_t azimuth_next = 0U;
  uint32_t azimuth_diff = 0U;
  output.clear();
  for (uint32_t idx = 0U; idx < NUM_BLOCKS_PER_PACKET; ++idx) {
    const DataBlock & block = pkt.blocks[idx];
    // make sure packet header is correct (UPPER BANK = 0xFFEE
    if ((block.flag[0U] == static_cast<uint8_t>(0xFF)) &&
      (block.flag[1U] == static_cast<uint8_t>(0xEE)))
    {
      // interpret/compute the azimuth for the first and second firing in this block
      // you can also just completely skip out on azimuth interpolation, see init_azimuth_table
      const uint32_t azimuth_base = (idx > 0U) ? azimuth_next :
        to_uint32(block.azimuth_bytes[1U], block.azimuth_bytes[0U]);
      if (idx < (NUM_BLOCKS_PER_PACKET - 1U)) {
        const DataBlock & next_blk = pkt.blocks[idx + 1U];
        azimuth_next = to_uint32(
          next_blk.azimuth_bytes[1U],
          next_blk.azimuth_bytes[0U]);
        azimuth_diff = (((AZIMUTH_ROTATION_RESOLUTION + azimuth_next) - azimuth_base) %
          AZIMUTH_ROTATION_RESOLUTION) / 2U;
      }
      const uint32_t azimuth_second = (azimuth_base + azimuth_diff);

      // get distance/intensity for each point in block, convert to cartesian
      for (uint16_t jdx = 0U; jdx < NUM_POINTS_PER_BLOCK; ++jdx) {
        const DataChannel & channel = block.channels[jdx];
        // rotation angle
        const uint32_t th = (((jdx < POINTS_PER_FIRE_SEQ) ? azimuth_base :
          azimuth_second) + m_azimuth_ind[jdx]) % AZIMUTH_ROTATION_RESOLUTION;
        // distance
        const float32_t r = compute_distance_m(channel.data[1U], channel.data[0U]);

        // altitude angle from firing order
        const uint32_t phi = m_altitude_ind[jdx];
        /// convert from polar to xyz, push to buffer
        PointXYZIF pt;
        polar_to_xyz(pt, r, th, phi);
        // beam intensity
        pt.intensity = m_intensity_table[channel.data[2U]];
        pt.id = static_cast<uint16_t>((jdx < POINTS_PER_FIRE_SEQ) ? m_fire_id :
          (m_fire_id + 1U));
        output.push_back(pt);
        // No need to check if this will overflow because of static asserts
      }
      // prevent overflow
      m_fire_id = static_cast<uint16_t>(m_fire_id + 2U);
      // check fire_id against count, push end of scan message
      if (m_fire_id >= m_num_firing_per_scan) {
        // I can use a point in the middle of a block because I know a packet is 384 points,
        // whereas a PointBlock can fit 512 points
        PointXYZIF pt;
        pt.id =
          static_cast<uint16_t>(PointXYZIF::END_OF_SCAN_ID);
        output.push_back(pt);

        m_fire_id = static_cast<uint16_t>(0U);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void Vlp16Translator::init_tables(const Config & config)
{
  init_azimuth_table_num_points(
    clamp<float32_t>(config.get_rpm(), MIN_RPM, MAX_RPM));
  init_trig_tables();
  init_altitude_table();
  init_intensity_table();
}

////////////////////////////////////////////////////////////////////////////////
void Vlp16Translator::init_altitude_table()
{
  // TODO(christopher.ho) from calibration file if available
  // altitude angles in degrees: from spec sheet
  float32_t altitude_deg[NUM_POINTS_PER_BLOCK];
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
  // repeat
  altitude_deg[16U] = -15.0F;
  altitude_deg[17U] = 1.0F;
  altitude_deg[18U] = -13.0F;
  altitude_deg[19U] = 3.0F;
  altitude_deg[20U] = -11.0F;
  altitude_deg[21U] = 5.0F;
  altitude_deg[22U] = -9.0F;
  altitude_deg[23U] = 7.0F;
  altitude_deg[24U] = -7.0F;
  altitude_deg[25U] = 9.0F;
  altitude_deg[26U] = -5.0F;
  altitude_deg[27U] = 11.0F;
  altitude_deg[28U] = -3.0F;
  altitude_deg[29U] = 13.0F;
  altitude_deg[30U] = -1.0F;
  altitude_deg[31U] = 15.0F;
  // convert to idx in lookup table
  for (uint16_t idx = 0U; idx < NUM_POINTS_PER_BLOCK; ++idx) {
    // multiply by 2/3 to get VLP16-HiRes 20 deg FoV (instead of 30 deg)
    const float32_t deg_raw = roundf(((altitude_deg[idx] * 2.0F) / 3.0F) * DEG2IDX);
    int32_t deg_idx = static_cast<int32_t>(deg_raw);
    if (deg_idx < 0) {
      deg_idx += static_cast<int32_t>(AZIMUTH_ROTATION_RESOLUTION);
    }
    m_altitude_ind[idx] = static_cast<uint32_t>(deg_idx);
  }
}

////////////////////////////////////////////////////////////////////////////////
void Vlp16Translator::init_azimuth_table_num_points(const float32_t rpm)
{
  // convert to a degree offset
  const float32_t US_TO_DEG = (rpm * 360.0F) / (60.0F * 1.0E6F);
  // us = 55.296*fire_seq + 2.304*(point idx % 16)
  for (uint32_t idx = 0U; idx < NUM_POINTS_PER_BLOCK; ++idx) {
    // This line may be removed if you do not want to do azimuth interpolation in convert_xyz
    const uint32_t jdx = (idx >= POINTS_PER_FIRE_SEQ) ? (idx - POINTS_PER_FIRE_SEQ) : idx;
    m_azimuth_ind[idx] = static_cast<uint32_t>(
      roundf(DEG2IDX * US_TO_DEG * (FIRE_DURATION_US * static_cast<float32_t>(jdx))));
  }
  //  32 [points/block] * (60E6 us/min * x min/rev = us/rev) * (BLOCK/us)
  m_num_firing_per_scan = static_cast<uint16_t>(std::ceil(60.0E6F / (rpm * FIRE_SEQ_OFFSET_US)));
}

////////////////////////////////////////////////////////////////////////////////
void Vlp16Translator::init_trig_tables()
{
  const float32_t IDX2RAD =
    TAU / static_cast<float32_t>(AZIMUTH_ROTATION_RESOLUTION);
  for (uint64_t idx = 0U; idx < AZIMUTH_ROTATION_RESOLUTION; ++idx) {
    m_cos_table[idx] = cosf((static_cast<float32_t>(idx)) * IDX2RAD);
    m_sin_table[idx] = sinf((static_cast<float32_t>(idx)) * IDX2RAD);
  }
}

////////////////////////////////////////////////////////////////////////////////
void Vlp16Translator::init_intensity_table()
{
  for (uint64_t idx = 0U; idx < NUM_INTENSITY_VALUES; ++idx) {
    m_intensity_table[idx] = static_cast<float32_t>(idx);
  }
}

}  // namespace velodyne_driver
}  // namespace drivers
}  // namespace autoware
