// Copyright 2017-2020 the Autoware Foundation, Arm Limited
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
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <gtest/gtest.h>
#include <common/types.hpp>

#include <tuple>
#include <vector>

#include "test_ray_ground_classifier_aux.hpp"

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

class RayGroundClassifierRaytraceVlp16 : public ::testing::Test
{
public:
  RayGroundClassifierRaytraceVlp16()
  : cfg{
      0.0F,        // sensor_height_m,
      5.0F,        // max_local_slope_deg,
      2.7F,        // max_global_slope_deg,
      25.0F,       // nonground_retro_thresh_deg,
      0.05F,       // min_height_thresh_m,
      0.9F,        // max_global_height_thresh_m,
      1.8F,        // max_last_local_ground_thresh_m,
      2.0,         // max_provisional_ground_distance_m
  }
  {
  }

protected:
  std::vector<std::tuple<float32_t, float32_t, uint8_t>> dat;
  std::vector<PointXYZIF> pts;
  std::vector<bool8_t> labels;
  Config cfg;
};


TEST_F(RayGroundClassifierRaytraceVlp16, TwentyMFlat)
{
  // one ray from ray_trace.py
  dat.push_back(std::make_tuple(5.69501861320422, 0, 0));
  dat.push_back(std::make_tuple(15.052089695573766, 0, 0));
  dat.push_back(std::make_tuple(6.1954674865359065, 0, 0));
  dat.push_back(std::make_tuple(18.766816572574122, 0, 0));
  dat.push_back(std::make_tuple(6.783612633309131, 0, 0));
  dat.push_back(std::make_tuple(20, 0.35107679215055887, 1));
  dat.push_back(std::make_tuple(7.485552032068151, 0, 0));
  dat.push_back(std::make_tuple(20, 0.81820889238293182, 1));
  dat.push_back(std::make_tuple(8.338871087793343, 0, 0));
  dat.push_back(std::make_tuple(20, 1.2842855955177668, 1));
  dat.push_back(std::make_tuple(9.399745637461052, 0, 0));
  dat.push_back(std::make_tuple(10.756059060537655, 0, 0));
  dat.push_back(std::make_tuple(12.553450412102858, 0, 0));

  generate_points(cfg, dat, pts, labels);
  label_and_check(cfg, pts, labels);
}

TEST_F(RayGroundClassifierRaytraceVlp16, TwentyfiveMFlat)
{
  // one ray from ray_trace.py
  dat.push_back(std::make_tuple(5.6950186132, 0.0, 0));
  dat.push_back(std::make_tuple(15.0520896956, 0.0, 0));
  dat.push_back(std::make_tuple(6.19546748654, 0.0, 0));
  dat.push_back(std::make_tuple(18.7668165726, 0.0, 0));
  dat.push_back(std::make_tuple(6.78361263331, 0.0, 0));
  dat.push_back(std::make_tuple(24.8797154738, 0.0, 1));
  dat.push_back(std::make_tuple(7.48555203207, 0.0, 0));
  dat.push_back(std::make_tuple(25.0, 0.575261115479, 1));
  dat.push_back(std::make_tuple(8.33887108779, 0.0, 0));
  dat.push_back(std::make_tuple(25.0, 1.1578569944, 1));
  dat.push_back(std::make_tuple(9.39974563746, 0.0, 0));
  dat.push_back(std::make_tuple(10.7560590605, 0.0, 0));
  dat.push_back(std::make_tuple(12.5534504121, 0.0, 0));

  generate_points(cfg, dat, pts, labels);
  label_and_check(cfg, pts, labels);
}

TEST_F(RayGroundClassifierRaytraceVlp16, ThirtyMFlat)
{
  // one ray from ray_trace.py
  dat.push_back(std::make_tuple(5.6950186132, 0.0, 0));
  dat.push_back(std::make_tuple(15.0520896956, 0.0, 0));
  dat.push_back(std::make_tuple(6.19546748654, 0.0, 0));
  dat.push_back(std::make_tuple(18.7668165726, 0.0, 0));
  dat.push_back(std::make_tuple(6.78361263331, 0.0, 0));
  dat.push_back(std::make_tuple(24.8797154738, 0.0, 0));
  dat.push_back(std::make_tuple(7.48555203207, 0.0, 0));
  dat.push_back(std::make_tuple(30.0, 0.332313338574, 1));
  dat.push_back(std::make_tuple(8.33887108779, 0.0, 0));
  dat.push_back(std::make_tuple(30.0, 1.03142839328, 1));
  dat.push_back(std::make_tuple(9.39974563746, 0.0, 0));
  dat.push_back(std::make_tuple(10.7560590605, 0.0, 0));
  dat.push_back(std::make_tuple(12.5534504121, 0.0, 0));

  generate_points(cfg, dat, pts, labels);
  label_and_check(cfg, pts, labels);
}

TEST_F(RayGroundClassifierRaytraceVlp16, ThirtyfiveMFlat)
{
  // one ray from ray_trace.py
  dat.push_back(std::make_tuple(5.6950186132, 0.0, 0));
  dat.push_back(std::make_tuple(15.0520896956, 0.0, 0));
  dat.push_back(std::make_tuple(6.19546748654, 0.0, 0));
  dat.push_back(std::make_tuple(18.7668165726, 0.0, 0));
  dat.push_back(std::make_tuple(6.78361263331, 0.0, 0));
  dat.push_back(std::make_tuple(24.8797154738, 0.0, 0));
  dat.push_back(std::make_tuple(7.48555203207, 0.0, 0));
  dat.push_back(std::make_tuple(35.0, 0.0893655616701, 1));
  dat.push_back(std::make_tuple(8.33887108779, 0.0, 0));
  dat.push_back(std::make_tuple(35.0, 0.904999792156, 1));
  dat.push_back(std::make_tuple(9.39974563746, 0.0, 0));
  dat.push_back(std::make_tuple(10.7560590605, 0.0, 0));
  dat.push_back(std::make_tuple(12.5534504121, 0.0, 0));

  generate_points(cfg, dat, pts, labels);
  label_and_check(cfg, pts, labels);
}

////////////////////////////////////////////////////////////////////
TEST_F(RayGroundClassifierRaytraceVlp16, TwentyMIncline)
{
  // one ray from ray_trace.py
  dat.push_back(std::make_tuple(5.46422494495, 0.0362703525863, 0));
  dat.push_back(std::make_tuple(12.0738486, 0.177086759009, 0));
  dat.push_back(std::make_tuple(5.90250107301, 0.0423220589369, 0));
  dat.push_back(std::make_tuple(13.8680085948, 0.233626897942, 0));
  dat.push_back(std::make_tuple(6.40579772957, 0.0498472358505, 0));
  dat.push_back(std::make_tuple(16.1126818117, 0.315377205011, 0));
  dat.push_back(std::make_tuple(6.98923945962, 0.0593409477938, 0));
  dat.push_back(std::make_tuple(19.9838150309, 0.818995312834, 1));
  dat.push_back(std::make_tuple(7.67258299401, 0.0715118194849, 0));
  dat.push_back(std::make_tuple(19.9611584508, 1.28526773206, 1));
  dat.push_back(std::make_tuple(8.48189243847, 0.0873937066786, 0));
  dat.push_back(std::make_tuple(19.9385797482, 1.74993743164, 1));
  dat.push_back(std::make_tuple(9.45182361709, 0.108524015657, 0));
  dat.push_back(std::make_tuple(10.6286330051, 0.137230125799, 0));

  generate_points(cfg, dat, pts, labels);
  label_and_check(cfg, pts, labels, false);
}

TEST_F(RayGroundClassifierRaytraceVlp16, TwentyfiveMIncline)
{
  // one ray from ray_trace.py
  dat.push_back(std::make_tuple(5.46422494495, 0.0362703525863, 0));
  dat.push_back(std::make_tuple(12.0738486, 0.177086759009, 0));
  dat.push_back(std::make_tuple(5.90250107301, 0.0423220589369, 0));
  dat.push_back(std::make_tuple(13.8680085948, 0.233626897942, 0));
  dat.push_back(std::make_tuple(6.40579772957, 0.0498472358505, 0));
  dat.push_back(std::make_tuple(16.1126818117, 0.315377205011, 0));
  dat.push_back(std::make_tuple(6.98923945962, 0.0593409477938, 0));
  dat.push_back(std::make_tuple(18.9270496556, 0.435171536281, 0));
  dat.push_back(std::make_tuple(7.67258299401, 0.0715118194849, 0));
  dat.push_back(std::make_tuple(24.9757508572, 1.15847015144, 1));
  dat.push_back(std::make_tuple(8.48189243847, 0.0873937066786, 0));
  dat.push_back(std::make_tuple(24.9404364102, 1.73988720605, 1));
  dat.push_back(std::make_tuple(9.45182361709, 0.108524015657, 0));
  dat.push_back(std::make_tuple(26.5105533045, 2.35374546304, 1));
  dat.push_back(std::make_tuple(10.6286330051, 0.137230125799, 0));

  generate_points(cfg, dat, pts, labels);
  label_and_check(cfg, pts, labels);
}

TEST_F(RayGroundClassifierRaytraceVlp16, ThirtyMIncline)
{
  // one ray from ray_trace.py
  dat.push_back(std::make_tuple(5.46422494495, 0.0362703525863, 0));
  dat.push_back(std::make_tuple(12.0738486, 0.177086759009, 0));
  dat.push_back(std::make_tuple(5.90250107301, 0.0423220589369, 0));
  dat.push_back(std::make_tuple(13.8680085948, 0.233626897942, 0));
  dat.push_back(std::make_tuple(6.40579772957, 0.0498472358505, 0));
  dat.push_back(std::make_tuple(16.1126818117, 0.315377205011, 0));
  dat.push_back(std::make_tuple(6.98923945962, 0.0593409477938, 0));
  dat.push_back(std::make_tuple(18.9270496556, 0.435171536281, 0));
  dat.push_back(std::make_tuple(7.67258299401, 0.0715118194849, 0));
  dat.push_back(std::make_tuple(22.4339184226, 0.611371107618, 0));
  dat.push_back(std::make_tuple(8.48189243847, 0.0873937066786, 0));
  dat.push_back(std::make_tuple(29.9536064295, 1.72981424855, 1));
  dat.push_back(std::make_tuple(9.45182361709, 0.108524015657, 0));
  dat.push_back(std::make_tuple(29.9028725594, 2.42588294607, 1));
  dat.push_back(std::make_tuple(10.6286330051, 0.137230125799, 0));

  generate_points(cfg, dat, pts, labels);
  label_and_check(cfg, pts, labels);
}

TEST_F(RayGroundClassifierRaytraceVlp16, ThirtyfiveMIncline)
{
  // one ray from ray_trace.py
  dat.push_back(std::make_tuple(5.46422494495, 0.0362703525863, 0));
  dat.push_back(std::make_tuple(12.0738486, 0.177086759009, 0));
  dat.push_back(std::make_tuple(5.90250107301, 0.0423220589369, 0));
  dat.push_back(std::make_tuple(13.8680085948, 0.233626897942, 0));
  dat.push_back(std::make_tuple(6.40579772957, 0.0498472358505, 0));
  dat.push_back(std::make_tuple(16.1126818117, 0.315377205011, 0));
  dat.push_back(std::make_tuple(6.98923945962, 0.0593409477938, 0));
  dat.push_back(std::make_tuple(18.9270496556, 0.435171536281, 0));
  dat.push_back(std::make_tuple(7.67258299401, 0.0715118194849, 0));
  dat.push_back(std::make_tuple(22.4339184226, 0.611371107618, 0));
  dat.push_back(std::make_tuple(8.48189243847, 0.0873937066786, 0));
  dat.push_back(std::make_tuple(34.9803045178, 1.7197141091, 1));
  dat.push_back(std::make_tuple(9.45182361709, 0.108524015657, 0));
  dat.push_back(std::make_tuple(34.9111998713, 2.53238475187, 1));
  dat.push_back(std::make_tuple(10.6286330051, 0.137230125799, 0));

  generate_points(cfg, dat, pts, labels);
  label_and_check(cfg, pts, labels, false);
}

//////////////////////////////////////////////////////////
TEST_F(RayGroundClassifierRaytraceVlp16, TwentyMDecline)
{
  // one ray from ray_trace.py
  dat.push_back(std::make_tuple(5.97056702129, -0.0433037786148, 0));
  dat.push_back(std::make_tuple(6.55700813257, -0.0522283231908, 0));
  dat.push_back(std::make_tuple(20.0178127905, -0.119321421476, 1));
  dat.push_back(std::make_tuple(7.27028348265, -0.0642092102989, 0));
  dat.push_back(std::make_tuple(20.0405281631, 0.348160946431, 1));
  dat.push_back(std::make_tuple(8.16247484072, -0.080935368714, 0));
  dat.push_back(std::make_tuple(20.0632189388, 0.815137112253, 1));
  dat.push_back(std::make_tuple(9.3225354862, -0.105575398313, 0));
  dat.push_back(std::make_tuple(10.9215375464, -0.144897937776, 0));
  dat.push_back(std::make_tuple(13.3630150178, -0.216921975664, 0));
  dat.push_back(std::make_tuple(18.1929635632, -0.402069877564, 0));

  generate_points(cfg, dat, pts, labels);
  label_and_check(cfg, pts, labels, false);
}

TEST_F(RayGroundClassifierRaytraceVlp16, TwentyfiveMDecline)
{
  // one ray from ray_trace.py
  dat.push_back(std::make_tuple(5.97056702129, -0.0433037786148, 0));
  dat.push_back(std::make_tuple(6.55700813257, -0.0522283231908, 0));
  dat.push_back(std::make_tuple(25.0099462801, -0.595476709286, 1));
  dat.push_back(std::make_tuple(7.27028348265, -0.0642092102989, 0));
  dat.push_back(std::make_tuple(25.045390705, -0.0119196967561, 1));
  dat.push_back(std::make_tuple(8.16247484072, -0.080935368714, 0));
  dat.push_back(std::make_tuple(25.0808167232, 0.57133426683, 1));
  dat.push_back(std::make_tuple(9.3225354862, -0.105575398313, 0));
  dat.push_back(std::make_tuple(10.9215375464, -0.144897937776, 0));
  dat.push_back(std::make_tuple(13.3630150178, -0.216921975664, 0));
  dat.push_back(std::make_tuple(18.1929635632, -0.402069877564, 0));

  generate_points(cfg, dat, pts, labels);
  label_and_check(cfg, pts, labels, false);
}

TEST_F(RayGroundClassifierRaytraceVlp16, ThirtyMDecline)
{
  // one ray from ray_trace.py
  dat.push_back(std::make_tuple(5.97056702129, -0.0433037786148, 0));
  dat.push_back(std::make_tuple(6.55700813257, -0.0522283231908, 0));
  dat.push_back(std::make_tuple(30.0015823548, -1.07158455311, 1));
  dat.push_back(std::make_tuple(7.27028348265, -0.0642092102989, 0));
  dat.push_back(std::make_tuple(30.0525603595, -0.372166327831, 1));
  dat.push_back(std::make_tuple(8.16247484072, -0.080935368714, 0));
  dat.push_back(std::make_tuple(30.1035405837, 0.327282347649, 1));
  dat.push_back(std::make_tuple(9.3225354862, -0.105575398313, 0));
  dat.push_back(std::make_tuple(10.9215375464, -0.144897937776, 0));
  dat.push_back(std::make_tuple(13.3630150178, -0.216921975664, 0));
  dat.push_back(std::make_tuple(18.1929635632, -0.402069877564, 0));

  generate_points(cfg, dat, pts, labels);
  label_and_check(cfg, pts, labels, false);
}
