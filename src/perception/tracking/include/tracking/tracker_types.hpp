// Copyright 2021 Apex.AI, Inc.
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

#ifndef TRACKING__TRACKER_TYPES_HPP_
#define TRACKING__TRACKER_TYPES_HPP_

#include <Eigen/Core>

namespace autoware
{
namespace perception
{
namespace tracking
{

/// \brief Maximum number of tracks possible in every timestep
constexpr uint16_t MAX_NUM_TRACKS = 256U;

/// \brief Number of dimensions needed to represent object position for tracking (x and y)
constexpr uint16_t NUM_OBJ_POSE_DIM = 2U;

}  // namespace tracking
}  // namespace perception
}  // namespace autoware

#endif  // TRACKING__TRACKER_TYPES_HPP_
