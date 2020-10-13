// Copyright 2020 Embotech AG, Zurich, Switzerland. All rights reserved.
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
#ifndef PARKING_PLANNER__ASTAR_PATH_PLANNER_HPP_
#define PARKING_PLANNER__ASTAR_PATH_PLANNER_HPP_

#include <common/types.hpp>
#include <cmath>
#include <vector>

#include "parking_planner_types.hpp"
#include "visibility_control.hpp"

namespace autoware
{
namespace motion
{
namespace planning
{
namespace parking_planner
{
using autoware::common::types::float64_t;
static constexpr float64_t MY_PI = 3.141592653589793;
static constexpr float64_t DELTA_LONGITUDINAL = 0.25;
static const float64_t DELTA_HEADING = MY_PI / 12.0;
static constexpr float64_t MAX_EXPLORATION_RADIUS = 20.0;
static constexpr size_t MAX_NUM_EXPLORATION_NODES = 100000;

class PARKING_PLANNER_PUBLIC AstarPathPlanner
{
public:
  /// \brief Create an A* path planner
  AstarPathPlanner() = default;

  /// \brief Plan a collision-free but not necessarily dynamically feasible path from a given
  ///        starting state to a given ending state.
  /// \param[in] current_state Starting vehicle state for the path planning
  /// \param[in] goal_state Desired final state for the path planning
  /// \param[in] vehicle_bounding_box Bounding box of the vehicle, used for collision checking
  /// \param[in] obstacles List of bounding boxes of the obstacles to be avoided
  /// \return A path is returned in all cases. On success, the path starts at current_state
  //          and ends at goal_state. On failure, the path is length 1 and only contains the
  //          current_state.
  std::vector<VehicleState<float64_t>>
  plan_astar(
    const VehicleState<float64_t> & current_state,
    const VehicleState<float64_t> & goal_state,
    const Polytope2D<float64_t> & vehicle_bounding_box,
    const std::vector<Polytope2D<float64_t>> & obstacles) const;

private:
  // Currently no state
};

}  // namespace parking_planner
}  // namespace planning
}  // namespace motion
}  // namespace autoware

#endif  // PARKING_PLANNER__ASTAR_PATH_PLANNER_HPP_
