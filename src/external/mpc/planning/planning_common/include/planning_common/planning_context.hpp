// Copyright 2019 Christopher Ho
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

#ifndef PLANNING_COMMON__PLANNING_CONTEXT_HPP_
#define PLANNING_COMMON__PLANNING_CONTEXT_HPP_

#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <time_utils/time_utils.hpp>
#include <tf2/buffer_core.h>
#include <tf2_msgs/msg/tf_message.hpp>

#include <planning_common/visibility_control.hpp>

#include <algorithm>
#include <chrono>
#include <list>
#include <string>

namespace motion
{
namespace planning
{
namespace planning_common
{
using State = autoware_auto_msgs::msg::VehicleKinematicState;
using Trajectory = autoware_auto_msgs::msg::Trajectory;
using Transform = geometry_msgs::msg::TransformStamped;
using Transforms = tf2_msgs::msg::TFMessage;

/// Controls the planning environment lookup and storage
class PLANNING_COMMON_PUBLIC EnvironmentConfig
{
public:
  explicit EnvironmentConfig(std::chrono::nanoseconds max_allowable_time_error);

  std::chrono::nanoseconds max_allowable_time_error() const noexcept;

private:
  std::chrono::nanoseconds m_max_allowable_time_error;
};  // class EnvironmntConfig

/// Represents the general setup of a motion planning problem at a given time stamp and in a given
/// coordinate frame. A context is only available through an Environment.
class PLANNING_COMMON_PUBLIC PlanningContext
{
public:
  /// A (planning) environment maintains some history of the environment: bounding boxes, lanes
  /// current state, target state, etc. When requested, it appropriately transforms and
  /// interpolates the full set of objects into a consistent coordinate frame at a given time stamp
  class Environment
  {
    template<typename T>
    using Buffer = std::list<T>;
    using Clock = std::chrono::system_clock;

public:
    explicit Environment(const EnvironmentConfig & cfg);

    /// Add state to queue; interpolation is done on these things
    void add_ego_state(const State & state);
    void add_ego_state(State && state);

    /// Add target states to queue; no interpolation is done
    void add_target_state(const State & state);
    void add_target_state(State && state);

    // TODO(c.ho) add lanes, bounding boxes

    /// Add transforms
    void add_transform(const Transforms & tfs);
    void add_transform(Transform && tf);
    void add_transform(const Transform & tf);

    /// Clear all data from before the given time point; doesn't affect transforms
    void clear_before(Clock::time_point stamp);

    /// Primary API: get a given planning context for a specified time and frame
    /// throws if a frame or object cannot be provided or populated.
    /// Interpolation only occurs if relevant (i.e. ego state). All other aspects are populated
    /// with data just after the specified time stamp
    PlanningContext context(const std::string & frame_id, Clock::time_point stamp) const;
    /// Check if it is possible to construct a valid context at a given frame and time
    bool has_valid_context(const std::string & frame_id, Clock::time_point stamp) const;

    /// Basic getter
    const EnvironmentConfig & config() const noexcept;

private:
    template<typename T>
    static typename Buffer<T>::const_iterator find_newer_than(
      const Buffer<T> & buff,
      const std::chrono::system_clock::time_point t)
    {
      const auto fn = [t](const T & s) -> bool {
          return time_utils::from_message(s.header.stamp) >= t;
        };
      return std::find_if(buff.cbegin(), buff.cend(), fn);
    }

    /// Check if it's possible to transform the frame w/ bonus equality check
    PLANNING_COMMON_LOCAL bool frame_ok(
      const std::string & target_frame,
      const std::string & current_frame,
      Clock::time_point stamp) const;
    /// Get target state based on time, untransformed
    PLANNING_COMMON_LOCAL const State * get_target_state(Clock::time_point stamp) const;
    /// Try to transform a state into the target frame; stamp from state is not respected
    PLANNING_COMMON_LOCAL State transform_state(
      const State & in,
      const std::string & frame,
      Clock::time_point stamp) const;
    /// Semi-duplicated logic, return true if interpolated_ego would succeeed
    PLANNING_COMMON_LOCAL
    bool can_interpolate_ego(const std::string & frame, Clock::time_point stamp) const;
    /// Compute ego state, possibly interpolated
    PLANNING_COMMON_LOCAL
    State interpolated_ego(const std::string & frame, Clock::time_point stamp) const;
    /// Throw error if duration is inadmissable
    PLANNING_COMMON_LOCAL bool check_duration(std::chrono::nanoseconds duration) const;

    EnvironmentConfig m_config;
    tf2::BufferCore m_tf_buffer{};
    Buffer<State> m_ego_states{};
    Buffer<State> m_target_states{};
  };  // class Environment

  /// Getters
  State ego_state() const noexcept;
  State target_state() const noexcept;

  // TODO(c.ho) more stuff

private:
  PlanningContext(State ego_state, State target_state);
  State m_ego_state;
  State m_target_state;
};  // class PlanningContext

using PlanningEnvironment = PlanningContext::Environment;
}  // namespace planning_common
}  // namespace planning
}  // namespace motion
#endif  // PLANNING_COMMON__PLANNING_CONTEXT_HPP_
