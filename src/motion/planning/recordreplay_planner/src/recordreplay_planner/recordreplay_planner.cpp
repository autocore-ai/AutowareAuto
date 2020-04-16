// Copyright 2020 Embotech AG, Zurich, Switzerland, inspired by Christopher Ho's mpc code
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

#include "recordreplay_planner/recordreplay_planner.hpp"

#include <geometry_msgs/msg/point32.hpp>
#include <geometry/common_2d.hpp>
#include <geometry/intersection.hpp>
#include <geometry/vehicle_bounding_box.hpp>
#include <time_utils/time_utils.hpp>
#include <motion_common/motion_common.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "rcl/types.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rmw/rmw.h"
#include "rmw/serialized_message.h"

namespace motion
{
namespace planning
{
namespace recordreplay_planner
{
using geometry_msgs::msg::Point32;
using motion::motion_common::to_angle;
using autoware::common::geometry::intersect;
using autoware::common::geometry::compute_boundingbox_from_trajectorypoint;

RecordReplayPlanner::RecordReplayPlanner(const VehicleConfig & vehicle_param)
: m_vehicle_param(vehicle_param)
{
}

// These may do more in the future
bool RecordReplayPlanner::is_recording() const noexcept
{
  return m_recordreplaystate == RecordReplayState::RECORDING;
}

bool RecordReplayPlanner::is_replaying() const noexcept
{
  return m_recordreplaystate == RecordReplayState::REPLAYING;
}

void RecordReplayPlanner::start_recording() noexcept
{
  m_recordreplaystate = RecordReplayState::RECORDING;
}

void RecordReplayPlanner::stop_recording() noexcept
{
  m_recordreplaystate = RecordReplayState::IDLE;
}

void RecordReplayPlanner::start_replaying() noexcept
{
  m_recordreplaystate = RecordReplayState::REPLAYING;
}

void RecordReplayPlanner::stop_replaying() noexcept
{
  m_recordreplaystate = RecordReplayState::IDLE;
}

void RecordReplayPlanner::clear_record() noexcept
{
  m_record_buffer.clear();
}

std::size_t RecordReplayPlanner::get_record_length() const noexcept
{
  return m_record_buffer.size();
}


void RecordReplayPlanner::set_heading_weight(double heading_weight)
{
  if (heading_weight < 0.0) {
    throw std::domain_error{"Negative weights do not make sense"};
  }
  m_heading_weight = heading_weight;
}

double RecordReplayPlanner::get_heading_weight()
{
  return m_heading_weight;
}

void RecordReplayPlanner::set_min_record_distance(double min_record_distance)
{
  if (min_record_distance < 0.0) {
    throw std::domain_error{"Negative minumum distance do not make sense"};
  }
  m_min_record_distance = min_record_distance;
}

double RecordReplayPlanner::get_min_record_distance() const
{
  return m_min_record_distance;
}


void RecordReplayPlanner::record_state(const State & state_to_record)
{
  if (m_record_buffer.empty()) {
    m_record_buffer.push_back(state_to_record);
    return;
  }

  auto previous_state = m_record_buffer.back();
  auto distance_sq = (state_to_record.state.x - previous_state.state.x) *
    (state_to_record.state.x - previous_state.state.x) +
    (state_to_record.state.y - previous_state.state.y) *
    (state_to_record.state.y - previous_state.state.y);

  if (distance_sq >= (m_min_record_distance * m_min_record_distance) ) {
    m_record_buffer.push_back(state_to_record);
  }
}

const Trajectory & RecordReplayPlanner::plan(const State & current_state)
{
  return from_record(current_state);
}


std::size_t RecordReplayPlanner::get_closest_state(const State & current_state)
{
  // Find the closest point to the current state in the stored states buffer
  const auto distance_from_current_state =
    [this, &current_state](State & other_state) {
      const auto s1 = current_state.state, s2 = other_state.state;
      return (s1.x - s2.x) * (s1.x - s2.x) + (s1.y - s2.y) * (s1.y - s2.y) +
             m_heading_weight * std::abs(to_angle(s1.heading - s2.heading));
    };
  const auto comparison_function =
    [&distance_from_current_state](State & one, State & two)
    {return distance_from_current_state(one) < distance_from_current_state(two);};


  const auto minimum_index_iterator =
    std::min_element(std::begin(m_record_buffer), std::end(m_record_buffer),
      comparison_function);
  auto minimum_idx = std::distance(std::begin(m_record_buffer), minimum_index_iterator);

  return minimum_idx;
}

const Trajectory & RecordReplayPlanner::from_record(const State & current_state)
{
  // Find out where on the recorded buffer we should start replaying
  auto minimum_idx = get_closest_state(current_state);

  // Determine how long the published trajectory will be
  auto & trajectory = m_trajectory;
  const auto record_length = get_record_length();
  const auto publication_length =
    std::min(record_length - minimum_idx, trajectory.points.max_size());

  // Assemble the trajectory as desired
  trajectory.points.resize(publication_length);
  trajectory.header = current_state.header;
  const auto t0 = time_utils::from_message(m_record_buffer[minimum_idx].header.stamp);
  for (std::size_t i = {}; i < publication_length; ++i) {
    // Make the time spacing of the points match the recorded timing
    trajectory.points[i] = m_record_buffer[minimum_idx + i].state;
    trajectory.points[i].time_from_start = time_utils::to_message(
      time_utils::from_message(m_record_buffer[minimum_idx + i].header.stamp) - t0);
  }

  // Create a function to obtain a polytope from our current state
  auto collision = false;
  for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
    // Obtain a bounding box for that step along the trajectory. (TODO(s.me) these boxes
    // could already be computed on recording and then cached so they don't have to be
    // recomputed every time a trajectory is published)
    const auto boundingbox = compute_boundingbox_from_trajectorypoint(trajectory.points[i],
        m_vehicle_param);

    // Check for collisions with all perceived obstacles
    for (const auto & obstaclebox : m_latest_bounding_boxes.boxes) {
      if (intersect(boundingbox.corners.begin(), boundingbox.corners.end(),
        obstaclebox.corners.begin(), obstaclebox.corners.end()) )
      {
        // Collision detected, drop everything larger than index i-1
        collision = true;
        trajectory.points.resize(i);

        // Mark the last point along the trajectory as "stopping" by setting all rates,
        // accelerations and velocities to zero. TODO(s.me) this is by no means
        // guaranteed to be dynamically feasible. One could implement a proper velocity
        // profile here in the future.
        trajectory.points[i - 1].longitudinal_velocity_mps = 0.0;
        trajectory.points[i - 1].lateral_velocity_mps = 0.0;
        trajectory.points[i - 1].acceleration_mps2 = 0.0;
        trajectory.points[i - 1].heading_rate_rps = 0.0;
        break;  // this only breaks the inner for loop
      }
    }
    if (collision) {
      break;
    }
  }

  return trajectory;
}

void RecordReplayPlanner::writeTrajectoryBufferToFile(const std::string & record_path)
{
  if (record_path.empty()) {
    throw std::runtime_error("record_path cannot be empty");
  }

  rcl_serialized_message_t serialized_msg_;

  serialized_msg_ = rmw_get_zero_initialized_serialized_message();
  auto allocator = rcutils_get_default_allocator();
  auto initial_capacity = 0u;
  auto ret = rmw_serialized_message_init(
    &serialized_msg_,
    initial_capacity,
    &allocator);
  if (ret != RCL_RET_OK) {
    throw std::runtime_error("failed to initialize serialized message");
  }

  std::ofstream file;
  file.open(record_path, std::fstream::binary);

  if (file.is_open()) {
    for (auto msg : m_record_buffer) {
      auto state_msg = std::make_shared<State>(msg);

      auto state_ts =
        rosidl_typesupport_cpp::get_message_type_support_handle<State>();

      auto ret = rmw_serialize(state_msg.get(), state_ts, &serialized_msg_);
      if (ret != RMW_RET_OK) {
        throw std::runtime_error("failed to serialize message");
      } else {
        const char * ref(reinterpret_cast<char *>(serialized_msg_.buffer));
        file.write(ref, serialized_msg_.buffer_length);
      }
    }
  } else {
    throw std::runtime_error("failed to open file for writing");
  }
  file.close();
}

void RecordReplayPlanner::readTrajectoryBufferFromFile(const std::string & replay_path)
{
  if (replay_path.empty()) {
    throw std::runtime_error("replay_path cannot be empty");
  }

  // Init serialized message buffer
  rcl_serialized_message_t serialized_state_msg =
    rmw_get_zero_initialized_serialized_message();
  auto allocator = rcutils_get_default_allocator();
  auto initial_capacity = 0u;
  auto ret = rmw_serialized_message_init(
    &serialized_state_msg,
    initial_capacity,
    &allocator);
  if (ret != RCL_RET_OK) {
    throw std::runtime_error("failed to initialize serialized message");
  }

  // Clear current trajectory deque
  clear_record();

  // Open file
  std::ifstream file;
  file.open(replay_path, std::fstream::binary);

  // Should be enough - long frame names might need a larger buffer size
  auto upper_bound_buffer_length = 200;
  auto current_read_buffer_length = upper_bound_buffer_length;
  char serialized_msg_buffer[200];

  rmw_serialized_message_t serialized_message_struct;
  serialized_message_struct.buffer =
    reinterpret_cast<unsigned char *>(serialized_msg_buffer);
  serialized_message_struct.buffer_length = upper_bound_buffer_length;
  serialized_message_struct.buffer_capacity = upper_bound_buffer_length;
  serialized_message_struct.allocator = rcutils_get_default_allocator();

  if (file.is_open()) {
    // Save end of file ptr
    file.seekg(0, std::ios::end);
    auto file_end = file.tellg();
    file.seekg(0, std::ios::beg);
    // Read from file
    while (file.read(serialized_msg_buffer, current_read_buffer_length)) {
      // Deserialize data
      auto state_msg = std::make_shared<State>();
      auto state_ts =
        rosidl_typesupport_cpp::get_message_type_support_handle<State>();
      ret = rmw_deserialize(&serialized_message_struct, state_ts,
          state_msg.get());
      if (ret != RMW_RET_OK) {
        throw std::runtime_error("failed to deserialize message");
      }

      // Fill deque buffer
      record_state(*state_msg.get());

      // Reserialize data to get its length in binary file
      ret = rmw_serialize(state_msg.get(), state_ts, &serialized_state_msg);
      if (ret != RMW_RET_OK) {
        throw std::runtime_error("failed to serialize message");
      }

      // Move filestream ptr to start of next message
      int32_t offset =
        serialized_state_msg.buffer_length - current_read_buffer_length;
      file.seekg(offset, std::ios_base::cur);

      // We have to resize the buffer to read the whole file
      auto file_remaining = file_end - file.tellg();
      if (file_remaining == 0) {
        break;
      } else if (file_remaining < current_read_buffer_length) {
        current_read_buffer_length = file_remaining;
      }
    }
  } else {
    throw std::runtime_error("failed to open file for reading");
  }
  file.close();
}

void RecordReplayPlanner::update_bounding_boxes(const BoundingBoxArray & bounding_boxes)
{
  m_latest_bounding_boxes = bounding_boxes;
}

std::size_t RecordReplayPlanner::get_number_of_bounding_boxes() const noexcept
{
  return m_latest_bounding_boxes.boxes.size();
}

}  // namespace recordreplay_planner
}  // namespace planning
}  // namespace motion
