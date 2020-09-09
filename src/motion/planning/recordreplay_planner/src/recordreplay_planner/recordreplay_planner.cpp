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
#include <time_utils/time_utils.hpp>
#include <motion_common/motion_common.hpp>
#include <common/types.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <map>

#include "recordreplay_planner/vehicle_bounding_box.hpp"

using autoware::common::types::bool8_t;
using autoware::common::types::char8_t;
using autoware::common::types::uchar8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using Csv = std::vector<std::vector<std::string>>;
using Association = std::map<std::string /* label */, int32_t /* row */>;
namespace
{
std::vector<std::string> split(const std::string & input, char delimiter)
{
  std::istringstream stream(input);
  std::string field;
  std::vector<std::string> result;
  while (std::getline(stream, field, delimiter)) {
    result.push_back(field);
  }
  return result;
}

void deleteHeadSpace(std::string & string)
{
  while (string.find_first_of(' ') == 0) {
    string.erase(string.begin());
    if (string.empty()) {
      break;
    }
  }
}

void deleteUnit(std::string & string)
{
  size_t start_pos, end_pos;
  start_pos = string.find_first_of('[');
  end_pos = string.find_last_of(']');
  if (start_pos != std::string::npos && end_pos != std::string::npos && start_pos < end_pos) {
    string.erase(start_pos, (end_pos + 1) - start_pos);
  }
}

bool loadData(
  const std::string & file_name, Association & label_row_association_map,
  Csv & file_data)
{
  file_data.clear();
  label_row_association_map.clear();
  std::ifstream ifs(file_name);

  // open file
  if (!ifs) {
    std::cerr << "Could not load " << file_name << std::endl;
    return false;
  }

  // create label-row association map
  std::string line;
  if (std::getline(ifs, line)) {
    std::vector<std::string> str_vec = split(line, ',');
    for (size_t i = 0; i < str_vec.size(); ++i) {
      deleteUnit(str_vec.at(i));
      deleteHeadSpace(str_vec.at(i));
      label_row_association_map[str_vec.at(i)] = static_cast<int>(i);
    }
  } else {
    std::cerr << "cannot create association map" << std::endl;
    return false;
  }

  // create file data
  while (std::getline(ifs, line)) {
    std::vector<std::string> str_vec = split(line, ',');
    file_data.push_back(str_vec);
  }

  return true;
}
}  // namespace
namespace motion
{
namespace planning
{
namespace recordreplay_planner
{
using geometry_msgs::msg::Point32;
using motion::motion_common::to_angle;
using autoware::common::geometry::intersect;
using autoware::common::geometry::dot_2d;

RecordReplayPlanner::RecordReplayPlanner(const VehicleConfig & vehicle_param)
: m_vehicle_param(vehicle_param)
{
}

// These may do more in the future
bool8_t RecordReplayPlanner::is_recording() const noexcept
{
  return m_recordreplaystate == RecordReplayState::RECORDING;
}

bool8_t RecordReplayPlanner::is_replaying() const noexcept
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
  m_cache_traj_bbox_arr.boxes.clear();
}

std::size_t RecordReplayPlanner::get_record_length() const noexcept
{
  return m_record_buffer.size();
}


void RecordReplayPlanner::set_heading_weight(float64_t heading_weight)
{
  if (heading_weight < 0.0) {
    throw std::domain_error{"Negative weights do not make sense"};
  }
  m_heading_weight = heading_weight;
}

float64_t RecordReplayPlanner::get_heading_weight()
{
  return m_heading_weight;
}

void RecordReplayPlanner::set_min_record_distance(float64_t min_record_distance)
{
  if (min_record_distance < 0.0) {
    throw std::domain_error{"Negative minumum distance do not make sense"};
  }
  m_min_record_distance = min_record_distance;
}

float64_t RecordReplayPlanner::get_min_record_distance() const
{
  return m_min_record_distance;
}


void RecordReplayPlanner::record_state(const State & state_to_record)
{
  m_cache_traj_bbox_arr.boxes.clear();

  if (m_record_buffer.empty()) {
    m_record_buffer.push_back(state_to_record);
    return;
  }

  auto previous_state = m_record_buffer.back();
  auto distance_sq = (state_to_record.state.x - previous_state.state.x) *
    (state_to_record.state.x - previous_state.state.x) +
    (state_to_record.state.y - previous_state.state.y) *
    (state_to_record.state.y - previous_state.state.y);

  if (static_cast<float64_t>(distance_sq) >= (m_min_record_distance * m_min_record_distance) ) {
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
             static_cast<float32_t>(m_heading_weight) * std::abs(to_angle(s1.heading - s2.heading));
    };
  const auto comparison_function =
    [&distance_from_current_state](State & one, State & two)
    {return distance_from_current_state(one) < distance_from_current_state(two);};


  const auto minimum_index_iterator =
    std::min_element(std::begin(m_record_buffer), std::end(m_record_buffer),
      comparison_function);
  auto minimum_idx = std::distance(std::begin(m_record_buffer), minimum_index_iterator);

  return static_cast<std::size_t>(minimum_idx);
}

const BoundingBoxArray & RecordReplayPlanner::get_traj_boxes()
{
  m_current_traj_bboxes.boxes.resize((m_traj_end_idx - m_traj_start_idx));
  for (std::size_t i = {}; i < (m_traj_end_idx - m_traj_start_idx); ++i) {
    m_current_traj_bboxes.boxes[i] = m_cache_traj_bbox_arr.boxes[i];
    // workaround to color Green
    m_current_traj_bboxes.boxes[i].vehicle_label = BoundingBox::MOTORCYCLE;
  }
  return m_current_traj_bboxes;
}
const BoundingBoxArray & RecordReplayPlanner::get_collision_boxes()
{
  if (!m_latest_collison_boxes.boxes.empty()) {
    // workaround to color Orange
    m_latest_collison_boxes.boxes[0].vehicle_label = BoundingBox::CYCLIST;
  }
  return m_latest_collison_boxes;
}

const Trajectory & RecordReplayPlanner::from_record(const State & current_state)
{
  // Find out where on the recorded buffer we should start replaying
  m_traj_start_idx = get_closest_state(current_state);

  // Determine how long the published trajectory will be
  auto & trajectory = m_trajectory;
  const auto record_length = get_record_length();
  m_traj_end_idx =
    std::min({record_length - m_traj_start_idx, trajectory.points.max_size(),
        m_cache_traj_bbox_arr.boxes.max_size()}) + m_traj_start_idx;

  // Build bounding box cache
  if (m_cache_traj_bbox_arr.boxes.empty() ||
    m_cache_traj_bbox_arr.boxes.size() != get_record_length())
  {
    m_cache_traj_bbox_arr.boxes.clear();
    m_cache_traj_bbox_arr.header = current_state.header;
    for (std::size_t i = m_traj_start_idx; i < m_traj_end_idx; ++i) {
      const auto boundingbox = compute_boundingbox_from_trajectorypoint(
        m_record_buffer[i].state, m_vehicle_param);
      m_cache_traj_bbox_arr.boxes.push_back(boundingbox);
    }
  }

  // Reset and setup debug msg
  m_latest_collison_boxes.boxes.clear();
  m_latest_collison_boxes.header = m_latest_bounding_boxes.header;
  m_current_traj_bboxes.boxes.clear();
  m_current_traj_bboxes.header = current_state.header;


  // Collision detection
  for (std::size_t i = m_traj_start_idx; i < m_traj_end_idx; ++i) {
    const size_t i_bbox = i - m_traj_start_idx;
    const auto & boundingbox = m_cache_traj_bbox_arr.boxes[i_bbox];

    // Check for collisions with all perceived obstacles
    for (const auto & obstaclebox : m_latest_bounding_boxes.boxes) {
      if (intersect(boundingbox.corners.begin(), boundingbox.corners.end(),
        obstaclebox.corners.begin(), obstaclebox.corners.end()) )
      {
        // Collision detected, set end index (non-inclusive)
        m_traj_end_idx = i;  // This also ends the outer loop

        // Visual Debug msg
        auto collison_box = obstaclebox;
        m_latest_collison_boxes.boxes.push_back(collison_box);

        break;
      }
    }
  }

  // Assemble the trajectory as desired
  trajectory.header = current_state.header;
  const auto publication_len = m_traj_end_idx - m_traj_start_idx;
  trajectory.points.resize(publication_len);

  const auto t0 = time_utils::from_message(m_record_buffer[m_traj_start_idx].header.stamp);
  for (std::size_t i = {}; i < publication_len; ++i) {
    // Make the time spacing of the points match the recorded timing
    trajectory.points[i] = m_record_buffer[m_traj_start_idx + i].state;
    trajectory.points[i].time_from_start = time_utils::to_message(
      time_utils::from_message(m_record_buffer[m_traj_start_idx + i].header.stamp) - t0);
  }

  // Adjust time stamp from velocity
  float32_t t = 0.0;
  for (std::size_t i = 1; i < publication_len; ++i) {
    auto & p0 = trajectory.points[i - 1];
    auto & p1 = trajectory.points[i];
    auto dx = p1.x - p0.x;
    auto dy = p1.y - p0.y;
    auto v = 0.5f * (p0.longitudinal_velocity_mps + p1.longitudinal_velocity_mps);
    t += std::sqrt(dx * dx + dy * dy) / std::max(std::fabs(v), 1.0e-5f);
    float32_t t_s = 0;
    float32_t t_ns = std::modf(t, &t_s) * 1.0e9f;
    trajectory.points[i].time_from_start.sec = static_cast<int32_t>(t_s);
    trajectory.points[i].time_from_start.nanosec = static_cast<uint32_t>(t_ns);
  }

  // Mark the last point along the trajectory as "stopping" by setting all rates,
  // accelerations and velocities to zero. TODO(s.me) this is by no means
  // guaranteed to be dynamically feasible. One could implement a proper velocity
  // profile here in the future.
  if (m_traj_end_idx > m_traj_start_idx) {
    const auto traj_last_idx = m_traj_end_idx - 1U;
    trajectory.points[traj_last_idx].longitudinal_velocity_mps = 0.0;
    trajectory.points[traj_last_idx].lateral_velocity_mps = 0.0;
    trajectory.points[traj_last_idx].acceleration_mps2 = 0.0;
    trajectory.points[traj_last_idx].heading_rate_rps = 0.0;
  }

  return trajectory;
}

void RecordReplayPlanner::writeTrajectoryBufferToFile(const std::string & record_path)
{
  if (record_path.empty()) {
    throw std::runtime_error("record_path cannot be empty");
  }

  std::ofstream ofs;
  ofs.open(record_path, std::ios::trunc);
  if (!ofs.is_open()) {
    throw std::runtime_error("Could not open file.");
  }
  ofs << "t_sec, t_nanosec, x, y, heading_real, heading_imag, longitudinal_velocity_mps, " <<
    "lateral_velocity_mps, acceleration_mps2, heading_rate_rps, front_wheel_angle_rad, " <<
    "rear_wheel_angle_rad" << std::endl;

  for (const auto & trajectory_point : m_record_buffer) {
    const auto & s = trajectory_point.state;
    const auto & t = s.time_from_start;
    ofs << t.sec << ", " << t.nanosec << ", " << s.x << ", " << s.y << ", " << s.heading.real <<
      ", " << s.heading.imag << ", " << s.longitudinal_velocity_mps << ", " <<
      s.lateral_velocity_mps << ", " << s.acceleration_mps2 << ", " << s.heading_rate_rps <<
      ", " << s.front_wheel_angle_rad << ", " << s.rear_wheel_angle_rad << std::endl;
  }
  ofs.close();
}

void RecordReplayPlanner::readTrajectoryBufferFromFile(const std::string & replay_path)
{
  if (replay_path.empty()) {
    throw std::runtime_error("replay_path cannot be empty");
  }

  // Clear current trajectory deque
  clear_record();

  Csv file_data;
  Association map;  // row labeled Association map
  if (!loadData(replay_path, map, file_data)) {
    std::cerr << "failed to open file : " << replay_path << std::endl;
    return;
  }

  for (size_t i = 0; i < file_data.size(); ++i) {
    State s;
    for (size_t j = 0; j < file_data.at(i).size(); ++j) {
      const int _j = static_cast<int>(j);
      if (map.at("t_sec") == _j) {
        s.state.time_from_start.sec = std::stoi(file_data.at(i).at(j));
      } else if (map.at("t_nanosec") == _j) {
        s.state.time_from_start.nanosec = static_cast<uint32_t>(std::stoi(file_data.at(i).at(j)));
      } else if (map.at("x") == _j) {
        s.state.x = std::stof(file_data.at(i).at(j));
      } else if (map.at("y") == _j) {
        s.state.y = std::stof(file_data.at(i).at(j));
      } else if (map.at("heading_real") == _j) {
        s.state.heading.real = std::stof(file_data.at(i).at(j));
      } else if (map.at("heading_imag") == _j) {
        s.state.heading.imag = std::stof(file_data.at(i).at(j));
      } else if (map.at("longitudinal_velocity_mps") == _j) {
        s.state.longitudinal_velocity_mps = std::stof(file_data.at(i).at(j));
      } else if (map.at("lateral_velocity_mps") == _j) {
        s.state.lateral_velocity_mps = std::stof(file_data.at(i).at(j));
      } else if (map.at("acceleration_mps2") == _j) {
        s.state.acceleration_mps2 = std::stof(file_data.at(i).at(j));
      } else if (map.at("heading_rate_rps") == _j) {
        s.state.heading_rate_rps = std::stof(file_data.at(i).at(j));
      } else if (map.at("front_wheel_angle_rad") == _j) {
        s.state.front_wheel_angle_rad = std::stof(file_data.at(i).at(j));
      } else if (map.at("rear_wheel_angle_rad") == _j) {
        s.state.rear_wheel_angle_rad = std::stof(file_data.at(i).at(j));
      }
    }
    record_state(s);
  }
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
