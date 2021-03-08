// Copyright 2021 Apex.AI, Inc.
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

/// \copyright Copyright 2021 Apex.AI, Inc.
/// All rights reserved.

#ifndef STATE_ESTIMATION_NODES__STEADY_TIME_GRID_HPP_
#define STATE_ESTIMATION_NODES__STEADY_TIME_GRID_HPP_

#include <state_estimation_nodes/measurement.hpp>

#include <algorithm>
#include <chrono>
#include <utility>

namespace autoware
{
namespace prediction
{

///
/// @brief      This is a utility class that allows querying timestamps on a 1D grid.
///
///             It is initialized with a start timestamp and an interval between consecutive
///             timestamps and allows to query the next timestamp on the grid for any given query
///             timestamp.
///
class SteadyTimeGrid
{
public:
  ///
  /// @brief      Allow empty initialization.
  ///
  SteadyTimeGrid() = default;

  ///
  /// @brief      Constructs a new instance from the time the event has occurred (measurement was
  ///             received) and its timestamp.
  ///
  /// @param[in]  measurement_timestamp        The timestamp of the message.
  /// @param[in]  interval_between_timestamps  Time interval between consecutive timestamps.
  ///
  SteadyTimeGrid(
    const std::chrono::system_clock::time_point & measurement_timestamp,
    const std::chrono::system_clock::duration & interval_between_timestamps)
  : m_first_measurement_timestamp{measurement_timestamp},
    m_interval_between_timestamps{interval_between_timestamps} {}

  ///
  /// @brief      Get the next timestamp on the grid given a query one.
  ///
  /// @param[in]  current_timestamp  The query timestamp.
  ///
  /// @return     The next timestamp on the 1D grid.
  ///
  inline std::chrono::system_clock::time_point get_next_timestamp_after(
    const std::chrono::system_clock::time_point & current_timestamp) const noexcept
  {
    const auto prev_time_index =
      (current_timestamp - m_first_measurement_timestamp) / m_interval_between_timestamps;
    return m_first_measurement_timestamp + (prev_time_index + 1L) * m_interval_between_timestamps;
  }

  ///
  /// @brief      Determine if initialized with a non-zero timestamp.
  ///
  /// @return     True if initialized, False otherwise.
  ///
  inline bool is_initialized() const noexcept
  {
    return m_first_measurement_timestamp > std::chrono::system_clock::time_point{};
  }

private:
  /// The timestamp of the received message.
  std::chrono::system_clock::time_point m_first_measurement_timestamp{};
  /// Interval between predictions.
  std::chrono::system_clock::duration m_interval_between_timestamps{};
};


}  // namespace prediction
}  // namespace autoware

#endif  // STATE_ESTIMATION_NODES__STEADY_TIME_GRID_HPP_
