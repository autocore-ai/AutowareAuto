// Copyright 2020 Apex.AI, Inc.
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

/// \copyright Copyright 2020 Apex.AI, Inc.
/// All rights reserved.

#ifndef STATE_ESTIMATION_NODE__MEASUREMENT_TIME_KEEPER_HPP_
#define STATE_ESTIMATION_NODE__MEASUREMENT_TIME_KEEPER_HPP_

#include <common/types.hpp>
#include <state_estimation_node/measurement.hpp>

#include <algorithm>
#include <chrono>
#include <utility>

namespace autoware
{
namespace prediction
{

///
/// @brief      This is a time storage class that keeps the time with respect to the clock that
///             timestamped the last measurement.
///
///             All the local times are with respect to the clock that timestamped the last
///             measurement, the global time is reported by the global system clock of the system
///             that received the message.
///
class MeasurementBasedTimeKeeper
{
public:
  ///
  /// @brief      Allow empty initialization.
  ///
  MeasurementBasedTimeKeeper() = default;

  ///
  /// @brief      Constructs a new instance from the time the event has occured (measurement was
  ///             received) and its timestamp.
  ///
  /// @param[in]  time_of_event  The time of event occurance (measurement received).
  /// @param[in]  timestamp      The timestamp of the message.
  ///
  MeasurementBasedTimeKeeper(
    const GlobalTime & time_of_event,
    const MeasurementBasedTime & timestamp)
  : m_time_last_measurement_received{time_of_event}, m_last_measurement_timestamp{timestamp}
  {
    m_last_temporal_update = convert_to_measurement_time(time_of_event);
  }

  ///
  /// @brief      Determines if initialized with proper timestamps.
  ///
  /// @return     True if initialized, False otherwise.
  ///
  inline common::types::bool8_t is_initialized() const noexcept
  {
    return m_time_last_measurement_received.is_valid() && m_last_temporal_update.is_valid();
  }

  ///
  /// @brief      Update when a new measurement is received. This changes the base of calculation of
  ///             the local time.
  ///
  /// @param[in]  time_of_event  The time of event occurance (time of message received).
  /// @param[in]  measurement    The measurement
  ///
  /// @tparam     MeasurementT   Type of measurement.
  ///
  template<typename MeasurementT>
  inline void update_with_measurement(
    const GlobalTime & time_of_event,
    const MeasurementT & measurement)
  {
    m_time_last_measurement_received = time_of_event;
    m_last_measurement_timestamp = measurement.get_acquisition_time();
  }

  ///
  /// @brief      Get the time since the last temporal update in measurement clock frame.
  ///
  /// @param[in]  time_point  The global time of the update reported by the clock.
  ///
  /// @return     Returns the duration that passed since the last temporal update.
  ///
  inline MeasurementBasedTime::duration time_since_last_temporal_update(
    const GlobalTime & time_point) const noexcept
  {
    return convert_to_measurement_time(time_point) - m_last_temporal_update;
  }
  ///
  /// @brief      Get the time since the last temporal update.
  ///
  /// @param[in]  time_point  The time of the update reported in the measurement clock frame.
  ///
  /// @return     Returns the duration that passed since the last temporal update.
  ///
  inline MeasurementBasedTime::duration time_since_last_temporal_update(
    const MeasurementBasedTime & time_point) const noexcept
  {
    return time_point - m_last_temporal_update;
  }

  ///
  /// @brief      Increment the time of last temporal update by a given amount.
  ///
  /// @param[in]  time_since_last_update  The time passed since last update.
  ///
  void increment_last_temporal_update_time(
    const MeasurementBasedTime::duration & time_since_last_update) noexcept
  {
    m_last_temporal_update += time_since_last_update;
  }

  ///
  /// @brief      Get the latest stored timestamp.
  ///
  /// @return     The latest time with respect to the clock that timestamped the measurement.
  ///
  MeasurementBasedTime latest_timestamp() const noexcept
  {
    return std::max(m_last_measurement_timestamp, m_last_temporal_update);
  }

private:
  ///
  /// @brief      Convert global time point into one relative to the clock that timestamped the last
  ///             measurement.
  ///
  /// @param[in]  global_time  The global time point
  ///
  /// @return     The time point with respect to the clock that timestamped the last measurement.
  ///
  inline MeasurementBasedTime convert_to_measurement_time(
    const GlobalTime & global_time) const noexcept
  {
    return m_last_measurement_timestamp + (global_time - m_time_last_measurement_received);
  }

  /// Time at which the last message was received as reported by some global clock.
  GlobalTime m_time_last_measurement_received{};
  /// The timestamp of the last received message.
  MeasurementBasedTime m_last_measurement_timestamp{};
  /// The time of the last temporal update with respect to the clock that timestamped the last
  /// measurement.
  MeasurementBasedTime m_last_temporal_update{};
};


}  // namespace prediction
}  // namespace autoware

#endif  // STATE_ESTIMATION_NODE__MEASUREMENT_TIME_KEEPER_HPP_
