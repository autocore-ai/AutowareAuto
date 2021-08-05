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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

/// \copyright Copyright 2021 Apex.AI, Inc.
/// All rights reserved.

#ifndef STATE_ESTIMATION_NODES__KALMAN_FILTER_WRAPPER_HPP_
#define STATE_ESTIMATION_NODES__KALMAN_FILTER_WRAPPER_HPP_

#include <common/types.hpp>
#include <measurement_conversion/measurement_typedefs.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <state_estimation/kalman_filter/kalman_filter.hpp>
#include <state_estimation_nodes/filter_typedefs.hpp>
#include <state_estimation_nodes/history.hpp>
#include <state_estimation_nodes/steady_time_grid.hpp>
#include <state_estimation_nodes/visibility_control.hpp>
#include <state_vector/common_states.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace autoware
{
namespace common
{
namespace state_estimation
{

///
/// @brief      This class provides a high level interface to the Kalman Filter allowing to predict
///             the state of the filter with time and observe it by receiving ROS messages.
///
/// @tparam     FilterT           Type of filter used internally.
///
template<typename FilterT>
class STATE_ESTIMATION_NODES_PUBLIC KalmanFilterWrapper
{
  using HistoryT = History<
    FilterT,
    PredictionEvent,
    ResetEvent<FilterT>,
    PoseMeasurementXYZ32,
    PoseMeasurementXYZRPY32>;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using State = typename FilterT::State;

  ///
  /// @brief      Create an EKF wrapper.
  ///
  /// @param[in]  motion_model              The motion model that is to be used.
  /// @param[in]  noise_model               The noise model that is to be used.
  /// @param[in]  initial_state_covariance  The initial covariances for the state. This is usually a
  ///                                       diagonal matrix with sigmas squared for each state
  ///                                       dimension on the diagonal.
  /// @param[in]  expected_dt               Expected time difference between updates of the filter.
  /// @param[in]  frame_id                  The frame id in which tracking takes place.
  /// @param[in]  history_duration          Length of the history of events.
  /// @param[in]  mahalanobis_threshold     The threshold on the Mahalanobis distance for outlier
  ///                                       rejection.
  ///
  KalmanFilterWrapper(
    const typename FilterT::MotionModel motion_model,
    const typename FilterT::NoiseModel noise_model,
    const typename FilterT::State::Matrix initial_state_covariance,
    const std::chrono::nanoseconds & expected_dt,
    const std::string & frame_id,
    const std::chrono::nanoseconds & history_duration = std::chrono::milliseconds{5000},
    common::types::float32_t mahalanobis_threshold =
    std::numeric_limits<common::types::float32_t>::max())
  : m_initial_covariance{initial_state_covariance},
    m_frame_id{frame_id},
    m_mahalanobis_threshold{mahalanobis_threshold},
    m_expected_prediction_period{expected_dt},
    m_filter{
      motion_model,
      noise_model,
      State{},
      initial_state_covariance,
    },
    m_history{
      m_filter,
      static_cast<std::size_t>(history_duration / expected_dt),
      m_mahalanobis_threshold} {}

  ///
  /// Reset the filter state using the default covariance and state derived from the measurement.
  ///
  /// @param[in]  measurement   The measurement from which we initialize the state.
  ///
  /// @tparam     MeasurementT  Type of measurement.
  ///
  template<typename MeasurementT>
  inline void add_reset_event_to_history(const MeasurementT & measurement)
  {
    add_reset_event_to_history(
      measurement.measurement.map_into(State{}),
      m_initial_covariance,
      measurement.timestamp);
  }

  ///
  /// Reset the filter state. This must be called at least once to start / tracking.
  ///
  /// @param[in]  state               The full state to set the system to.
  /// @param[in]  initial_covariance  The initial covariance.
  /// @param[in]  event_timestamp     The event timestamp. Ideally this should be in the same clock
  ///                                 as the one that timestamps the messages.
  ///
  inline void add_reset_event_to_history(
    const State & state,
    const typename State::Matrix & initial_covariance,
    const std::chrono::system_clock::time_point & event_timestamp)
  {
    m_history.emplace_event(event_timestamp, ResetEvent<FilterT>{state, initial_covariance});
    m_time_grid = SteadyTimeGrid{event_timestamp, m_expected_prediction_period};
  }

  ///
  /// Predict state of filter at the next timestep defined by the period of this node.
  ///
  /// @return     true if the update was successful and false otherwise. In case false is returned,
  ///             this update had no effect on the state of the filter.
  ///
  inline common::types::bool8_t add_next_temporal_update_to_history()
  {
    if (!is_initialized()) {return false;}
    const auto next_prediction_timestamp =
      m_time_grid.get_next_timestamp_after(m_history.get_last_timestamp());
    m_history.emplace_event(next_prediction_timestamp, PredictionEvent{});
    return true;
  }

  ///
  /// Update the filter state with a measurement.
  ///
  /// @param[in]  measurement            The measurement. It is expected to be a concrete
  ///                                    instantiation of the Measurement class.
  ///
  /// @tparam     MeasurementT           Measurement type that is a concrete template specialization
  ///                                    of the Measurement class.
  ///
  /// @return     true if the observation was successful, false otherwise. In case of an
  ///             unsuccessful update, the state of the underlying filter has not been changed.
  ///
  template<typename MeasurementT>
  common::types::bool8_t add_observation_to_history(const MeasurementT & measurement)
  {
    if (!is_initialized()) {return false;}
    m_history.emplace_event(measurement.timestamp, measurement.measurement);
    return true;
  }

  /// Check if the filter is is_initialized with a state.
  inline common::types::bool8_t is_initialized() const noexcept
  {
    return (!m_history.empty()) && m_time_grid.is_initialized();
  }

  /// Get the current state of the system as an odometry message.
  nav_msgs::msg::Odometry get_state() const;

private:
  /// Initial covariance of the filter.
  typename State::Matrix m_initial_covariance{};
  /// Time represented in a frame based on the last measurement timestamp.
  SteadyTimeGrid m_time_grid{};
  /// Frame in which the estimation happens, e.g. "odom".
  std::string m_frame_id{};
  /// The threshold on the Mahalanobis distance used to reject outliers.
  common::types::float32_t m_mahalanobis_threshold{};
  /// What duration passes between prediction events.
  std::chrono::nanoseconds m_expected_prediction_period{};
  /// Wrapper owns the filter implementation.
  FilterT m_filter{};
  /// History of all events is stored here.
  HistoryT m_history{};
};

using ConstantAccelerationFilterWrapperXY =
  KalmanFilterWrapper<ConstAccelerationKalmanFilterXY>;

using ConstantAccelerationFilterWrapperXYZRPY =
  KalmanFilterWrapper<ConstAccelerationKalmanFilterXYZRPY>;

}  // namespace state_estimation
}  // namespace common
}  // namespace autoware

#endif  // STATE_ESTIMATION_NODES__KALMAN_FILTER_WRAPPER_HPP_
