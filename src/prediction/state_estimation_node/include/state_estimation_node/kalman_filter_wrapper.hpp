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

#ifndef STATE_ESTIMATION_NODE__KALMAN_FILTER_WRAPPER_HPP_
#define STATE_ESTIMATION_NODE__KALMAN_FILTER_WRAPPER_HPP_

#include <common/types.hpp>
#include <kalman_filter/esrcf.hpp>
#include <motion_model/constant_acceleration.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <state_estimation_node/measurement.hpp>
#include <state_estimation_node/measurement_time_keeper.hpp>
#include <state_estimation_node/visibility_control.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <limits>
#include <cstdint>
#include <memory>
#include <string>
#include <chrono>

namespace autoware
{
namespace prediction
{

///
/// @brief      This class provides a high level interface to the Kalman Filter allowing to predict
///             the state of the filter with time and observe it by receiving ROS messages.
///
/// @tparam     MotionModelT  An underlying motion model.
/// @tparam     kNumOfStates  Number of states of the system.
///
template<typename MotionModelT, std::int32_t kNumOfStates, std::int32_t kProcessNoiseDim>
class STATE_ESTIMATION_NODE_PUBLIC KalmanFilterWrapper
{
  using FilterT = prediction::kalman_filter::Esrcf<kNumOfStates, kProcessNoiseDim>;

  template<std::int32_t kRows, std::int32_t kCols>
  using RectangularMatrixT = Eigen::Matrix<common::types::float32_t, kRows, kCols>;

  template<std::int32_t kNum>
  using SquareMatrixT = Eigen::Matrix<common::types::float32_t, kNum, kNum>;

  template<std::int32_t kLength>
  using VectorT = Eigen::Matrix<common::types::float32_t, kLength, 1>;

public:
  ///
  /// @brief      Create an EKF wrapper.
  ///
  /// @param[in]  initial_covariance_factor  The initial covariances for the state. This is usually
  ///                                        a diagonal matrix with sigmas for each state dimention
  ///                                        on the diagonal.
  /// @param[in]  process_noise              A thin matrix that has as many rows as there are states
  ///                                        and as many rows as the dimentionality of our space,
  ///                                        e.g. for state of position, velocity, and acceleration
  ///                                        in 2D this will be 6x2; for state of position and
  ///                                        velocity in 1D, it will be 2x1.
  /// @param[in]  expected_dt                Expected time difference between updates of the filter.
  /// @param[in]  frame_id                   The frame id in which tracking takes place.
  /// @param[in]  mahalanobis_threshold      The threshold on the Mahalanobis distance for ourlier
  ///                                        rejection.
  /// @param[in]  motion_model               The motion model that is to be used. Mostly present
  ///                                        here to avoid passign the type explicitly.
  ///
  KalmanFilterWrapper(
    const SquareMatrixT<kNumOfStates> & initial_covariance_factor,
    const RectangularMatrixT<kNumOfStates, kProcessNoiseDim> & process_noise,
    const std::chrono::nanoseconds & expected_dt,
    const std::string & frame_id,
    common::types::float32_t mahalanobis_threshold =
    std::numeric_limits<common::types::float32_t>::max(),
    const MotionModelT & motion_model = MotionModelT{})
  : m_motion_model{motion_model},
    m_initial_covariance_factor{initial_covariance_factor},
    m_frame_id{frame_id},
    m_mahalanobis_threshold{mahalanobis_threshold}
  {
    static_assert(
      motion_model.get_num_states() == kNumOfStates,
      "Wrong number of states in the motion model.");

    SquareMatrixT<kNumOfStates> F;
    m_motion_model.compute_jacobian(F, expected_dt);
    m_GQ_left_factor = F * process_noise;
    m_ekf = std::make_unique<FilterT>(m_motion_model, m_GQ_left_factor);
  }

  ///
  /// Reset the filter state using the default covariance and state derived from the measurement.
  ///
  /// @param[in]  measurement              The measurement form which we intitialize the state.
  /// @param[in]  time_of_event_occurance  The time of event occurance from some global clock.
  ///
  /// @tparam     MeasurementT             Type of measurement.
  ///
  template<typename MeasurementT>
  void reset(const MeasurementT & measurement, const GlobalTime & time_of_event_occurance);

  ///
  /// Reset the filter state. This must be called at least once to start / tracking.
  ///
  /// @param[in]  state                    The full state to set the system to.
  /// @param[in]  initial_covariance_chol  The initial covariance cholesky factor. For a diagonal
  ///                                      matrix with squared variances on the diagonal:
  ///                                      diag([s^2]), its cholesky factor is a diagonal matrix
  ///                                      with variances on the diagonal: diag([s]).
  /// @param[in]  event_timestamp          The event timestamp. Ideally this should be in the same
  ///                                      clock as the one that timestamps the messages.
  /// @param[in]  time_of_event_occurance  The time of event occurance from some global clock.
  ///
  void reset(
    const VectorT<kNumOfStates> & state,
    const SquareMatrixT<kNumOfStates> & initial_covariance_chol,
    const MeasurementBasedTime & event_timestamp,
    const GlobalTime & time_of_event_occurance);

  ///
  /// Predict state of filter at a provided time. The update will be ignored if this time is before
  /// the last recorded update time.
  ///
  /// @param[in]  time_of_update       The time of update as reported by some global clock.
  ///
  /// @tparam     kTimeReferenceFrame  The enum value that defines the basis of our clock's time.
  ///
  /// @return     true if the update was successful and false otherwise. In case false is returned,
  ///             this update had no effect on the state of the filter.
  ///
  template<TimeReferenceFrame kTimeReferenceFrame>
  common::types::bool8_t temporal_update(
    const Time<kTimeReferenceFrame> & time_of_update);

  ///
  /// Update the filter state with a measurement.
  ///
  /// @param[in]  message_received_time  The time from some global clock when the message has been
  ///                                    received.
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
  common::types::bool8_t observation_update(
    const GlobalTime & message_received_time,
    const MeasurementT & measurement);

  /// Check if the filter is is_initialized with a state.
  common::types::bool8_t is_initialized() const noexcept
  {
    return m_ekf_initialized && m_time_keeper.is_initialized();
  }

  /// Get the current state of the system as an odometry message.
  nav_msgs::msg::Odometry get_state() const;

private:
  /// Check if the vectors pass the Mahalanobis gate.
  common::types::bool8_t passes_mahalanobis_gate(
    const VectorT<kNumOfStates> & sample,
    const VectorT<kNumOfStates> & mean,
    const SquareMatrixT<kNumOfStates> & covariance_factor) const;

  /// Shows if this filter has been initialized with a stateful measurement.
  common::types::bool8_t m_ekf_initialized{};
  /// Time represented in a frame based on the last measurement timestamp.
  MeasurementBasedTimeKeeper m_time_keeper{};
  /// We own our motion model and store it here.
  MotionModelT m_motion_model;
  /// We own the left factor of the matrix GQ = m_GQ_left_factor * m_GQ_left_factor.T
  RectangularMatrixT<kNumOfStates, kProcessNoiseDim> m_GQ_left_factor;
  /// We own the initial state covariance Cholesky factor. In the most common case, for the diagonal
  /// covariance matrix  with squared variances on the diagonal: diag([s^2]), it's Cholesky factor
  /// will be diag([s]). Otherwise this is a lower triangular Cholesky factor of the state
  /// covariance matrix.
  SquareMatrixT<kNumOfStates> m_initial_covariance_factor;
  /// The kalman filter class that takes care of the state estimation.
  std::unique_ptr<FilterT> m_ekf{};
  /// Frame in which the estimation happens, e.g. "odom".
  std::string m_frame_id{};
  /// The threshold on the Mahalanobis distance used to reject outliers.
  common::types::float32_t m_mahalanobis_threshold{};
};

using ConstantAccelerationFilter =
  KalmanFilterWrapper<motion::motion_model::ConstantAcceleration, 6, 2>;

}  // namespace prediction
}  // namespace autoware

#endif  // STATE_ESTIMATION_NODE__KALMAN_FILTER_WRAPPER_HPP_
