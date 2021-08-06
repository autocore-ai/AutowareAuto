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

#ifndef STATE_ESTIMATION__KALMAN_FILTER__KALMAN_FILTER_HPP_
#define STATE_ESTIMATION__KALMAN_FILTER__KALMAN_FILTER_HPP_

#include <helper_functions/float_comparisons.hpp>
#include <motion_model/motion_model_interface.hpp>
#include <motion_model/stationary_motion_model.hpp>
#include <state_estimation/noise_model/noise_interface.hpp>
#include <state_estimation/state_estimation_interface.hpp>
#include <state_estimation/visibility_control.hpp>

#include <Eigen/LU>

#include <limits>
#include <vector>

namespace autoware
{
namespace common
{
namespace state_estimation
{
///
/// @brief      A Kalman filter implementation.
///
/// @tparam     MotionModelT  Type of the motion model.
/// @tparam     NoiseModelT   Type of the noise model.
///
template<typename MotionModelT, typename NoiseModelT>
class STATE_ESTIMATION_PUBLIC KalmanFilter
  : public StateEstimationInterface<KalmanFilter<MotionModelT, NoiseModelT>>
{
  static_assert(
    std::is_base_of<common::motion_model::MotionModelInterface<MotionModelT>, MotionModelT>::value,
    "\n\nMotion model must inherit from MotionModelInterface\n\n");
  static_assert(
    std::is_base_of<NoiseInterface<NoiseModelT>, NoiseModelT>::value,
    "\n\nNoise model must inherit from NoiseInterface\n\n");
  static_assert(
    std::is_same<typename MotionModelT::State, typename NoiseModelT::State>::value,
    "\n\nMotion model and noise model must have the same underlying state\n\n");

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using State = typename MotionModelT::State;
  using StateMatrix = typename State::Matrix;
  using MotionModel = MotionModelT;
  using NoiseModel = NoiseModelT;

  ///
  /// @brief      Constructs a new instance of a Kalman filter.
  ///
  /// @param[in]  motion_model        The motion model to be used to predict the movement.
  /// @param[in]  noise_model         The noise model that models the motion noise.
  /// @param[in]  initial_state       The initial state of the filter.
  /// @param[in]  initial_covariance  The initial state covariance.
  ///
  explicit KalmanFilter(
    MotionModelT motion_model,
    NoiseModelT noise_model,
    const State & initial_state,
    const StateMatrix & initial_covariance)
  : m_motion_model{motion_model},
    m_noise_model{noise_model},
    m_state{initial_state},
    m_covariance{initial_covariance} {}

  ///
  /// @brief      Predict next state.
  ///
  /// @param[in]  dt    Time difference to the time at which prediction is needed.
  ///
  /// @return     Predicted state.
  ///
  State crtp_predict(const std::chrono::nanoseconds & dt)
  {
    m_state = m_motion_model.predict(m_state, dt);
    const auto & motion_jacobian = m_motion_model.jacobian(m_state, dt);
    m_covariance =
      motion_jacobian * m_covariance * motion_jacobian.transpose() + m_noise_model.covariance(dt);
    return m_state;
  }

  ///
  /// @brief      Correct the predicted state given a measurement
  ///
  /// @note       It is expected that a prediction step was done right before the correction.
  ///
  /// @param[in]  measurement   Current measurement.
  ///
  /// @tparam     MeasurementT  Measurement type.
  ///
  /// @return     State corrected with the measurement.
  ///
  template<typename MeasurementT>
  State crtp_correct(const MeasurementT & measurement)
  {
    const auto expected_measurement = measurement.create_new_instance_from(m_state);
    const auto innovation = wrap_all_angles(measurement.state() - expected_measurement);
    const auto mapping_matrix = measurement.mapping_matrix_from(m_state);
    const auto innovation_covariance =
      mapping_matrix * m_covariance * mapping_matrix.transpose() + measurement.covariance();
    const auto kalman_gain =
      m_covariance * mapping_matrix.transpose() * innovation_covariance.inverse();
    m_state += kalman_gain * innovation.vector();
    m_state.wrap_all_angles();
    m_covariance = (State::Matrix::Identity() - kalman_gain * mapping_matrix) * m_covariance;
    return m_state;
  }

  ///
  /// @brief      Reset the state of the filter to a given state and covariance.
  ///
  /// @param[in]  state       The new state that overwrites one stored in the filter.
  /// @param[in]  covariance  The new covariance that overwrites one stored in the filter.
  ///
  void crtp_reset(const State & state, const StateMatrix & covariance)
  {
    m_state = state;
    m_covariance = covariance;
  }

  /// @brief      Get current state.
  auto & crtp_state() {return m_state;}
  /// @brief      Get current state.
  const auto & crtp_state() const {return m_state;}

  /// @brief      Get current covariance.
  auto & crtp_covariance() {return m_covariance;}
  /// @brief      Get current covariance.
  const auto & crtp_covariance() const {return m_covariance;}

private:
  /// Motion model used to predict the state forward.
  MotionModelT m_motion_model{};
  /// Noise model of the movement.
  NoiseModelT m_noise_model{};
  /// State of the tracked object.
  State m_state{};
  /// Covariance of the state of the tracked object.
  StateMatrix m_covariance{StateMatrix::Zero()};
};

///
/// @brief      A utility function that creates a Kalman filter.
///
/// @details    Mostly this is needed to avoid passing the template parameters explicitly and let
///             the compiler infer them from the objects passed into this function.
///
/// @param[in]  motion_model        A motion model.
/// @param[in]  noise_model         A noise model.
/// @param[in]  initial_state       The initial state
/// @param[in]  initial_covariance  The initial covariance
///
/// @tparam     MotionModelT        Type of the motion model.
/// @tparam     NoiseModelT         Type of the noise model.
///
/// @return     Returns a valid KalmanFilter instance.
///
template<typename MotionModelT, typename NoiseModelT>
auto make_kalman_filter(
  const MotionModelT & motion_model,
  const NoiseModelT & noise_model,
  const typename MotionModelT::State & initial_state,
  const typename MotionModelT::State::Matrix & initial_covariance)
{
  return KalmanFilter<MotionModelT, NoiseModelT>{
    motion_model, noise_model, initial_state, initial_covariance};
}

///
/// @brief      A utility function that creates a Kalman filter that is to be used for correction
///             only, i.e., this Kalman filter cannot predict the state forward in time.
///
/// @details    Mostly this is needed to avoid passing the template parameters explicitly and let
///             the compiler infer them from the objects passed into this function.
///
/// @param[in]  initial_state       The initial state
/// @param[in]  initial_covariance  The initial covariance
///
/// @tparam     StateT              { description }
/// @tparam     MotionModelT  Type of the motion model.
/// @tparam     NoiseModelT   Type of the noise model.
///
/// @return     Returns a valid KalmanFilter instance.
///
template<typename StateT>
auto make_correction_only_kalman_filter(
  const StateT & initial_state,
  const typename StateT::Matrix & initial_covariance)
{
  struct DummyNoise : public NoiseInterface<DummyNoise>
  {
    using State = StateT;
    typename State::Matrix crtp_covariance(const std::chrono::nanoseconds &) const
    {
      throw std::runtime_error(
              "Trying to use a correction-only Kalman filter to predict the state.");
    }
  };

  using MotionModel = common::motion_model::StationaryMotionModel<StateT>;

  return make_kalman_filter(
    MotionModel{}, DummyNoise{}, initial_state, initial_covariance);
}

///
/// @brief      A utility function that creates a Kalman filter from a vector of variances.
///
///             Mostly this is needed to avoid passing the template parameters explicitly and let
///             the compiler infer them from the objects passed into this function.
///
/// @param[in]  motion_model       A motion model.
/// @param[in]  noise_model        A noise model.
/// @param[in]  initial_state      Initial state.
/// @param[in]  initial_variances  Initial variances as a vector.
///
/// @tparam     MotionModelT       Type of the motion model.
/// @tparam     NoiseModelT        Type of the noise model.
///
/// @return     Returns a valid KalmanFilter instance.
///
template<typename MotionModelT, typename NoiseModelT>
auto make_kalman_filter(
  const MotionModelT & motion_model,
  const NoiseModelT & noise_model,
  const typename MotionModelT::State & initial_state,
  const std::vector<typename MotionModelT::State::Scalar> & initial_variances)
{
  using State = typename MotionModelT::State;
  if (initial_variances.size() != static_cast<std::size_t>(State::size())) {
    std::runtime_error(
      "Cannot create Kalman filter - dimensions mismatch. Provided " +
      std::to_string(initial_variances.size()) + " variances, but " +
      std::to_string(State::size()) + " required.");
  }
  typename State::Vector variances{State::Vector::Zero()};
  // A small enough epsilon to compare a floating point variance with zero.
  const auto epsilon = 5.0F * std::numeric_limits<common::types::float32_t>::epsilon();
  for (std::uint32_t i = 0; i < initial_variances.size(); ++i) {
    if (common::helper_functions::comparisons::abs_lte(initial_variances[i], 0.0F, epsilon)) {
      throw std::domain_error("Variances must be positive");
    }
    variances[static_cast<std::int32_t>(i)] = initial_variances[i] * initial_variances[i];
  }
  return KalmanFilter<MotionModelT, NoiseModelT>{
    motion_model, noise_model, initial_state, variances.asDiagonal()};
}

}  // namespace state_estimation
}  // namespace common
}  // namespace autoware

#endif  // STATE_ESTIMATION__KALMAN_FILTER__KALMAN_FILTER_HPP_
