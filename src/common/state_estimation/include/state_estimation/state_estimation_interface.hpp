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


#ifndef STATE_ESTIMATION__STATE_ESTIMATION_INTERFACE_HPP_
#define STATE_ESTIMATION__STATE_ESTIMATION_INTERFACE_HPP_


#include <helper_functions/crtp.hpp>
#include <state_estimation/measurement/measurement_interface.hpp>
#include <state_estimation/visibility_control.hpp>

#include <chrono>

namespace autoware
{
namespace common
{
namespace state_estimation
{

///
/// @brief      Interface for a filter that can be used to track an object.
///
/// @tparam     Derived  An implementation of this filter.
///
template<typename Derived>
class STATE_ESTIMATION_PUBLIC StateEstimationInterface
  : public common::helper_functions::crtp<Derived>
{
public:
  ///
  /// @brief      Predict the state by dt into the future.
  ///
  /// @param[in]  dt    Time for prediction.
  ///
  /// @return     A new predicted state.
  ///
  auto predict(const std::chrono::nanoseconds & dt) {return this->impl().crtp_predict(dt);}

  ///
  /// @brief      Correct the state with a measurement.
  ///
  /// @note       It is expected that prediction step was called before-wise.
  ///
  /// @param[in]  measurement   The measurement
  ///
  /// @tparam     MeasurementT  Measurement type
  ///
  /// @return     A corrected state.
  ///
  template<typename MeasurementT>
  auto correct(const MeasurementT & measurement)
  {
    static_assert(
      std::is_base_of<MeasurementInterface<MeasurementT>, MeasurementT>::value,
      "\n\nMeasurement must inherit from MeasurementInterface\n\n");
    return this->impl().template crtp_correct<MeasurementT>(measurement);
  }

  ///
  /// @brief      Reset the internal state of the vector to the given state and covariance.
  ///
  /// @param[in]  state       The new state
  /// @param[in]  covariance  The new covariance
  ///
  /// @tparam     StateT      The state type. It must match the one already stored in the filter.
  ///
  template<typename StateT>
  void reset(const StateT & state, const typename StateT::Matrix & covariance)
  {
    static_assert(
      std::is_same<typename Derived::State, StateT>::value,
      "\n\nProvided type StateT must match the filter implementation State type.\n\n");
    this->impl().template crtp_reset(state, covariance);
  }

  /// @brief      Return current state.
  auto & state() {return this->impl().crtp_state();}
  /// @brief      Return current state.
  const auto & state() const {return this->impl().crtp_state();}

  /// @brief      Return current covariance.
  auto & covariance() {return this->impl().crtp_covariance();}
  /// @brief      Return current covariance.
  const auto & covariance() const {return this->impl().crtp_covariance();}
};


}  // namespace state_estimation
}  // namespace common
}  // namespace autoware

#endif  // STATE_ESTIMATION__STATE_ESTIMATION_INTERFACE_HPP_
