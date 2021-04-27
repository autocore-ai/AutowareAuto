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

#ifndef STATE_ESTIMATION__MEASUREMENT__MEASUREMENT_INTERFACE_HPP_
#define STATE_ESTIMATION__MEASUREMENT__MEASUREMENT_INTERFACE_HPP_

#include <helper_functions/crtp.hpp>
#include <state_estimation/visibility_control.hpp>
#include <state_vector/generic_state.hpp>

#include <type_traits>

namespace autoware
{
namespace common
{
namespace state_estimation
{

///
/// @brief      A CRTP interface to any measurement.
///
/// @tparam     Derived  Derived class that is expected to inherit from this class.
///
template<typename Derived>
struct STATE_ESTIMATION_PUBLIC MeasurementInterface : public common::helper_functions::crtp<Derived>
{
  /// @brief      Get measurement as a state vector.
  auto & state()
  {
    using ReturnType = std::decay_t<decltype(this->impl().crtp_state())>;
    static_assert(
      common::state_vector::is_state<ReturnType>::value,
      "\n\nFunction crtp_state must return a state.\n\n");
    return this->impl().crtp_state();
  }
  /// @brief      Get measurement as a state vector.
  const auto & state() const
  {
    using ReturnType = std::decay_t<decltype(this->impl().crtp_state())>;
    static_assert(
      common::state_vector::is_state<ReturnType>::value,
      "\n\nFunction crtp_state must return a state.\n\n");
    return this->impl().crtp_state();
  }

  /// @brief      Get covariance of the measurement as a matrix.
  auto & covariance() {return this->impl().crtp_covariance();}
  const auto & covariance() const {return this->impl().crtp_covariance();}

  /// @brief      Get variances as a vector. Note that these represent sigmas, not sigmas^2.
  auto & variances() {return this->impl().crtp_variances();}
  /// @brief      Get variances as a vector. Note that these represent sigmas, not sigmas^2.
  const auto & variances() const {return this->impl().crtp_variances();}

  ///
  /// @brief      Create a new instance of the measurement from another state.
  ///
  /// @note       While it might seem like this function could be static, it is explicitly not
  ///             static to allow for future implementation to cache some internal workspace
  ///             required for this computation along with the mapping matrix computation. To allow
  ///             these optimized implementation to be a drop-in replacement for the naive
  ///             implementations, this function is not static.
  ///
  /// @param[in]  other_state  The other state.
  ///
  /// @tparam     OtherStateT  Type of the other state.
  ///
  /// @return     Return a state that is a mapping of the OtherStateT to this measurement's state.
  ///
  template<typename OtherStateT>
  auto create_new_instance_from(const OtherStateT & other_state) const
  {
    static_assert(
      common::state_vector::is_state<OtherStateT>::value,
      "\n\nThe other state must be a state.\n\n");
    using ReturnType =
      std::decay_t<decltype(this->impl().crtp_create_new_instance_from(other_state))>;
    static_assert(
      common::state_vector::is_state<ReturnType>::value,
      "\n\nFunction crtp_create_new_instance_from must return a state.\n\n");
    return this->impl().crtp_create_new_instance_from(other_state);
  }

  ///
  /// @brief      Map this measurement into another state space.
  ///
  /// @note       It is implementation defined, but the other_state values are usually maintained
  ///             for all variables not updated through this mapping. Please see the concrete
  ///             implementation documentation when in doubt.
  ///
  /// @param[in]  other_state  The other state.
  ///
  /// @tparam     OtherStateT  Type of the other state.
  ///
  /// @return     An instance of OtherStateT with updated variables observed by this measurement.
  ///
  template<typename OtherStateT>
  OtherStateT map_into(const OtherStateT & other_state) const
  {
    static_assert(
      common::state_vector::is_state<OtherStateT>::value,
      "\n\nThe other state must be a state.\n\n");
    return this->impl().crtp_map_into(other_state);
  }

  ///
  /// @brief      Get a matrix that maps a state to this measurement's state space.
  ///
  /// @details    For a measurement of dimensionality M the resulting matrix converts a state vector
  ///             of type OtherStateT (of dimensionality N) to a state vector of this measurement
  ///             (of dimensionality M). This matrix is, generally speaking, the Jacobian of a
  ///             mapping function h, such as mapped_state = h(other_state), implemented in
  ///             create_new_instance_from.
  ///
  /// @param[in]  other_state  The other state to be converted into the measurement state space.
  ///
  /// @tparam     OtherStateT  Type of the other state.
  ///
  /// @return     Returns a mapping matrix that maps from OtherStateT to this measurement's state.
  ///
  template<typename OtherStateT>
  auto mapping_matrix_from(const OtherStateT & other_state) const
  {
    static_assert(
      common::state_vector::is_state<OtherStateT>::value,
      "The other state must be a state.");
    return this->impl().crtp_mapping_matrix_from(other_state);
  }
};


}  // namespace state_estimation
}  // namespace common
}  // namespace autoware

#endif  // STATE_ESTIMATION__MEASUREMENT__MEASUREMENT_INTERFACE_HPP_
