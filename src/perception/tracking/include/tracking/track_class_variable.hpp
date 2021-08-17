// Copyright 2021 Apex.AI, Inc.
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

// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef TRACKING__TRACK_CLASS_VARIABLE_HPP_
#define TRACKING__TRACK_CLASS_VARIABLE_HPP_


#include <autoware_auto_msgs/msg/object_classification.hpp>
#include <state_vector/generic_state.hpp>
#include <state_vector/variable.hpp>


namespace autoware
{
namespace perception
{
namespace tracking
{

///
/// @brief      A struct to create a variable type from a given `uint8_t` value
///
/// @details    This is used to ensure each of the values gets its own unique type.
///
/// @tparam     kClassificationConstant  A given constant value.
///
template<std::uint8_t kClassificationConstant>
struct ClassificationVariable : public common::state_vector::Variable,
  public std::integral_constant<std::uint8_t, kClassificationConstant> {};

///
/// @brief      Check that all the indices in the state vector correspond to the integral constant
///             that the variable type represents.
///
/// @details    This check is needed to ensure that all the values from ObjectClassification
///             constants are covered by the variables used in the state. The function itself does
///             nothing important but it will not compile if there is a mismatch in variable values.
///
/// @tparam     StateT  A state that needs checking.
///
template<typename StateT>
static inline void assert_indices_match_classification_constants()
{
  static_assert(
    autoware::common::state_vector::is_state<StateT>::value,
    "The provided type must represent a GenericState type.");

  auto assert_indices_match_classification_constants = [](auto variable) {
      using VariableT = decltype(variable);
      static_assert(
        StateT::template index_of<VariableT>() == VariableT::value,
        "The index of the variable must correspond to the integral constant stored inside of "
        "this variable class.");
    };
  common::type_traits::visit(StateT::variables(), assert_indices_match_classification_constants);
}

/// A common state used for classification in the tracker.
using ObjectClassificationState = common::state_vector::FloatState<
  ClassificationVariable<autoware_auto_msgs::msg::ObjectClassification::UNKNOWN>,
  ClassificationVariable<autoware_auto_msgs::msg::ObjectClassification::CAR>,
  ClassificationVariable<autoware_auto_msgs::msg::ObjectClassification::TRUCK>,
  ClassificationVariable<autoware_auto_msgs::msg::ObjectClassification::TRAILER>,
  ClassificationVariable<autoware_auto_msgs::msg::ObjectClassification::MOTORCYCLE>,
  ClassificationVariable<autoware_auto_msgs::msg::ObjectClassification::BICYCLE>,
  ClassificationVariable<autoware_auto_msgs::msg::ObjectClassification::PEDESTRIAN>>;

}  // namespace tracking
}  // namespace perception
}  // namespace autoware

#endif  // TRACKING__TRACK_CLASS_VARIABLE_HPP_
