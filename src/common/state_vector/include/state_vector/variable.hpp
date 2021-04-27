// Copyright 2021 the Autoware Foundation
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
// Developed by Apex.AI, Inc.

/// \copyright Copyright 2021 the Autoware Foundation
/// All rights reserved.
/// \file
/// \brief Contains base tag structs that define variables and traits to check if a type is one.

#ifndef STATE_VECTOR__VARIABLE_HPP_
#define STATE_VECTOR__VARIABLE_HPP_

#include <state_vector/visibility_control.hpp>

#include <type_traits>


namespace autoware
{
namespace common
{
namespace state_vector
{

///
/// @brief      A tag struct used to disambiguate variables from other types.
///
struct STATE_VECTOR_PUBLIC Variable {};

///
/// @brief      A tag struct used to disambiguate variables that store angles from other types.
///
///             Inheriting from AngleVariable allows to automatically wrap angles upon need.
///
struct STATE_VECTOR_PUBLIC AngleVariable : Variable {};

///
/// @brief      A trait to check if a type is a variable by checking if it inherits from Variable.
///
/// @tparam     T     Query type.
///
template<typename T>
struct STATE_VECTOR_PUBLIC is_variable : std::conditional_t<
    std::is_base_of<Variable, T>::value, std::true_type, std::false_type> {};

///
/// @brief      A trait to check if a variable represents an angle.
///
/// @tparam     T     Variable type.
///
template<typename T>
struct STATE_VECTOR_PUBLIC is_angle : std::conditional_t<
    std::is_base_of<AngleVariable, T>::value, std::true_type, std::false_type> {};


}  // namespace state_vector
}  // namespace common
}  // namespace autoware

#endif  // STATE_VECTOR__VARIABLE_HPP_
