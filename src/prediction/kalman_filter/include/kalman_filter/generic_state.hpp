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
/// \brief This file defines a class for a generic state vector representation.

#ifndef KALMAN_FILTER__GENERIC_STATE_HPP_
#define KALMAN_FILTER__GENERIC_STATE_HPP_

#include <common/type_traits.hpp>
#include <common/types.hpp>
#include <helper_functions/type_name.hpp>
#include <kalman_filter/common_variables.hpp>
#include <kalman_filter/visibility_control.hpp>

#include <Eigen/Core>

#include <tuple>

namespace autoware
{
namespace prediction
{

///
/// @brief      A representation of a generic state vectors with specified variables.
///
/// @tparam     VariableTs  Variables which this state consists of.
///
template<typename ScalarT, typename ... VariableTs>
class KALMAN_FILTER_PUBLIC GenericState
{
  static_assert(
    common::type_traits::conjunction<is_variable<VariableTs>...>::value,
    "\n\nState can only be generated from variables, i.e. types that inherit from Variable.\n\n");
  static_assert(
    std::is_arithmetic<ScalarT>::value, "\n\nThe provided scalar type is not arithmetic.\n\n");
  static_assert(
    sizeof...(VariableTs) > 0, "\n\nCannot create state without variables.\n\n");

public:
  constexpr static std::int32_t kSize = sizeof...(VariableTs);
  using Variables = std::tuple<VariableTs...>;
  using Vector = Eigen::Matrix<ScalarT, kSize, 1>;
  using Matrix = Eigen::Matrix<ScalarT, kSize, kSize>;
  using Scalar = ScalarT;

  ///
  /// @brief      Default constructor.
  ///
  GenericState() = default;

  ///
  /// @brief      Constructs a new instance from an Eigen vector
  ///
  /// @param[in]  vector  An Eigen vector.
  ///
  explicit GenericState(const Vector & vector)
  : m_state{vector} {}

  ///
  /// @brief      Get underlying Eigen vector.
  ///
  /// @return     A reference to the underlying Eigen vector.
  ///
  inline Vector & vector() noexcept {return m_state;}
  ///
  /// @brief      Get underlying Eigen vector.
  ///
  /// @return     A const reference to the underlying Eigen vector.
  ///
  inline const Vector & vector() const noexcept {return m_state;}

  ///
  /// @brief      Get an element of the vector.
  ///
  /// @param[in]  idx   The index of the vector element.
  ///
  /// @return     A reference to the vector element.
  ///
  inline ScalarT & operator[](const Eigen::Index idx) {return m_state[idx];}
  ///
  /// @brief      Get an element of the vector.
  ///
  /// @param[in]  idx   The index of the vector element.
  ///
  /// @return     A reference to the vector element.
  ///
  inline const ScalarT & operator[](const Eigen::Index idx) const {return m_state[idx];}

  ///
  /// @brief      Get vector element at a given variable.
  ///
  /// @tparam     VariableT  A variable from the state.
  ///
  /// @return     A reference to the element representing this variable.
  ///
  template<typename VariableT>
  inline ScalarT & at() noexcept
  {
    static_assert(is_variable<VariableT>::value, "Type is not a Variable.");
    return m_state[common::type_traits::index<VariableT, Variables>::value];
  }
  ///
  /// @brief      Get vector element at a given variable.
  ///
  /// @tparam     VariableT  A variable from the state.
  ///
  /// @return     A reference to the element representing this variable.
  ///
  template<typename VariableT>
  inline const ScalarT & at() const noexcept
  {
    static_assert(is_variable<VariableT>::value, "Type is not a Variable.");
    return m_state[common::type_traits::index<VariableT, Variables>::value];
  }

  ///
  /// @brief      Get an index of the variable in the state.
  ///
  /// @tparam     VariableT  A variable from the state.
  ///
  /// @return     An index of the given variable in the state vector.
  ///
  template<typename VariableT>
  constexpr static Eigen::Index index_of() noexcept
  {
    static_assert(is_variable<VariableT>::value, "Type is not a Variable.");
    return common::type_traits::index<VariableT, Variables>::value;
  }

  /// @brief      Get the size of the state instance.
  inline Eigen::Index size() const noexcept {return kSize;}

  /// @brief      An equality operator.
  friend bool operator==(const GenericState & lhs, const GenericState & rhs)
  {
    return lhs.m_state.isApprox(rhs.m_state);
  }

  /// @brief      Allow using << operator with this class.
  ///
  ///             This function adds the variables to the output stream along with their names.
  friend std::ostream & operator<<(std::ostream & out, const GenericState & state)
  {
    out << "State:";
    int counter = 0;
    auto print = [&out, &state, &counter](auto element) {
        using VariableT = std::decay_t<decltype(element)>;
        out << "\n  " <<
          helper_functions::get_type_name<VariableT>() << ": " << state[counter++];
      };
    Variables dummy_variables_tuple;
    common::type_traits::visit(dummy_variables_tuple, print);
    return out;
  }

private:
  /// Underlying Eigen vector.
  Vector m_state{Vector::Zero()};
};

///
/// @brief      Check if a given type is a state.
///
/// @tparam     StateT  A query potential state type.
///
template<typename StateT>
struct KALMAN_FILTER_PUBLIC is_state : public std::false_type {};

///
/// @brief      A specialization of this trait for GenericState type.
///
/// @tparam     ScalarT     A scalar type of a state.
/// @tparam     VariableTs  Variable types.
///
template<typename ScalarT, typename ... VariableTs>
struct KALMAN_FILTER_PUBLIC is_state<GenericState<ScalarT, VariableTs...>>
  : public std::true_type {};

/// A typedef for the 32 bit floating point state.
template<typename ... Ts>
using FloatState = GenericState<common::types::float32_t, Ts...>;

/// A typedef for the 64 bit floating point state.
template<typename ... Ts>
using DoubleState = GenericState<common::types::float64_t, Ts...>;

}  // namespace prediction
}  // namespace autoware

#endif  // KALMAN_FILTER__GENERIC_STATE_HPP_
