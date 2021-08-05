// Copyright 2021 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
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

#ifndef STATE_VECTOR__GENERIC_STATE_HPP_
#define STATE_VECTOR__GENERIC_STATE_HPP_

#include <common/type_traits.hpp>
#include <common/types.hpp>
#include <helper_functions/angle_utils.hpp>
#include <helper_functions/type_name.hpp>
#include <state_vector/common_variables.hpp>
#include <state_vector/visibility_control.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wold-style-cast"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wuseless-cast"
#include <Eigen/Core>
#pragma GCC diagnostic pop

#include <tuple>

namespace autoware
{
namespace common
{
namespace state_vector
{

/// Forward-declare is_state trait.
template<typename StateT>
struct is_state;


///
/// @brief      A representation of a generic state vectors with specified variables.
///
/// @tparam     VariableTs  Variables which this state consists of.
///
template<typename ScalarT, typename ... VariableTs>
class STATE_VECTOR_PUBLIC GenericState
{
  static_assert(
    common::type_traits::conjunction<is_variable<VariableTs>...>::value,
    "\n\nState can only be generated from variables, i.e. types that inherit from Variable.\n\n");
  static_assert(
    std::is_arithmetic<ScalarT>::value, "\n\nThe provided scalar type is not arithmetic.\n\n");
  static_assert(
    sizeof...(VariableTs) > 0, "\n\nCannot create state without variables.\n\n");
  // Hide this under private to make sure it is never ODR-used.
  constexpr static std::int32_t kSize = sizeof...(VariableTs);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
  inline ScalarT & at(const VariableT) noexcept
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
  /// @brief      Get vector element at a given variable.
  ///
  /// @tparam     VariableT  A variable from the state.
  ///
  /// @return     A reference to the element representing this variable.
  ///
  template<typename VariableT>
  inline const ScalarT & at(const VariableT) const noexcept
  {
    static_assert(is_variable<VariableT>::value, "Type is not a Variable.");
    return m_state[common::type_traits::index<VariableT, Variables>::value];
  }


  ///
  /// @brief      Copy values of this state into another state.
  ///
  /// @details    This function copies (a subset of) variables from the current state into another
  ///             state. If `other_state` is provided, it is used as a "donor" state, i.e., the
  ///             values already in this `other_state` are unchanged unless overwritten by the ones
  ///             present in the current state. If no `other_state` is given, a new zero-initialized
  ///             instance of OtherStateT will be created and this state will be used for the
  ///             further update using the variables from the current state. Please see tests for
  ///             examples of usage.
  ///
  /// @note       This function also handles a rearrangement of the variables if the OtherStateT has
  ///             the variables in a different order from the current state.
  ///
  /// @note       This function copies only variables that are present in both states. It silently
  ///             does nothing if there are no common variables between the states.
  ///
  /// @param[in]  other_state  Other state.
  ///
  /// @tparam     OtherStateT  Type of the other state.
  ///
  /// @return     The updated other state.
  ///
  template<typename OtherStateT>
  inline OtherStateT copy_into(OtherStateT other_state = OtherStateT{}) const noexcept
  {
    static_assert(
      is_state<OtherStateT>::value,
      "\n\nOtherStateT is not a GenericState.\n\n");
    auto copy_variables = [&other_state, this](auto variable) {
        other_state.at(variable) = this->at(variable);
      };
    using CommonVariablesTuple = typename autoware::common::type_traits::intersect<
      Variables, typename OtherStateT::Variables>::type;
    common::type_traits::visit(CommonVariablesTuple{}, copy_variables);
    return other_state;
  }

  ///
  /// @brief      Get an index of the variable in the state.
  ///
  /// @tparam     VariableT  A variable from the state.
  ///
  /// @return     An index of the given variable in the state vector.
  ///
  template<typename VariableT>
  inline constexpr static Eigen::Index index_of() noexcept
  {
    static_assert(is_variable<VariableT>::value, "Type is not a Variable.");
    return common::type_traits::index<VariableT, Variables>::value;
  }

  ///
  /// @brief      Get the size of the state.
  ///
  /// @note       This can also be called on an instance of the state.
  ///
  /// @return     Number of variables in this state.
  ///
  inline constexpr static Eigen::Index size() noexcept {return kSize;}

  /// @brief      Get a variables tuple used for iteration over all variables.
  inline constexpr static Variables variables() noexcept{return Variables{};}

  /// @brief      An equality operator.
  friend constexpr bool operator==(const GenericState & lhs, const GenericState & rhs)
  {
    return lhs.m_state.isApprox(rhs.m_state);
  }

  /// @brief      Allow using << operator with this class.
  ///
  ///             This function adds the variables to the output stream along with their names.
  friend std::ostream & operator<<(std::ostream & out, const GenericState & state)
  {
    out << "State:";
    auto print = [&out, &state](auto element) {
        out << "\n  " <<
          autoware::helper_functions::get_type_name(element) << ": " << state.at(element);
      };
    common::type_traits::visit(GenericState::variables(), print);
    return out;
  }

  ///
  /// @brief      Addition assignment operator.
  ///
  /// @param[in]  rhs   Other state vector
  ///
  /// @return     A reference to this state.
  ///
  inline GenericState & operator+=(const GenericState & rhs) noexcept
  {
    return this->operator+=(rhs.vector());
  }

  ///
  /// @brief      Addition assignment operator.
  ///
  /// @param[in]  rhs   Other state vector as an Eigen vector.
  ///
  /// @warning    This can be unsafe if the order of the variables in the Eigen vector is different
  ///             from the one used within this state class.
  ///
  /// @return     A reference to this state.
  ///
  inline GenericState & operator+=(const Vector & rhs) noexcept
  {
    this->vector() += rhs;
    return *this;
  }

  ///
  /// @brief      Subtraction assignment operator.
  ///
  /// @param[in]  rhs   Other state vector as an Eigen vector.
  ///
  /// @return     A reference to this state.
  ///
  inline GenericState & operator-=(const GenericState & rhs) noexcept
  {
    return this->operator-=(rhs.vector());
  }

  ///
  /// @brief      Subtraction assignment operator.
  ///
  /// @param[in]  rhs   Other state vector as an Eigen vector.
  ///
  /// @warning    This can be unsafe if the order of the variables in the Eigen vector is different
  ///             from the one used within this state class.
  ///
  /// @return     A reference to this state.
  ///
  inline GenericState & operator-=(const Vector & rhs) noexcept
  {
    this->vector() -= rhs;
    return *this;
  }


  ///
  /// @brief      Subtraction operator.
  ///
  /// @param[in]  lhs   Left hand side of the subtraction.
  /// @param[in]  rhs   Right hand side of the subtraction.
  ///
  /// @tparam     T  Either a state vector type or an underlying Eigen vector type.
  ///
  /// @return     A state that results from this operation.
  ///
  template<typename T>
  inline friend GenericState operator-(GenericState lhs, const T & rhs) noexcept
  {
    lhs -= rhs;
    return lhs;
  }

  ///
  /// @brief      Addition operator.
  ///
  /// @param[in]  lhs   Left hand side of the addition.
  /// @param[in]  rhs   Right hand side of the addition.
  ///
  /// @tparam     T  Either a state vector type or an underlying Eigen vector type.
  ///
  /// @return     A state that results from this operation.
  ///
  template<typename T>
  inline friend GenericState operator+(GenericState lhs, const T & rhs) noexcept
  {
    lhs += rhs;
    return lhs;
  }

  ///
  /// @brief      Wrap all variables that represent angles using the
  ///             common::helper_functions::wrap_angle.
  ///
  constexpr void wrap_all_angles() noexcept
  {
    const auto wrap_angles = [this](auto variable) {
        using VariableT = decltype(variable);
        if (is_angle<VariableT>::value) {
          this->at(variable) = common::helper_functions::wrap_angle(this->at(variable));
        }
      };
    common::type_traits::visit(GenericState::variables(), wrap_angles);
  }

  ///
  /// @brief      Wrap all variables that represent angles using the
  ///             common::helper_functions::wrap_angle.
  ///
  constexpr friend GenericState wrap_all_angles(GenericState state) noexcept
  {
    state.wrap_all_angles();
    return state;
  }

  ///
  /// @brief      Cast to another scalar type.
  ///
  /// @return     The state with a different scalar type.
  ///
  template<typename NewScalarT>
  GenericState<NewScalarT, VariableTs...> cast() const noexcept
  {
    return GenericState<NewScalarT, VariableTs...>{m_state.template cast<NewScalarT>()};
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
struct STATE_VECTOR_PUBLIC is_state : public std::false_type {};

///
/// @brief      A specialization of this trait for GenericState type.
///
/// @tparam     ScalarT     A scalar type of a state.
/// @tparam     VariableTs  Variable types.
///
template<typename ScalarT, typename ... VariableTs>
struct STATE_VECTOR_PUBLIC is_state<GenericState<ScalarT, VariableTs...>>
  : public std::true_type {};

/// A typedef for the 32 bit floating point state.
template<typename ... Ts>
using FloatState = GenericState<common::types::float32_t, Ts...>;

/// A typedef for the 64 bit floating point state.
template<typename ... Ts>
using DoubleState = GenericState<common::types::float64_t, Ts...>;

}  // namespace state_vector
}  // namespace common
}  // namespace autoware

#endif  // STATE_VECTOR__GENERIC_STATE_HPP_
