// Copyright 2021 Apex.AI, Inc.
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
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.


#ifndef STATE_ESTIMATION__MEASUREMENT__LINEAR_MEASUREMENT_HPP_
#define STATE_ESTIMATION__MEASUREMENT__LINEAR_MEASUREMENT_HPP_

#include <common/type_traits.hpp>
#include <state_estimation/measurement/measurement_interface.hpp>

#include <Eigen/Core>

#include <chrono>
#include <tuple>

namespace autoware
{
namespace common
{
namespace state_estimation
{

///
/// @brief      A class that represents a linear measurement.
///
/// @details    A linear measurement is one that measures directly variables from some unknown real
///             state, thus requiring no measurement Jacobian computation. The mapping matrix will
///             only have 1s and 0s in it.
///
/// @tparam     StateT  A measured state.
///
template<typename StateT>
class STATE_ESTIMATION_PUBLIC LinearMeasurement
  : public MeasurementInterface<LinearMeasurement<StateT>>
{
  template<typename OtherStateT>
  using MappingMatrixFrom =
    Eigen::Matrix<typename StateT::Scalar, StateT::size(), OtherStateT::size()>;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using State = StateT;
  /// @brief      Default constructor.
  LinearMeasurement() = default;
  ///
  /// @brief      Construct a new instance of the measurement from a state vector.
  ///
  /// @param[in]  measurement  The measurement
  /// @param[in]  covariance   The covariance
  ///
  explicit LinearMeasurement(
    const typename StateT::Vector & measurement,
    const typename StateT::Matrix & covariance)
  : m_measurement{measurement},
    m_covariance{covariance} {}
  ///
  /// @brief      Convenience factory function to construct a measurement from a vector of standard
  ///             deviations.
  ///
  /// @details    This results in a diagonal covariance matrix, with the squared standard
  ///             deviations as entries. In a diagonal covariance matrix, the variables are
  ///             uncorrelated.
  ///
  /// @param[in]  measurement         The measurement
  /// @param[in]  standard_deviation  The standard deviation for each variable.
  ///
  static LinearMeasurement create_with_stddev(
    const typename StateT::Vector & measurement,
    const typename StateT::Vector & standard_deviation)
  {
    return LinearMeasurement{measurement,
      standard_deviation.array().square().matrix().asDiagonal()};
  }

  ///
  /// @brief      Cast to another scalar type.
  ///
  /// @return     The measurement with a different scalar type.
  ///
  template<typename NewScalarT>
  auto cast() const noexcept
  {
    using NewState = decltype(m_measurement.template cast<NewScalarT>());
    return LinearMeasurement<NewState>{
      m_measurement.vector().template cast<NewScalarT>(),
      m_covariance.template cast<NewScalarT>()
    };
  }

protected:
  // Allow the CRTP interface to call private functions from this class.
  friend MeasurementInterface<LinearMeasurement<StateT>>;

  /// @brief      Get state vector function that is to be called by the CRTP interface.
  inline StateT & crtp_state() noexcept {return m_measurement;}
  /// @brief      Get state vector function that is to be called by the CRTP interface.
  inline const StateT & crtp_state() const noexcept {return m_measurement;}
  /// @brief      Get covariance function that is to be called by the CRTP interface.
  inline typename StateT::Matrix & crtp_covariance() noexcept {return m_covariance;}
  /// @brief      Get covariance function that is to be called by the CRTP interface.
  inline const typename StateT::Matrix & crtp_covariance() const noexcept {return m_covariance;}
  ///
  /// @brief      Create a new instance of the measurement from another state.
  ///
  /// @details    This is a linear measurement, i.e., it directly observes the variables of another
  ///             state, thus this mapping function is linear and just copied values for the
  ///             matching variables in both states.
  ///
  /// @note       It is expected that the measurement StateT is a direct sub-state of the
  ///             OtherStateT.
  ///
  /// @param[in]  other_state  The other state
  ///
  /// @tparam     OtherStateT  State to which we map the internal state StateT.
  ///
  /// @return     A vector that represents a mapping of another state to this measurement's frame.
  ///
  template<typename OtherStateT>
  StateT crtp_create_new_instance_from(const OtherStateT & other_state) const
  {
    return other_state.template copy_into<StateT>();
  }
  ///
  /// @brief      Copy this measurement into another state space.
  ///
  /// @note       Values present in other_state but not in this measurement will stay intact.
  ///
  /// @param[in]  other_state  The other state
  ///
  /// @tparam     OtherStateT  State to which we map the internal state StateT.
  ///
  /// @return     A vector that represents a mapping of another state to this measurement's frame.
  ///
  template<typename OtherStateT>
  OtherStateT crtp_map_into(const OtherStateT & other_state) const noexcept
  {
    return m_measurement.copy_into(other_state);
  }
  ///
  /// @brief      Get the mapping matrix between the two states called by the CRTP interface.
  ///
  /// @details    As this is a linear measurement (one that directly measures variables of some
  ///             state) this function only fills the entries of the mapping matrix with 1s and 0s.
  ///
  /// @note       The OtherStateT input parameter is not used in this implementation. It is required
  ///             for other implementation that must map into the measurement state space in a
  ///             non-linear way.
  ///
  /// @tparam     OtherStateT  State to which we map the internal state StateT.
  ///
  /// @return     A matrix M, such that other_state = M * this_state.
  ///
  template<typename OtherStateT>
  MappingMatrixFrom<OtherStateT> crtp_mapping_matrix_from(const OtherStateT &) const
  {
    MappingMatrixFrom<OtherStateT> m{MappingMatrixFrom<OtherStateT>::Zero()};
    auto fill_mapping_matrix = [&m, this](auto variable) {
        using VariableT = std::decay_t<decltype(variable)>;
        constexpr auto index_in_this_state = StateT::template index_of<VariableT>();
        constexpr auto index_in_other_state = OtherStateT::template index_of<VariableT>();
        m(index_in_this_state, index_in_other_state) = 1;
      };
    using CommonVariablesTuple = typename autoware::common::type_traits::intersect<
      typename StateT::Variables, typename OtherStateT::Variables>::type;
    common::type_traits::visit(CommonVariablesTuple{}, fill_mapping_matrix);
    return m;
  }

  /// @brief An equality operator.
  friend bool operator==(const LinearMeasurement & lhs, const LinearMeasurement & rhs)
  {
    return (lhs.state() == rhs.state()) &&
           lhs.covariance().isApprox(rhs.covariance());
  }

private:
  /// Current measurement vector.
  StateT m_measurement{};
  /// Current measurement covariance matrix.
  typename StateT::Matrix m_covariance{StateT::Matrix::Zero()};
};

}  // namespace state_estimation
}  // namespace common
}  // namespace autoware

#endif  // STATE_ESTIMATION__MEASUREMENT__LINEAR_MEASUREMENT_HPP_
