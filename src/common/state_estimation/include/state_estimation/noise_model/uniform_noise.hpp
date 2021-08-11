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

#ifndef STATE_ESTIMATION__NOISE_MODEL__UNIFORM_NOISE_HPP_
#define STATE_ESTIMATION__NOISE_MODEL__UNIFORM_NOISE_HPP_

#include <state_estimation/noise_model/noise_interface.hpp>
#include <state_estimation/visibility_control.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <chrono>
#include <type_traits>
#include <vector>

namespace autoware
{
namespace common
{
namespace state_estimation
{

///
/// @brief      This class contains the logic for a uniform noise model.
///
/// @tparam     StateT  State vector type for which this noise model is created.
///
template<typename StateT>
class STATE_ESTIMATION_PUBLIC UniformNoise : public NoiseInterface<UniformNoise<StateT>>
{
  using Scalar = typename StateT::Scalar;
  using Matrix = typename StateT::Matrix;
  template<typename OtherScalarT>
  using VarianceVector = Eigen::Matrix<OtherScalarT, StateT::size(), 1>;

public:
  /// A convenience typedef for the state for which this noise model is to be used.
  using State = StateT;

  ///
  /// @brief      Create the noise model from an existing covariance matrix.
  ///
  /// @param[in]  covariance  The covariance
  ///
  explicit UniformNoise(const Matrix & covariance) noexcept
  : m_covariance{covariance} {}

  ///
  /// @brief      Create the noise model from an array
  ///
  /// @param[in]  variances     The values for variances, i.e., values for σ (not squared)
  ///
  /// @tparam     OtherScalarT  Some scalar type.
  ///
  template<typename OtherScalarT>
  explicit UniformNoise(
    const std::array<OtherScalarT, State::size()> & variances) noexcept
  : m_covariance{
      Eigen::Map<const VarianceVector<OtherScalarT>>(variances.data())
      .template cast<Scalar>().array().square().matrix().asDiagonal()} {}

  ///
  /// @brief      Create the noise model from a vector
  ///
  /// @param[in]  variances     The values for variances, i.e., values for σ (not squared)
  ///
  /// @throws     std::runtime_error  if the vector is of a wrong size.
  ///
  /// @tparam     OtherScalarT  Some scalar type.
  ///
  template<typename OtherScalarT>
  explicit UniformNoise(const std::vector<OtherScalarT> & variances)
  {
    if (variances.size() != static_cast<std::size_t>(State::size())) {
      throw std::runtime_error(
              "There must be " + std::to_string(State::size()) +
              " variances for initializing the uniform noise model, but " +
              std::to_string(variances.size()) + " provided");
    }
    m_covariance =
      Eigen::Map<const VarianceVector<OtherScalarT>>(variances.data())
      .template cast<Scalar>().array().square().matrix().asDiagonal();
  }


  ///
  /// @brief      Create noise model from variances directly.
  ///
  /// @param[in]  variance    The first variance, i.e., value for σ (not squared)
  /// @param[in]  variances   The variances for all the other variables
  ///
  /// @tparam     VarianceTs  Types for other variances.
  ///
  template<typename ... VarianceTs>
  explicit UniformNoise(const Scalar variance, const VarianceTs ... variances)
  : UniformNoise{std::array<Scalar, State::size()> {variance, variances ...}}
  {
    static_assert(
      sizeof...(VarianceTs) + 1 == State::size(),
      "Wrong number of variances passed into the UniformNoise constructor");
  }

protected:
  /// Allow the interface to access the protected and private members to visually encapsulate the
  /// implementation.
  friend NoiseInterface<UniformNoise<StateT>>;

  ///
  /// @brief      A CRTP-called covariance getter.
  ///
  /// @return     A covariance of the noise process over a given time span.
  ///
  Matrix crtp_covariance(const std::chrono::nanoseconds & dt) const noexcept
  {
    return m_covariance * std::chrono::duration_cast<std::chrono::duration<Scalar>>(dt).count();
  }

private:
  /// Store the covariance matrix internally.
  Matrix m_covariance{};
};

}  // namespace state_estimation
}  // namespace common
}  // namespace autoware

#endif  // STATE_ESTIMATION__NOISE_MODEL__UNIFORM_NOISE_HPP_
