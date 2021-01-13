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

#ifndef STATE_ESTIMATION_NODES__MEASUREMENT_HPP_
#define STATE_ESTIMATION_NODES__MEASUREMENT_HPP_

#include <common/types.hpp>
#include <state_estimation_nodes/visibility_control.hpp>
#include <state_estimation_nodes/time.hpp>

#include <Eigen/Core>

#include <cstdint>
#include <tuple>

namespace autoware
{
namespace prediction
{
///
/// A class that wraps a measurement.
///
/// It can be used to create a measurement and produce valid values for:
/// - Full state of the system generated from this measurement.
/// - Mapping matrix to map measurement to the state.
/// - Variances of this measurement.
///
/// @tparam     T                       Type of measurement underlying type, e.g. float32_t
/// @tparam     kMeasurementModalities  Which modalities does this measurement represent.
///
template<typename T, std::uint32_t... kMeasurementModalities>
class STATE_ESTIMATION_NODES_PUBLIC Measurement
{
  static constexpr std::int32_t EntriesCount = sizeof...(kMeasurementModalities);

  using MeasurementModalityArray =
    std::array<std::uint32_t, static_cast<std::size_t>(EntriesCount)>;

  template<typename S, std::int32_t kSize>
  using Vector = Eigen::Matrix<S, kSize, 1U>;
  template<typename S, std::int32_t kNumOfColumns>
  using MapMatrix = Eigen::Matrix<S, EntriesCount, kNumOfColumns>;

  using MeasurementVector = Vector<T, EntriesCount>;
  using VarianceVector = Vector<T, EntriesCount>;

public:
  /// Create a measurement from the measured values and their variances.
  ///
  /// @param[in]  acquisition_time  The acquisition time for this measurement.
  /// @param[in]  values            The measured values.
  /// @param[in]  variances         The variances of the measured values.
  ///
  explicit Measurement(
    const MeasurementBasedTime & acquisition_time,
    const MeasurementVector & values, const VarianceVector & variances = {})
  : m_acquisition_time{acquisition_time}, m_measurement_values{values}, m_variances{variances} {}

  /// Get stored measured values.
  ///
  /// @return     The values measured.
  ///
  inline const MeasurementVector & get_values() const noexcept {return m_measurement_values;}

  /// Get stored variances.
  ///
  /// @return     Variances of this measurement as a vector of elements on the diagonal of R.
  ///
  inline const VarianceVector & get_variances() const noexcept {return m_variances;}

  /// Get the time at which this measurement was acquired.
  ///
  /// @return     The acquisition time of the measurement.
  ///
  inline const MeasurementBasedTime & get_acquisition_time() const noexcept
  {
    return m_acquisition_time;
  }

  ///
  /// Get a matrix that maps the observation to state given the state dimensionality.
  ///
  /// @note       We can optimize this by allocating memory just once in the class static context,
  ///             but this is left for the future if needed.
  ///
  /// @tparam     kNumOfStates  Number of states of the state vector.
  ///
  /// @return     The observation to state mapping matrix H such that: state = H * measurement.
  ///
  template<std::int32_t kNumOfStates>
  static MapMatrix<T, kNumOfStates> get_observation_to_state_mapping()
  {
    using MatrixT = MapMatrix<T, kNumOfStates>;
    MatrixT observation_to_state_mapping{MatrixT::Zero()};
    for (auto i = 0U; i < kMeasurementModalitiesArray.size(); ++i) {
      observation_to_state_mapping(i, kMeasurementModalitiesArray[i]) = static_cast<T>(1.0);
    }
    return observation_to_state_mapping;
  }

  ///
  /// Get a full state vector populated by the values from this measurement.
  ///
  /// Note that the ordering of these values depends on two factors:
  /// - which order of kMeasurementModalities is provided
  /// - integer values for measurement modalities in the enum.
  ///
  /// It is assumed that state is ordered exactly as values in kMeasurementModalities, i.e., a
  /// measurement modality 0 is the first one in the state and so on..
  ///
  /// @param[in]  donor_state     The state that will be reused to set values missing in this
  ///                             measurement.
  ///
  /// @tparam     kFullStateSize  The dimentionality of the state.
  ///
  /// @return     The values from a measurement represented as a vector state dimentionality.
  ///
  template<std::int32_t kFullStateSize>
  Vector<T, kFullStateSize> get_values_in_full_state(
    const Vector<T, kFullStateSize> & donor_state = Vector<T, kFullStateSize>::Zero()) const
  {
    static_assert(kFullStateSize >= EntriesCount, "State too small to contain this measurement.");
    Vector<T, kFullStateSize> state{donor_state};
    for (auto i = 0U; i < kMeasurementModalitiesArray.size(); ++i) {
      state[kMeasurementModalitiesArray[i]] = m_measurement_values[i];
    }
    return state;
  }

private:
  /// Stores the order of the modalities that the user wants.
  static constexpr MeasurementModalityArray kMeasurementModalitiesArray {
    kMeasurementModalities ...};
  /// Time of the acquisition.
  MeasurementBasedTime m_acquisition_time;
  /// Stores values for the current measurement.
  MeasurementVector m_measurement_values{};
  /// Stores variances of the current measurement.
  VarianceVector m_variances{};
};

template<typename T, std::uint32_t... kMeasurementModalities>
constexpr typename Measurement<T, kMeasurementModalities...>::MeasurementModalityArray
Measurement<T, kMeasurementModalities...>::kMeasurementModalitiesArray;

}  // namespace prediction
}  // namespace autoware


#endif  // STATE_ESTIMATION_NODES__MEASUREMENT_HPP_
