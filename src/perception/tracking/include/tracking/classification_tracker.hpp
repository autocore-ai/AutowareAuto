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

#ifndef TRACKING__CLASSIFICATION_TRACKER_HPP_
#define TRACKING__CLASSIFICATION_TRACKER_HPP_

#include <autoware_auto_msgs/msg/detected_object.hpp>
#include <autoware_auto_msgs/msg/object_classification.hpp>
#include <helper_functions/float_comparisons.hpp>
#include <state_estimation/kalman_filter/kalman_filter.hpp>
#include <state_estimation/measurement/linear_measurement.hpp>
#include <state_vector/variable.hpp>
#include <tracking/track_class_variable.hpp>
#include <tracking/visibility_control.hpp>

#include <iostream>
#include <limits>

namespace autoware
{
namespace perception
{
namespace tracking
{

///
/// @brief      A class for tracking classification information that works on any set of variables.
///
/// @details    Under the hood, this class uses the autoware::common::state_estimation::KalmanFilter
///             class with a special state vector containing all the relevant classes that we aim to
///             track. The state vector directly holds the probabilities in its values. If the
///             observation covariance for any particular observation is the same for all observed
///             classes, then the underlying vector is guaranteed to represent probabilities that
///             always sum up to 1.
///
/// @tparam     ClassificationStateT  A state that defines all classes that can be tracked.
///
template<typename ClassificationStateT>
class TRACKING_PUBLIC GenericClassificationTracker
{
  /// A convenience typedef for the underlying Kalman filter.
  using ClassTrackerKalmanFilter =
    decltype(common::state_estimation::make_correction_only_kalman_filter(
      std::declval<ClassificationStateT>(),
      std::declval<typename ClassificationStateT::Matrix>()));

public:
  /// Default constructor.
  GenericClassificationTracker() = default;

  /// Allow specifying a custom observation covariance.
  explicit GenericClassificationTracker(
    const autoware::common::types::float32_t default_observation_covariance,
    const autoware::common::types::float32_t initial_state_covariance)
  : m_default_observation_covariance{default_observation_covariance},
    m_initial_state_covariance{initial_state_covariance} {}

  ///
  /// @brief      Update the class probabilities given a classification update.
  ///
  /// @note       If the probabilities in the classification vector add up to a number less than 1,
  ///             all the "missing" probability mass will be assigned to the UNKNOWN state. If the
  ///             sum is above 1 an exception `std::domain_error` will be thrown instead.
  ///
  /// @param[in]  classification_vector  A vector of classifications with their probabilities.
  ///
  void update(
    const autoware_auto_msgs::msg::DetectedObject::_classification_type & classification_vector)
  {
    update(classification_vector, m_default_observation_covariance);
  }

  ///
  /// @brief      Update the class probabilities given a classification update.
  ///
  /// @note       If the probabilities in the classification vector add up to a number less than 1,
  ///             all the "missing" probability mass will be assigned to the UNKNOWN state. If the
  ///             sum is above 1 an exception `std::domain_error` will be thrown instead.
  ///
  /// @param[in]  classification_vector   A vector of classifications with their probabilities.
  /// @param[in]  observation_covariance  A custom observation covariance.
  ///
  void update(
    const autoware_auto_msgs::msg::DetectedObject::_classification_type & classification_vector,
    const common::types::float32_t observation_covariance)
  {
    auto measurement =
      common::state_estimation::LinearMeasurement<ClassificationStateT>::create_with_stddev(
      ClassificationStateT::Vector::Constant(0.0F),
      ClassificationStateT::Vector::Constant(observation_covariance));
    for (const auto & classification : classification_vector) {
      // We can use the classification as a direct index into the state because within this class we
      // guarantee that the variable that corresponds to a certain index within the
      // ObjectClassification constants is exactly on the same position within the state vector used
      // here. See autoware::perception::tracking::assert_indices_match_classification_constants.
      if (std::isnan(classification.probability)) {
        throw std::domain_error("Provided classification probability is NAN.");
      }
      measurement.state()[classification.classification] = classification.probability;
    }
    const auto sum = measurement.state().vector().sum();
    if (sum > 1.0F) {
      throw std::domain_error("Sum of all probabilities in the classification of an object is > 1");
    } else if (sum < 1.0F) {
      // Any gap in the total probability mass contributes to the likelihood of an unknown state.
      std::cerr << "WARNING: Sum of all classification probabilities is less than one. "
        "Assigning the missing probability to the UNKNOWN class." << std::endl;
      measurement.state()[autoware_auto_msgs::msg::ObjectClassification::UNKNOWN] = 1.0F - sum;
    }
    m_tracker.correct(measurement);
  }

  ///
  /// @brief      Gets the most likely class from the current state vector.
  ///
  /// @return     The most likely classification value.
  ///
  std::uint8_t most_likely_class() const
  {
    std::uint8_t index_of_the_max_value{};
    m_tracker.state().vector().maxCoeff(&index_of_the_max_value);
    return index_of_the_max_value;
  }

  ///
  /// @brief      Gets the object classification vector to be set directly into the DetectedObject
  ///             message.
  ///
  /// @return     The object classification vector.
  ///
  autoware_auto_msgs::msg::DetectedObject::_classification_type object_classification_vector() const
  {
    autoware_auto_msgs::msg::DetectedObject::_classification_type classification_vector;
    for (uint8_t label = 0U; label < ClassificationStateT::size(); ++label) {
      autoware_auto_msgs::msg::ObjectClassification object_classification;
      object_classification.classification = label;
      object_classification.probability = m_tracker.state()[label];
      classification_vector.emplace_back(object_classification);
    }
    return classification_vector;
  }

  /// @brief      Expose the underlying state for utility purposes.
  const ClassificationStateT & state() const noexcept {return m_tracker.state();}

  /// @brief      Expose the observation covariance.
  autoware::common::types::float32_t default_observation_covariance() const noexcept
  {
    return m_default_observation_covariance;
  }

private:
  /// @brief      Create an initial classification vector, with 100% probability for the UNKNOWN
  ///             state.
  static ClassificationStateT create_initial_classification_vector()
  {
    assert_indices_match_classification_constants<ClassificationStateT>();

    ClassificationStateT initial_state{};
    initial_state[autoware_auto_msgs::msg::ObjectClassification::UNKNOWN] = 1.0F;
    return initial_state;
  }

  /// The default observation covariance.
  autoware::common::types::float32_t m_default_observation_covariance{0.1F};
  autoware::common::types::float32_t m_initial_state_covariance{100000.0F};

  /// The underlying Kalman filter.
  ClassTrackerKalmanFilter m_tracker = common::state_estimation::make_correction_only_kalman_filter(
    create_initial_classification_vector(),
    m_initial_state_covariance * ClassificationStateT::Matrix::Identity());
};


using ClassificationTracker = GenericClassificationTracker<ObjectClassificationState>;

}  // namespace tracking
}  // namespace perception
}  // namespace autoware

#endif  // TRACKING__CLASSIFICATION_TRACKER_HPP_
