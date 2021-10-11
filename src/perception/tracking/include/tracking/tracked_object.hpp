// Copyright 2021 The Autoware Foundation
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

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the multi_object_tracking class.

#ifndef TRACKING__TRACKED_OBJECT_HPP_
#define TRACKING__TRACKED_OBJECT_HPP_

#include <autoware_auto_msgs/msg/detected_object.hpp>
#include <autoware_auto_msgs/msg/detected_objects.hpp>
#include <autoware_auto_msgs/msg/shape.hpp>
#include <autoware_auto_msgs/msg/tracked_objects.hpp>
#include <common/types.hpp>
#include <motion_model/linear_motion_model.hpp>
#include <state_estimation/kalman_filter/kalman_filter.hpp>
#include <state_estimation/noise_model/wiener_noise.hpp>
#include <state_vector/common_states.hpp>
#include <tracking/classification_tracker.hpp>
#include <tracking/visibility_control.hpp>

#include <chrono>
#include <cstddef>
#include <vector>

namespace autoware
{
namespace perception
{
namespace tracking
{

/// \brief Internal class containing the object state and other information.
class TRACKING_PUBLIC TrackedObject
{
public:
  // Aliases for convenience
  using CA = autoware::common::state_vector::ConstAccelerationXY64;
  using MotionModel = autoware::common::motion_model::LinearMotionModel<CA>;
  using NoiseModel = autoware::common::state_estimation::WienerNoise<CA>;
  using EKF = autoware::common::state_estimation::KalmanFilter<MotionModel, NoiseModel>;
  using TrackedObjectMsg = autoware_auto_msgs::msg::TrackedObject;
  using DetectedObjectMsg = autoware_auto_msgs::msg::DetectedObject;
  using ObjectClassifications = autoware_auto_msgs::msg::TrackedObject::_classification_type;
  using ShapeMsg = autoware_auto_msgs::msg::Shape;

  /// Constructor
  /// \param detection A detection from which to initialize this object. Must have a pose.
  /// \param default_variance All variables will initially have this variance where the detection
  /// does not contain one.
  /// \param noise_variance The sigma for the acceleration noise
  /// \throws std::runtime_error if the detection does not have a pose.
  TrackedObject(
    const DetectedObjectMsg & detection, common::types::float64_t default_variance,
    common::types::float64_t noise_variance);
  /// Extrapolate the track forward.
  // TODO(nikolai.morin): Change signature to use absolute time after #1002
  void predict(std::chrono::nanoseconds dt);

  /// Adjust the track to the detection.
  void update(const DetectedObjectMsg & detection);

  /// Update just the classification state of the track
  void update(
    const ObjectClassifications & obj_type, const common::types::float32_t covariance);

  /// Call when no correspondence for this track was found.
  void no_update();

  /// Returns whether the tracked object can be removed.
  bool should_be_removed(
    const std::chrono::nanoseconds time_threshold,
    const std::size_t ticks_threshold) const;

  /// Getter for the message.
  const TrackedObjectMsg & msg();

  /// Getter for position covariance array
  inline Eigen::Matrix2d position_covariance() const
  {
    return m_ekf.covariance().topLeftCorner<2, 2>();
  }

  /// Getter for centroid position
  inline Eigen::Vector2d centroid() const
  {
    return Eigen::Vector2d {m_ekf.state().at<autoware::common::state_vector::variable::X>(),
      m_ekf.state().at<autoware::common::state_vector::variable::Y>()};
  }

  /// Getter for shape
  inline const ShapeMsg & shape() const
  {
    return m_msg.shape[0];
  }

private:
  /// The final to-be-published object.
  TrackedObjectMsg m_msg;
  /// The state estimator.
  EKF m_ekf;
  /// The time since this object has last been seen.
  std::chrono::nanoseconds m_time_since_last_seen = std::chrono::nanoseconds::zero();
  /// The number of updates where this object has not been seen
  std::size_t m_ticks_since_last_seen = 0;
  /// The number of updates (seen or not) since this object has been created
  std::size_t m_ticks_alive = 1;
  /// All variables will initially have this variance where the detection
  /// does not contain one.
  common::types::float64_t m_default_variance = -1.0;
  /// Track class classifier.
  ClassificationTracker m_classifier;
};


/// A collection of tracked objects.
struct TRACKING_PUBLIC TrackedObjects
{
  /// Tracked objects.
  std::vector<TrackedObject> objects;
  /// Frame in which the objects exist.
  std_msgs::msg::Header::_frame_id_type frame_id;
};

}  // namespace tracking
}  // namespace perception
}  // namespace autoware

#endif  // TRACKING__TRACKED_OBJECT_HPP_
