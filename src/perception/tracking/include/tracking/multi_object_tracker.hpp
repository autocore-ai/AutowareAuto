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

#ifndef TRACKING__MULTI_OBJECT_TRACKER_HPP_
#define TRACKING__MULTI_OBJECT_TRACKER_HPP_

#include <tracking/detected_object_associator.hpp>
#include <tracking/greedy_roi_associator.hpp>
#include <tracking/track_creator.hpp>
#include <tracking/tracked_object.hpp>
#include <tracking/visibility_control.hpp>

#include <autoware_auto_msgs/msg/detected_object.hpp>
#include <autoware_auto_msgs/msg/tracked_object.hpp>
#include <autoware_auto_msgs/msg/tracked_objects.hpp>
#include <common/types.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <motion_model/linear_motion_model.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <state_estimation/kalman_filter/kalman_filter.hpp>
#include <state_estimation/noise_model/wiener_noise.hpp>
#include <state_vector/common_states.hpp>
#include <tf2/buffer_core.h>

#include <chrono>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <vector>


namespace autoware
{
namespace perception
{
namespace tracking
{


/// \brief A return code for the tracker update.
enum class TrackerUpdateStatus
{
  /// Success.
  Ok,
  /// The provided detections were older than the previous detections.
  /// The Kalman filter can only extrapolate forward, so this is an error.
  WentBackInTime,
  /// The frame of the detections does not match the source frame of the transform.
  DetectionFrameMismatch,
  /// The target frame of the transform does not match the frame in which the tracker operates.
  TrackerFrameMismatch,
  /// The provided detections are not in a usable frame – the detection frame must be
  /// gravity-aligned.
  FrameNotGravityAligned,
  /// At least one of the provided detections has an invalid shape.
  InvalidShape,
};


/// \brief Output of MultiObjectTracker::update.
struct TRACKING_PUBLIC DetectedObjectsUpdateResult
{
  /// The existing tracks output.
  autoware_auto_msgs::msg::TrackedObjects tracks;
  /// Any unassigned clusters left after the update.
  autoware_auto_msgs::msg::DetectedObjects unassigned_clusters;
  /// Indicates the success or failure, and kind of failure, of the tracking operation.
  TrackerUpdateStatus status;
  /// Timestamps of ROI msgs used for track creation. Useful for debugging purposes.
  MaybeRoiStampsT maybe_roi_stamps;
};

/// \brief Options for object tracking, with sensible defaults.
struct TRACKING_PUBLIC MultiObjectTrackerOptions
{
  /// Detected object association parameters.
  DataAssociationConfig object_association_config;
  /// Vision ROI association parameters.
  GreedyRoiAssociatorConfig vision_association_config;
  /// Track creator parameters.
  TrackCreatorConfig track_creator_config;
  /// Time after which unseen tracks should be pruned.
  std::chrono::nanoseconds pruning_time_threshold = std::chrono::nanoseconds::max();
  /// Number of updates after which unseen tracks should be pruned.
  std::size_t pruning_ticks_threshold = std::numeric_limits<std::size_t>::max();
  /// The frame in which to do tracking.
  std::string frame = "map";
};

/// \brief A class for multi-object tracking.
class TRACKING_PUBLIC MultiObjectTracker
{
private:
  using DetectedObjectsMsg = autoware_auto_msgs::msg::DetectedObjects;
  using ClassifiedRoiArrayMsg = autoware_auto_msgs::msg::ClassifiedRoiArray;
  using TrackedObjectsMsg = autoware_auto_msgs::msg::TrackedObjects;

public:
  /// Constructor
  explicit MultiObjectTracker(
    MultiObjectTrackerOptions options, const tf2::BufferCore & tf2_buffer);

  /// \brief Update the tracks with the specified detections and return the tracks at the current
  /// timestamp.
  /// \param[in] detections An array of detections.
  /// \param[in] detection_frame_odometry An odometry message for the detection frame in the
  /// tracking frame, which is defined in MultiObjectTrackerOptions.
  /// \return A result object containing tracks, unless an error occurred.
  DetectedObjectsUpdateResult update(
    const DetectedObjectsMsg & detections,
    const nav_msgs::msg::Odometry & detection_frame_odometry);

  /// \brief Update the tracks with the specified detections
  /// \param[in] rois An array of vision detections.
  void update(const ClassifiedRoiArrayMsg & rois);

private:
  /// Check that the input data is valid.
  TrackerUpdateStatus validate(
    const DetectedObjectsMsg & detections,
    const nav_msgs::msg::Odometry & detection_frame_odometry);

  /// Transform the detections into the tracker frame.
  DetectedObjectsMsg transform(
    const DetectedObjectsMsg & detections,
    const nav_msgs::msg::Odometry & detection_frame_odometry);

  /// Convert the internal tracked object representation to the ROS message type.
  TrackedObjectsMsg convert_to_msg(const builtin_interfaces::msg::Time & stamp) const;

  /// The tracked objects, also called "tracks".
  TrackedObjects m_tracks;

  /// Timestamp of the last update.
  std::chrono::system_clock::time_point m_last_update;

  /// Configuration values.
  MultiObjectTrackerOptions m_options;

  /// Associator for matching observations to tracks.
  DetectedObjectAssociator m_object_associator;

  GreedyRoiAssociator m_vision_associator;

  /// Creator for creating tracks based on unassociated observations
  TrackCreator m_track_creator;
};

}  // namespace tracking
}  // namespace perception
}  // namespace autoware

#endif  // TRACKING__MULTI_OBJECT_TRACKER_HPP_
