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

#include "tracking/multi_object_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

#include "autoware_auto_tf2/tf2_autoware_auto_msgs.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_eigen/tf2_eigen.h"
#include "time_utils/time_utils.hpp"

namespace autoware
{
namespace perception
{
namespace tracking
{

namespace
{
bool is_gravity_aligned(const geometry_msgs::msg::Quaternion & quat)
{
  // Check that the transformation is still roughly 2D, i.e. does not have substantial pitch and
  // roll. That means that either the rotation angle is small, or the rotation axis is
  // approximately equal to the z axis.
  constexpr double kAngleThresh = 0.1;  // rad
  constexpr double kAxisTiltThresh = 0.1;  // rad
  // rotation angle small
  // ⇔ |θ| <= kAngleThresh  (angles are assumed to be between -π and π)
  // ⇔ cos(θ/2) => std::cos(kAngleThresh/2)
  // ⇔ w => std::cos(kAngleThresh/2)
  if (quat.w < std::cos(0.5 * kAngleThresh)) {
    // From Wikipedia: (x, y, z) = cos(θ/2) * (u_x, u_y, u_z), where u is the rotation axis.
    // The cosine of the angle α between the rotation axis and the z axis is the dot product of the
    // rotation axis u and the the z axis, so cos(α) = u_z.
    const double u_z = quat.z / std::sqrt(quat.x * quat.x + quat.y * quat.y + quat.z * quat.z);
    if (u_z < std::cos(kAxisTiltThresh)) {
      return false;
    }
  }
  return true;
}

geometry_msgs::msg::TransformStamped to_transform(const nav_msgs::msg::Odometry & odometry)
{
  geometry_msgs::msg::TransformStamped tfs;
  tfs.header = odometry.header;
  tfs.child_frame_id = odometry.child_frame_id;
  tfs.transform.translation.x = odometry.pose.pose.position.x;
  tfs.transform.translation.y = odometry.pose.pose.position.y;
  tfs.transform.translation.z = odometry.pose.pose.position.z;
  tfs.transform.rotation = odometry.pose.pose.orientation;
  return tfs;
}
}  // anonymous namespace


MultiObjectTracker::MultiObjectTracker(MultiObjectTrackerOptions options)
: m_options(options), m_associator(options.association_config) {}

TrackerUpdateResult MultiObjectTracker::update(
  DetectedObjectsMsg detections,
  const nav_msgs::msg::Odometry & detection_frame_odometry)
{
  TrackerUpdateResult result;
  result.status = this->validate(detections, detection_frame_odometry);
  if (result.status != TrackerUpdateStatus::Ok) {
    return result;
  }

  // ==================================
  // Transform detections
  // ==================================
  this->transform(detections, detection_frame_odometry);

  // ==================================
  // Predict tracks forward
  // ==================================
  // TODO(nikolai.morin): Simplify after #1002
  const auto target_time = time_utils::from_message(detections.header.stamp);
  const auto dt = target_time - m_last_update;
  for (auto & object : m_objects) {
    object.predict(dt);
  }

  // ==================================
  // Associate observations with tracks
  // ==================================
  TrackedObjectsMsg tracked_objects_msg = this->convert_to_msg();
  AssociatorResult association;
  try {
    association = m_associator.assign(detections, tracked_objects_msg);
  } catch (const std::runtime_error & e) {
    result.status = TrackerUpdateStatus::InvalidShape;
    return result;
  }

  // ==================================
  // Update tracks with observations
  // ==================================
  for (size_t track_idx = 0; track_idx < m_objects.size(); ++track_idx) {
    size_t detection_idx = association.track_assignments[track_idx];
    if (detection_idx == AssociatorResult::UNASSIGNED) {
      continue;
    }
    const auto & detection = detections.objects[detection_idx];
    m_objects[track_idx].update(detection);
  }
  for (const size_t track_idx : association.unassigned_track_indices) {
    m_objects[track_idx].no_update();
  }

  // ==================================
  // Initialize new tracks
  // ==================================
  for (size_t new_detection_idx : association.unassigned_detection_indices) {
    const auto & detection = detections.objects[new_detection_idx];
    if (!detection.kinematics.has_pose) {
      continue;
    }
    m_objects.push_back(
      TrackedObject(
        detection,
        m_options.default_variance, m_options.noise_variance));
  }

  // ==================================
  // Prune tracks
  // ==================================
  const auto last = std::remove_if(
    m_objects.begin(), m_objects.end(), [this](const auto & object) {
      return object.should_be_removed(
        this->m_options.pruning_time_threshold,
        this->m_options.pruning_ticks_threshold);
    });
  m_objects.erase(last, m_objects.end());

  // ==================================
  // Build result
  // ==================================
  result.objects = std::make_unique<TrackedObjectsMsg>(this->convert_to_msg());
  result.status = TrackerUpdateStatus::Ok;
  m_last_update = target_time;
  return result;
}


TrackerUpdateStatus MultiObjectTracker::validate(
  const DetectedObjectsMsg & detections,
  const nav_msgs::msg::Odometry & detection_frame_odometry)
{
  const auto target_time = time_utils::from_message(detections.header.stamp);
  if (target_time < m_last_update) {
    return TrackerUpdateStatus::WentBackInTime;
  }
  if (detections.header.frame_id != detection_frame_odometry.child_frame_id) {
    return TrackerUpdateStatus::DetectionFrameMismatch;
  }
  if (detection_frame_odometry.header.frame_id != m_options.frame) {
    return TrackerUpdateStatus::TrackerFrameMismatch;
  }
  if (!is_gravity_aligned(detection_frame_odometry.pose.pose.orientation)) {
    return TrackerUpdateStatus::FrameNotGravityAligned;
  }
  for (const auto & detection : detections.objects) {
    if (!detection.kinematics.has_pose && !detection.kinematics.has_twist) {
      return TrackerUpdateStatus::EmptyDetection;
    }
  }
  // Could also validate
  // * classes
  // * object shapes
  // * detection poses are gravity aligned
  return TrackerUpdateStatus::Ok;
}

void MultiObjectTracker::transform(
  DetectedObjectsMsg & detections,
  const nav_msgs::msg::Odometry & detection_frame_odometry)
{
  // Convert the odometry to Eigen objects.
  Eigen::Isometry3d tf__tracking__detection = Eigen::Isometry3d::Identity();
  tf2::fromMsg(detection_frame_odometry.pose.pose, tf__tracking__detection);
  const Eigen::Matrix3d rot_d = tf__tracking__detection.linear();
  // Convert the odometry to TransformStamped for use with tf2::doTransform.
  const geometry_msgs::msg::TransformStamped tf_msg__tracking__detection = to_transform(
    detection_frame_odometry);
  // Hoisted outside the loop
  Eigen::Isometry3d tf__detection__object = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf__tracking__object = Eigen::Isometry3d::Identity();

  detections.header.frame_id = m_options.frame;
  for (auto & detection : detections.objects) {
    // Transform the shape. If needed, this can potentially be made more efficient by not using
    // tf2::doTransform, which converts the TransformStamped message to a different representation
    // in each call.
    tf2::doTransform(detection.shape.polygon, detection.shape.polygon, tf_msg__tracking__detection);
    // Transform the pose.
    if (detection.kinematics.has_pose) {
      tf2::fromMsg(detection.kinematics.pose.pose, tf__detection__object);
      tf__tracking__object = tf__tracking__detection * tf__detection__object;
      detection.kinematics.pose.pose = tf2::toMsg(tf__tracking__object);
      if (detection.kinematics.has_pose_covariance) {
        // Doing this properly is difficult. We'll ignore the rotational part. This is a practical
        // solution since only the yaw covariance is relevant, and the yaw covariance is
        // unaffected by the transformation, which preserves the z axis.
        // An even more accurate implementation could additionally include the odometry covariance.
        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> cov(
          detection.kinematics.pose.covariance.data());
        cov.topLeftCorner<3, 3>() = rot_d * cov.topLeftCorner<3, 3>() * rot_d.transpose();
      }
    }
    // Transform the twist.
    if (detection.kinematics.has_twist) {
      auto & linear = detection.kinematics.twist.twist.linear;
      const auto & frame_linear = detection_frame_odometry.twist.twist.linear;
      const Eigen::Vector3d eigen_linear{linear.x, linear.y, linear.z};
      const Eigen::Vector3d eigen_linear_transformed = rot_d * eigen_linear;
      // This assumes the detection frame has no angular velocity wrt the tracking frame.
      // TODO(nikolai.morin): Implement the full formula, to be found in
      // Craig's "Introduction to robotics" book, third edition, formula 5.13
      linear.x = frame_linear.x + eigen_linear_transformed.x();
      linear.y = frame_linear.y + eigen_linear_transformed.y();
      linear.z = frame_linear.z + eigen_linear_transformed.z();
    }
  }
}

MultiObjectTracker::TrackedObjectsMsg MultiObjectTracker::convert_to_msg() const
{
  TrackedObjectsMsg array;
  array.objects.reserve(m_objects.size());
  std::transform(
    m_objects.begin(), m_objects.end(), std::back_inserter(array.objects), [](
      TrackedObject o) {return o.msg();});
  return array;
}


}  // namespace tracking
}  // namespace perception
}  // namespace autoware
