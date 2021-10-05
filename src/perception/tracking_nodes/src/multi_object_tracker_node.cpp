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

#include <tracking_nodes/multi_object_tracker_node.hpp>

#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <time_utils/time_utils.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>


namespace autoware
{
namespace tracking_nodes
{

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::perception::tracking::MultiObjectTracker;
using autoware::perception::tracking::MultiObjectTrackerOptions;
using autoware::perception::tracking::TrackCreatorConfig;
using autoware::perception::tracking::DetectedObjectsUpdateResult;
using autoware::perception::tracking::TrackerUpdateStatus;
using autoware::perception::tracking::GreedyRoiAssociatorConfig;
using autoware::perception::tracking::CameraIntrinsics;
using autoware::perception::tracking::VisionPolicyConfig;
using autoware_auto_msgs::msg::DetectedObjects;
using autoware_auto_msgs::msg::TrackedObjects;
using nav_msgs::msg::Odometry;
using std::placeholders::_1;
using std::placeholders::_2;

namespace
{
constexpr std::chrono::milliseconds kMaxLidarEgoStateStampDiff{100};
constexpr std::chrono::milliseconds kMaxVisionEgoStateStampDiff{100};
constexpr std::int64_t kDefaultHistoryDepth{20};
constexpr std::int64_t kDefaultPoseHistoryDepth{100};

MultiObjectTracker init_tracker(
  rclcpp::Node & node,
  const bool8_t use_vision,
  tf2::BufferCore & tf_buffer)
{
  const float32_t max_distance =
    static_cast<float32_t>(node.declare_parameter(
      "object_association.max_distance").get<float64_t>());
  const float32_t max_area_ratio =
    static_cast<float32_t>(node.declare_parameter(
      "object_association.max_area_ratio").get<float64_t>());
  const bool consider_edge_for_big_detections = node.declare_parameter(
    "object_association.consider_edge_for_big_detection").get<bool>();

  auto creation_policy = perception::tracking::TrackCreationPolicy::LidarClusterOnly;
  const auto default_variance = node.declare_parameter(
    "ekf_default_variance").get<float64_t>();
  const auto noise_variance = node.declare_parameter(
    "ekf_noise_variance").get<float64_t>();
  const std::chrono::nanoseconds pruning_time_threshold =
    std::chrono::milliseconds(
    node.declare_parameter(
      "pruning_time_threshold_ms").get<int64_t>());
  const std::size_t pruning_ticks_threshold =
    static_cast<std::size_t>(node.declare_parameter(
      "pruning_ticks_threshold").get<int64_t>());
  const std::string frame = node.declare_parameter("track_frame_id", "odom");

  TrackCreatorConfig creator_config{};
  GreedyRoiAssociatorConfig vision_config{};

  if (use_vision) {
    // There is no reason to have vision and use LidarClusterOnly policy. So, update the policy
    creation_policy = perception::tracking::TrackCreationPolicy::LidarClusterIfVision;
    vision_config.intrinsics = {
      static_cast<std::size_t>(node.declare_parameter(
        "vision_association.intrinsics.width").get<int64_t>()),
      static_cast<std::size_t>(node.declare_parameter(
        "vision_association.intrinsics.height").get<int64_t>()),
      static_cast<float32_t>(node.declare_parameter(
        "vision_association.intrinsics.fx").get<float32_t>()),
      static_cast<float32_t>(node.declare_parameter(
        "vision_association.intrinsics.fy").get<float32_t>()),
      static_cast<float32_t>(node.declare_parameter(
        "vision_association.intrinsics.ox").get<float32_t>()),
      static_cast<float32_t>(node.declare_parameter(
        "vision_association.intrinsics.oy").get<float32_t>()),
      static_cast<float32_t>(node.declare_parameter(
        "vision_association.intrinsics.skew").get<float32_t>())
    };


    vision_config.iou_threshold = static_cast<float32_t>(node.declare_parameter(
        "vision_association.iou_threshold").get<float32_t>());

    VisionPolicyConfig vision_policy_cfg;
    vision_policy_cfg.associator_cfg = vision_config;
    vision_policy_cfg.max_vision_lidar_timestamp_diff = std::chrono::milliseconds(
      node.declare_parameter(
        "vision_association.timestamp_diff_ms").get<int64_t>());
    creator_config.vision_policy_config.emplace(vision_policy_cfg);
  }

  creator_config.policy = creation_policy;
  creator_config.default_variance = default_variance;
  creator_config.noise_variance = noise_variance;

  MultiObjectTrackerOptions options{
    {max_distance, max_area_ratio, consider_edge_for_big_detections}, vision_config,
    creator_config, pruning_time_threshold, pruning_ticks_threshold, frame};
  return MultiObjectTracker{options, tf_buffer};
}

std::string status_to_string(TrackerUpdateStatus status)
{
  // Use a switch statement without default since it warns when not all cases are handled.
  switch (status) {
    case TrackerUpdateStatus::Ok: return "Ok";
    case TrackerUpdateStatus::WentBackInTime: return "WentBackInTime";
    case TrackerUpdateStatus::DetectionFrameMismatch: return "DetectionFrameMismatch";
    case TrackerUpdateStatus::TrackerFrameMismatch: return "TrackerFrameMismatch";
    case TrackerUpdateStatus::FrameNotGravityAligned: return "FrameNotGravityAligned";
    case TrackerUpdateStatus::InvalidShape: return "InvalidShape";
  }
  return "Invalid status";
}

// Convert pose msg to odom msg
Odometry::SharedPtr to_odom(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & pose_msg)
{
  nav_msgs::msg::Odometry::SharedPtr odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
  odom_msg->header = pose_msg->header;
  odom_msg->child_frame_id = "base_link";
  odom_msg->pose = pose_msg->pose;
  return odom_msg;
}

// Get msg closest to the given timestamp from the list of messages
template<typename T>
T get_closest_match(const std::vector<T> & matched_msgs, const rclcpp::Time & stamp)
{
  return *std::min_element(
    matched_msgs.begin(), matched_msgs.end(), [&stamp](const auto &
    a, const auto & b) {
      const rclcpp::Time t1(a->header.stamp);
      const rclcpp::Time t2(b->header.stamp);
      return std::abs((t1 - stamp).nanoseconds()) < std::abs((t2 - stamp).nanoseconds());
    });
}

}  // namespace

MultiObjectTrackerNode::MultiObjectTrackerNode(const rclcpp::NodeOptions & options)
:  Node("multi_object_tracker_node", options),
  m_use_ndt{this->declare_parameter("use_ndt", true)},
  m_use_vision{this->declare_parameter("use_vision", true)},
  m_history_depth{static_cast<std::size_t>(
      declare_parameter("history_depth", kDefaultHistoryDepth))},
  m_tf_listener{m_tf_buffer},
  m_tracker{init_tracker(*this, m_use_vision, m_tf_buffer)},
  m_track_publisher{create_publisher<TrackedObjects>("tracked_objects", m_history_depth)},
  m_leftover_publisher{create_publisher<DetectedObjects>("leftover_clusters", m_history_depth)},
  m_visualize_track_creation{this->declare_parameter("visualize_track_creation", false)}
{
  const auto pose_history_depth =
    static_cast<size_t>(declare_parameter("pose_history_depth", kDefaultPoseHistoryDepth));
  m_odom_cache = std::make_unique<OdomCache>();
  m_odom_cache->setCacheSize(static_cast<std::uint32_t>(pose_history_depth));
  if (m_use_ndt) {
    m_odom_subscription = create_subscription<OdometryMsg>(
      "ego_state", rclcpp::QoS{pose_history_depth},
      std::bind(&MultiObjectTrackerNode::odometry_callback, this, std::placeholders::_1));
  } else {
    m_pose_subscription = create_subscription<PoseMsg>(
      "ego_state", rclcpp::QoS{pose_history_depth},
      std::bind(&MultiObjectTrackerNode::pose_callback, this, std::placeholders::_1));
  }

  m_detected_objects_subscription = create_subscription<DetectedObjects>(
    "detected_objects", rclcpp::QoS{m_history_depth},
    std::bind(&MultiObjectTrackerNode::detected_objects_callback, this, std::placeholders::_1));

  // Initialize vision callbacks if vision is configured to be used:
  if (m_use_vision) {
    auto num_vision_topics = static_cast<size_t>(this->declare_parameter("num_vision_topics", 1));

    m_vision_subscriptions.resize(num_vision_topics);

    for (size_t i = 0U; i < m_vision_subscriptions.size(); ++i) {
      const auto topic_name = "classified_rois" + std::to_string(i + 1);
      m_vision_subscriptions[i] = create_subscription<ClassifiedRoiArray>(
        topic_name, rclcpp::QoS{m_history_depth},
        std::bind(
          &MultiObjectTrackerNode::classified_roi_callback, this,
          std::placeholders::_1));
    }
  }

  if (m_visualize_track_creation) {
    if (!m_use_vision) {
      throw std::runtime_error(
              "Visualization can only be enabled if the vision detections are enabled.");
    }
    m_track_creating_clusters_pub =
      create_publisher<DetectedObjects>("associated_detections", m_history_depth);
  }
}

void MultiObjectTrackerNode::odometry_callback(const OdometryMsg::ConstSharedPtr odom)
{
  m_odom_cache->add(odom);
}

void MultiObjectTrackerNode::pose_callback(const PoseMsg::ConstSharedPtr pose)
{
  m_odom_cache->add(to_odom(pose));
}

void MultiObjectTrackerNode::detected_objects_callback(const DetectedObjects::ConstSharedPtr objs)
{
  const rclcpp::Time msg_stamp{objs->header.stamp.sec, objs->header.stamp.nanosec};
  const auto earliest_time = msg_stamp - kMaxLidarEgoStateStampDiff;
  const auto latest_time = msg_stamp + kMaxLidarEgoStateStampDiff;
  const auto matched_msgs = m_odom_cache->getInterval(earliest_time, latest_time);
  if (matched_msgs.empty()) {
    RCLCPP_WARN(get_logger(), "No matching odom msg received for obj msg");
    return;
  }
  const auto result = m_tracker.update(
    *objs, *get_closest_match(matched_msgs, objs->header.stamp));
  if (result.status == TrackerUpdateStatus::Ok && result.maybe_roi_stamps) {
    m_track_publisher->publish(result.tracks);
    m_leftover_publisher->publish(result.unassigned_clusters);
    maybe_visualize(*(result.maybe_roi_stamps), *objs);
  } else {
    RCLCPP_WARN(
      get_logger(), "Tracker update for vision detection at time %d.%d failed. Reason: %s",
      objs->header.stamp.sec, objs->header.stamp.nanosec,
      status_to_string(result.status).c_str());
  }
}

void MultiObjectTrackerNode::classified_roi_callback(const ClassifiedRoiArray::ConstSharedPtr rois)
{
  m_tracker.update(*rois);
}

void MultiObjectTrackerNode::maybe_visualize(
  const perception::tracking::MaybeRoiStampsT::value_type & roi_stamps,
  DetectedObjects all_objects)
{
  if (!m_visualize_track_creation) {
    return;
  }
  // Align the detections on time with the rois they are associated to.
  for (const auto & stamp : roi_stamps) {
    all_objects.header.stamp = stamp;
    m_track_creating_clusters_pub->publish(all_objects);
  }
}
}  // namespace tracking_nodes
}  // namespace autoware

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::tracking_nodes::MultiObjectTrackerNode)
