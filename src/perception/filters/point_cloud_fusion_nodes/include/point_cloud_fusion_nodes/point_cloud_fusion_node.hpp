// Copyright 2019-2021 the Autoware Foundation
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

#ifndef POINT_CLOUD_FUSION_NODES__POINT_CLOUD_FUSION_NODE_HPP_
#define POINT_CLOUD_FUSION_NODES__POINT_CLOUD_FUSION_NODE_HPP_

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <lidar_utils/point_cloud_utils.hpp>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>
#include <point_cloud_fusion/point_cloud_fusion.hpp>
#include <point_cloud_fusion_nodes/visibility_control.hpp>
#include <common/types.hpp>
#include <string>
#include <memory>
#include <vector>

using autoware::common::types::bool8_t;

namespace autoware
{
namespace perception
{
namespace filters
{
namespace point_cloud_fusion_nodes
{
/// \brief Class that fuses multiple point clouds from different sources into one by concatanating
/// them.
class POINT_CLOUD_FUSION_NODES_PUBLIC PointCloudFusionNode : public rclcpp::Node
{
public:
  /// \brief constructor
  /// \param[in] node_options An rclcpp::NodeOptions object
  explicit PointCloudFusionNode(
    const rclcpp::NodeOptions & node_options);

private:
  using PointT = common::types::PointXYZIF;
  using PointCloudMsgT = sensor_msgs::msg::PointCloud2;
  using PointCloudT = sensor_msgs::msg::PointCloud2;
  using SyncPolicyT = message_filters::sync_policies::ApproximateTime<PointCloudMsgT,
      PointCloudMsgT, PointCloudMsgT, PointCloudMsgT, PointCloudMsgT, PointCloudMsgT,
      PointCloudMsgT, PointCloudMsgT>;

  void init();

  std::chrono::nanoseconds convert_msg_time(builtin_interfaces::msg::Time stamp);

  void pointcloud_callback(
    const PointCloudMsgT::ConstSharedPtr & msg1, const PointCloudMsgT::ConstSharedPtr & msg2,
    const PointCloudMsgT::ConstSharedPtr & msg3, const PointCloudMsgT::ConstSharedPtr & msg4,
    const PointCloudMsgT::ConstSharedPtr & msg5, const PointCloudMsgT::ConstSharedPtr & msg6,
    const PointCloudMsgT::ConstSharedPtr & msg7, const PointCloudMsgT::ConstSharedPtr & msg8);

  std::unique_ptr<point_cloud_fusion::PointCloudFusion> m_core;
  PointCloudT m_cloud_concatenated;
  std::unique_ptr<message_filters::Subscriber<PointCloudMsgT>> m_cloud_subscribers[8];
  std::unique_ptr<message_filters::Synchronizer<SyncPolicyT>> m_cloud_synchronizer;
  rclcpp::Publisher<PointCloudMsgT>::SharedPtr m_cloud_publisher;

  std::vector<std::string> m_input_topics;
  std::string m_output_frame_id;
  uint32_t m_cloud_capacity;
};
}  // namespace point_cloud_fusion_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware


#endif  // POINT_CLOUD_FUSION_NODES__POINT_CLOUD_FUSION_NODE_HPP_
