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

#ifndef SPINNAKER_CAMERA_NODE__SPINNAKER_CAMERA_NODE_HPP_
#define SPINNAKER_CAMERA_NODE__SPINNAKER_CAMERA_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <spinnaker_camera_driver/system_wrapper.hpp>
#include <spinnaker_camera_node/visibility_control.hpp>

#include <memory>
#include <string>
#include <vector>
#include <mutex>

namespace autoware
{
namespace drivers
{
namespace camera
{

class SPINNAKER_CAMERA_NODE_PUBLIC SpinnakerCameraNode : public ::rclcpp::Node
{
public:
  /// ROS 2 parameter contructor.
  /// \param[in] node_name The name of the node.
  /// \param[in] node_namespace Namespace of the node.
  /// \param[in] node_options Node options for this node.
  explicit SpinnakerCameraNode(
    const std::string & node_name,
    const std::string & node_namespace = "",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions{});

private:
  /// A wrapper around a publisher that handles proper multithreading protection.
  class SPINNAKER_CAMERA_NODE_LOCAL ProtectedPublisher;

  /// This funciton is called to publish an image.
  SPINNAKER_CAMERA_NODE_LOCAL void publish_image(
    std::uint32_t camera_index,
    std::unique_ptr<sensor_msgs::msg::Image> image);

  /// Helper function to parse camera-related params and create cameras from them.
  SPINNAKER_CAMERA_NODE_LOCAL spinnaker::CameraListWrapper & create_cameras_from_params(
    spinnaker::SystemWrapper * spinnaker_wrapper);

  /// Helper function to create publishers.
  SPINNAKER_CAMERA_NODE_LOCAL static std::vector<ProtectedPublisher> create_publishers(
    ::rclcpp::Node * node,
    size_t number_of_cameras,
    bool use_publisher_per_camera);

  std::unique_ptr<spinnaker::SystemWrapper> m_spinnaker_wrapper{};
  std::vector<ProtectedPublisher> m_publishers{};
  bool m_use_publisher_per_camera{};
};

class SpinnakerCameraNode::ProtectedPublisher
{
  using PublisherT = ::rclcpp::Publisher<::sensor_msgs::msg::Image>;

public:
  /// Co-share ownership of a rclcpp publisher.
  void set_publisher(PublisherT::SharedPtr publisher);
  /// Publish an image.
  void publish(std::unique_ptr<sensor_msgs::msg::Image> image);

private:
  std::mutex m_publish_mutex{};
  PublisherT::SharedPtr m_publisher{};
};

}  // namespace camera
}  // namespace drivers
}  // namespace autoware

#endif  // SPINNAKER_CAMERA_NODE__SPINNAKER_CAMERA_NODE_HPP_
