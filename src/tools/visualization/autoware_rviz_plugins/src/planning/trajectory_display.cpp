// Copyright 2019 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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

#include <planning/trajectory_display.hpp>

#include <memory>
#include <string>

namespace autoware
{
namespace rviz_plugins
{


TrajectoryDisplay::TrajectoryDisplay()
: rviz_common::RosTopicDisplay<autoware_auto_msgs::msg::Trajectory>(),
  m_marker_common(std::make_unique<MarkerCommon>(this))
{}

void TrajectoryDisplay::onInitialize()
{
  RTDClass::onInitialize();
  m_marker_common->initialize(context_, scene_node_);

  topic_property_->setValue("trajectory");
  topic_property_->setDescription("Trajectory topic to subscribe to.");
}

void TrajectoryDisplay::load(const rviz_common::Config & config)
{
  Display::load(config);
  m_marker_common->load(config);
}

void TrajectoryDisplay::update(float wall_dt, float ros_dt)
{
  m_marker_common->update(wall_dt, ros_dt);
}

void TrajectoryDisplay::reset()
{
  RosTopicDisplay::reset();
  m_marker_common->clearMarkers();
}

void TrajectoryDisplay::processMessage(const Trajectory::ConstSharedPtr msg)
{
  m_marker_common->clearMarkers();
  auto count = 0;
  const auto update = [&count, this, header = msg->header](auto marker) -> void {
      marker->header = header;
      marker->id = ++count;
      m_marker_common->addMessage(marker);
    };
  for (std::size_t idx = 0U; idx < msg->points.size(); ++idx) {
    const auto & point = msg->points[idx];
    {
      const auto traj_marker = create_pose_marker(point);
      update(traj_marker);
    }
    {
      const auto vel_marker = create_velocity_marker(point);
      update(vel_marker);
    }
  }
}

visualization_msgs::msg::Marker::SharedPtr TrajectoryDisplay::create_pose_marker(
  const TrajectoryPoint & point) const
{
  auto marker = std::make_shared<Marker>();
  marker->type = visualization_msgs::msg::Marker::ARROW;
  marker->action = visualization_msgs::msg::Marker::ADD;
  marker->ns = "trajectory_arrow";
  marker->pose.position.x = static_cast<double>(point.x);
  marker->pose.position.y = static_cast<double>(point.y);
  marker->pose.position.z = 0.0;
  marker->pose.orientation.x = 0.0;
  marker->pose.orientation.y = 0.0;
  marker->pose.orientation.z = point.heading.imag;
  marker->pose.orientation.w = point.heading.real;
  marker->scale.x = 1;
  marker->scale.y = 0.1;
  marker->scale.z = 0.1;
  marker->color.a = 1.0;  // Don't forget to set the alpha!
  marker->color.r = 0.0;
  marker->color.g = 1.0;
  marker->color.b = 0.0;

  return marker;
}

visualization_msgs::msg::Marker::SharedPtr TrajectoryDisplay::create_velocity_marker(
  const TrajectoryPoint & point) const
{
  auto marker = std::make_shared<Marker>();
  marker->type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker->action = visualization_msgs::msg::Marker::ADD;
  marker->ns = "trajectory_velocity";
  marker->pose.position.x = static_cast<double>(point.x);
  marker->pose.position.y = static_cast<double>(point.y);
  marker->pose.position.z = 0.1;
  marker->scale.z = 0.2;
  marker->color.a = 0.85F;  // Don't forget to set the alpha!
  marker->color.r = 0.0F;
  marker->color.g = 1.0F;
  marker->color.b = 0.0F;
  marker->text = std::to_string(point.longitudinal_velocity_mps) + "mps";

  return marker;
}

}  // namespace rviz_plugins
}  // namespace autoware

// Export the plugin
#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(autoware::rviz_plugins::TrajectoryDisplay, rviz_common::Display)
