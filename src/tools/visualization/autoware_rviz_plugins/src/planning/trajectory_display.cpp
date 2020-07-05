// Copyright 2019 Apex.AI, Inc.
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

#include <planning/trajectory_display.hpp>
#include <common/types.hpp>

#include <memory>
#include <string>

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

namespace autoware
{
namespace rviz_plugins
{


TrajectoryDisplay::TrajectoryDisplay()
: rviz_common::RosTopicDisplay<autoware_auto_msgs::msg::Trajectory>(),
  m_marker_common(std::make_unique<MarkerCommon>(this))
{
  color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(0, 255, 0), "Color to draw the arrow.",
    this, SLOT(updateProperty()));

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1.0, "Amount of transparency to apply to the arrow.",
    this, SLOT(updateProperty()));
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);

  scale_property_ = new rviz_common::properties::FloatProperty(
    "Scale", 1, "Scale of the arrow.",
    this, SLOT(updateProperty()));

  text_scale_property_ = new rviz_common::properties::FloatProperty(
    "Text Scale", 1, "Scale of the text showing velocity.",
    this, SLOT(updateProperty()));

  text_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Text Alpha", 1.0, "Amount of transparency to apply to the velocity text.",
    this, SLOT(updateProperty()));
  text_alpha_property_->setMin(0);
  text_alpha_property_->setMax(1);
}

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

void TrajectoryDisplay::update(float32_t wall_dt, float32_t ros_dt)
{
  m_marker_common->update(wall_dt, ros_dt);
}

void TrajectoryDisplay::reset()
{
  RosTopicDisplay::reset();
  m_marker_common->clearMarkers();
}

void TrajectoryDisplay::updateProperty()
{
  if (msg_cache != nullptr) {
    processMessage(msg_cache);
  }
}

void TrajectoryDisplay::processMessage(const Trajectory::ConstSharedPtr msg)
{
  msg_cache = msg;
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
  auto color = color_property_->getColor();
  auto alpha = alpha_property_->getFloat();
  auto scale = scale_property_->getFloat();

  auto marker = std::make_shared<Marker>();
  marker->type = visualization_msgs::msg::Marker::ARROW;
  marker->action = visualization_msgs::msg::Marker::ADD;
  marker->ns = "trajectory_arrow";
  marker->pose.position.x = static_cast<float64_t>(point.x);
  marker->pose.position.y = static_cast<float64_t>(point.y);
  marker->pose.position.z = 0.0;
  marker->pose.orientation.x = 0.0;
  marker->pose.orientation.y = 0.0;
  marker->pose.orientation.z = point.heading.imag;
  marker->pose.orientation.w = point.heading.real;
  marker->scale.x = 1 * scale;
  marker->scale.y = 0.1 * scale;
  marker->scale.z = 0.1 * scale;
  marker->color.a = alpha;
  marker->color.r = color.redF();
  marker->color.g = color.greenF();
  marker->color.b = color.blueF();

  return marker;
}

visualization_msgs::msg::Marker::SharedPtr TrajectoryDisplay::create_velocity_marker(
  const TrajectoryPoint & point) const
{
  auto color = color_property_->getColor();
  auto text_alpha = text_alpha_property_->getFloat();
  auto text_scale = text_scale_property_->getFloat();

  auto marker = std::make_shared<Marker>();
  marker->type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker->action = visualization_msgs::msg::Marker::ADD;
  marker->ns = "trajectory_velocity";
  marker->pose.position.x = static_cast<float64_t>(point.x);
  marker->pose.position.y = static_cast<float64_t>(point.y);
  marker->pose.position.z = 0.1;
  marker->scale.z = 0.2 * text_scale;
  marker->color.a = text_alpha;
  marker->color.r = color.redF();
  marker->color.g = color.greenF();
  marker->color.b = color.blueF();
  marker->text = std::to_string(point.longitudinal_velocity_mps) + "mps";

  return marker;
}

}  // namespace rviz_plugins
}  // namespace autoware

// Export the plugin
#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(autoware::rviz_plugins::TrajectoryDisplay, rviz_common::Display)
