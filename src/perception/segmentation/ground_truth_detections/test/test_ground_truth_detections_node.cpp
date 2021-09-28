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

#include <ground_truth_detections/ground_truth_detections_node.hpp>

#include <geometry_msgs/msg/vector3.hpp>

#include <fake_test_node/fake_test_node.hpp>

#include <math.h>
#include <memory>

#include "gtest/gtest.h"

namespace
{

using autoware::ground_truth_detections::GroundTruthDetectionsNode;
using autoware_auto_msgs::msg::ClassifiedRoiArray;
using autoware_auto_msgs::msg::DetectedObjects;
using geometry_msgs::msg::Vector3;
using lgsvl_msgs::msg::Detection2DArray;
using lgsvl_msgs::msg::Detection2D;
using lgsvl_msgs::msg::Detection3DArray;
using lgsvl_msgs::msg::Detection3D;

using GroundTruth2dDetectionsTest = autoware::tools::testing::FakeTestNode;
using GroundTruth3dDetectionsTest = autoware::tools::testing::FakeTestNode;

using namespace std::chrono_literals;

static constexpr float_t CAR_CENTER_X = 15.3F;
static constexpr float_t CAR_CENTER_Y = 17.4F;
static constexpr float_t CAR_BBOX_HEIGHT = 5.2F;
static constexpr float_t CAR_BBOX_WIDTH = 2.7F;

static constexpr double_t CAR_CENTER_3D_X = CAR_CENTER_X;
static constexpr double_t CAR_CENTER_3D_Y = CAR_CENTER_Y;
static constexpr double_t CAR_CENTER_3D_Z = 9.9F;

// unit quaternion representing a rotation of 61.1° around an axis (-0.14, 0.16, 0.98)
static constexpr double_t CAR_ORIENTATION_X = -0.0696078;
static constexpr double_t CAR_ORIENTATION_Y = 0.0795518;
static constexpr double_t CAR_ORIENTATION_Z = 0.497199;
static constexpr double_t CAR_ORIENTATION_W = 0.861173;

static constexpr float_t CAR_BBOX_3D_HEIGHT = 2.1F;
static constexpr float_t CAR_BBOX_3D_LENGTH = 4.8F;
static constexpr float_t CAR_BBOX_3D_WIDTH = 2.15F;

static constexpr double_t CAR_TWIST_LINEAR_X = 2.0F;
static constexpr double_t CAR_TWIST_LINEAR_Y = 2.3F;
static constexpr double_t CAR_TWIST_LINEAR_Z = 2.5F;

// One detection for each supported label + one detection with unsupported label
Detection2DArray make_sample_detections_2d()
{
  Detection2DArray detections;
  detections.header.stamp.sec = 24;
  detections.header.stamp.nanosec = 8723U;

  const auto add_detection = [&detections](const char * label) -> Detection2D & {
      detections.detections.emplace_back(detections.detections.back());
      auto & d = detections.detections.back();
      d.label = label;
      ++d.id;

      return d;
    };
  {
    Detection2D d;

    d.label = "Hatchback";
    d.header = detections.header;
    d.bbox.x = CAR_CENTER_X;
    d.bbox.y = CAR_CENTER_Y;
    d.bbox.height = CAR_BBOX_HEIGHT;
    d.bbox.width = CAR_BBOX_WIDTH;
    d.id = 0;
    d.score = 1.0F;
    d.velocity.linear = geometry_msgs::build<Vector3>()
      .x(CAR_TWIST_LINEAR_X)
      .y(CAR_TWIST_LINEAR_Y)
      .z(CAR_TWIST_LINEAR_Z);

    detections.detections.emplace_back(d);
  }

  add_detection("Jeep");
  add_detection("Sedan");
  add_detection("SUV");
  add_detection("BoxTruck");

  // add a position that would give rise to a lower-left corner just outside the allowed
  // range if half of the width (or height) were subtracted
  {
    Detection2D & d = add_detection("Pedestrian");
    d.bbox.x = 5.0F;
    d.bbox.width = 10.00003F;

    d.bbox.y = 6.0F;
    d.bbox.height = 12.00002F;
  }

  add_detection("Unsupported_label");

  return detections;
}

// cppcheck-suppress syntaxError
TEST_F(GroundTruth2dDetectionsTest, ReceiveDetections)
{
  rclcpp::NodeOptions options{};
  const auto node = std::make_shared<GroundTruthDetectionsNode>(options);

  ClassifiedRoiArray::SharedPtr last_received_msg{};
  const auto input_topic = "/simulator/ground_truth/detections2D";
  const auto output_topic = "/perception/ground_truth_detections_2d";

  auto fake_publisher = create_publisher<Detection2DArray>(input_topic, 1s);

  auto result_subscription = create_subscription<ClassifiedRoiArray>(
    output_topic, *node,                                                                  //
    [&last_received_msg](
      const ClassifiedRoiArray::SharedPtr msg) {
      last_received_msg = msg;
    });

  Detection2DArray input_msg = make_sample_detections_2d();

  const auto dt{100ms};
  const auto max_wait_time{std::chrono::seconds{1LL}};
  auto time_passed{std::chrono::milliseconds{0LL}};
  while (!last_received_msg) {
    fake_publisher->publish(input_msg);
    rclcpp::spin_some(node);
    rclcpp::spin_some(get_fake_node());
    std::this_thread::sleep_for(dt);
    time_passed += dt;
    if (time_passed > max_wait_time) {
      FAIL() << "Did not receive a message soon enough.";
    }
  }

  ASSERT_TRUE(last_received_msg);
  ASSERT_EQ(last_received_msg->rois.size(), input_msg.detections.size());

  const auto & car_detection = last_received_msg->rois.front();

  ASSERT_EQ(car_detection.classifications.size(), 1U);
  EXPECT_FLOAT_EQ(car_detection.classifications.front().probability, 1.0);
  EXPECT_EQ(
    car_detection.classifications.front().classification,
    autoware_auto_msgs::msg::ObjectClassification::CAR);

  ASSERT_EQ(car_detection.polygon.points.size(), 4U);

  {
    const auto & lower_left = car_detection.polygon.points[0];
    EXPECT_FLOAT_EQ(lower_left.x, CAR_CENTER_X - 0.5F * CAR_BBOX_WIDTH);
    EXPECT_FLOAT_EQ(lower_left.y, CAR_CENTER_Y - 0.5F * CAR_BBOX_HEIGHT);
    EXPECT_EQ(lower_left.z, 0.0F);

    const auto & lower_right = car_detection.polygon.points[1];
    EXPECT_FLOAT_EQ(lower_right.x, CAR_CENTER_X + 0.5F * CAR_BBOX_WIDTH);
    EXPECT_FLOAT_EQ(lower_right.y, CAR_CENTER_Y - 0.5F * CAR_BBOX_HEIGHT);
    EXPECT_EQ(lower_right.z, 0.0F);

    const auto & upper_right = car_detection.polygon.points[2];
    EXPECT_FLOAT_EQ(upper_right.x, CAR_CENTER_X + 0.5F * CAR_BBOX_WIDTH);
    EXPECT_FLOAT_EQ(upper_right.y, CAR_CENTER_Y + 0.5F * CAR_BBOX_HEIGHT);
    EXPECT_EQ(upper_right.z, 0.0F);

    const auto & upper_left = car_detection.polygon.points[3];
    EXPECT_FLOAT_EQ(upper_left.x, CAR_CENTER_X - 0.5F * CAR_BBOX_WIDTH);
    EXPECT_FLOAT_EQ(upper_left.y, CAR_CENTER_Y + 0.5F * CAR_BBOX_HEIGHT);
    EXPECT_EQ(upper_left.z, 0.0F);
  }

  static constexpr size_t N_CARS = 4;
  for (size_t i = 1U; i < N_CARS; ++i) {
    const auto & other_car_roi = last_received_msg->rois[i];
    EXPECT_EQ(other_car_roi.classifications, car_detection.classifications);
    EXPECT_EQ(other_car_roi.polygon, car_detection.polygon);
  }

  {
    const auto & truck_roi = last_received_msg->rois[N_CARS];
    EXPECT_EQ(
      truck_roi.classifications.at(0).classification,
      autoware_auto_msgs::msg::ObjectClassification::TRUCK);
    EXPECT_EQ(truck_roi.polygon, car_detection.polygon);
  }

  const auto & pedestrian_roi = *(last_received_msg->rois.rbegin() + 1);
  {
    EXPECT_EQ(
      pedestrian_roi.classifications.at(0).classification,
      autoware_auto_msgs::msg::ObjectClassification::PEDESTRIAN);
    EXPECT_NE(pedestrian_roi.polygon, car_detection.polygon);

    // check clipping to non-negative values
    const auto & lower_left = pedestrian_roi.polygon.points.at(0);
    EXPECT_EQ(lower_left.x, 0.0F);
    EXPECT_EQ(lower_left.y, 0.0F);
  }

  {
    const auto & unknown_roi = last_received_msg->rois.back();
    EXPECT_EQ(
      unknown_roi.classifications.at(0).classification,
      autoware_auto_msgs::msg::ObjectClassification::UNKNOWN);
    EXPECT_EQ(unknown_roi.polygon, pedestrian_roi.polygon);
  }
}

Detection3DArray make_sample_detections_3d()
{
  Detection3DArray detections;
  detections.header.stamp.sec = 24;
  detections.header.stamp.nanosec = 8723U;

  {
    Detection3D d;

    d.header = detections.header;
    d.id = 14224;
    d.label = "Hatchback";
    d.score = 1.0F;
    d.bbox.position.position.x = CAR_CENTER_3D_X;
    d.bbox.position.position.y = CAR_CENTER_3D_Y;
    d.bbox.position.position.z = CAR_CENTER_3D_Z;
    d.bbox.position.orientation.x = CAR_ORIENTATION_X;
    d.bbox.position.orientation.y = CAR_ORIENTATION_Y;
    d.bbox.position.orientation.z = CAR_ORIENTATION_Z;
    d.bbox.position.orientation.w = CAR_ORIENTATION_W;
    d.bbox.size.x = CAR_BBOX_3D_LENGTH;
    d.bbox.size.y = CAR_BBOX_3D_WIDTH;
    d.bbox.size.z = CAR_BBOX_3D_HEIGHT;

    d.velocity.linear = geometry_msgs::build<Vector3>()
      .x(CAR_TWIST_LINEAR_X)
      .y(CAR_TWIST_LINEAR_Y)
      .z(CAR_TWIST_LINEAR_Z);
    detections.detections.emplace_back(d);
  }

  return detections;
}

TEST_F(GroundTruth3dDetectionsTest, ReceiveDetections)
{
  rclcpp::NodeOptions options{};
  const auto node = std::make_shared<GroundTruthDetectionsNode>(options);

  DetectedObjects::SharedPtr last_received_msg{};
  const auto input_topic = "/simulator/ground_truth/detections3D";
  const auto output_topic = "/perception/ground_truth_detections_3d";

  auto fake_publisher = create_publisher<Detection3DArray>(input_topic, 1s);

  auto result_subscription = create_subscription<DetectedObjects>(
    output_topic, *node,                                                                  //
    [&last_received_msg](
      const DetectedObjects::SharedPtr msg) {
      last_received_msg = msg;
    });

  Detection3DArray input_msg = make_sample_detections_3d();

  const auto dt{100ms};
  const auto max_wait_time{std::chrono::seconds{1LL}};
  auto time_passed{std::chrono::milliseconds{0LL}};
  while (!last_received_msg) {
    fake_publisher->publish(input_msg);
    rclcpp::spin_some(node);
    rclcpp::spin_some(get_fake_node());
    std::this_thread::sleep_for(dt);
    time_passed += dt;
    if (time_passed > max_wait_time) {
      FAIL() << "Did not receive a message soon enough.";
    }
  }

  ASSERT_TRUE(last_received_msg);
  ASSERT_EQ(last_received_msg->objects.size(), input_msg.detections.size());

  const auto & car_detection = last_received_msg->objects.front();

  ASSERT_EQ(car_detection.existence_probability, 1.0F);

  // classification
  {
    ASSERT_EQ(car_detection.classification.size(), 1U);

    EXPECT_FLOAT_EQ(car_detection.classification.front().probability, 1.0);
    EXPECT_EQ(
      car_detection.classification.front().classification,
      autoware_auto_msgs::msg::ObjectClassification::CAR);
  }

  // kinematics
  {
    const auto & k = car_detection.kinematics;
    EXPECT_EQ(k.centroid_position.x, CAR_CENTER_3D_X);
    EXPECT_EQ(k.centroid_position.y, CAR_CENTER_3D_Y);
    EXPECT_EQ(k.centroid_position.z, CAR_CENTER_3D_Z);

    EXPECT_FALSE(k.has_position_covariance);

    ASSERT_EQ(
      k.orientation_availability,
      autoware_auto_msgs::msg::DetectedObjectKinematics::AVAILABLE);
    EXPECT_EQ(k.orientation.x, CAR_ORIENTATION_X);
    EXPECT_EQ(k.orientation.y, CAR_ORIENTATION_Y);
    EXPECT_EQ(k.orientation.z, CAR_ORIENTATION_Z);
    EXPECT_EQ(k.orientation.w, CAR_ORIENTATION_W);

    EXPECT_EQ(k.twist.twist.linear.x, CAR_TWIST_LINEAR_X);
    EXPECT_EQ(k.twist.twist.linear.y, CAR_TWIST_LINEAR_Y);
    EXPECT_EQ(k.twist.twist.linear.z, CAR_TWIST_LINEAR_Z);
  }

  // shape
  {
    const auto & s = car_detection.shape;

    EXPECT_EQ(s.height, CAR_BBOX_3D_HEIGHT);

    ASSERT_EQ(s.polygon.points.size(), 4UL);

    // contract: all corners have zero z value
    for (size_t i = 1; i < 4; ++i) {
      EXPECT_EQ(s.polygon.points[i].z, 0.0F) << i;
    }

    {
      const auto & rear_left_corner = s.polygon.points[0];
      EXPECT_FLOAT_EQ(rear_left_corner.x, -0.5F * CAR_BBOX_3D_LENGTH);
      EXPECT_FLOAT_EQ(rear_left_corner.y, -0.5F * CAR_BBOX_3D_WIDTH);
    }

    {
      const auto & rear_right_corner = s.polygon.points[1];
      EXPECT_FLOAT_EQ(rear_right_corner.x, -0.5F * CAR_BBOX_3D_LENGTH);
      EXPECT_FLOAT_EQ(rear_right_corner.y, +0.5F * CAR_BBOX_3D_WIDTH);
    }

    {
      const auto & front_right_corner = s.polygon.points[2];
      EXPECT_FLOAT_EQ(front_right_corner.x, +0.5F * CAR_BBOX_3D_LENGTH);
      EXPECT_FLOAT_EQ(front_right_corner.y, +0.5F * CAR_BBOX_3D_WIDTH);
    }

    {
      const auto & front_left_corner = s.polygon.points[3];
      EXPECT_FLOAT_EQ(front_left_corner.x, +0.5F * CAR_BBOX_3D_LENGTH);
      EXPECT_FLOAT_EQ(front_left_corner.y, -0.5F * CAR_BBOX_3D_WIDTH);
    }
  }
}

}  // namespace
