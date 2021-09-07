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

#include <memory>

#include "gtest/gtest.h"

namespace
{

using autoware::ground_truth_detections::GroundTruthDetectionsNode;
using autoware_auto_msgs::msg::ClassifiedRoiArray;
using geometry_msgs::msg::Vector3;
using lgsvl_msgs::msg::Detection2DArray;
using lgsvl_msgs::msg::Detection2D;

using GroundTruth2dDetectionsTest = autoware::tools::testing::FakeTestNode;

using namespace std::chrono_literals;

static constexpr float_t CAR_CENTER_X = 15.3F;
static constexpr float_t CAR_CENTER_Y = 17.4F;
static constexpr float_t CAR_BBOX_HEIGHT = 5.2F;
static constexpr float_t CAR_BBOX_WIDTH = 2.7F;

// One detection for each supported label + one detection with unsupported label
Detection2DArray make_sample_detections()
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
    d.velocity.linear = geometry_msgs::build<Vector3>().x(1.1).y(2.2).z(3.3);
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

  Detection2DArray input_msg = make_sample_detections();

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

  const auto & car_roi = last_received_msg->rois.front();

  ASSERT_EQ(car_roi.classifications.size(), 1U);
  EXPECT_FLOAT_EQ(car_roi.classifications.front().probability, 1.0);
  EXPECT_EQ(
    car_roi.classifications.front().classification,
    autoware_auto_msgs::msg::ObjectClassification::CAR);

  ASSERT_EQ(car_roi.polygon.points.size(), 4U);

  {
    const auto & lower_left = car_roi.polygon.points[0];
    EXPECT_FLOAT_EQ(lower_left.x, CAR_CENTER_X - 0.5F * CAR_BBOX_WIDTH);
    EXPECT_FLOAT_EQ(lower_left.y, CAR_CENTER_Y - 0.5F * CAR_BBOX_HEIGHT);
    EXPECT_EQ(lower_left.z, 0.0F);

    const auto & lower_right = car_roi.polygon.points[1];
    EXPECT_FLOAT_EQ(lower_right.x, CAR_CENTER_X + 0.5F * CAR_BBOX_WIDTH);
    EXPECT_FLOAT_EQ(lower_right.y, CAR_CENTER_Y - 0.5F * CAR_BBOX_HEIGHT);
    EXPECT_EQ(lower_right.z, 0.0F);

    const auto & upper_right = car_roi.polygon.points[2];
    EXPECT_FLOAT_EQ(upper_right.x, CAR_CENTER_X + 0.5F * CAR_BBOX_WIDTH);
    EXPECT_FLOAT_EQ(upper_right.y, CAR_CENTER_Y + 0.5F * CAR_BBOX_HEIGHT);
    EXPECT_EQ(upper_right.z, 0.0F);

    const auto & upper_left = car_roi.polygon.points[3];
    EXPECT_FLOAT_EQ(upper_left.x, CAR_CENTER_X - 0.5F * CAR_BBOX_WIDTH);
    EXPECT_FLOAT_EQ(upper_left.y, CAR_CENTER_Y + 0.5F * CAR_BBOX_HEIGHT);
    EXPECT_EQ(upper_left.z, 0.0F);
  }

  static constexpr size_t N_CARS = 4;
  for (size_t i = 1U; i < N_CARS; ++i) {
    const auto & other_car_roi = last_received_msg->rois[i];
    EXPECT_EQ(other_car_roi.classifications, car_roi.classifications);
    EXPECT_EQ(other_car_roi.polygon, car_roi.polygon);
  }

  {
    const auto & truck_roi = last_received_msg->rois[N_CARS];
    EXPECT_EQ(
      truck_roi.classifications.at(0).classification,
      autoware_auto_msgs::msg::ObjectClassification::TRUCK);
    EXPECT_EQ(truck_roi.polygon, car_roi.polygon);
  }

  const auto & pedestrian_roi = *(last_received_msg->rois.rbegin() + 1);
  {
    EXPECT_EQ(
      pedestrian_roi.classifications.at(0).classification,
      autoware_auto_msgs::msg::ObjectClassification::PEDESTRIAN);
    EXPECT_NE(pedestrian_roi.polygon, car_roi.polygon);

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

}  // namespace
