// Copyright 2020 Apex.AI, Inc.
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

#include <experimental/optional>
#include <gtest/gtest.h>
#include <joystick_vehicle_interface/joystick_vehicle_interface_node.hpp>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>

using joystick_vehicle_interface::Axes;
using joystick_vehicle_interface::Buttons;
using joystick_vehicle_interface::JoystickVehicleInterfaceNode;

enum class PubType
{
  Raw,
  Basic,
  HighLevel
};

struct JoyMapping
{
  PubType pub_type;
  JoystickVehicleInterfaceNode::AxisMap axis_map;
  JoystickVehicleInterfaceNode::AxisScaleMap axis_scale_map;
  JoystickVehicleInterfaceNode::AxisScaleMap axis_offset_map;
  JoystickVehicleInterfaceNode::ButtonMap button_map;
};  // struct JoyMapping

class joy_vi_test : public ::testing::TestWithParam<JoyMapping>
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }
  void TearDown() override
  {
    (void)rclcpp::shutdown();
  }
};  // class joy_vi_test

template<typename T>
struct SubAndMsg
{
  typename rclcpp::Subscription<T>::SharedPtr sub_{nullptr};
  std::experimental::optional<T> msg_{};
  SubAndMsg(rclcpp::Node & nd, const std::string & topic)
  {
    if ("null" != topic) {
      sub_ = nd.create_subscription<T>(topic, rclcpp::QoS{10U}.reliable().durability_volatile(),
          [this](std::shared_ptr<T> msg) {msg_ = *msg;});
    }
  }
};

TEST_P(joy_vi_test, basic_mapping)
{
  const auto param = GetParam();
  const auto high_level_command_topic =
    (PubType::HighLevel == param.pub_type) ? "test_joystick_high_level" : "null";
  const auto raw_command_topic = (PubType::Raw == param.pub_type) ? "test_joystick_raw" : "null";
  const auto basic_command_topic =
    (PubType::Basic == param.pub_type) ? "test_joystick_basic" : "null";
  constexpr auto state_command_topic = "test_state_command_topic";
  constexpr auto joy_topic = "test_joy_topic";

  const auto test_nd = std::make_shared<rclcpp::Node>("test_joystick_vehicle_interface_talker");
  const auto qos = rclcpp::SensorDataQoS{};
  const auto joy_pub = test_nd->create_publisher<sensor_msgs::msg::Joy>(joy_topic, qos);
  SubAndMsg<autoware_auto_msgs::msg::RawControlCommand> raw{*test_nd, raw_command_topic};
  SubAndMsg<autoware_auto_msgs::msg::HighLevelControlCommand>
  high_level{*test_nd, high_level_command_topic};
  SubAndMsg<autoware_auto_msgs::msg::VehicleControlCommand> basic{*test_nd, basic_command_topic};
  SubAndMsg<autoware_auto_msgs::msg::VehicleStateCommand> state{*test_nd, state_command_topic};

  ASSERT_NE(state.sub_, nullptr);
  switch (param.pub_type) {
    case PubType::Raw:
      ASSERT_NE(raw.sub_, nullptr);
      break;
    case PubType::Basic:
      ASSERT_NE(basic.sub_, nullptr);
      break;
    case PubType::HighLevel:
      ASSERT_NE(high_level.sub_, nullptr);
      break;
  }

  const auto nd = std::make_shared<JoystickVehicleInterfaceNode>(
    "test_joystick_vehicle_interface",
    "",
    high_level_command_topic,
    raw_command_topic,
    basic_command_topic,
    state_command_topic,
    joy_topic,
    param.axis_map,
    param.axis_scale_map,
    param.axis_offset_map,
    param.button_map);

  // make joy
  sensor_msgs::msg::Joy joy_msg{};
  joy_msg.axes = {-0.1F, -0.2F, -0.3F, -0.4F, -0.5F};
  joy_msg.buttons = {1, 0, 1, 0, 1, 0, 1, 0, 1, 0};
  const auto timer = test_nd->create_wall_timer(std::chrono::milliseconds{1LL},
      [&joy_msg, &joy_pub]() {joy_pub->publish(joy_msg);});
  // Execute
  rclcpp::executors::SingleThreadedExecutor exec{};
  exec.add_node(test_nd);
  exec.add_node(nd);
  while ((!raw.msg_) && (!basic.msg_) && (!high_level.msg_)) {
    exec.spin_some(std::chrono::milliseconds{1LL});
    std::this_thread::sleep_for(std::chrono::milliseconds{1LL});
  }
  // check that you got stuff
  {
    if (raw.sub_) {
      EXPECT_TRUE(raw.msg_);
    }
    if (basic.sub_) {
      EXPECT_TRUE(basic.msg_);
    }
    if (high_level.sub_) {
      EXPECT_TRUE(high_level.msg_);
    }
    if (HasFailure()) {FAIL();}
  }
  // Helper
  const auto axis_check_fn = [ = ](Axes axis, auto value) -> bool {
      const auto it = param.axis_map.find(axis);
      if ((param.axis_map.end() != it) && (it->second < joy_msg.axes.size())) {
        const auto scale_it = param.axis_scale_map.find(axis);
        const auto scale = param.axis_scale_map.end() == scale_it ?
          JoystickVehicleInterfaceNode::DEFAULT_SCALE : scale_it->second;
        const auto offset_it = param.axis_offset_map.find(axis);
        const auto offset = param.axis_offset_map.end() == offset_it ?
          JoystickVehicleInterfaceNode::DEFAULT_OFFSET : offset_it->second;
        using ValT = decltype(value);
        const auto expect_val =
          static_cast<ValT>((scale * joy_msg.axes[it->second]) + offset);
        if (std::is_floating_point<ValT>::value) {
          return std::fabs(value - expect_val) < std::numeric_limits<ValT>::epsilon();
        } else {
          return value == expect_val;
        }
      }
      return true;
    };
  // Check raw message
  if (raw.msg_) {
    EXPECT_EQ(joy_msg.header.stamp, raw.msg_->stamp);
    constexpr auto BIG_NUM = 999999;  // Mostly checking for unsigned underflow
    // Throttle
    EXPECT_TRUE(axis_check_fn(Axes::THROTTLE, raw.msg_->throttle));
    EXPECT_LT(raw.msg_->throttle, static_cast<decltype(raw.msg_->throttle)>(BIG_NUM));
    // Brake
    EXPECT_TRUE(axis_check_fn(Axes::BRAKE, raw.msg_->brake));
    EXPECT_LT(raw.msg_->brake, static_cast<decltype(raw.msg_->brake)>(BIG_NUM));
    // Front steer
    EXPECT_TRUE(axis_check_fn(Axes::FRONT_STEER, raw.msg_->front_steer));
    // Rear steer
    EXPECT_TRUE(axis_check_fn(Axes::REAR_STEER, raw.msg_->rear_steer));
  }
  // Check basic message
  if (basic.msg_) {
    EXPECT_EQ(joy_msg.header.stamp, basic.msg_->stamp);
    // Front steer
    EXPECT_TRUE(axis_check_fn(Axes::FRONT_STEER, basic.msg_->front_wheel_angle_rad));
    // Rear steer
    EXPECT_TRUE(axis_check_fn(Axes::REAR_STEER, basic.msg_->rear_wheel_angle_rad));
    // Acceleration
    EXPECT_TRUE(axis_check_fn(Axes::ACCELERATION, basic.msg_->long_accel_mps2));
  }
  // Button helper
  const auto button_check_fn = [ = ](Buttons button) -> auto {
      const auto it = param.button_map.find(button);
      if ((param.button_map.end() != it) && (it->second < joy_msg.buttons.size())) {
        return 1 == joy_msg.buttons[it->second];
      }
      return false;
    };
  // check high level message
  if (high_level.msg_) {
    // Exactly one of the high level buttons should be on.. otherwise there's no point in this test
    ASSERT_TRUE(button_check_fn(Buttons::VELOCITY_UP) ^ (button_check_fn(Buttons::VELOCITY_DOWN)));
    // Check sign
    const auto velocity = high_level.msg_->velocity_mps;
    if (button_check_fn(Buttons::VELOCITY_UP)) {
      EXPECT_GT(velocity, 0.0F);
    } else {
      EXPECT_LT(velocity, 0.0F);
    }
    // Must be modulo the increment
    EXPECT_LT(std::fabs(std::fmod(velocity, JoystickVehicleInterfaceNode::VELOCITY_INCREMENT)),
      std::numeric_limits<decltype(velocity)>::epsilon());
    // Curvature
    EXPECT_TRUE(axis_check_fn(Axes::CURVATURE, high_level.msg_->curvature));
  }
  // Check state message
  if (state.msg_) {
    EXPECT_EQ(joy_msg.header.stamp, state.msg_->stamp);
    // It's all buttons here
    using VSC = autoware_auto_msgs::msg::VehicleStateCommand;
    // Toggle buttons depend on state
    // Since joy message is always the same, these buttons can be in one of two states
    const auto toggle_case1 =
      (!button_check_fn(Buttons::HORN_TOGGLE) || state.msg_->horn) &&
      (!button_check_fn(Buttons::HAND_BRAKE_TOGGLE) || state.msg_->hand_brake) &&
      (button_check_fn(Buttons::AUTONOMOUS_TOGGLE) == (state.msg_->mode == VSC::MODE_AUTONOMOUS)) &&
      (button_check_fn(Buttons::HEADLIGHTS_TOGGLE) ==
      (state.msg_->headlight == VSC::HEADLIGHT_ON)) &&
      (button_check_fn(Buttons::WIPER_TOGGLE) == (state.msg_->wiper == VSC::WIPER_LOW));
    const auto toggle_case2 =
      (!button_check_fn(Buttons::HORN_TOGGLE) || !state.msg_->horn) &&
      (!button_check_fn(Buttons::HAND_BRAKE_TOGGLE) || !state.msg_->hand_brake) &&
      (button_check_fn(Buttons::AUTONOMOUS_TOGGLE) == (state.msg_->mode == VSC::MODE_MANUAL)) &&
      (button_check_fn(Buttons::HEADLIGHTS_TOGGLE) ==
      (state.msg_->headlight == VSC::HEADLIGHT_OFF)) &&
      (button_check_fn(Buttons::WIPER_TOGGLE) == (state.msg_->wiper == VSC::WIPER_OFF));
    if (toggle_case1 || toggle_case2) {
      EXPECT_TRUE(true);  // Pushed logic into conditional for printing purposes
    } else {
      std::cerr << "Fail with toggle case:\n";
      const auto err_print =
        [ = ](auto name, auto val, auto expected_val1, auto expected_val2, auto button) {
          if (button_check_fn(button)) {
            std::cerr << name << " = " << static_cast<int32_t>(val) << ", expected " <<
              static_cast<int32_t>(expected_val1) << ", or " <<
              static_cast<int32_t>(expected_val2) << "\n";
          }
        };
      err_print("Horn", state.msg_->horn, true, false, Buttons::HORN_TOGGLE);
      err_print("hand_brake", state.msg_->hand_brake, true, false, Buttons::HAND_BRAKE_TOGGLE);
      err_print("mode", state.msg_->mode, VSC::MODE_AUTONOMOUS, VSC::MODE_MANUAL,
        Buttons::AUTONOMOUS_TOGGLE);
      err_print("headlight", state.msg_->headlight, VSC::HEADLIGHT_ON,
        VSC::HEADLIGHT_OFF, Buttons::HEADLIGHTS_TOGGLE);
      err_print("wiper", state.msg_->wiper, VSC::WIPER_LOW, VSC::WIPER_OFF, Buttons::WIPER_TOGGLE);
      EXPECT_TRUE(false);
    }
    // Easy checks w/o state
    EXPECT_EQ(button_check_fn(Buttons::GEAR_DRIVE), state.msg_->gear == VSC::GEAR_DRIVE);
    EXPECT_EQ(button_check_fn(Buttons::GEAR_REVERSE), state.msg_->gear == VSC::GEAR_REVERSE);
    EXPECT_EQ(button_check_fn(Buttons::GEAR_NEUTRAL), state.msg_->gear == VSC::GEAR_NEUTRAL);
    EXPECT_EQ(button_check_fn(Buttons::GEAR_PARK), state.msg_->gear == VSC::GEAR_PARK);
    EXPECT_EQ(button_check_fn(Buttons::BLINKER_LEFT), state.msg_->blinker == VSC::BLINKER_LEFT);
    EXPECT_EQ(button_check_fn(Buttons::BLINKER_RIGHT), state.msg_->blinker == VSC::BLINKER_RIGHT);
    EXPECT_EQ(button_check_fn(Buttons::BLINKER_HAZARD), state.msg_->blinker == VSC::BLINKER_HAZARD);
  }
}

INSTANTIATE_TEST_CASE_P(
  test,
  joy_vi_test,
  ::testing::Values(
    // Raw control command
    JoyMapping{
  PubType::Raw,
  {{Axes::THROTTLE, 0U}, {Axes::BRAKE, 1U}, {Axes::FRONT_STEER, 2U}},
  {{Axes::THROTTLE, -1.0F}, {Axes::BRAKE, 2.0F}, {Axes::FRONT_STEER, -3.0F}},
  {{Axes::THROTTLE, 2.0F}, {Axes::BRAKE, 5.0F}, {Axes::FRONT_STEER, -10.0F}},
  {}
},
    // Basic control command
    JoyMapping{
  PubType::Basic,
  {{Axes::ACCELERATION, 3U}, {Axes::FRONT_STEER, 0U}, {Axes::REAR_STEER, 4U}},
  {{Axes::ACCELERATION, 12.0F}, {Axes::FRONT_STEER, -3.55F}, {Axes::REAR_STEER, 5.0F}},
  {{Axes::ACCELERATION, -1.1F}, {Axes::FRONT_STEER, 5.2F}, {Axes::REAR_STEER, -2.3F}},
  {}
},
    // State
    JoyMapping{
  PubType::Basic,
  {},
  {},
  {},
  {{Buttons::AUTONOMOUS_TOGGLE, 0U}, {Buttons::HEADLIGHTS_TOGGLE, 1U}, {Buttons::WIPER_TOGGLE, 2U},
    {Buttons::GEAR_DRIVE, 3U}, {Buttons::BLINKER_LEFT, 4U}}
},
    // High level
    JoyMapping{
  PubType::HighLevel,
  {{Axes::CURVATURE, 0U}},
  {{Axes::CURVATURE, 2.5F}},
  {{Axes::CURVATURE, 0.4F}},
  {{Buttons::VELOCITY_UP, 0U}, {Buttons::VELOCITY_DOWN, 1U}}
}
));
