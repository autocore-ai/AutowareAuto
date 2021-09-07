// Copyright 2020-2021 the Autoware Foundation
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

#include <experimental/optional>
#include <gtest/gtest.h>
#include <joystick_vehicle_interface/joystick_vehicle_interface.hpp>
#include <joystick_vehicle_interface_nodes/joystick_vehicle_interface_node.hpp>
#include <common/types.hpp>

#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using joystick_vehicle_interface::Axes;
using joystick_vehicle_interface::Buttons;
using joystick_vehicle_interface_nodes::JoystickVehicleInterfaceNode;
using autoware::common::types::bool8_t;

enum class PubType
{
  Raw,
  Basic,
  HighLevel
};

struct JoyMapping
{
  PubType pub_type;
  joystick_vehicle_interface::AxisMap axis_map;
  joystick_vehicle_interface::AxisScaleMap axis_scale_map;
  joystick_vehicle_interface::AxisScaleMap axis_offset_map;
  joystick_vehicle_interface::ButtonMap button_map;
};  // struct JoyMapping

class JoyViTest : public ::testing::TestWithParam<JoyMapping>
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

  template<typename T, typename K, typename V>
  void add_map_value_to_parameters(
    std::vector<rclcpp::Parameter> & params,
    const std::string & parameter_key,
    const std::map<K, V> & map,
    const K & map_key)
  {
    auto && it = map.find(map_key);
    if (it != map.end()) {
      params.emplace_back(parameter_key, static_cast<T>(map.at(map_key)));
    }
  }
};  // class JoyViTest

template<typename T>
struct SubAndMsg
{
  typename rclcpp::Subscription<T>::SharedPtr sub_{nullptr};
  std::experimental::optional<T> msg_{};
  SubAndMsg(rclcpp::Node & nd, const std::string & topic)
  {
    if ("null" != topic) {
      sub_ = nd.create_subscription<T>(
        topic, rclcpp::QoS{10U}.reliable().durability_volatile(),
        [this](std::shared_ptr<T> msg) {msg_ = *msg;});
    }
  }
};

TEST_P(JoyViTest, BasicMapping)
{
  using autoware_auto_msgs::msg::HeadlightsCommand;
  using autoware_auto_msgs::msg::WipersCommand;

  const auto param = GetParam();
  const std::string control_command =
    (PubType::HighLevel == param.pub_type) ? "high_level" :
    (PubType::Raw == param.pub_type) ? "raw" :
    (PubType::Basic == param.pub_type) ? "basic" : "null";
  const bool recordreplay_command_enabled = true;

  const auto test_nd = std::make_shared<rclcpp::Node>("test_joystick_vehicle_interface_talker");
  const auto qos = rclcpp::SensorDataQoS{};
  const auto joy_pub = test_nd->create_publisher<sensor_msgs::msg::Joy>("joy", qos);
  SubAndMsg<autoware_auto_msgs::msg::RawControlCommand>
  raw{*test_nd, (control_command == "raw") ? "raw_command" : "null"};
  SubAndMsg<autoware_auto_msgs::msg::HighLevelControlCommand>
  high_level{*test_nd, (control_command == "high_level") ? "high_level_command" : "null"};
  SubAndMsg<autoware_auto_msgs::msg::VehicleControlCommand>
  basic{*test_nd, (control_command == "basic") ? "basic_command" : "null"};
  SubAndMsg<autoware_auto_msgs::msg::VehicleStateCommand> state{*test_nd, "state_command"};
  SubAndMsg<std_msgs::msg::UInt8> recordreplay{*test_nd, "recordreplay_cmd"};

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

  rclcpp::NodeOptions node_options;

  std::vector<rclcpp::Parameter> params;
  params.emplace_back("control_command", control_command);
  params.emplace_back("recordreplay_command_enabled", recordreplay_command_enabled);

  add_map_value_to_parameters<uint8_t>(params, "axes.throttle", param.axis_map, Axes::THROTTLE);
  add_map_value_to_parameters<uint8_t>(params, "axes.brake", param.axis_map, Axes::BRAKE);
  add_map_value_to_parameters<uint8_t>(
    params, "axes.front_steer", param.axis_map,
    Axes::FRONT_STEER);
  add_map_value_to_parameters<uint8_t>(params, "axes.rear_steer", param.axis_map, Axes::REAR_STEER);
  add_map_value_to_parameters<uint8_t>(params, "axes.curvature", param.axis_map, Axes::CURVATURE);
  add_map_value_to_parameters<uint8_t>(
    params, "axes.acceleration", param.axis_map,
    Axes::ACCELERATION);

  add_map_value_to_parameters<double>(
    params, "axis_scale.throttle", param.axis_scale_map,
    Axes::THROTTLE);
  add_map_value_to_parameters<double>(
    params, "axis_scale.brake", param.axis_scale_map,
    Axes::BRAKE);
  add_map_value_to_parameters<double>(
    params, "axis_scale.front_steer", param.axis_scale_map,
    Axes::FRONT_STEER);
  add_map_value_to_parameters<double>(
    params, "axis_scale.rear_steer", param.axis_scale_map,
    Axes::REAR_STEER);
  add_map_value_to_parameters<double>(
    params, "axis_scale.curvature", param.axis_scale_map,
    Axes::CURVATURE);
  add_map_value_to_parameters<double>(
    params, "axis_scale.acceleration", param.axis_scale_map,
    Axes::ACCELERATION);

  add_map_value_to_parameters<double>(
    params, "axis_offset.throttle", param.axis_offset_map,
    Axes::THROTTLE);
  add_map_value_to_parameters<double>(
    params, "axis_offset.brake", param.axis_offset_map,
    Axes::BRAKE);
  add_map_value_to_parameters<double>(
    params, "axis_offset.front_steer", param.axis_offset_map,
    Axes::FRONT_STEER);
  add_map_value_to_parameters<double>(
    params, "axis_offset.rear_steer", param.axis_offset_map,
    Axes::REAR_STEER);
  add_map_value_to_parameters<double>(
    params, "axis_offset.curvature", param.axis_offset_map,
    Axes::CURVATURE);
  add_map_value_to_parameters<double>(
    params, "axis_offset.acceleration", param.axis_offset_map,
    Axes::ACCELERATION);

  add_map_value_to_parameters<uint8_t>(
    params, "buttons.autonomous", param.button_map,
    Buttons::AUTONOMOUS_TOGGLE);
  add_map_value_to_parameters<uint8_t>(
    params, "buttons.headlights", param.button_map,
    Buttons::HEADLIGHTS_TOGGLE);
  add_map_value_to_parameters<uint8_t>(
    params, "buttons.wiper", param.button_map,
    Buttons::WIPER_TOGGLE);
  add_map_value_to_parameters<uint8_t>(
    params, "buttons.gear_drive", param.button_map,
    Buttons::GEAR_DRIVE);
  add_map_value_to_parameters<uint8_t>(
    params, "buttons.gear_reverse", param.button_map,
    Buttons::GEAR_REVERSE);
  add_map_value_to_parameters<uint8_t>(
    params, "buttons.gear_park", param.button_map,
    Buttons::GEAR_PARK);
  add_map_value_to_parameters<uint8_t>(
    params, "buttons.gear_neutral", param.button_map,
    Buttons::GEAR_NEUTRAL);
  add_map_value_to_parameters<uint8_t>(
    params, "buttons.gear_low", param.button_map,
    Buttons::GEAR_LOW);
  add_map_value_to_parameters<uint8_t>(
    params, "buttons.blinker_left", param.button_map,
    Buttons::BLINKER_LEFT);
  add_map_value_to_parameters<uint8_t>(
    params, "buttons.blinker_right", param.button_map,
    Buttons::BLINKER_RIGHT);
  add_map_value_to_parameters<uint8_t>(
    params, "buttons.blinker_hazard", param.button_map,
    Buttons::BLINKER_HAZARD);
  add_map_value_to_parameters<uint8_t>(
    params, "buttons.velocity_up", param.button_map,
    Buttons::VELOCITY_UP);
  add_map_value_to_parameters<uint8_t>(
    params, "buttons.velocity_down", param.button_map,
    Buttons::VELOCITY_DOWN);
  add_map_value_to_parameters<uint8_t>(
    params, "buttons.recordreplay_start_record",
    param.button_map,
    Buttons::RECORDREPLAY_START_RECORD);
  add_map_value_to_parameters<uint8_t>(
    params, "buttons.recordreplay_start_replay",
    param.button_map,
    Buttons::RECORDREPLAY_START_REPLAY);
  add_map_value_to_parameters<uint8_t>(
    params, "buttons.recordreplay_stop", param.button_map,
    Buttons::RECORDREPLAY_STOP);

  node_options.parameter_overrides(params);

  const auto nd = std::make_shared<JoystickVehicleInterfaceNode>(
    node_options);

  // make joy
  sensor_msgs::msg::Joy joy_msg{};
  joy_msg.axes = {-0.1F, -0.2F, -0.3F, -0.4F, -0.5F};
  joy_msg.buttons = {1, 0, 1, 0, 1, 0, 1, 0, 1, 0};
  const auto timer = test_nd->create_wall_timer(
    std::chrono::milliseconds{1LL},
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
  const auto axis_check_fn = [ = ](Axes axis, auto value) -> bool8_t {
      const auto it = param.axis_map.find(axis);
      if ((param.axis_map.end() != it) && (it->second < joy_msg.axes.size())) {
        const auto scale_it = param.axis_scale_map.find(axis);
        const auto scale = param.axis_scale_map.end() == scale_it ?
          joystick_vehicle_interface::DEFAULT_SCALE : scale_it->second;
        const auto offset_it = param.axis_offset_map.find(axis);
        const auto offset = param.axis_offset_map.end() == offset_it ?
          joystick_vehicle_interface::DEFAULT_OFFSET : offset_it->second;
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
    EXPECT_LT(
      std::fabs(std::fmod(velocity, joystick_vehicle_interface::VELOCITY_INCREMENT)),
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
      (state.msg_->headlight == HeadlightsCommand::ENABLE_LOW)) &&
      (button_check_fn(Buttons::WIPER_TOGGLE) == (state.msg_->wiper == WipersCommand::ENABLE_LOW));
    const auto toggle_case2 =
      (!button_check_fn(Buttons::HORN_TOGGLE) || !state.msg_->horn) &&
      (!button_check_fn(Buttons::HAND_BRAKE_TOGGLE) || !state.msg_->hand_brake) &&
      (button_check_fn(Buttons::AUTONOMOUS_TOGGLE) == (state.msg_->mode == VSC::MODE_MANUAL)) &&
      (button_check_fn(Buttons::HEADLIGHTS_TOGGLE) ==
      (state.msg_->headlight == HeadlightsCommand::DISABLE)) &&
      (button_check_fn(Buttons::WIPER_TOGGLE) == (state.msg_->wiper == WipersCommand::DISABLE));
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
      err_print(
        "mode", state.msg_->mode, VSC::MODE_AUTONOMOUS, VSC::MODE_MANUAL,
        Buttons::AUTONOMOUS_TOGGLE);
      err_print(
        "headlight", state.msg_->headlight, HeadlightsCommand::ENABLE_LOW,
        HeadlightsCommand::DISABLE, Buttons::HEADLIGHTS_TOGGLE);
      err_print(
        "wiper", state.msg_->wiper, WipersCommand::ENABLE_LOW, WipersCommand::DISABLE,
        Buttons::WIPER_TOGGLE);
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

  // Record Replay
  if (recordreplay.msg_) {
    EXPECT_EQ(button_check_fn(Buttons::RECORDREPLAY_START_RECORD), recordreplay.msg_->data == 1u);
    EXPECT_EQ(button_check_fn(Buttons::RECORDREPLAY_START_REPLAY), recordreplay.msg_->data == 2u);
    EXPECT_EQ(button_check_fn(Buttons::RECORDREPLAY_STOP), recordreplay.msg_->data == 3u);
    EXPECT_EQ(
      !button_check_fn(Buttons::RECORDREPLAY_START_RECORD) &&
      !button_check_fn(Buttons::RECORDREPLAY_START_REPLAY) &&
      !button_check_fn(Buttons::RECORDREPLAY_STOP), recordreplay.msg_->data == 0u);
  }
}

INSTANTIATE_TEST_CASE_P(
  Test,
  JoyViTest,
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
},
    // RecordReplay
    JoyMapping{
  PubType::Basic,
  {},
  {},
  {},
  {{Buttons::RECORDREPLAY_START_RECORD, 1U}}
},
    JoyMapping{
  PubType::Basic,
  {},
  {},
  {},
  {{Buttons::RECORDREPLAY_START_REPLAY, 1U}}
},
    JoyMapping {
  PubType::Basic,
  {},
  {},
  {},
  {{Buttons::RECORDREPLAY_STOP, 1U}}
}
    // cppcheck-suppress syntaxError
  ), );
