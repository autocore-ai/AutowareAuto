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

#include "ne_raptor_interface/ne_raptor_interface.hpp"

#include <rclcpp/logging.hpp>
#include <time_utils/time_utils.hpp>

#include <cmath>
#include <stdexcept>
#include <iostream>

namespace autoware
{
namespace ne_raptor_interface
{

NERaptorInterface::NERaptorInterface(
  rclcpp::Node & node,
  uint16_t ecu_build_num,
  float32_t front_axle_to_cog,
  float32_t rear_axle_to_cog,
  float32_t steer_to_tire_ratio,
  float32_t max_steer_angle,
  float32_t acceleration_limit,
  float32_t deceleration_limit,
  float32_t acceleration_positive_jerk_limit,
  float32_t deceleration_negative_jerk_limit,
  uint32_t pub_period
)
: m_logger{node.get_logger()},
  m_ecu_build_num{ecu_build_num},
  m_front_axle_to_cog{front_axle_to_cog},
  m_rear_axle_to_cog{rear_axle_to_cog},
  m_steer_to_tire_ratio{steer_to_tire_ratio},
  m_max_steer_angle{max_steer_angle},
  m_acceleration_limit{acceleration_limit},
  m_deceleration_limit{std::fabs(deceleration_limit)},
  m_acceleration_positive_jerk_limit{acceleration_positive_jerk_limit},
  m_deceleration_negative_jerk_limit{deceleration_negative_jerk_limit},
  m_pub_period{std::chrono::milliseconds(pub_period)},
  m_dbw_state_machine(new DbwStateMachine{3}),
  m_rolling_counter{0},
  m_clock{RCL_SYSTEM_TIME}
{
  // Publishers (to Raptor DBW)
  m_accel_cmd_pub = node.create_publisher<AcceleratorPedalCmd>("accelerator_pedal_cmd", 1);
  m_brake_cmd_pub = node.create_publisher<BrakeCmd>("brake_cmd", 1);
  m_gear_cmd_pub = node.create_publisher<GearCmd>("gear_cmd", 1);
  m_gl_en_cmd_pub = node.create_publisher<GlobalEnableCmd>("global_enable_cmd", 1);
  m_misc_cmd_pub = node.create_publisher<MiscCmd>("misc_cmd", 1);
  m_steer_cmd_pub = node.create_publisher<SteeringCmd>("steering_cmd", 1);
  m_dbw_enable_cmd_pub = node.create_publisher<std_msgs::msg::Empty>("enable", 10);
  m_dbw_disable_cmd_pub = node.create_publisher<std_msgs::msg::Empty>("disable", 10);

  // Publishers (to Autoware)
  m_vehicle_kin_state_pub = node.create_publisher<VehicleKinematicState>(
    "vehicle_kinematic_state",
    10);

  // Subscribers (from Raptor DBW)
  m_brake_rpt_sub =
    node.create_subscription<BrakeReport>(
    "brake_report", rclcpp::QoS{20},
    [this](BrakeReport::SharedPtr msg) {on_brake_report(msg);});
  m_gear_rpt_sub =
    node.create_subscription<GearReport>(
    "gear_report", rclcpp::QoS{20},
    [this](GearReport::SharedPtr msg) {on_gear_report(msg);});
  m_misc_rpt_sub =
    node.create_subscription<MiscReport>(
    "misc_report", rclcpp::QoS{2},
    [this](MiscReport::SharedPtr msg) {on_misc_report(msg);});
  m_other_acts_rpt_sub =
    node.create_subscription<OtherActuatorsReport>(
    "other_actuators_report", rclcpp::QoS{20},
    [this](OtherActuatorsReport::SharedPtr msg) {on_other_actuators_report(msg);});
  m_steering_rpt_sub =
    node.create_subscription<SteeringReport>(
    "steering_report", rclcpp::QoS{20},
    [this](SteeringReport::SharedPtr msg) {on_steering_report(msg);});
  m_wheel_spd_rpt_sub =
    node.create_subscription<WheelSpeedReport>(
    "wheel_speed_report", rclcpp::QoS{20},
    [this](WheelSpeedReport::SharedPtr msg) {on_wheel_spd_report(msg);});

  // Initialize command values
  m_gl_en_cmd.ecu_build_number = m_ecu_build_num;
  m_gl_en_cmd.enable_joystick_limits = false;

  m_accel_cmd.control_type.value = ActuatorControlMode::CLOSED_LOOP_VEHICLE;   // vehicle speed
  m_accel_cmd.ignore = false;
  m_accel_cmd.accel_limit = m_acceleration_limit;
  m_accel_cmd.accel_positive_jerk_limit = m_acceleration_positive_jerk_limit;

  m_brake_cmd.control_type.value = ActuatorControlMode::CLOSED_LOOP_VEHICLE;   // vehicle speed
  m_brake_cmd.decel_limit = m_deceleration_limit;
  m_brake_cmd.decel_negative_jerk_limit = m_deceleration_negative_jerk_limit;

  m_steer_cmd.control_type.value = ActuatorControlMode::CLOSED_LOOP_ACTUATOR;  // angular position
  m_steer_cmd.ignore = false;

  m_gear_cmd.cmd.gear = Gear::NONE;

  m_misc_cmd.door_request_right_rear.value = DoorRequest::NO_REQUEST;
  m_misc_cmd.door_request_left_rear.value = DoorRequest::NO_REQUEST;
  m_misc_cmd.door_request_lift_gate.value = DoorRequest::NO_REQUEST;
  m_misc_cmd.rear_wiper_cmd.status = WiperRear::OFF;
  m_misc_cmd.ignition_cmd.status = Ignition::NO_REQUEST;
  m_misc_cmd.cmd.value = TurnSignal::NONE;
  m_misc_cmd.low_beam_cmd.status = LowBeam::OFF;
  m_misc_cmd.high_beam_cmd.status = HighBeam::OFF;
  m_misc_cmd.front_wiper_cmd.status = WiperFront::OFF;

  m_timer = node.create_wall_timer(m_pub_period, std::bind(&NERaptorInterface::cmdCallback, this));
}

void NERaptorInterface::cmdCallback()
{
  // Increment rolling counter
  m_rolling_counter++;
  if (m_rolling_counter > 15) {
    m_rolling_counter = 0;
  }
  std::lock_guard<std::mutex> guard_ac(m_accel_cmd_mutex);
  std::lock_guard<std::mutex> guard_bc(m_brake_cmd_mutex);
  std::lock_guard<std::mutex> guard_gc(m_gear_cmd_mutex);
  std::lock_guard<std::mutex> guard_ec(m_gl_en_cmd_mutex);
  std::lock_guard<std::mutex> guard_mc(m_misc_cmd_mutex);
  std::lock_guard<std::mutex> guard_sc(m_steer_cmd_mutex);

  // Set rolling counters
  m_accel_cmd.rolling_counter = m_rolling_counter;
  m_brake_cmd.rolling_counter = m_rolling_counter;
  m_gear_cmd.rolling_counter = m_rolling_counter;
  m_gl_en_cmd.rolling_counter = m_rolling_counter;
  m_misc_cmd.rolling_counter = m_rolling_counter;
  m_steer_cmd.rolling_counter = m_rolling_counter;

  const auto is_dbw_enabled = m_dbw_state_machine->get_state() != DbwState::DISABLED;

  // Set enables based on current DBW mode
  if (is_dbw_enabled) {
    m_accel_cmd.enable = true;
    m_brake_cmd.enable = true;
    m_gear_cmd.enable = true;
    m_gl_en_cmd.global_enable = true;
    m_misc_cmd.block_standard_cruise_buttons = true;
    m_misc_cmd.block_adaptive_cruise_buttons = true;
    m_misc_cmd.block_turn_signal_stalk = true;
    m_steer_cmd.enable = true;
  } else {
    m_accel_cmd.enable = false;
    m_brake_cmd.enable = false;
    m_gear_cmd.enable = false;
    m_gl_en_cmd.global_enable = false;
    m_misc_cmd.block_standard_cruise_buttons = false;
    m_misc_cmd.block_adaptive_cruise_buttons = false;
    m_misc_cmd.block_turn_signal_stalk = false;
    m_steer_cmd.enable = false;
  }

  // Publish commands to NE Raptor DBW
  m_accel_cmd_pub->publish(m_accel_cmd);
  m_brake_cmd_pub->publish(m_brake_cmd);
  m_gear_cmd_pub->publish(m_gear_cmd);
  m_gl_en_cmd_pub->publish(m_gl_en_cmd);
  m_misc_cmd_pub->publish(m_misc_cmd);
  m_steer_cmd_pub->publish(m_steer_cmd);

  // Set state flags
  m_dbw_state_machine->control_cmd_sent();
  m_dbw_state_machine->state_cmd_sent();
}

bool8_t NERaptorInterface::update(std::chrono::nanoseconds timeout)
{
  (void)timeout;
  return true;
}

bool8_t NERaptorInterface::send_state_command(const VehicleStateCommand & msg)
{
  bool8_t ret{true};

  std::lock_guard<std::mutex> guard_gc(m_gear_cmd_mutex);
  std::lock_guard<std::mutex> guard_ec(m_gl_en_cmd_mutex);
  std::lock_guard<std::mutex> guard_mc(m_misc_cmd_mutex);

  // Set gear values
  switch (msg.gear) {
    case VehicleStateCommand::GEAR_NO_COMMAND:
      m_gear_cmd.cmd.gear = Gear::NONE;
      break;
    case VehicleStateCommand::GEAR_DRIVE:
      m_gear_cmd.cmd.gear = Gear::DRIVE;
      break;
    case VehicleStateCommand::GEAR_REVERSE:
      m_gear_cmd.cmd.gear = Gear::REVERSE;
      break;
    case VehicleStateCommand::GEAR_PARK:
      m_gear_cmd.cmd.gear = Gear::PARK;
      break;
    case VehicleStateCommand::GEAR_LOW:
      m_gear_cmd.cmd.gear = Gear::LOW;
      break;
    case VehicleStateCommand::GEAR_NEUTRAL:
      m_gear_cmd.cmd.gear = Gear::NEUTRAL;
      break;
    default:  // error
      m_gear_cmd.cmd.gear = Gear::NONE;
      RCLCPP_ERROR_THROTTLE(
        m_logger, m_clock, CLOCK_1_SEC,
        "Received command for invalid gear state.");
      ret = false;
      break;
  }

  // Set misc command values
  m_misc_cmd.horn_cmd = msg.horn;

  switch (msg.blinker) {
    case VehicleStateCommand::BLINKER_NO_COMMAND:
      // Keep previous
      break;
    case VehicleStateCommand::BLINKER_OFF:
      m_misc_cmd.cmd.value = TurnSignal::NONE;
      break;
    case VehicleStateCommand::BLINKER_LEFT:
      m_misc_cmd.cmd.value = TurnSignal::LEFT;
      break;
    case VehicleStateCommand::BLINKER_RIGHT:
      m_misc_cmd.cmd.value = TurnSignal::RIGHT;
      break;
    case VehicleStateCommand::BLINKER_HAZARD:
      m_misc_cmd.cmd.value = TurnSignal::HAZARDS;
      break;
    default:
      m_misc_cmd.cmd.value = TurnSignal::SNA;
      RCLCPP_ERROR_THROTTLE(
        m_logger, m_clock, CLOCK_1_SEC,
        "Received command for invalid turn signal state.");
      ret = false;
      break;
  }

  std::lock_guard<std::mutex> guard_bc(m_brake_cmd_mutex);
  m_brake_cmd.park_brake_cmd.status =
    (msg.hand_brake) ? ParkingBrake::ON : ParkingBrake::OFF;

  m_seen_vehicle_state_cmd = true;

  return ret;
}

/* Apparently HighLevelControlCommand will be obsolete soon.
 */
bool8_t NERaptorInterface::send_control_command(const HighLevelControlCommand & msg)
{
  bool8_t ret{true};
  float32_t velocity_checked{0.0F};

  std::lock_guard<std::mutex> guard_ac(m_accel_cmd_mutex);
  std::lock_guard<std::mutex> guard_bc(m_brake_cmd_mutex);
  std::lock_guard<std::mutex> guard_sc(m_steer_cmd_mutex);

  // Using curvature for control
  m_accel_cmd.control_type.value = ActuatorControlMode::CLOSED_LOOP_VEHICLE;  // vehicle speed
  m_steer_cmd.control_type.value = ActuatorControlMode::CLOSED_LOOP_VEHICLE;  // vehicle curvature
  m_brake_cmd.control_type.value = ActuatorControlMode::CLOSED_LOOP_VEHICLE;  // vehicle speed

  // Set limits
  m_steer_cmd.angle_velocity = m_max_steer_angle;

  // Check for invalid changes in direction
  if ( ( (state_report().gear == VehicleStateReport::GEAR_DRIVE) &&
    (msg.velocity_mps < 0.0F) ) ||
    ( (state_report().gear == VehicleStateReport::GEAR_REVERSE) &&
    (msg.velocity_mps > 0.0F) ) )
  {
    velocity_checked = 0.0F;
    RCLCPP_ERROR_THROTTLE(
      m_logger, m_clock, CLOCK_1_SEC,
      "Got invalid speed request value: speed direction does not match current gear.");
    ret = false;
  } else {
    velocity_checked = std::fabs(msg.velocity_mps);
  }

  // Set commands
  m_accel_cmd.speed_cmd = velocity_checked;
  m_steer_cmd.vehicle_curvature_cmd = msg.curvature;

  return ret;
}

/* Apparently RawControlCommand will be obsolete soon.
 * Function not supported - AutoWare RawControlCommand message units are undefined.
 */
bool8_t NERaptorInterface::send_control_command(const RawControlCommand & msg)
{
  (void)msg;
  RCLCPP_ERROR_THROTTLE(
    m_logger, m_clock, CLOCK_1_SEC,
    "NE Raptor does not support sending raw pedal controls directly.");
  return false;
}

bool8_t NERaptorInterface::send_control_command(const VehicleControlCommand & msg)
{
  bool8_t ret{true};
  float32_t velocity_checked{0.0F};
  float32_t angle_checked{0.0F};

  std::lock_guard<std::mutex> guard_ac(m_accel_cmd_mutex);
  std::lock_guard<std::mutex> guard_bc(m_brake_cmd_mutex);
  std::lock_guard<std::mutex> guard_sc(m_steer_cmd_mutex);

  // Using steering wheel angle for control
  m_accel_cmd.control_type.value = ActuatorControlMode::CLOSED_LOOP_VEHICLE;   // vehicle speed
  m_steer_cmd.control_type.value = ActuatorControlMode::CLOSED_LOOP_ACTUATOR;  // angular position
  m_brake_cmd.control_type.value = ActuatorControlMode::CLOSED_LOOP_VEHICLE;   // vehicle speed

  // Set limits
  m_steer_cmd.angle_velocity = m_max_steer_angle;

  if (msg.long_accel_mps2 > 0.0F && msg.long_accel_mps2 < m_acceleration_limit) {
    m_accel_cmd.accel_limit = msg.long_accel_mps2;
  } else {
    m_accel_cmd.accel_limit = m_acceleration_limit;
  }

  if (msg.long_accel_mps2 < 0.0F && msg.long_accel_mps2 > (-1.0F * m_deceleration_limit)) {
    m_brake_cmd.decel_limit = std::fabs(msg.long_accel_mps2);
  } else {
    m_brake_cmd.decel_limit = m_deceleration_limit;
  }

  // Check for invalid changes in direction
  if ( ( (state_report().gear == VehicleStateReport::GEAR_DRIVE) &&
    (msg.velocity_mps < 0.0F) ) ||
    ( (state_report().gear == VehicleStateReport::GEAR_REVERSE) &&
    (msg.velocity_mps > 0.0F) ) )
  {
    velocity_checked = 0.0F;
    RCLCPP_ERROR_THROTTLE(
      m_logger, m_clock, CLOCK_1_SEC,
      "Got invalid speed request value: speed direction does not match current gear.");
    ret = false;
  } else {
    velocity_checked = std::fabs(msg.velocity_mps);
  }

  // Limit steering angle to valid range
  /* Steering -> tire angle conversion is linear except for extreme angles */
  angle_checked = (msg.front_wheel_angle_rad * m_steer_to_tire_ratio) / DEGREES_TO_RADIANS;
  if (angle_checked > m_max_steer_angle) {
    angle_checked = m_max_steer_angle;
    RCLCPP_ERROR_THROTTLE(
      m_logger, m_clock, CLOCK_1_SEC,
      "Got invalid steering angle value: request exceeds max angle.");
    ret = false;
  }
  if (angle_checked < (-1.0F * m_max_steer_angle)) {
    angle_checked = -1.0F * m_max_steer_angle;
    RCLCPP_ERROR_THROTTLE(
      m_logger, m_clock, CLOCK_1_SEC,
      "Got invalid steering angle value: request exceeds max angle.");
    ret = false;
  }

  // Set commands
  m_accel_cmd.speed_cmd = velocity_checked;
  m_steer_cmd.angle_cmd = angle_checked;

  return ret;
}

bool8_t NERaptorInterface::handle_mode_change_request(ModeChangeRequest::SharedPtr request)
{
  bool8_t ret{true};
  std_msgs::msg::Empty send_req{};
  if (request->mode == ModeChangeRequest::MODE_MANUAL) {
    m_dbw_state_machine->user_request(false);
    m_dbw_disable_cmd_pub->publish(send_req);
  } else if (request->mode == ModeChangeRequest::MODE_AUTONOMOUS) {
    m_dbw_state_machine->user_request(true);
    m_dbw_enable_cmd_pub->publish(send_req);
  } else {
    RCLCPP_ERROR_THROTTLE(
      m_logger, m_clock, CLOCK_1_SEC,
      "Got invalid autonomy mode request value.");
    ret = false;
  }
  return ret;
}

void NERaptorInterface::send_headlights_command(const HeadlightsCommand & msg)
{
  switch (msg.command) {
    case HeadlightsCommand::NO_COMMAND:
      // Keep previous
      break;
    case HeadlightsCommand::DISABLE:
      m_misc_cmd.low_beam_cmd.status = LowBeam::OFF;
      m_misc_cmd.high_beam_cmd.status = HighBeam::OFF;
      break;
    case HeadlightsCommand::ENABLE_LOW:
      m_misc_cmd.low_beam_cmd.status = LowBeam::ON;
      m_misc_cmd.high_beam_cmd.status = HighBeam::OFF;
      break;
    case HeadlightsCommand::ENABLE_HIGH:
      m_misc_cmd.low_beam_cmd.status = LowBeam::OFF;
      m_misc_cmd.high_beam_cmd.status = HighBeam::ON;
      break;
    default:
      // Keep previous
      RCLCPP_ERROR_THROTTLE(
        m_logger, m_clock, CLOCK_1_SEC,
        "Received command for invalid headlight state.");
      break;
  }
}

void NERaptorInterface::send_wipers_command(const WipersCommand & msg)
{
  switch (msg.command) {
    case WipersCommand::NO_COMMAND:
      // Keep previous
      break;
    case WipersCommand::DISABLE:
      m_misc_cmd.front_wiper_cmd.status = WiperFront::OFF;
      m_misc_cmd.rear_wiper_cmd.status = WiperRear::OFF;
      break;
    case WipersCommand::ENABLE_LOW:
      m_misc_cmd.front_wiper_cmd.status = WiperFront::CONSTANT_LOW;
      m_misc_cmd.rear_wiper_cmd.status = WiperRear::CONSTANT_LOW;
      break;
    case WipersCommand::ENABLE_HIGH:
      m_misc_cmd.front_wiper_cmd.status = WiperFront::CONSTANT_HIGH;
      m_misc_cmd.rear_wiper_cmd.status = WiperRear::CONSTANT_HIGH;
      break;
    case WipersCommand::ENABLE_CLEAN:
      m_misc_cmd.front_wiper_cmd.status = WiperFront::WASH_BRIEF;
      m_misc_cmd.rear_wiper_cmd.status = WiperRear::WASH_BRIEF;
      break;
    default:
      // Keep previous
      RCLCPP_ERROR_THROTTLE(
        m_logger, m_clock, CLOCK_1_SEC,
        "Received command for invalid wiper state.");
      break;
  }
}

void NERaptorInterface::on_brake_report(const BrakeReport::SharedPtr & msg)
{
  switch (msg->parking_brake.status) {
    case ParkingBrake::OFF:
      state_report().hand_brake = false;
      break;
    case ParkingBrake::ON:
      state_report().hand_brake = true;
      break;
    case ParkingBrake::NO_REQUEST:
    case ParkingBrake::FAULT:
    default:
      state_report().hand_brake = false;
      RCLCPP_WARN_THROTTLE(
        m_logger, m_clock, CLOCK_1_SEC,
        "Received invalid parking brake value from NE Raptor DBW.");
      break;
  }
  m_seen_brake_rpt = true;
}

void NERaptorInterface::on_gear_report(const GearReport::SharedPtr & msg)
{
  switch (msg->state.gear) {
    case Gear::PARK:
      state_report().gear = VehicleStateReport::GEAR_PARK;
      break;
    case Gear::REVERSE:
      state_report().gear = VehicleStateReport::GEAR_REVERSE;
      break;
    case Gear::NEUTRAL:
      state_report().gear = VehicleStateReport::GEAR_NEUTRAL;
      break;
    case Gear::DRIVE:
      state_report().gear = VehicleStateReport::GEAR_DRIVE;
      break;
    case Gear::LOW:
      state_report().gear = VehicleStateReport::GEAR_LOW;
      break;
    case Gear::NONE:
    default:
      state_report().gear = 0;
      RCLCPP_WARN_THROTTLE(
        m_logger, m_clock, CLOCK_1_SEC,
        "Received invalid gear value from NE Raptor DBW.");
      break;
  }
  m_seen_gear_rpt = true;
}

void NERaptorInterface::on_misc_report(const MiscReport::SharedPtr & msg)
{
  const float32_t speed_mps = msg->vehicle_speed * KPH_TO_MPS_RATIO * m_travel_direction;
  const float32_t wheelbase = m_rear_axle_to_cog + m_front_axle_to_cog;
  float32_t delta{0.0F};
  float32_t beta{0.0F};
  float32_t prev_speed_mps{0.0F};
  float32_t dT{0.0F};

  odometry().velocity_mps = speed_mps;

  state_report().fuel = static_cast<uint8_t>(msg->fuel_level);

  if (msg->drive_by_wire_enabled) {
    state_report().mode = VehicleStateReport::MODE_AUTONOMOUS;
  } else {
    state_report().mode = VehicleStateReport::MODE_MANUAL;
  }
  m_dbw_state_machine->dbw_feedback(msg->by_wire_ready && !msg->general_driver_activity);

  std::lock_guard<std::mutex> guard_vks(m_vehicle_kin_state_mutex);

  /**
   * Input velocity is (assumed to be) measured at the rear axle, but we're
   * producing a velocity at the center of gravity.
   * Lateral velocity increases linearly from 0 at the rear axle to the maximum
   * at the front axle, where it is tan(δ)*v_lon.
   */
  delta = m_vehicle_kin_state.state.front_wheel_angle_rad;
  if (m_seen_misc_rpt &&
    m_seen_wheel_spd_rpt)
  {
    prev_speed_mps = m_vehicle_kin_state.state.longitudinal_velocity_mps;
  }
  m_vehicle_kin_state.state.longitudinal_velocity_mps = speed_mps;
  m_vehicle_kin_state.state.lateral_velocity_mps = (m_rear_axle_to_cog / wheelbase) * speed_mps *
    std::tan(delta);

  m_vehicle_kin_state.header.frame_id = "odom";

  // need >1 message in to calculate dT
  if (!m_seen_misc_rpt) {
    m_seen_misc_rpt = true;
    m_vehicle_kin_state.header.stamp = msg->header.stamp;
    // Position = (0,0) at time = 0
    m_vehicle_kin_state.state.x = 0.0F;
    m_vehicle_kin_state.state.y = 0.0F;
    m_vehicle_kin_state.state.heading =
      motion::motion_common::from_angle(0.0F);
    return;
  }

  // Calculate dT (seconds)
  dT = static_cast<float32_t>(msg->header.stamp.sec - m_vehicle_kin_state.header.stamp.sec);
  dT +=
    static_cast<float32_t>(msg->header.stamp.nanosec - m_vehicle_kin_state.header.stamp.nanosec) /
    1000000000.0F;

  if (dT < 0.0F) {
    RCLCPP_ERROR_THROTTLE(
      m_logger, m_clock, CLOCK_1_SEC,
      "Received inconsistent timestamps.");
    return;
  }

  m_vehicle_kin_state.header.stamp = msg->header.stamp;

  if (m_seen_steering_rpt &&
    m_seen_wheel_spd_rpt)
  {
    m_vehicle_kin_state.state.acceleration_mps2 = (speed_mps - prev_speed_mps) / dT;  // m/s^2

    beta = std::atan2(m_rear_axle_to_cog * std::tan(delta), wheelbase);
    m_vehicle_kin_state.state.heading_rate_rps = std::cos(beta) * std::tan(delta) / wheelbase;

    // Update position (x, y), yaw
    kinematic_bicycle_model(dT, &m_vehicle_kin_state);

    m_vehicle_kin_state_pub->publish(m_vehicle_kin_state);
  }
}

void NERaptorInterface::on_other_actuators_report(const OtherActuatorsReport::SharedPtr & msg)
{
  switch (msg->horn_state.status) {
    case HornState::OFF:
      state_report().horn = false;
      break;
    case HornState::ON:
      state_report().horn = true;
      break;
    case HornState::SNA:
    default:
      state_report().horn = false;
      RCLCPP_WARN_THROTTLE(
        m_logger, m_clock, CLOCK_1_SEC,
        "Received invalid horn value from NE Raptor DBW.");
      break;
  }

  switch (msg->turn_signal_state.value) {
    case TurnSignal::NONE:
      state_report().blinker = VehicleStateReport::BLINKER_OFF;
      break;
    case TurnSignal::LEFT:
      state_report().blinker = VehicleStateReport::BLINKER_LEFT;
      break;
    case TurnSignal::RIGHT:
      state_report().blinker = VehicleStateReport::BLINKER_RIGHT;
      break;
    case TurnSignal::HAZARDS:
      state_report().blinker = VehicleStateReport::BLINKER_HAZARD;
      break;
    case TurnSignal::SNA:
    default:
      state_report().blinker = 0;
      RCLCPP_WARN_THROTTLE(
        m_logger, m_clock, CLOCK_1_SEC,
        "Received invalid turn signal value from NE Raptor DBW.");
      break;
  }

  switch (msg->high_beam_state.value) {
    case HighBeamState::OFF:
      state_report().headlight = VehicleStateReport::HEADLIGHT_OFF;
      break;
    case HighBeamState::ON:
      state_report().headlight = VehicleStateReport::HEADLIGHT_HIGH;
      break;
    case HighBeamState::RESERVED:
    case HighBeamState::SNA:
    default:
      state_report().headlight = 0;
      RCLCPP_WARN_THROTTLE(
        m_logger, m_clock, CLOCK_1_SEC,
        "Received invalid headlight value from NE Raptor DBW.");
      break;
  }

  switch (msg->front_wiper_state.status) {
    case WiperFront::OFF:
      state_report().wiper = VehicleStateReport::WIPER_OFF;
      break;
    case WiperFront::CONSTANT_LOW:
      state_report().wiper = VehicleStateReport::WIPER_LOW;
      break;
    case WiperFront::CONSTANT_HIGH:
      state_report().wiper = VehicleStateReport::WIPER_HIGH;
      break;
    case WiperFront::WASH_BRIEF:
      state_report().wiper = VehicleStateReport::WIPER_CLEAN;
      break;
    case WiperFront::SNA:
    default:
      state_report().wiper = 0;
      RCLCPP_WARN_THROTTLE(
        m_logger, m_clock, CLOCK_1_SEC,
        "Received invalid wiper value from NE Raptor DBW.");
      break;
  }

  state_report().stamp = msg->header.stamp;
}

void NERaptorInterface::on_steering_report(const SteeringReport::SharedPtr & msg)
{
  /* Steering -> tire angle conversion is linear except for extreme angles */
  const float32_t f_wheel_angle_rad = (msg->steering_wheel_angle * DEGREES_TO_RADIANS) /
    m_steer_to_tire_ratio;

  odometry().front_wheel_angle_rad = f_wheel_angle_rad;
  odometry().rear_wheel_angle_rad = 0.0F;

  std::lock_guard<std::mutex> guard_vks(m_vehicle_kin_state_mutex);
  m_vehicle_kin_state.state.front_wheel_angle_rad = f_wheel_angle_rad;
  m_vehicle_kin_state.state.rear_wheel_angle_rad = 0.0F;

  m_seen_steering_rpt = true;
  odometry().stamp = msg->header.stamp;
}

void NERaptorInterface::on_wheel_spd_report(const WheelSpeedReport::SharedPtr & msg)
{
  // Detect direction of travel
  float32_t fr = msg->front_right;
  float32_t fl = msg->front_left;
  float32_t rr = msg->rear_right;
  float32_t rl = msg->rear_left;

  if ((fr == 0.0F) && (fl == 0.0F) && (rr == 0.0F) && (rl == 0.0F)) {
    // car is not moving
    m_travel_direction = 0.0F;
  } else if ((fr >= 0.0F) && (fl >= 0.0F) && (rr >= 0.0F) && (rl >= 0.0F)) {
    // car is moving forward
    m_travel_direction = 1.0F;
  } else if ((fr <= 0.0F) && (fl <= 0.0F) && (rr <= 0.0F) && (rl <= 0.0F)) {
    // car is moving backward
    m_travel_direction = -1.0F;
  } else {
    // Wheels are moving different directions. This is probably bad.
    m_travel_direction = 0.0F;
    RCLCPP_WARN_THROTTLE(
      m_logger, m_clock, CLOCK_1_SEC,
      "Received inconsistent wheel speeds.");
  }

  m_seen_wheel_spd_rpt = true;
}

// Update x, y, heading, and heading_rate
void NERaptorInterface::kinematic_bicycle_model(
  float32_t dt, VehicleKinematicState * vks)
{
  const float32_t wheelbase = m_rear_axle_to_cog + m_front_axle_to_cog;

  // convert to yaw – copied from trajectory_spoofer.cpp
  // The below formula could probably be simplified if it would be derived directly for heading
  const float32_t sin_y = 2.0F * vks->state.heading.real * vks->state.heading.imag;
  const float32_t cos_y = 1.0F - 2.0F * vks->state.heading.imag * vks->state.heading.imag;
  float32_t yaw = std::atan2(sin_y, cos_y);
  if (yaw < 0) {
    yaw += TAU;
  }

  /* δ: tire angle (relative to car's main axis)
   * φ: heading/yaw
   * β: direction of movement at point of reference (relative to car's main axis)
   * m_rear_axle_to_cog: distance of point of reference to rear axle
   * m_front_axle_to_cog: distance of point of reference to front axle
   * wheelbase: m_rear_axle_to_cog + m_front_axle_to_cog
   * x, y, v are at the point of reference
   * x' = v cos(φ + β)
   * y' = v sin(φ + β)
   * φ' = (cos(β)tan(δ)) / wheelbase
   * v' = a
   * β = arctan((m_rear_axle_to_cog*tan(δ))/wheelbase)
   */

  float32_t v0_lat = vks->state.lateral_velocity_mps;
  float32_t v0_lon = vks->state.longitudinal_velocity_mps;
  float32_t v0 = std::sqrt(v0_lat * v0_lat + v0_lon * v0_lon);
  float32_t delta = vks->state.front_wheel_angle_rad;
  float32_t a = vks->state.acceleration_mps2;
  float32_t beta = std::atan2(m_rear_axle_to_cog * std::tan(delta), wheelbase);

  // This is the direction in which the POI is moving at the beginning of the
  // integration step. "Course" may not be super accurate, but it's to
  // emphasize that the POI doesn't travel in the heading direction.
  const float32_t course = yaw + beta;

  // How much the yaw changes per meter traveled (at the reference point)
  const float32_t yaw_change =
    std::cos(beta) * std::tan(delta) / wheelbase;

  // How much the yaw rate
  const float32_t yaw_rate = yaw_change * v0;

  // Threshold chosen so as to not result in division by 0
  if (std::abs(yaw_rate) < 1e-18f) {
    vks->state.x += std::cos(course) * (v0 * dt + 0.5f * a * dt * dt);
    vks->state.y += std::sin(course) * (v0 * dt + 0.5f * a * dt * dt);
  } else {
    vks->state.x +=
      (v0 + a * dt) / yaw_rate * std::sin(course + yaw_rate * dt) -
      v0 / yaw_rate * std::sin(course) +
      a / (yaw_rate * yaw_rate) * std::cos(course + yaw_rate * dt) -
      a / (yaw_rate * yaw_rate) * std::cos(course);
    vks->state.y +=
      -(v0 + a * dt) / yaw_rate * std::cos(course + yaw_rate * dt) +
      v0 / yaw_rate * std::cos(course) +
      a / (yaw_rate * yaw_rate) * std::sin(course + yaw_rate * dt) -
      a / (yaw_rate * yaw_rate) * std::sin(course);
  }
  yaw += std::cos(beta) * std::tan(delta) / wheelbase * (v0 * dt + 0.5f * a * dt * dt);
  vks->state.heading.real = std::cos(yaw / 2.0f);
  vks->state.heading.imag = std::sin(yaw / 2.0f);

  // Rotations per second or rad per second?
  vks->state.heading_rate_rps = yaw_rate;
}

}  // namespace ne_raptor_interface
}  // namespace autoware
