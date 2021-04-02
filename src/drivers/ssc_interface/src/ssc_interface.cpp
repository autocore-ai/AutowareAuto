// Copyright 2020 The Autoware Foundation
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

#include "ssc_interface/ssc_interface.hpp"

#include <automotive_platform_msgs/msg/gear.hpp>
#include <rclcpp/logging.hpp>
#include <time_utils/time_utils.hpp>

#include <cmath>
#include <stdexcept>

using SscGear = automotive_platform_msgs::msg::Gear;

namespace ssc_interface
{

SscInterface::SscInterface(
  rclcpp::Node & node,
  float32_t front_axle_to_cog,
  float32_t rear_axle_to_cog,
  float32_t max_accel_mps2,
  float32_t max_decel_mps2,
  float32_t max_yaw_rate_rad
)
: m_logger{node.get_logger()},
  m_front_axle_to_cog{front_axle_to_cog},
  m_rear_axle_to_cog{rear_axle_to_cog},
  m_accel_limit{max_accel_mps2},
  m_decel_limit{max_decel_mps2},
  m_max_yaw_rate{max_yaw_rate_rad},
  m_dbw_state_machine(new DbwStateMachine{3})
{
  // Publishers (to SSC)
  m_gear_cmd_pub = node.create_publisher<GearCommand>("gear_select", 10);
  m_speed_cmd_pub = node.create_publisher<SpeedMode>("arbitrated_speed_commands", 10);
  m_steer_cmd_pub = node.create_publisher<SteerMode>("arbitrated_steering_commands", 10);
  m_turn_signal_cmd_pub = node.create_publisher<TurnSignalCommand>(
    "turn_signal_command", 10);

  // Publishers (to Autoware)
  m_kinematic_state_pub =
    node.create_publisher<VehicleKinematicState>("vehicle_kinematic_state_cog", 10);

  // Subscribers (from SSC)
  m_dbw_state_sub =
    node.create_subscription<std_msgs::msg::Bool>(
    "dbw_enabled_feedback", rclcpp::QoS{10},
    [this](std_msgs::msg::Bool::SharedPtr msg) {on_dbw_state_report(msg);});
  m_gear_feedback_sub =
    node.create_subscription<GearFeedback>(
    "gear_feedback", rclcpp::QoS{10},
    [this](GearFeedback::SharedPtr msg) {on_gear_report(msg);});
  m_vel_accel_sub =
    node.create_subscription<VelocityAccelCov>(
    "velocity_accel_cov", rclcpp::QoS{10},
    [this](VelocityAccelCov::SharedPtr msg) {on_vel_accel_report(msg);});
  m_steer_sub =
    node.create_subscription<SteeringFeedback>(
    "steering_feedback", rclcpp::QoS{10},
    [this](SteeringFeedback::SharedPtr msg) {on_steer_report(msg);});
}

bool8_t SscInterface::update(std::chrono::nanoseconds timeout)
{
  (void)timeout;
  return true;
}

bool8_t SscInterface::send_state_command(const VehicleStateCommand & msg)
{
  // Turn signal command
  TurnSignalCommand tsc;
  tsc.mode = m_dbw_state_machine->enabled() ? 1 : 0;
  tsc.turn_signal = TurnSignalCommand::NONE;

  switch (msg.blinker) {
    case VehicleStateCommand::BLINKER_NO_COMMAND:
      break;
    case VehicleStateCommand::BLINKER_OFF:
      tsc.mode = 1;
      tsc.turn_signal = TurnSignalCommand::NONE;
      break;
    case VehicleStateCommand::BLINKER_LEFT:
      tsc.mode = 1;
      tsc.turn_signal = TurnSignalCommand::LEFT;
      break;
    case VehicleStateCommand::BLINKER_RIGHT:
      tsc.mode = 1;
      tsc.turn_signal = TurnSignalCommand::RIGHT;
      break;
    case VehicleStateCommand::BLINKER_HAZARD:
      tsc.mode = 1;
      tsc.turn_signal = TurnSignalCommand::NONE;
      RCLCPP_WARN(m_logger, "Received command for unsuported turn signal state.");
      break;
    default:
      RCLCPP_ERROR(m_logger, "Received command for invalid turn signal state.");
  }

  tsc.header.stamp = msg.stamp;
  m_turn_signal_cmd_pub->publish(tsc);

  // Gear command
  GearCommand gc;
  // Has no mode - only listens if at least one
  // other DBW system is enabled
  gc.command.gear = SscGear::NONE;

  switch (msg.gear) {
    case VehicleStateCommand::GEAR_NO_COMMAND:
      break;
    case VehicleStateCommand::GEAR_DRIVE:
      gc.command.gear = SscGear::DRIVE;
      break;
    case VehicleStateCommand::GEAR_REVERSE:
      gc.command.gear = SscGear::REVERSE;
      break;
    case VehicleStateCommand::GEAR_PARK:
      gc.command.gear = SscGear::PARK;
      break;
    case VehicleStateCommand::GEAR_LOW:
      gc.command.gear = SscGear::LOW;
      break;
    case VehicleStateCommand::GEAR_NEUTRAL:
      gc.command.gear = SscGear::NEUTRAL;
      break;
    default:
      RCLCPP_ERROR(m_logger, "Received command for invalid gear state.");
  }

  gc.header.stamp = msg.stamp;
  m_gear_cmd_pub->publish(gc);

  m_dbw_state_machine->state_cmd_sent();

  return true;
}

bool8_t SscInterface::send_control_command(const HighLevelControlCommand & msg)
{
  auto desired_velocity{0.0F};

  // Handle velocities opposite the current direction of travel
  if (
    (state_report().gear == VehicleStateReport::GEAR_DRIVE && msg.velocity_mps < 0.0F) ||
    (state_report().gear == VehicleStateReport::GEAR_REVERSE && msg.velocity_mps > 0.0F))
  {
    desired_velocity = 0.0F;
  } else {
    desired_velocity = std::fabs(msg.velocity_mps);
  }

  // Publish speed command
  SpeedMode speed_mode;
  speed_mode.mode = m_dbw_state_machine->enabled() ? 1 : 0;
  speed_mode.speed = desired_velocity;
  speed_mode.acceleration_limit = m_accel_limit;
  speed_mode.deceleration_limit = m_decel_limit;
  speed_mode.header.stamp = msg.stamp;
  m_speed_cmd_pub->publish(speed_mode);

  // Publish steering command
  SteerMode steer_mode;
  constexpr float32_t curvature_rate = 0.15F;  // assume the rate is constant.
  steer_mode.mode = m_dbw_state_machine->enabled() ? 1 : 0;
  steer_mode.curvature = msg.curvature;
  steer_mode.max_curvature_rate = curvature_rate;  // should be positive
  steer_mode.header.stamp = msg.stamp;
  m_steer_cmd_pub->publish(steer_mode);

  m_dbw_state_machine->control_cmd_sent();

  return true;
}

bool8_t SscInterface::send_control_command(const RawControlCommand & msg)
{
  (void)msg;
  RCLCPP_ERROR(m_logger, "SSC does not support sending raw pedal controls directly.");
  return false;
}

bool8_t SscInterface::send_control_command(const VehicleControlCommand & msg)
{
  auto signed_velocity = msg.velocity_mps;

  if (msg.velocity_mps > 0.0F && state_report().gear == VehicleStateReport::GEAR_REVERSE) {
    signed_velocity = -msg.velocity_mps;
  }

  const auto wheelbase = m_front_axle_to_cog + m_rear_axle_to_cog;

  HighLevelControlCommand hlc_cmd;
  hlc_cmd.stamp = msg.stamp;

  // Convert from center-of-mass velocity to rear-axle-center velocity
  const auto beta =
    std::atan(m_front_axle_to_cog * std::tan(msg.front_wheel_angle_rad) / (wheelbase));
  hlc_cmd.velocity_mps = std::cos(beta) * signed_velocity;

  // Calculate curvature from desired steering angle
  hlc_cmd.curvature = std::tan(msg.front_wheel_angle_rad) / (wheelbase);

  return send_control_command(hlc_cmd);
}

bool8_t SscInterface::handle_mode_change_request(ModeChangeRequest::SharedPtr request)
{
  if (request->mode == ModeChangeRequest::MODE_MANUAL) {
    m_dbw_state_machine->user_request(false);
    return true;
  } else if (request->mode == ModeChangeRequest::MODE_AUTONOMOUS) {
    m_dbw_state_machine->user_request(true);
    return true;
  } else {
    RCLCPP_ERROR(m_logger, "Got invalid autonomy mode request value.");
    return false;
  }
}

void SscInterface::on_dbw_state_report(const std_msgs::msg::Bool::SharedPtr & msg)
{
  if (msg->data) {
    state_report().mode = VehicleStateReport::MODE_AUTONOMOUS;
  } else {
    state_report().mode = VehicleStateReport::MODE_MANUAL;
  }

  m_dbw_state_machine->dbw_feedback(msg->data);
}

void SscInterface::on_gear_report(const GearFeedback::SharedPtr & msg)
{
  switch (msg->current_gear.gear) {
    case SscGear::PARK:
      state_report().gear = VehicleStateReport::GEAR_PARK;
      break;
    case SscGear::REVERSE:
      state_report().gear = VehicleStateReport::GEAR_REVERSE;
      break;
    case SscGear::NEUTRAL:
      state_report().gear = VehicleStateReport::GEAR_NEUTRAL;
      break;
    case SscGear::DRIVE:
      state_report().gear = VehicleStateReport::GEAR_DRIVE;
      break;
    case SscGear::LOW:
      state_report().gear = VehicleStateReport::GEAR_LOW;
      break;
    case SscGear::NONE:
    default:
      state_report().gear = 0;
      RCLCPP_WARN(m_logger, "Received invalid gear value from SSC.");
  }
}

void SscInterface::on_steer_report(const SteeringFeedback::SharedPtr & msg)
{
  const auto front_wheel_angle_rad = msg->steering_wheel_angle * STEERING_TO_TIRE_RATIO;
  odometry().stamp = msg->header.stamp;
  odometry().front_wheel_angle_rad = front_wheel_angle_rad;
  odometry().rear_wheel_angle_rad = 0.0F;

  std::lock_guard<std::mutex> guard(m_vehicle_kinematic_state_mutex);
  m_vehicle_kinematic_state.state.front_wheel_angle_rad = front_wheel_angle_rad;
  m_seen_steer = true;
}

void SscInterface::on_vel_accel_report(const VelocityAccelCov::SharedPtr & msg)
{
  odometry().stamp = msg->header.stamp;
  odometry().velocity_mps = msg->velocity;

  std::lock_guard<std::mutex> guard(m_vehicle_kinematic_state_mutex);
  // Input velocity is (assumed to be) measured at the rear axle, but we're
  // producing a velocity at the center of gravity.
  // Lateral velocity increases linearly from 0 at the rear axle to the maximum
  // at the front axle, where it is tan(δ)*v_lon.
  const float32_t wheelbase = m_rear_axle_to_cog + m_front_axle_to_cog;
  const float32_t delta = m_vehicle_kinematic_state.state.front_wheel_angle_rad;
  m_vehicle_kinematic_state.header.frame_id = "odom";
  m_vehicle_kinematic_state.state.longitudinal_velocity_mps = msg->velocity;
  m_vehicle_kinematic_state.state.lateral_velocity_mps = (m_rear_axle_to_cog / wheelbase) *
    msg->velocity * std::tan(delta);
  m_vehicle_kinematic_state.state.acceleration_mps2 = msg->accleration;
  // Dt can not be calculated from the first message alone
  if (!m_seen_vel_accel) {
    m_seen_vel_accel = true;
    m_vehicle_kinematic_state.header.stamp = msg->header.stamp;
    return;
  }

  // Calculate dt
  float32_t dt = static_cast<float32_t>(
    msg->header.stamp.sec - m_vehicle_kinematic_state.header.stamp.sec);
  dt += static_cast<float32_t>(
    msg->header.stamp.sec - m_vehicle_kinematic_state.header.stamp.sec) / 1000000000.0F;

  if (dt < 0.0F) {
    RCLCPP_WARN(m_logger, "Received inconsistent timestamps.");
  }

  m_vehicle_kinematic_state.header.stamp = msg->header.stamp;

  if (m_seen_steer) {
    // TODO(Takamasa Horibe): modify after AVP with TF specifications
    // position or yaw is 0 since odom=baselink with static TF in AVP demo
    m_vehicle_kinematic_state.state.x = 0.0F;
    m_vehicle_kinematic_state.state.y = 0.0F;
    m_vehicle_kinematic_state.state.heading.real = std::cos(/*yaw*/ 0.0F / 2.0F);
    m_vehicle_kinematic_state.state.heading.imag = std::sin(/*yaw*/ 0.0F / 2.0F);
    const float32_t beta = std::atan2(m_rear_axle_to_cog * std::tan(delta), wheelbase);
    m_vehicle_kinematic_state.state.heading_rate_rps = std::cos(beta) * std::tan(delta) / wheelbase;
    m_kinematic_state_pub->publish(m_vehicle_kinematic_state);
  }
}


// Update x, y, heading, and heading_rate from the other variables
// TODO(nikolai.morin): Clean up and implement as a motion model
void SscInterface::kinematic_bicycle_model(
  float32_t dt, float32_t l_r, float32_t l_f, VehicleKinematicState * vks)
{
  // convert to yaw – copied from trajectory_spoofer.cpp
  // The below formula could probably be simplified if it would be derived directly for heading
  const float32_t sin_y = 2.0F * vks->state.heading.real * vks->state.heading.imag;
  const float32_t cos_y = 1.0F - 2.0F * vks->state.heading.imag * vks->state.heading.imag;
  float32_t yaw = std::atan2(sin_y, cos_y);
  if (yaw < 0) {
    yaw += TAU;
  }
  // δ: tire angle (relative to car's main axis)
  // φ: heading/yaw
  // β: direction of movement at point of reference (relative to car's main axis)
  // l_r: distance of point of reference to rear axle
  // l_f: distance of point of reference to front axle
  // x, y, v are at the point of reference
  // x' = v cos(φ + β)
  // y' = v sin(φ + β)
  // φ' = (cos(β)tan(δ)) / (l_r + l_f)
  // v' = a
  // β = arctan((l_r*tan(δ))/(l_r + l_f))

  // TODO(nikolai.morin): Decouple from VehicleKinematicState, use only v0 as
  // input. Currently v0_lat + v0_lon are redundant with beta/delta via
  // beta = atan2(v_lat, v_lon).
  float32_t v0_lat = vks->state.lateral_velocity_mps;
  float32_t v0_lon = vks->state.longitudinal_velocity_mps;
  float32_t v0 = std::sqrt(v0_lat * v0_lat + v0_lon * v0_lon);
  float32_t delta = vks->state.front_wheel_angle_rad;
  float32_t a = vks->state.acceleration_mps2;
  float32_t beta = std::atan2(l_r * std::tan(delta), l_r + l_f);
  // This is the direction in which the POI is moving at the beginning of the
  // integration step. "Course" may not be super accurate, but it's to
  // emphasize that the POI doesn't travel in the heading direction.
  const float32_t course = yaw + beta;
  // How much the yaw changes per meter traveled (at the reference point)
  const float32_t yaw_change =
    std::cos(beta) * std::tan(delta) / (l_r + l_f);
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
  yaw += std::cos(beta) * std::tan(delta) / (l_r + l_f) * (v0 * dt + 0.5f * a * dt * dt);
  vks->state.heading.real = std::cos(yaw / 2.0f);
  vks->state.heading.imag = std::sin(yaw / 2.0f);

  // Rotations per second or rad per second?
  vks->state.heading_rate_rps = yaw_rate;
}

}  // namespace ssc_interface
