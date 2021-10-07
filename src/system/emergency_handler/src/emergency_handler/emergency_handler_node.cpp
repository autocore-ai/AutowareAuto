// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

#include <memory>
#include <string>
#include <utility>

#include "emergency_handler/emergency_handler_node.hpp"
#include "emergency_handler/visibility_control.hpp"

namespace autoware
{
namespace emergency_handler
{

using namespace std::chrono_literals;

diagnostic_msgs::msg::DiagnosticArray convertHazardStatusToDiagnosticArray(
  rclcpp::Clock::SharedPtr clock, const autoware_auto_msgs::msg::HazardStatus & hazard_status)
{
  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = clock->now();

  const auto decorateDiag = [](const auto & hazard_diag, const std::string & label) {
      auto diag = hazard_diag;
      diag.message = label + diag.message;
      return diag;
    };

  for (const auto & hazard_diag : hazard_status.diag_no_fault) {
    diag_array.status.push_back(decorateDiag(hazard_diag, "[No Fault]"));
  }
  for (const auto & hazard_diag : hazard_status.diag_safe_fault) {
    diag_array.status.push_back(decorateDiag(hazard_diag, "[Safe Fault]"));
  }
  for (const auto & hazard_diag : hazard_status.diag_latent_fault) {
    diag_array.status.push_back(decorateDiag(hazard_diag, "[Latent Fault]"));
  }
  for (const auto & hazard_diag : hazard_status.diag_single_point_fault) {
    diag_array.status.push_back(decorateDiag(hazard_diag, "[Single Point Fault]"));
  }

  return diag_array;
}

EmergencyHandlerNode::EmergencyHandlerNode(const rclcpp::NodeOptions & node_options)
: Node("emergency_handler", node_options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;

  // Parameters
  update_rate_ = this->declare_parameter<double>("update_rate", 10.0);
  data_ready_timeout_ = this->declare_parameter<double>("data_ready_timeout", 30.0);
  timeout_driving_capability_ =
    this->declare_parameter<double>("timeout_driving_capability", 0.5);
  emergency_hazard_level_ = this->declare_parameter<int>("emergency_hazard_level", 2);
  use_emergency_hold_ = this->declare_parameter<bool>("use_emergency_hold", false);
  emergency_stop_acceleration_mps2_ =
    this->declare_parameter("emergency_stop_acceleration_mps2", -2.5);
  use_parking_after_stopped_ = this->declare_parameter<bool>("use_parking_after_stopped", true);
  stopped_velocity_threshold_ =
    this->declare_parameter<double>("stopped_velocity_threshold", 0.001);

  // Subscribers
  sub_autoware_state_ = create_subscription<autoware_auto_msgs::msg::AutowareState>(
    "input/autoware_state", rclcpp::QoS{1},
    std::bind(&EmergencyHandlerNode::onAutowareState, this, _1));
  sub_driving_capability_ = create_subscription<autoware_auto_msgs::msg::DrivingCapability>(
    "input/driving_capability", rclcpp::QoS{1},
    std::bind(&EmergencyHandlerNode::onDrivingCapability, this, _1));
  sub_prev_control_command_ = create_subscription<autoware_auto_msgs::msg::VehicleControlCommand>(
    "input/prev_control_command", rclcpp::QoS{1},
    std::bind(&EmergencyHandlerNode::onPrevControlCommand, this, _1));
  sub_state_report_ = create_subscription<autoware_auto_msgs::msg::VehicleStateReport>(
    "input/state_report", rclcpp::QoS{1},
    std::bind(&EmergencyHandlerNode::onStateReport, this, _1));
  sub_odometry_ = create_subscription<autoware_auto_msgs::msg::VehicleOdometry>(
    "input/odometry", rclcpp::QoS{1}, std::bind(&EmergencyHandlerNode::onOdometry, this, _1));

  // Publishers
  pub_control_command_ = create_publisher<autoware_auto_msgs::msg::VehicleControlCommand>(
    "output/control_command", rclcpp::QoS{1});
  pub_state_command_ = create_publisher<autoware_auto_msgs::msg::VehicleStateCommand>(
    "output/state_command", rclcpp::QoS{1});
  pub_is_emergency_ = create_publisher<autoware_auto_msgs::msg::EmergencyState>(
    "output/is_emergency", rclcpp::QoS{1});
  pub_hazard_status_ = create_publisher<autoware_auto_msgs::msg::HazardStatusStamped>(
    "output/hazard_status", rclcpp::QoS{1});
  pub_diagnostics_err_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "output/diagnostics_err", rclcpp::QoS{1});

  // Heartbeat
  heartbeat_driving_capability_ =
    std::make_shared<HeartbeatChecker<autoware_auto_msgs::msg::DrivingCapability>>(
    *this, "input/driving_capability", timeout_driving_capability_);

  // Service
  srv_clear_emergency_ = this->create_service<std_srvs::srv::Trigger>(
    "service/clear_emergency",
    std::bind(&EmergencyHandlerNode::onClearEmergencyService, this, _1, _2, _3));

  // Initialize messages
  odometry_ = autoware_auto_msgs::msg::VehicleOdometry::ConstSharedPtr(
    new autoware_auto_msgs::msg::VehicleOdometry);
  prev_control_command_ = autoware_auto_msgs::msg::VehicleControlCommand::ConstSharedPtr(
    new autoware_auto_msgs::msg::VehicleControlCommand);

  // Timer
  initialized_time_ = this->now();
  auto timer_callback = std::bind(&EmergencyHandlerNode::onTimer, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / update_rate_));

  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

void EmergencyHandlerNode::onAutowareState(
  const autoware_auto_msgs::msg::AutowareState::ConstSharedPtr msg)
{
  autoware_state_ = msg;
}

void EmergencyHandlerNode::onDrivingCapability(
  const autoware_auto_msgs::msg::DrivingCapability::ConstSharedPtr msg)
{
  driving_capability_ = msg;
}

void EmergencyHandlerNode::onPrevControlCommand(
  const autoware_auto_msgs::msg::VehicleControlCommand::ConstSharedPtr msg)
{
  prev_control_command_ = msg;
}

void EmergencyHandlerNode::onStateReport(
  const autoware_auto_msgs::msg::VehicleStateReport::ConstSharedPtr msg)
{
  state_report_ = msg;
}

void EmergencyHandlerNode::onOdometry(
  const autoware_auto_msgs::msg::VehicleOdometry::ConstSharedPtr msg)
{
  odometry_ = msg;
}

bool EmergencyHandlerNode::onClearEmergencyService(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request_header;
  (void)request;

  const auto hazard_status = judgeHazardStatus();

  if (!isEmergency(hazard_status)) {
    is_emergency_ = false;
    hazard_status_ = hazard_status;

    response->success = true;
    response->message = "Emergency state has been cleared.";
  } else {
    response->success = false;
    response->message = "There are still errors, can't clear the emergency state.";
  }

  return true;
}

void EmergencyHandlerNode::publishHazardStatus(
  const autoware_auto_msgs::msg::HazardStatus & hazard_status)
{
  using autoware_auto_msgs::msg::EmergencyState;

  // Create EmergencyState msg
  EmergencyState emergency_state;
  if (isEmergency(hazard_status)) {
    emergency_state.state = EmergencyState::MRM_OPERATING;
  } else {
    emergency_state.state = EmergencyState::NORMAL;
  }

  // Create HazardStatusStamped msg
  autoware_auto_msgs::msg::HazardStatusStamped hazard_status_stamped;
  hazard_status_stamped.stamp = this->now();
  hazard_status_stamped.status = hazard_status;

  // Publish data
  pub_is_emergency_->publish(emergency_state);
  pub_hazard_status_->publish(hazard_status_stamped);
  pub_diagnostics_err_->publish(
    convertHazardStatusToDiagnosticArray(this->get_clock(), hazard_status_stamped.status));
}

void EmergencyHandlerNode::publishControlAndStateCommands()
{
  const auto stamp = this->now();

  // Publish ControlCommand
  {
    autoware_auto_msgs::msg::VehicleControlCommand msg;
    msg.stamp = stamp;
    msg.front_wheel_angle_rad = prev_control_command_->front_wheel_angle_rad;
    msg.velocity_mps = 0.0;
    msg.long_accel_mps2 = static_cast<float>(emergency_stop_acceleration_mps2_);

    pub_control_command_->publish(msg);
  }

  // Publish StateCommand
  {
    autoware_auto_msgs::msg::VehicleStateCommand msg;
    msg.stamp = stamp;
    msg.blinker = autoware_auto_msgs::msg::VehicleStateCommand::BLINKER_HAZARD;
    msg.headlight = autoware_auto_msgs::msg::VehicleStateCommand::HEADLIGHT_NO_COMMAND;
    msg.wiper = autoware_auto_msgs::msg::VehicleStateCommand::WIPER_NO_COMMAND;

    if (use_parking_after_stopped_ && isStopped()) {
      msg.gear = autoware_auto_msgs::msg::VehicleStateCommand::GEAR_PARK;
    } else {
      msg.gear = autoware_auto_msgs::msg::VehicleStateCommand::GEAR_NO_COMMAND;
    }

    pub_state_command_->publish(msg);
  }
}

bool EmergencyHandlerNode::isDataReady()
{
  if (!autoware_state_) {
    return false;
  }

  if (!driving_capability_) {
    return false;
  }

  if (!state_report_) {
    return false;
  }

  return true;
}

void EmergencyHandlerNode::onTimer()
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  // Wait for data ready
  if (!isDataReady()) {
    if ((this->now() - initialized_time_).seconds() > data_ready_timeout_) {
      autoware_auto_msgs::msg::HazardStatus hazard_status;
      hazard_status.level = autoware_auto_msgs::msg::HazardStatus::SINGLE_POINT_FAULT;
      hazard_status.emergency = true;

      const auto diag = createDiagnosticStatus(
        DiagnosticStatus::ERROR, "input_data_timeout");
      hazard_status.diag_single_point_fault.push_back(diag);
      hazard_status_ = hazard_status;
    }
  } else {
    // Check if emergency
    if (use_emergency_hold_) {
      if (!is_emergency_) {
        // Update only when it is not emergency
        hazard_status_ = judgeHazardStatus();
      }
    } else {
      // Update always
      hazard_status_ = judgeHazardStatus();
    }
  }

  publishHazardStatus(hazard_status_);
  is_emergency_ = isEmergency(hazard_status_);

  // Handle the emergency state by emergency stopping of the vehicle
  if (is_emergency_) {
    publishControlAndStateCommands();
  }
}

bool EmergencyHandlerNode::isStopped()
{
  if (static_cast<double>(odometry_->velocity_mps) < stopped_velocity_threshold_) {
    return true;
  }
  return false;
}

bool EmergencyHandlerNode::isEmergency(
  const autoware_auto_msgs::msg::HazardStatus & hazard_status)
{
  return hazard_status.emergency;
}

autoware_auto_msgs::msg::HazardStatus EmergencyHandlerNode::judgeHazardStatus()
{
  using autoware_auto_msgs::msg::AutowareState;
  using autoware_auto_msgs::msg::HazardStatus;
  using autoware_auto_msgs::msg::VehicleStateReport;

  if (!state_report_ || !autoware_state_) {
    throw std::runtime_error(std::string(__func__) + ": No state report or autoware state.");
  }

  const auto vehicle_mode = state_report_->mode;

  // Get hazard status
  auto hazard_status = vehicle_mode == VehicleStateReport::MODE_AUTONOMOUS ?
    driving_capability_->autonomous_driving :
    driving_capability_->remote_control;

  // Ignore initializing and finalizing state
  {
    const auto is_in_auto_ignore_state =
      (autoware_state_->state == AutowareState::INITIALIZING) ||
      (autoware_state_->state == AutowareState::WAITING_FOR_ROUTE) ||
      (autoware_state_->state == AutowareState::PLANNING) ||
      (autoware_state_->state == AutowareState::FINALIZING);

    if (vehicle_mode == VehicleStateReport::MODE_AUTONOMOUS && is_in_auto_ignore_state) {
      hazard_status.level = HazardStatus::NO_FAULT;
      hazard_status.emergency = false;
    }

    const auto is_in_remote_ignore_state =
      (autoware_state_->state == AutowareState::INITIALIZING) ||
      (autoware_state_->state == AutowareState::FINALIZING);

    if (vehicle_mode == VehicleStateReport::MODE_MANUAL && is_in_remote_ignore_state) {
      hazard_status.level = HazardStatus::NO_FAULT;
      hazard_status.emergency = false;
    }
  }

  // Check timeout
  {
    using diagnostic_msgs::msg::DiagnosticStatus;

    const auto is_in_heartbeat_timeout_ignore_state =
      (autoware_state_->state == AutowareState::INITIALIZING);

    if (!is_in_heartbeat_timeout_ignore_state && heartbeat_driving_capability_->isTimeout()) {
      hazard_status.level = HazardStatus::SINGLE_POINT_FAULT;
      hazard_status.emergency = true;
      hazard_status.diag_single_point_fault.push_back(
        createDiagnosticStatus(
          DiagnosticStatus::ERROR, "heartbeat_timeout",
          "heartbeat_driving_capability is timeout"));
    }
  }

  return hazard_status;
}

diagnostic_msgs::msg::DiagnosticStatus EmergencyHandlerNode::createDiagnosticStatus(
  const int level, const std::string & name, const std::string & message)
{
  diagnostic_msgs::msg::DiagnosticStatus diag;

  diag.level = static_cast<unsigned char>(level);
  diag.name = name;
  diag.message = message;
  diag.hardware_id = "emergency_handler";

  return diag;
}

}  // namespace emergency_handler
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::emergency_handler::EmergencyHandlerNode)
