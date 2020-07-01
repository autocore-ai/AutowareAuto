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

#include <common/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <autoware_auto_msgs/msg/trajectory_point.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/control_diagnostic.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <motion_common/motion_common.hpp>
#include <pure_pursuit/pure_pursuit.hpp>
#include <time_utils/time_utils.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <std_msgs/msg/string.hpp>
#include <atomic>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

using autoware::common::types::char8_t;
using autoware::common::types::float32_t;
using autoware::common::types::bool8_t;

namespace
{

using autoware_auto_msgs::msg::Trajectory;
using autoware_auto_msgs::msg::TrajectoryPoint;
using TrajectoryPointStamped = autoware_auto_msgs::msg::VehicleKinematicState;
using autoware::motion::control::pure_pursuit::ControllerDiagnostic;
using autoware_auto_msgs::msg::VehicleControlCommand;

constexpr auto PI = 3.14159F;

void create_traj(
  Trajectory & traj,
  uint32_t traj_size,
  float32_t heading = PI / 4.0F,
  float32_t offset = 1.0F,
  float32_t slope = 1.0F)
{
  traj.points.resize(traj_size);
  traj.header.frame_id = "traj";
  for (uint32_t idx = 0U; idx < traj_size; ++idx) {
    traj.points[idx].x = (static_cast<float32_t>(idx) * slope) + offset;
    traj.points[idx].y = (static_cast<float32_t>(idx) * slope) + offset;
    float32_t velocity = static_cast<float32_t>(idx) + offset;
    traj.points[idx].longitudinal_velocity_mps = velocity;
    traj.points[idx].heading = ::motion::motion_common::from_angle(heading);
  }
}

void create_current_pose(
  TrajectoryPointStamped & current_stamp,
  float32_t x,
  float32_t y,
  float32_t heading,
  float32_t velocity,
  float32_t acceleration,
  float32_t heading_rate,
  std::string frame_id)
{
  current_stamp.state.x = x;
  current_stamp.state.y = y;
  current_stamp.state.heading = ::motion::motion_common::from_angle(heading);
  current_stamp.state.longitudinal_velocity_mps = velocity;
  current_stamp.state.acceleration_mps2 = acceleration;
  current_stamp.state.heading_rate_rps = heading_rate;
  current_stamp.header.frame_id = frame_id;
}

class MotionControllerPubSub : public rclcpp::Node
{
public:
  MotionControllerPubSub(
    const char8_t * const name,
    const char8_t * const pose_topic,
    const char8_t * const trajectory_topic,
    const char8_t * const command_topic,
    const char8_t * const diag_topic,
    const char8_t * const traj_frame_id,
    const char8_t * const pose_frame_id,
    const uint32_t num_msgs,
    const std::chrono::nanoseconds pub_period)
  : Node(name),
    m_pose_pub_ptr(create_publisher<TrajectoryPointStamped>(pose_topic, rclcpp::QoS{10})),
    m_trajectory_pub_ptr(create_publisher<Trajectory>(trajectory_topic, rclcpp::QoS{10})),
    m_tf2_pub_ptr(create_publisher<tf2_msgs::msg::TFMessage>("tf", rclcpp::QoS{10})),
    m_pose_frame_id(pose_frame_id),
    m_trajectory_frame_id(traj_frame_id),
    m_last_pub(std::chrono::steady_clock::now()),
    m_num_msgs(num_msgs),
    m_pub_period(pub_period),
    m_iteration(0U),
    m_done(false),
    m_sub_ptr(
      create_subscription<VehicleControlCommand>(
        command_topic,
        rclcpp::QoS{10},
        [this](VehicleControlCommand::SharedPtr msg) {
          this->update(msg);
        }
      )
    ),
    m_num_received_msgs(),  // zero initialization
    m_num_correct_output()  // zero initialization
  {
    // initialize trajectory
    m_traj_msg.header.stamp = time_utils::to_message(std::chrono::system_clock::now());
    create_traj(m_traj_msg, 100U, 0.0F);
    m_traj_msg.header.frame_id = m_trajectory_frame_id.c_str();
    m_trajectory_pub_ptr->publish(m_traj_msg);
    // initialize tf transform
    m_tf_msg.transform.translation.x = 0.0;
    m_tf_msg.transform.translation.y = 0.0;
    m_tf_msg.transform.translation.z = 0.0;
    m_tf_msg.transform.rotation.w = 1.0;
    m_tf_msg.transform.rotation.x = 0.0;
    m_tf_msg.transform.rotation.y = 0.0;
    m_tf_msg.transform.rotation.z = 0.0;
    m_tf_msg.child_frame_id = m_trajectory_frame_id.c_str();
    m_tf_msg.header.frame_id = m_pose_frame_id.c_str();
    m_tf_msg.header.stamp = m_traj_msg.header.stamp;
    m_tf2_msg.transforms.push_back(m_tf_msg);
    m_tf2_pub_ptr->publish(m_tf2_msg);
    RCLCPP_INFO(get_logger(), "PubTask initialized");
    RCLCPP_INFO(get_logger(), "MotionControllerPubSub initialized. Now start pub task.");
    (void)diag_topic;
    m_timer = create_wall_timer(m_pub_period, [this]() -> void {task_function();});
  }

  bool8_t done()
  {
    const bool8_t pub_done = is_done();
    const bool8_t wait_done =
      (std::chrono::steady_clock::now() - get_last_pub() > std::chrono::seconds(1LL));
    return pub_done && wait_done;
  }

  bool8_t success() const
  {
    using namespace std::string_literals;
    RCLCPP_INFO(get_logger(), "MotionControllerPubSub checking...");
    RCLCPP_INFO(get_logger(), "\tShould receive: "s + std::to_string(m_num_msgs * 2U));
    RCLCPP_INFO(get_logger(), "\tActual received: "s + std::to_string(m_num_received_msgs));
    // Relatively loose check: make sure you got at least something
    bool8_t message_received = m_num_received_msgs > 0U;
    // Make sure you're close enough
    message_received = message_received && (m_num_received_msgs > (m_num_msgs * 7 / 10));
    // Definitely shouldn't get too many
    message_received = message_received && (m_num_received_msgs <= m_num_msgs * 2U);

    RCLCPP_INFO(get_logger(), "\tShould have correct answer: "s + std::to_string(m_num_msgs));
    RCLCPP_INFO(get_logger(),
      "\tActual have correct answer: "s + std::to_string(m_num_correct_output));
    bool8_t is_correct = m_num_correct_output <= m_num_msgs;
    is_correct = is_correct && ((m_num_correct_output * 2U) > (m_num_msgs * 7 / 10));
    return message_received && is_correct;
  }

private:
  bool8_t is_done() const {return m_done;}

  uint32_t get_iteration() const {return m_iteration;}

  const std::chrono::steady_clock::time_point & get_last_pub() const {return m_last_pub;}

  void task_function()
  {
    RCLCPP_INFO(get_logger(), "PubTask task starts.");
    const uint32_t total_msgs = m_num_msgs * 3U;
    if (m_iteration > total_msgs) {
      m_done = true;
      RCLCPP_INFO(get_logger(), "task_function is finished");
    } else {
      m_last_pub = std::chrono::steady_clock::now();
      const auto right_now = std::chrono::system_clock::now();
      // Given the trajectory (slope = PI / 4) and various velocity, angle, and position values
      if ((m_iteration % 3U) == 0U) {
        // tf update
        m_tf_msg.transform.translation.x += 1.0;
        m_tf_msg.transform.translation.y += 1.0;
        m_tf_msg.transform.rotation.z += 0.007;  // From 0 to PI/2
        const float32_t z_float = static_cast<float32_t>(m_tf_msg.transform.rotation.z);
        m_tf_msg.transform.rotation.w = sqrtf(1.0F - z_float * z_float);
        m_tf_msg.header.stamp = time_utils::to_message(right_now);
        m_tf2_msg.transforms.push_back(m_tf_msg);
        m_tf2_pub_ptr->publish(m_tf2_msg);
        m_tf2_msg.transforms.clear();
        // traj update
        m_traj_msg.header.stamp = m_tf_msg.header.stamp;
        create_traj(
          m_traj_msg, 100U, PI / 4.0F, static_cast<float32_t>(m_tf_msg.transform.translation.x));
        m_traj_msg.header.frame_id = m_trajectory_frame_id.c_str();
        m_trajectory_pub_ptr->publish(m_traj_msg);
      } else {
        // pose update
        const float32_t offset = static_cast<float32_t>(m_iteration % 3U - 1);
        create_current_pose(
          m_pose_msg, offset * 2.0F, offset * 2.0F, 0.0F, offset * 10,
          0.0F, 0.0F, m_pose_frame_id.c_str());
        m_pose_msg.header.stamp = time_utils::to_message(right_now);
        m_pose_pub_ptr->publish(m_pose_msg);
      }
      m_iteration++;
    }
  }

  void update(VehicleControlCommand::SharedPtr msg)
  {
    const uint32_t iteration = get_iteration();
    if ((iteration % 3) == 2U) {
      // Check: estimated accel and steering angle are the desired values
      // Poses that are on the trajectory and those velocity equals to 0 are checked
      // The first pose topic is always on the origin of the trajectory by the TF transform,
      // and the velocity is 0 (constant), and only the yaw angle is changing (from 0 to PI/2).
      // Since the angle of the trajectory is invariant PI/4 throughout the test and the velocity
      // of each trajectory point N equals to the origin x value of the trajectory + K
      // (linearly increasing), the desired steering angle and acceleration can be estimated.
      constexpr float32_t TOL = 1.0E-5F;
      const float32_t target_velocity = static_cast<float32_t>(iteration / 3U) + 6.0F;
      constexpr float32_t distance = 5.0F * sqrtf(2.0F);  // minimum distance = 6.0
      const float32_t accel = ((target_velocity * target_velocity) / (2.0F * distance));
      constexpr float32_t denominator = distance * distance;
      const float32_t tf_x = static_cast<float32_t>((iteration + 1U) / 3U);
      const float32_t tf_z = 0.007F * tf_x;
      const float32_t tf_w = sqrtf(1.0F - tf_z * tf_z);
      const float32_t siny = 2.0F * (tf_w * tf_z);
      const float32_t cosy = 1.0F - (2.0F * tf_z * tf_z);
      const float32_t heading_rad = atan2f(siny, cosy);
      const float32_t relative_y = (-sinf(heading_rad) * 5.0F) + (cosf(heading_rad) * 5.0F);
      const float32_t carvature = (2.0F * relative_y) / denominator;
      const float32_t angle = atanf(carvature * 2.7F);
      if (
        (fabsf(accel - msg->long_accel_mps2) < TOL) &&
        (fabsf(angle - msg->front_wheel_angle_rad) < TOL))
      {
        ++m_num_correct_output;
      }
    }
    ++m_num_received_msgs;
  }

  const rclcpp::Publisher<TrajectoryPointStamped>::SharedPtr m_pose_pub_ptr;
  const rclcpp::Publisher<Trajectory>::SharedPtr m_trajectory_pub_ptr;
  const rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr m_tf2_pub_ptr;
  const std::string m_pose_frame_id;
  const std::string m_trajectory_frame_id;
  std::chrono::steady_clock::time_point m_last_pub;
  const uint32_t m_num_msgs;
  const std::chrono::nanoseconds m_pub_period;
  TrajectoryPointStamped m_pose_msg;
  Trajectory m_traj_msg;
  geometry_msgs::msg::TransformStamped m_tf_msg;
  tf2_msgs::msg::TFMessage m_tf2_msg;
  std::atomic<uint32_t> m_iteration;
  std::atomic<bool8_t> m_done;
  rclcpp::Subscription<VehicleControlCommand>::SharedPtr m_sub_ptr;
  uint32_t m_num_received_msgs;
  uint32_t m_num_correct_output;
  rclcpp::TimerBase::SharedPtr m_timer{};
};  // class MotionControllerPubSub
}  // namespace

int32_t main(const int32_t argc, char8_t ** const argv)
{
  rclcpp::init(argc, argv);

  std::string pose_topic = "current_pose";
  std::string trajectory_topic = "trajectory";
  std::string command_topic = "ctrl_cmd";
  std::string diag_topic = "ctrl_diag";
  std::string traj_frame_id = "base_link";
  std::string pose_frame_id = "map";
  uint32_t num_msgs = 100U;

  const uint32_t period_ms = 30U;
  const std::chrono::nanoseconds period = std::chrono::milliseconds(period_ms);
  std::shared_ptr<MotionControllerPubSub> tester_ptr =
    std::make_shared<MotionControllerPubSub>(
    "pure_puresuit_test",
    pose_topic.c_str(),
    trajectory_topic.c_str(),
    command_topic.c_str(),
    diag_topic.c_str(),
    traj_frame_id.c_str(),
    pose_frame_id.c_str(),
    num_msgs,
    period);

  rclcpp::executors::SingleThreadedExecutor exec;

  while (!tester_ptr->done()) {
    exec.spin_node_some(tester_ptr);
  }

  if (tester_ptr->success()) {
    printf("success\n");
  } else {
    printf("failed\n");
  }

  rclcpp::shutdown();
  return 0;
}
