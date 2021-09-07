// Copyright 2020 The Autoware Foundation
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

/// \copyright Copyright 2021 The Autoware Foundation
/// \file test_ne_raptor_interface.hpp
/// \brief This file defines the NERaptorInterfaceTest class.

#ifndef NE_RAPTOR_INTERFACE__TEST_NE_RAPTOR_INTERFACE_HPP_
#define NE_RAPTOR_INTERFACE__TEST_NE_RAPTOR_INTERFACE_HPP_

#include <gtest/gtest.h>

#include <ne_raptor_interface/ne_raptor_interface.hpp>
#include <ne_raptor_interface/test_ne_raptor_interface_listener.hpp>
#include <ne_raptor_interface/test_ne_raptor_interface_talker.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>
#include <thread>

using autoware::ne_raptor_interface::NERaptorInterface;
using autoware::ne_raptor_interface::NERaptorInterfaceListener;
using autoware::ne_raptor_interface::NERaptorInterfaceTalker;
using autoware::drivers::vehicle_interface::DbwStateMachine;
using autoware::drivers::vehicle_interface::DbwState;

/* Node init values */
const uint16_t c_ecu_build_num = 0xABCD;
const float32_t c_front_axle_to_cog = 1.5F;
const float32_t c_rear_axle_to_cog = 0.5F;
const float32_t c_steer_to_tire_ratio = 2.0F;
const float32_t c_max_steer_angle = 500.0F;
const float32_t c_accel_limit = 3.0F;
const float32_t c_decel_limit = 3.0F;
const float32_t c_pos_jerk_limit = 9.0F;
const float32_t c_neg_jerk_limit = 9.0F;
const uint32_t c_pub_period = 100;  // Publishing period in ms

/* Other useful constants */
static constexpr uint8_t SERVICE_TIMEOUT = 5;
static constexpr uint8_t kNumRollOver{0x10};

static constexpr uint8_t kNumTests_HMCR{7};
static constexpr uint8_t kTestValid_VSC{5};
static constexpr uint8_t kTestInvalid_VSC{14};
static constexpr uint8_t kNumTests_VSC{kTestValid_VSC + kTestInvalid_VSC};
static constexpr uint8_t kNumTests_HLCC{8};
static constexpr uint8_t kNumTests_VCC{12};
static constexpr uint8_t kTestValid_VSR{4};
static constexpr uint8_t kTestInvalid_VSR{1};
static constexpr uint8_t kNumTests_VSR{kTestValid_VSR + kTestInvalid_VSR};
static constexpr uint8_t kNumTests_VO{18};
static constexpr uint8_t kTestValid_VKS{18};
static constexpr uint8_t kTestInvalid_VKS{1};
static constexpr uint8_t kNumTests_VKS{kTestValid_VKS + kTestInvalid_VKS};

using namespace std::literals::chrono_literals; //NOLINT
const std::chrono::nanoseconds C_TIMEOUT_NANO = 1000000000ns;
const std::chrono::milliseconds C_TIMEOUT_MILLI = std::chrono::milliseconds(c_pub_period);
const uint8_t C_TIMEOUT_ITERATIONS = 25;

/// \brief Class for testing NERaptorInterface
class NERaptorInterfaceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    i_node_ = std::make_shared<rclcpp::Node>("ne_raptor_interface_test_node", "/gtest");
    c_node_ = std::make_shared<rclcpp::Node>("ne_raptor_interface_test_client_node", "/gtest");
    ne_raptor_interface_ = std::make_unique<NERaptorInterface>(
      *i_node_,
      c_ecu_build_num,
      c_front_axle_to_cog,
      c_rear_axle_to_cog,
      c_steer_to_tire_ratio,
      c_max_steer_angle,
      c_accel_limit,
      c_decel_limit,
      c_pos_jerk_limit,
      c_neg_jerk_limit,
      c_pub_period
    );

    rclcpp::NodeOptions options{};
    test_client_ = c_node_->create_client<AutonomyModeChange>("autonomy_mode");
    test_listener_ = std::make_unique<NERaptorInterfaceListener>(options);
    test_talker_ = std::make_unique<NERaptorInterfaceTalker>(options);
    test_service_ = i_node_->create_service<AutonomyModeChange>(
      "autonomy_mode", [this](
        ModeChangeRequest::SharedPtr request,
        ModeChangeResponse::SharedPtr response) -> void
      {
        (void)response;
        if (!ne_raptor_interface_->handle_mode_change_request(request)) {
          throw std::runtime_error{"Changing autonomy mode failed"};
        }
      });
  }

  void TearDown() override
  {
    (void)rclcpp::shutdown();
  }

public:
  rclcpp::Node::SharedPtr i_node_, c_node_;
  std::unique_ptr<NERaptorInterface> ne_raptor_interface_;
  rclcpp::Client<AutonomyModeChange>::SharedPtr test_client_;
  std::unique_ptr<NERaptorInterfaceListener> test_listener_;
  std::unique_ptr<NERaptorInterfaceTalker> test_talker_;
  rclcpp::Service<AutonomyModeChange>::SharedPtr test_service_;
  rclcpp::Clock test_clock{RCL_SYSTEM_TIME};

  // Struct types for test sets
  struct test_hmcr  /** Test handle_mode_change_request */
  {
    uint8_t in_mcr;       /**< Input: ModeChangeRequest */
    bool8_t exp_success;  /**< Expected output: handle_mode_change_request */
    bool8_t exp_enable;   /**< Expected output: dbw enable command sent */
    bool8_t exp_disable;  /**< Expected output: dbw disable command sent */
  };
  struct test_vsc  /** Test VehicleStateCommand */
  {
    VehicleStateCommand in_vsc;  /**< Input: VehicleStateCommand */
    uint8_t in_mcr;              /**< Input: Mode Change Request */ /* enable DBW */
    MiscReport in_mr;            /**< Input: MiscReport */ /* DBW state feedback */
    GearCmd exp_gc;              /**< Expected output: GearCmd */
    GlobalEnableCmd exp_gec;     /**< Expected output: GlobalEnableCmd */
    MiscCmd exp_mc;              /**< Expected output: MiscCmd */
    bool8_t exp_success;         /**< Expected output: send_state_command */
    bool8_t exp_dbw_success;     /**< Expected output: handle_mode_change_request */
    bool8_t exp_dbw_enable;      /**< Expected output: dbw enable command sent */
    bool8_t exp_dbw_disable;     /**< Expected output: dbw disable command sent */
  };
  struct test_hlcc  /** Test HighLevelControlCommand */
  {
    HighLevelControlCommand in_hlcc;  /**< Input: HighLevelControlCommand */
    VehicleStateCommand in_vsc;       /**< Input: VehicleStateCommand */ /* parking brake, gear */
    uint8_t in_mcr;                   /**< Input: Mode Change Request */ /* enable DBW */
    GearReport in_gr;                 /**< Input: GearReport */ /* set current gear */
    AcceleratorPedalCmd exp_apc;      /**< Expected output: AcceleratorPedalCmd */
    BrakeCmd exp_bc;                  /**< Expected output: BrakeCmd */
    SteeringCmd exp_sc;               /**< Expected output: SteeringCmd */
    bool8_t exp_success;              /**< Expected output: send_control_command */
  };
  struct test_vcc  /** Test vehicle control command */
  {
    VehicleControlCommand in_vcc;  /**< Input: vehicle control command */
    VehicleStateCommand in_vsc;    /**< Input: VehicleStateCommand */ /* parking brake, gear */
    uint8_t in_mcr;                /**< Input: Mode Change Request */ /* enable DBW */
    GearReport in_gr;              /**< Input: GearReport */ /* set current gear */
    AcceleratorPedalCmd exp_apc;   /**< Expected output: AcceleratorPedalCmd */
    BrakeCmd exp_bc;               /**< Expected output: BrakeCmd */
    SteeringCmd exp_sc;            /**< Expected output: SteeringCmd */
    bool8_t exp_success;           /**< Expected output: send_control_command */
  };
  struct test_vsr  /** Test VehicleStateReport */
  {
    BrakeReport in_br;            /**< Input: BrakeReport */
    GearReport in_gr;             /**< Input: GearReport */
    MiscReport in_mr;             /**< Input: MiscReport */
    OtherActuatorsReport in_oar;  /**< Input: OtherActuatorsReport */ /* send this last */
    uint8_t in_mcr;               /**< Input: Mode Change Request */ /* enable DBW */
    VehicleStateReport exp_vsr;   /**< Expected output: VehicleStateReport */
  };
  struct test_vo  /** Test VehicleOdometry */
  {
    GearReport in_gr;         /**< Input: GearReport */ /* set current gear */
    MiscReport in_mr;         /**< Input: MiscReport */
    SteeringReport in_sr;     /**< Input: SteeringReport */ /* send this last */
    WheelSpeedReport in_wsr;  /**< Input: WheelSpeedReport */
    VehicleOdometry exp_vo;   /**< Expected output: VehicleOdometry */
  };
  struct test_vks  /** Test VehicleKinematicState */
  {
    GearReport in_gr;               /**< Input: GearReport */ /* set current gear */
    MiscReport in_mr;               /**< Input: MiscReport */ /* send this last */
    SteeringReport in_sr;           /**< Input: SteeringReport */
    WheelSpeedReport in_wsr;        /**< Input: WheelSpeedReport */
    VehicleKinematicState exp_vks;  /**< Expected output: VehicleKinematicState */
  };
};  // class NERaptorInterfaceTest

template<typename T>
void wait_for_subscriber(
  const T & pub_ptr,
  const uint32_t num_expected_subs = 1U,
  std::chrono::milliseconds match_timeout = std::chrono::seconds{10U})
{
  const auto match_start = std::chrono::steady_clock::now();
  // Ensure map publisher has a map that is listening.
  while (pub_ptr->get_subscription_count() < num_expected_subs) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    if (std::chrono::steady_clock::now() - match_start > match_timeout) {
      throw std::runtime_error("timed out waiting for subscriber");
    }
  }
}


template<typename T>
void wait_for_publisher(
  const T & sub_ptr,
  const uint32_t num_expected_pubs = 1U,
  std::chrono::milliseconds match_timeout = std::chrono::seconds{10U})
{
  const auto match_start = std::chrono::steady_clock::now();
  // Ensure map publisher has a map that is listening.
  while (sub_ptr->get_publisher_count() < num_expected_pubs) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    if (std::chrono::steady_clock::now() - match_start > match_timeout) {
      throw std::runtime_error("timed out waiting for publisher");
    }
  }
}

#endif  // NE_RAPTOR_INTERFACE__TEST_NE_RAPTOR_INTERFACE_HPP_
