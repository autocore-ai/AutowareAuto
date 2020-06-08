// Copyright 2020 Apex.AI, Inc.
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

#include "state_machine.hpp"

struct WiperHeadlight
{
  uint8_t wiper;
  uint8_t headlight;
  uint8_t headlight_result;
};

class wiper_headlight_state_machine
  : public state_machine, public ::testing::WithParamInterface<WiperHeadlight>
{
protected:
  // Make sure the state report and command constants are the same because we're mixing stuff
  void SetUp()
  {
    // Wiper
    ASSERT_EQ(VSR::WIPER_LOW, VSC::WIPER_LOW);
    ASSERT_EQ(VSR::WIPER_HIGH, VSC::WIPER_HIGH);
    ASSERT_EQ(VSR::WIPER_OFF, VSC::WIPER_OFF);
    // headlight
    ASSERT_EQ(VSR::HEADLIGHT_OFF, VSC::HEADLIGHT_OFF);
    ASSERT_EQ(VSR::HEADLIGHT_ON, VSC::HEADLIGHT_ON);
    ASSERT_EQ(VSR::HEADLIGHT_HIGH, VSC::HEADLIGHT_HIGH);
  }
};

class wipers_on_headlights_on : public wiper_headlight_state_machine
{
};

// Turning wipers on should turn on headlights: wiper command, light command, light result
TEST_P(wipers_on_headlights_on, basic)
{
  const auto param = GetParam();
  const auto state = VSC{}.set__wiper(param.wiper).set__headlight(param.headlight);
  const auto cmd = sm_.compute_safe_commands(Command{ctrl, state});
  EXPECT_EQ(cmd.control(), ctrl);
  // All equal except for lamp
  EXPECT_EQ(cmd.state()->wiper, state.wiper);
  EXPECT_EQ(cmd.state()->mode, state.mode);
  EXPECT_EQ(cmd.state()->blinker, state.blinker);
  EXPECT_EQ(cmd.state()->gear, state.gear);
  EXPECT_EQ(cmd.state()->hand_brake, state.hand_brake);
  EXPECT_EQ(cmd.state()->horn, state.horn);
  // Lamp
  EXPECT_EQ(cmd.state()->headlight, param.headlight_result);
  // report only if changed
  if (param.headlight_result != param.headlight) {
    EXPECT_TRUE(has_report(StateMachineReport::WIPERS_ON_HEADLIGHTS_ON));
  }
}

INSTANTIATE_TEST_CASE_P(
  test,
  wipers_on_headlights_on,
  ::testing::Values(
    WiperHeadlight{VSC::WIPER_LOW, VSC::HEADLIGHT_NO_COMMAND, VSC::HEADLIGHT_ON},
    WiperHeadlight{VSC::WIPER_LOW, VSC::HEADLIGHT_OFF, VSC::HEADLIGHT_ON},
    WiperHeadlight{VSC::WIPER_LOW, VSC::HEADLIGHT_ON, VSC::HEADLIGHT_ON},
    WiperHeadlight{VSC::WIPER_LOW, VSC::HEADLIGHT_HIGH, VSC::HEADLIGHT_HIGH},
    WiperHeadlight{VSC::WIPER_HIGH, VSC::HEADLIGHT_NO_COMMAND, VSC::HEADLIGHT_ON},
    WiperHeadlight{VSC::WIPER_HIGH, VSC::HEADLIGHT_OFF, VSC::HEADLIGHT_ON},
    WiperHeadlight{VSC::WIPER_HIGH, VSC::HEADLIGHT_ON, VSC::HEADLIGHT_ON},
    WiperHeadlight{VSC::WIPER_HIGH, VSC::HEADLIGHT_HIGH, VSC::HEADLIGHT_HIGH}
  )
);

class wipers_off_headlight_no_change : public wiper_headlight_state_machine
{
};

// Turning off wipers should keep headlights on: wiper state, light state, light command
TEST_P(wipers_off_headlight_no_change, basic)
{
  const auto param = GetParam();
  // Set state to wipers on, headlights on
  sm_.update(VO{}, VSR{}.set__headlight(param.headlight).set__wiper(param.wiper));
  // Turn off wipers
  const auto state = VSC{}.set__wiper(VSC::WIPER_OFF).set__headlight(param.headlight_result);
  const auto cmd = sm_.compute_safe_commands(Command{ctrl, state});
  // Nothing should change
  EXPECT_EQ(cmd.control(), ctrl);
  EXPECT_EQ(cmd.state(), state);
  // report
  EXPECT_TRUE(sm_.reports().empty());
}

INSTANTIATE_TEST_CASE_P(
  test,
  wipers_off_headlight_no_change,
  ::testing::Values(
    WiperHeadlight{VSR::WIPER_LOW, VSR::HEADLIGHT_ON, VSC::HEADLIGHT_NO_COMMAND},
    WiperHeadlight{VSR::WIPER_LOW, VSR::HEADLIGHT_ON, VSC::HEADLIGHT_OFF},
    WiperHeadlight{VSR::WIPER_LOW, VSR::HEADLIGHT_ON, VSC::HEADLIGHT_OFF},
    WiperHeadlight{VSR::WIPER_LOW, VSR::HEADLIGHT_ON, VSC::HEADLIGHT_HIGH},
    WiperHeadlight{VSR::WIPER_LOW, VSR::HEADLIGHT_HIGH, VSC::HEADLIGHT_NO_COMMAND},
    WiperHeadlight{VSR::WIPER_LOW, VSR::HEADLIGHT_HIGH, VSC::HEADLIGHT_OFF},
    WiperHeadlight{VSR::WIPER_LOW, VSR::HEADLIGHT_HIGH, VSC::HEADLIGHT_OFF},
    WiperHeadlight{VSR::WIPER_LOW, VSR::HEADLIGHT_HIGH, VSC::HEADLIGHT_HIGH},
    WiperHeadlight{VSR::WIPER_HIGH, VSR::HEADLIGHT_ON, VSC::HEADLIGHT_NO_COMMAND},
    WiperHeadlight{VSR::WIPER_HIGH, VSR::HEADLIGHT_ON, VSC::HEADLIGHT_OFF},
    WiperHeadlight{VSR::WIPER_HIGH, VSR::HEADLIGHT_ON, VSC::HEADLIGHT_OFF},
    WiperHeadlight{VSR::WIPER_HIGH, VSR::HEADLIGHT_ON, VSC::HEADLIGHT_HIGH},
    WiperHeadlight{VSR::WIPER_HIGH, VSR::HEADLIGHT_HIGH, VSC::HEADLIGHT_NO_COMMAND},
    WiperHeadlight{VSR::WIPER_HIGH, VSR::HEADLIGHT_HIGH, VSC::HEADLIGHT_OFF},
    WiperHeadlight{VSR::WIPER_HIGH, VSR::HEADLIGHT_HIGH, VSC::HEADLIGHT_OFF},
    WiperHeadlight{VSR::WIPER_HIGH, VSR::HEADLIGHT_HIGH, VSC::HEADLIGHT_HIGH}
  )
);
