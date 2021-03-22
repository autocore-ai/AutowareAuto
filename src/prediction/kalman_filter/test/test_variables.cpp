// Copyright 2021 the Autoware Foundation
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
// Developed by Apex.AI, Inc.

/// \copyright Copyright 2021 the Autoware Foundation
/// All rights reserved.
/// \file
/// \brief This file defines tests for the variables.

#include <kalman_filter/variable.hpp>

#include <gtest/gtest.h>

struct NotAVariable {};
struct CustomVariable : autoware::prediction::Variable {};
struct CustomAngle : autoware::prediction::AngleVariable {};

/// @test Variable traits work as expected.
TEST(VariableTest, CheckVariables) {
  EXPECT_FALSE(autoware::prediction::is_variable<NotAVariable>::value);

  EXPECT_TRUE(autoware::prediction::is_variable<CustomVariable>::value);
  EXPECT_FALSE(autoware::prediction::is_angle<CustomVariable>::value);

  EXPECT_TRUE(autoware::prediction::is_variable<CustomAngle>::value);
  EXPECT_TRUE(autoware::prediction::is_angle<CustomAngle>::value);
}
