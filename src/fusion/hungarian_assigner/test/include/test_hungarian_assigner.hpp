// Copyright 2018 the Autoware Foundation
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

#ifndef TEST_HUNGARIAN_ASSIGNER_HPP_
#define TEST_HUNGARIAN_ASSIGNER_HPP_

#include <hungarian_assigner/hungarian_assigner.hpp>
#include <iostream>
#include <vector>
#include <thread>
#include "common/types.hpp"

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::fusion::hungarian_assigner::hungarian_assigner_c;

template<uint16_t N>
void set_weights(
  hungarian_assigner_c<N> & assign,
  const std::vector<std::vector<float32_t>> & weights)
{
  for (uint64_t idx = {}; idx < weights.size(); ++idx) {
    const std::vector<float32_t> & w = weights[idx];
    for (uint64_t jdx = {}; jdx < w.size(); ++jdx) {
      assign.set_weight(w[jdx], static_cast<int64_t>(idx), static_cast<int64_t>(jdx));
    }
  }
}

// test various assumptions such as NAN and memset
TEST(HungarianAssigner, Assumptions)
{
  bool8_t bval = true;
  memset(&bval, 0, sizeof(bval));
  ASSERT_FALSE(bval);
  uint64_t uval = 99U;
  memset(&uval, 0, sizeof(uval));
  ASSERT_EQ(uval, 0U);
}

// absolutely minimal example
TEST(HungarianAssigner, Minimal)
{
  hungarian_assigner_c<16U> assign;
  ASSERT_THROW(assign.set_size(5U, 4U), std::domain_error);
  ASSERT_THROW(assign.set_size(15, 17), std::length_error);
  assign.set_size(3U, 3U);
  assign.set_weight(1.0F, 0U, 1U);
  assign.set_weight(1.0F, 1U, 2U);
  assign.set_weight(1.0F, 2U, 0U);
  ASSERT_TRUE(assign.assign());
  ASSERT_EQ(assign.get_assignment(0U), 1U);
  ASSERT_EQ(assign.get_assignment(1U), 2U);
  ASSERT_EQ(assign.get_assignment(2U), 0U);
  ASSERT_THROW(assign.get_unassigned(0U), std::range_error);
  // reset
  ASSERT_NO_THROW(assign.reset());
  ASSERT_THROW(assign.get_assignment(0U), std::range_error);
  ASSERT_THROW(assign.get_unassigned(0U), std::range_error);
  // make sure the result is the same
  assign.set_size(3U, 3U);
  assign.set_weight(1.0F, 0U, 1U);
  assign.set_weight(1.0F, 1U, 2U);
  assign.set_weight(1.0F, 2U, 0U);
  ASSERT_TRUE(assign.assign());
  ASSERT_EQ(assign.get_assignment(0U), 1U);
  ASSERT_EQ(assign.get_assignment(1U), 2U);
  ASSERT_EQ(assign.get_assignment(2U), 0U);
  ASSERT_THROW(assign.get_unassigned(0U), std::range_error);
  ASSERT_THROW(assign.get_assignment(3U), std::range_error);
}


// basic example, one degenerate case
/*
0   10   1
0   10   10
1   0    1
*/
TEST(HungarianAssigner, Basic)
{
  hungarian_assigner_c<16U> assign(3, 3);
  assign.set_weight(0.0F, 0U, 0U);
  assign.set_weight(10.0F, 0U, 1U);
  assign.set_weight(1.0F, 0U, 2U);
  assign.set_weight(0.0F, 1U, 0U);
  assign.set_weight(10.0F, 1U, 1U);
  assign.set_weight(10.0F, 1U, 2U);
  assign.set_weight(1.0F, 2U, 0U);
  assign.set_weight(0.0F, 2U, 1U);
  assign.set_weight(1.0F, 2U, 2U);
  ASSERT_THROW(assign.set_weight(1.0F, 2U, 3U), std::out_of_range);
  ASSERT_THROW(assign.set_weight(1.0, 3U, 0U), std::out_of_range);
  ASSERT_TRUE(assign.assign());
  ASSERT_EQ(assign.get_assignment(0U), 2U);
  ASSERT_EQ(assign.get_assignment(1U), 0U);
  ASSERT_EQ(assign.get_assignment(2U), 1U);
  ASSERT_THROW(assign.get_unassigned(0U), std::range_error);
  ASSERT_THROW(assign.get_assignment(3U), std::range_error);
}

// exercise basic unbalanced logic
// https://www.wisdomjobs.com/e-university/quantitative-techniques-for-management-tutorial-297/unbalanced-assignment-problem-9899.html
TEST(HungarianAssigner, Unbalanced1)
{
  hungarian_assigner_c<16U> assign;
  assign.set_size(4U, 5U);
  const std::vector<std::vector<float32_t>> weights =
  {
    {5, 7, 11, 6, 7},
    {8, 5, 5, 6, 5},
    {6, 7, 10, 7, 3},
    {10, 4, 8, 2, 4}
  };
  set_weights(assign, weights);

  ASSERT_TRUE(assign.assign());
  ASSERT_EQ(assign.get_assignment(0U), 0U);
  ASSERT_EQ(assign.get_assignment(1U), 1U);
  ASSERT_EQ(assign.get_assignment(2U), 4U);
  ASSERT_EQ(assign.get_assignment(3U), 3U);
  ASSERT_EQ(assign.get_unassigned(0U), 2U);
  ASSERT_THROW(assign.get_unassigned(1U), std::range_error);
  ASSERT_THROW(assign.get_assignment(4U), std::range_error);
}

// exercise unbalanced logic in a complicated case
// http://naagustutorial.blogspot.com/2013/12/hungarian-method-unbalanced-assignment.html
TEST(HungarianAssigner, Unbalanced2)
{
  hungarian_assigner_c<16U> assign;
  assign.set_size(4U, 5U);
  const std::vector<std::vector<float32_t>> weights =
  {
    {4, 3, 6, 2, 7},
    {10, 12, 11, 14, 16},
    {4, 3, 2, 1, 5},
    {8, 7, 6, 9, 6}
  };
  set_weights(assign, weights);

  ASSERT_TRUE(assign.assign());
  ASSERT_EQ(assign.get_assignment(0U), 1U);
  ASSERT_EQ(assign.get_assignment(1U), 0U);
  ASSERT_EQ(assign.get_assignment(2U), 3U);
  ASSERT_EQ(assign.get_assignment(3U), 2U);
  ASSERT_EQ(assign.get_unassigned(0U), 4U);
  ASSERT_THROW(assign.get_unassigned(1U), std::range_error);
  ASSERT_THROW(assign.get_assignment(4U), std::range_error);
}

// test a complicated case
// http://file.scirp.org/pdf/AJOR_2016063017275082.pdf
TEST(HungarianAssigner, Complicated)
{
  hungarian_assigner_c<16U> assign;
  assign.set_size(8U, 10U);
  const std::vector<std::vector<float32_t>> weights =
  {
    {300, 290, 280, 290, 210, 300, 290, 280, 290, 210},
    {250, 310, 290, 300, 200, 250, 310, 290, 300, 200},
    {180, 190, 300, 190, 180, 180, 190, 300, 190, 180},
    {320, 180, 190, 240, 170, 320, 180, 190, 240, 170},
    {270, 210, 190, 250, 160, 270, 210, 190, 250, 160},
    {190, 200, 220, 190, 140, 190, 200, 220, 190, 140},
    {220, 300, 230, 180, 160, 220, 300, 230, 180, 160},
    {260, 190, 260, 210, 180, 260, 190, 260, 210, 180}
  };
  set_weights(assign, weights);

  ASSERT_TRUE(assign.assign());
  // modulo here since the problem is a machine (column) can do at most 2 jobs
  ASSERT_EQ(assign.get_assignment(0U) % 5U, 4U);
  ASSERT_EQ(assign.get_assignment(1U) % 5U, 0U);
  ASSERT_EQ(assign.get_assignment(2U) % 5U, 0U);
  ASSERT_EQ(assign.get_assignment(3U) % 5U, 1U);
  ASSERT_EQ(assign.get_assignment(4U) % 5U, 2U);
  ASSERT_EQ(assign.get_assignment(5U) % 5U, 4U);
  ASSERT_EQ(assign.get_assignment(6U) % 5U, 3U);
  ASSERT_EQ(assign.get_assignment(7U) % 5U, 1U);
  ASSERT_EQ(assign.get_unassigned(0U) % 5U, 2U);
  ASSERT_EQ(assign.get_unassigned(1U) % 5U, 3U);
  ASSERT_THROW(assign.get_unassigned(2U), std::range_error);
  ASSERT_THROW(assign.get_assignment(8U), std::range_error);
}

// simple case, test parallel fill
TEST(HungarianAssigner, Parallel)
{
  const uint64_t SZ = 256U;
  hungarian_assigner_c<SZ> assign;
  ASSERT_NO_THROW(assign.set_size(SZ, SZ));
  auto fn = [ =, &assign](const uint64_t row) {
      for (uint64_t idx = 0U; idx < SZ; ++idx) {
        const float32_t val = (idx == row) ? 0.0F : 10.0F;
        assign.set_weight(val, static_cast<int64_t>(row), static_cast<int64_t>(idx));
      }
    };
  std::vector<std::thread> threads(SZ);
  for (uint64_t idx = 0U; idx < SZ; ++idx) {
    threads[idx] = std::thread(fn, idx);
  }
  for (uint64_t idx = 0U; idx < SZ; ++idx) {
    threads[idx].join();
  }
  ASSERT_TRUE(assign.assign());
  for (uint64_t idx = 0U; idx < SZ; ++idx) {
    ASSERT_EQ(assign.get_assignment(static_cast<int64_t>(idx)), static_cast<int64_t>(idx));
  }
}

// solvable problem with missing links with one unique assignment
/*
0 1 2 3
0 1 2 -
0 1 - -
0 - - -
*/
TEST(HungarianAssigner, IllConditioned)
{
  hungarian_assigner_c<16U> assign;
  assign.set_size(4U, 4U);
  assign.set_weight(0.0F, 0U, 0U);
  assign.set_weight(1.0F, 0U, 1U);
  assign.set_weight(2.0F, 0U, 2U);
  assign.set_weight(3.0F, 0U, 3U);
  assign.set_weight(0.0F, 1U, 0U);
  assign.set_weight(1.0F, 1U, 1U);
  assign.set_weight(2.0F, 1U, 2U);
  assign.set_weight(0.0F, 2U, 0U);
  assign.set_weight(1.0F, 2U, 1U);
  assign.set_weight(0.0F, 3U, 0U);
  ASSERT_TRUE(assign.assign());
  ASSERT_EQ(assign.get_assignment(0U), 3U);
  ASSERT_EQ(assign.get_assignment(1U), 2U);
  ASSERT_EQ(assign.get_assignment(2U), 1U);
  ASSERT_EQ(assign.get_assignment(3U), 0U);
  ASSERT_THROW(assign.get_assignment(4U), std::range_error);
  ASSERT_THROW(assign.get_unassigned(0U), std::range_error);
}

// unsolvable problem
/*
1 1 1 1
1 - - -
1 - - -
1 - - -
*/
TEST(HungarianAssigner, Degenerate1)
{
  hungarian_assigner_c<16U> assign;
  assign.set_size(4U, 4U);
  assign.set_weight(0.0, 0U, 0U);
  assign.set_weight(0.0, 0U, 1U);
  assign.set_weight(0.0, 0U, 2U);
  assign.set_weight(0.0, 0U, 3U);
  assign.set_weight(0.0, 1U, 0U);
  assign.set_weight(0.0, 2U, 0U);
  assign.set_weight(0.0, 3U, 0U);
  EXPECT_TRUE(assign.assign());
  EXPECT_EQ(assign.get_assignment(0), 3);
  EXPECT_EQ(assign.get_assignment(1), assign.UNASSIGNED);
  EXPECT_EQ(assign.get_assignment(2), assign.UNASSIGNED);
  EXPECT_EQ(assign.get_assignment(3), 0);
}

TEST(HungarianAssigner, Degenerate2)
{
  hungarian_assigner_c<16U> assign;
  assign.set_size(4U, 4U);
  /*
  0 X X X
  X X X X
  X X X X
  X X X X
  */
  assign.set_weight(0, 0, 0);
  EXPECT_TRUE(assign.assign());
  EXPECT_EQ(assign.get_assignment(0), 0);
  EXPECT_EQ(assign.get_assignment(1), assign.UNASSIGNED);
  EXPECT_EQ(assign.get_assignment(2), assign.UNASSIGNED);
  EXPECT_EQ(assign.get_assignment(3), assign.UNASSIGNED);
  assign.reset(3, 3);
  /*
  X X 1
  X X 1
  1 X X
  */
  assign.set_weight(1.0, 0, 2);
  assign.set_weight(1.0, 1, 2);
  assign.set_weight(1.0, 2, 0);
  EXPECT_TRUE(assign.assign());
  EXPECT_EQ(assign.get_assignment(0), assign.UNASSIGNED);
  EXPECT_EQ(assign.get_assignment(1), 2);
  EXPECT_EQ(assign.get_assignment(2), 0);
  assign.reset(2, 2);
  /*
  X X
  X X
  */
  EXPECT_TRUE(assign.assign());
  EXPECT_EQ(assign.get_assignment(0), assign.UNASSIGNED);
  EXPECT_EQ(assign.get_assignment(1), assign.UNASSIGNED);
  assign.reset(1, 1);
  // [1]
  assign.set_weight(1, 0, 0);
  EXPECT_TRUE(assign.assign());
  EXPECT_EQ(assign.get_assignment(0), 0);
}

// semi-solvable problem
/*
1 2 X X
X 2 1 X
X X X X
2 1 X X
*/
TEST(HungarianAssigner, Partial)
{
  hungarian_assigner_c<16U> assign;
  assign.set_size(4U, 4U);
  assign.set_weight(1, 0U, 0U);
  assign.set_weight(2, 0U, 1U);
  assign.set_weight(2, 3U, 0U);
  assign.set_weight(1, 3U, 1U);
  assign.set_weight(2, 1U, 1U);
  assign.set_weight(1, 1U, 2U);
  ASSERT_TRUE(assign.assign());
  ASSERT_EQ(assign.get_assignment(0), 0);
  ASSERT_EQ(assign.get_assignment(1), 2);
  ASSERT_EQ(assign.get_assignment(2), assign.UNASSIGNED);
  ASSERT_EQ(assign.get_assignment(3), 1);
}

/*
   x  4 x 12 x
   x  0 x  8 x
   x  4 x  4 x
   x  8 x  0 x
   x 12 x  4 x
*/
TEST(HungarianAssigner, Degenerate3)
{
  hungarian_assigner_c<32U> assign;
//  assign.reset();
  assign.set_size(5U, 5U);
  assign.set_weight(4, 0U, 1U);
  assign.set_weight(0, 1U, 1U);
  assign.set_weight(4, 2U, 1U);
  assign.set_weight(8, 3U, 1U);
  assign.set_weight(12, 4U, 1U);
  assign.set_weight(12, 0U, 3U);
  assign.set_weight(8, 1U, 3U);
  assign.set_weight(4, 2U, 3U);
  assign.set_weight(0, 3U, 3U);
  assign.set_weight(4, 4U, 3U);

  EXPECT_EQ(assign.assign(), true);
  std::cout << "Running test " << std::endl;
  EXPECT_EQ(assign.get_assignment(0), assign.UNASSIGNED);
  EXPECT_EQ(assign.get_assignment(1), 1);
  EXPECT_EQ(assign.get_assignment(2), assign.UNASSIGNED);
  EXPECT_EQ(assign.get_assignment(3), 3);
  EXPECT_EQ(assign.get_assignment(4), assign.UNASSIGNED);
}

#endif  // TEST_HUNGARIAN_ASSIGNER_HPP_
