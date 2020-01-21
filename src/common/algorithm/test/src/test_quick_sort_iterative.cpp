// Copyright 2017-2019 Apex.AI, Inc.
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

#include "autoware_auto_algorithm/algorithm.hpp"
#include <gtest/gtest.h>

template <typename Container, typename RandomIt = typename Container::iterator>
using QuickSorter = ::autoware::common::algorithm::QuickSorter<Container, RandomIt>;

TEST(quick_sort_iterative, empty) {
  ::std::vector<int> vector;
  QuickSorter<::std::vector<int>> sorter;
  sorter.sort(vector.begin(), vector.end());
  ASSERT_EQ(vector, ::std::vector<int>({}));
  ASSERT_EQ(sorter.capacity(), 0);
}

TEST(quick_sort_iterative, single_elem) {
  ::std::vector<int> vector = {42};
  QuickSorter<::std::vector<int>> sorter(vector.capacity());
  ASSERT_EQ(sorter.capacity(), 1);
  sorter.sort(vector.begin(), vector.end());
  ASSERT_EQ(vector, ::std::vector<int>({42}));
  ASSERT_EQ(sorter.capacity(), 1);
}

TEST(quick_sort_iterative, two_elems) {
  ::std::vector<int> vector = {42, 43};
  QuickSorter<::std::vector<int>> sorter(vector.capacity());
  ASSERT_EQ(sorter.capacity(), 2);
  sorter.sort(vector.begin(), vector.end());
  ASSERT_EQ(vector, ::std::vector<int>({42, 43}));
  ASSERT_EQ(sorter.capacity(), 2);
}

TEST(quick_sort_iterative, already_sorted) {
  ::std::vector<int> vector = {1, 2, 3, 4, 5, 6};
  QuickSorter<::std::vector<int>> sorter(vector.capacity());
  ASSERT_EQ(sorter.capacity(), 6);
  sorter.sort(vector.begin(), vector.end());
  ASSERT_EQ(vector, ::std::vector<int>({1, 2, 3, 4, 5, 6}));
  ASSERT_EQ(sorter.capacity(), 6);
}

TEST(quick_sort_iterative, descending) {
  ::std::vector<int> vector = {6, 5, 4, 3, 2, 1};
  QuickSorter<::std::vector<int>> sorter(vector.capacity());
  ASSERT_EQ(sorter.capacity(), 6);
  sorter.sort(vector.begin(), vector.end());
  ASSERT_EQ(vector, ::std::vector<int>({1, 2, 3, 4, 5, 6}));
  ASSERT_EQ(sorter.capacity(), 6);
}

TEST(quick_sort_iterative, random) {
  ::std::vector<int> vector = {3, 5, 1, 6, 4, 2};
  QuickSorter<::std::vector<int>> sorter(vector.capacity());
  ASSERT_EQ(sorter.capacity(), 6);
  sorter.sort(vector.begin(), vector.end());
  ASSERT_EQ(vector, ::std::vector<int>({1, 2, 3, 4, 5, 6}));
  ASSERT_EQ(sorter.capacity(), 6);
}

TEST(quick_sort_iterative, sub_range_begin) {
  ::std::vector<int> vector = {3, 5, 1, 6, 4, 2};
  QuickSorter<::std::vector<int>> sorter(vector.capacity());
  ASSERT_EQ(sorter.capacity(), 6);
  sorter.sort(vector.begin(), vector.end()-2);
  ASSERT_EQ(vector, ::std::vector<int>({1, 3, 5, 6, 4, 2}));
  ASSERT_EQ(sorter.capacity(), 6);
}

TEST(quick_sort_iterative, sub_range_end) {
  ::std::vector<int> vector = {3, 5, 1, 6, 4, 2};
  QuickSorter<::std::vector<int>> sorter(vector.capacity());
  ASSERT_EQ(sorter.capacity(), 6);
  sorter.sort(vector.begin()+2, vector.end());
  ASSERT_EQ(vector, ::std::vector<int>({3, 5, 1, 2, 4, 6}));
  ASSERT_EQ(sorter.capacity(), 6);
}

TEST(quick_sort_iterative, sub_range) {
  ::std::vector<int> vector = {3, 5, 1, 6, 4, 2};
  QuickSorter<::std::vector<int>> sorter(vector.capacity());
  ASSERT_EQ(sorter.capacity(), 6);
  sorter.sort(vector.begin()+1, vector.end()-3);
  ASSERT_EQ(vector, ::std::vector<int>({3, 1, 5, 6, 4, 2}));
  ASSERT_EQ(sorter.capacity(), 6);
}