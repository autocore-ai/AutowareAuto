// Copyright 2021 Apex.AI, Inc.
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
#ifndef TRACKING__ROI_ASSOCIATION_HPP_
#define TRACKING__ROI_ASSOCIATION_HPP_

#include <tracking/visibility_control.hpp>
#include <geometry/intersection.hpp>

namespace autoware
{
namespace perception
{
namespace tracking
{

/// \brief Simple heuristic functor that returns the IoU between two shapes.
class IOUHeuristic
{
  /// \brief Get the match score of a projection and a roi
  /// \tparam Iterable1T A container class that has stl style iterators defined.
  /// \tparam Iterable2T A container class that has stl style iterators defined.
  /// \tparam Point1T Point type that have the adapters for the x and y fields.
  /// \tparam Point2T Point type that have the adapters for the x and y fields.
  /// \param shape1 Polygon 1
  /// \param shape2 Polygon 2
  /// \return The  IOU between two given shapes.
  template<template<typename ...> class Iterable1T,
    template<typename ...> class Iterable2T, typename Point1T, typename Point2T>
  common::types::float32_t operator()(
    const Iterable1T<Point1T> & shape1, const Iterable2T<Point2T> & shape2)
  {
    return common::geometry::convex_intersection_over_union_2d(shape1, shape2);
  }
};

}  // namespace tracking
}  // namespace perception
}  // namespace autoware

#endif  // TRACKING__ROI_ASSOCIATION_HPP_
