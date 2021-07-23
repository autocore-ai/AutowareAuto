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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <tracking_test_framework/shapes.hpp>
#include <tracking_test_framework/utils.hpp>
#include <helper_functions/float_comparisons.hpp>

#include <algorithm>
#include <vector>

namespace comparison = autoware::common::helper_functions::comparisons;
using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;


namespace autoware
{
namespace tracking_test_framework
{

Line::Line(const Eigen::Vector2f & start, const Eigen::Vector2f & end)
: m_start(start), m_end(end)
{
  this->m_line_length = (end - start).norm();
  this->m_line_direction = (end - start) / this->m_line_length;
}

EigenStlVector<Eigen::Vector2f> Line::intersect_with_line(
  const Line & line, const bool8_t) const
{
  /// If we represent two lines in the form of:
  /// \f$(𝑝_1 = \hat{𝑝_1} + 𝑡_1⋅𝑑_1)\f$  \f$(𝑝_2 = \hat{𝑝_2} + 𝑡_2⋅𝑑_2)\f$
  /// where  \f$(𝑝_1)\f$  and  \f$(𝑝_2)\f$  are arbitrary points on the given lines,
  /// \f$(\hat{𝑝_1})\f$  and  \f$(\hat{𝑝_2})\f$  are the starting points of the lines,
  /// \f$(𝑡_1)\f$  and  \f$(𝑡_2)\f$  are scalar scaling parameters that scale the corresponding
  /// direction unit vectors  \f$(𝑑_1)\f$  and  \f$(𝑑_2)\f$ , thus defining any point on the line.
  const Eigen::Vector2f p_delta = this->m_start - line.m_start;
  float32_t denominator = utils::cross_2d(this->m_line_direction, line.m_line_direction);

  // To prevent divide by zero error check if denominator is non-zero
  if (comparison::abs_eq_zero<float32_t>(denominator, 0.0001F)) {
    return EigenStlVector<Eigen::Vector2f>{};
  }

  float32_t t1 = utils::cross_2d(line.m_line_direction, p_delta) / denominator;
  float32_t t2 = utils::cross_2d(this->m_line_direction, p_delta) / denominator;

  const bool8_t t1_out_of_bounds = (t1<0.0F || t1> this->m_line_length);
  const bool8_t t2_out_of_bounds = (t2<0.0F || t2> line.m_line_length);

  if (t1_out_of_bounds || t2_out_of_bounds) {
    return EigenStlVector<Eigen::Vector2f>{};
  }
  return EigenStlVector<Eigen::Vector2f>{this->m_start + (t1 * this->m_line_direction)};
}

Eigen::Vector2f Line::get_point(const float32_t scale) const
{
  return this->m_start + scale * this->m_line_direction;
}


Rectangle::Rectangle(
  const Eigen::Vector2f & center, const Eigen::Vector2f & size, const float32_t orientation_degrees)
: m_center(center), m_size(size), m_orientation_radians(utils::to_radians(orientation_degrees))
{
  const Eigen::Rotation2D<float32_t> rotation(m_orientation_radians);
  const Eigen::Matrix2f rotation_matrix = rotation.toRotationMatrix();

  m_corners = {m_center + rotation_matrix * ((Eigen::Vector2f{-m_size[0], -m_size[1]} *0.5F)),
    m_center + rotation_matrix * ((Eigen::Vector2f{m_size[0], -m_size[1]} *0.5F)),
    m_center + rotation_matrix * ((Eigen::Vector2f{m_size[0], m_size[1]} *0.5F)),
    m_center + rotation_matrix * ((Eigen::Vector2f{-m_size[0], m_size[1]} *0.5F))};

  m_borders = {
    Line{m_corners[0], m_corners[1]},
    Line{m_corners[1], m_corners[2]},
    Line{m_corners[2], m_corners[3]},
    Line{m_corners[3], m_corners[0]}};
}

EigenStlVector<Eigen::Vector2f> Rectangle::intersect_with_line(
  const Line & line, const bool8_t closest_point_only) const
{
  ///  We can pass the rectangle borders to the Line::intersect_with_line function
  /// one by one to see if the line is intersecting any of the 4 sides of the rectangle and get
  /// the intersection points if any from there.
  EigenStlVector<Eigen::Vector2f> intersections{};
  for (const auto & border : this->m_borders) {
    const auto & current_intersection_vec = border.intersect_with_line(line, true);
    if (!current_intersection_vec.empty()) {
      intersections.emplace_back(current_intersection_vec[0]);
    }
  }
  if (intersections.empty()) {
    return EigenStlVector<Eigen::Vector2f>{};
  }
  if (closest_point_only) {
    sort(
      intersections.begin(), intersections.end(), [&](const Eigen::Vector2f & point_a,
      const Eigen::Vector2f & point_b) {
        return (point_a - line.starting_point()).norm() < (point_b -
        line.starting_point()).norm();
      });
    return EigenStlVector<Eigen::Vector2f>{intersections[0]};
  }
  return intersections;
}

Circle::Circle(const Eigen::Vector2f & center, const float32_t radius)
: m_center(center), m_radius(radius) {}


EigenStlVector<Eigen::Vector2f> Circle::intersect_with_line(
  const Line & line, const bool8_t closest_point_only) const
{
  /// Given a line in the form of  \f$(𝑝=𝑝_0+𝑡⋅𝑑)\f$ , where \f$(𝑝)\f$ is a point on a line,
  /// \f$(𝑝_0)\f$ is the 2D starting point of the line, \f$(𝑡)\f$ is a scale parameter and \f$
  /// (𝑑)\f$ is the normalized direction vector and a circle in the form of  \f$(𝑥−𝑥_c)^2+(𝑦−𝑦_c)
  /// ^2 = 𝑟^2\f$ , where \f$(𝑥,𝑦)\f$ are coordinates of a point on a circle,\f$(𝑥_c,𝑦_c)\f$ are
  /// the coordinates of the center of the circle and \f$(𝑟)\f$ is the radius of this circle, we
  /// can form a single equation from which we look for such values of \f$(𝑡)\f$ that the
  /// resulting point lies both on the point and on the circle.
  const Eigen::Vector2f delta = m_center - line.starting_point();
  const float32_t root_part = powf32(m_radius, 2) - powf32(
    utils::cross_2d(
      delta, line.direction()), 2);
  if (root_part < 0.0F) {
    return EigenStlVector<Eigen::Vector2f>();
  }
  const float32_t prefix_part = line.direction().transpose() * delta;
  std::vector<float32_t> distances {};
  if (comparison::abs_eq_zero<float32_t>(root_part, 0.0001F)) {
    distances.emplace_back(prefix_part);
  } else {
    distances.emplace_back(prefix_part + sqrtf32(root_part));
    distances.emplace_back(prefix_part - sqrtf32(root_part));
  }
  /// Sort the intersections with closest being the first
  std::sort(distances.begin(), distances.end());

  /// If invalid distance return empty vector
  if (distances[0] < 0.0F || distances[0] > line.length()) {
    return EigenStlVector<Eigen::Vector2f>{};
  }

  EigenStlVector<Eigen::Vector2f> intersections{};
  if (closest_point_only) {
    intersections.emplace_back(line.get_point(distances[0]));
  } else {
    for (const auto & scale  : distances) {
      intersections.emplace_back(line.get_point(scale));
    }
  }
  return intersections;
}


}  // namespace tracking_test_framework
}  // namespace autoware
