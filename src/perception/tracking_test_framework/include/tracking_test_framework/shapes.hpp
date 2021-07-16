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

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the tracking_test_framework classes which are responsible for
/// generating the 2D objects for unit testing tracking algorithms.

#ifndef TRACKING_TEST_FRAMEWORK__SHAPES_HPP_
#define TRACKING_TEST_FRAMEWORK__SHAPES_HPP_

#include <tracking_test_framework/visibility_control.hpp>
#include <tracking_test_framework/eigen_stl_vector.hpp>

#include <common/types.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <memory>
#include <vector>

namespace autoware
{
namespace tracking_test_framework
{

/// Forward declaration of the Line class
class TRACKING_TEST_FRAMEWORK_PUBLIC Line;


/// \brief This is the base class for the 2D shapes
class TRACKING_TEST_FRAMEWORK_PUBLIC Shape
{
public:
  /// \brief virtual method to get intersection points between the line and different shapes
  /// \param[in] line the Line object
  /// \param[in] closest_point_only the boolean to determine if closest intersection to be
  /// returned or all
  /// \return returns the intersection points
  virtual EigenStlVector<Eigen::Vector2f> intersect_with_line(
    const Line & line, const autoware::common::types::bool8_t closest_point_only) const = 0;

  /// \brief Virtual destructor
  virtual ~Shape() = default;
};

/// \brief This is the class which represents the line and defines the function
/// intersect_with_line which gives the intersection points with another line
class TRACKING_TEST_FRAMEWORK_PUBLIC Line : public Shape
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// \brief default constructor
  Line() = default;

  /// \brief constructor
  Line(const Eigen::Vector2f & start, const Eigen::Vector2f & end);

  /// \brief Method to get intersection points of a line with another line
  /// \param[in] line the Line object
  /// \param[in] closest_point_only the boolean to determine if closest intersection to be
  /// returned or all
  /// \return returns the intersection points
  EigenStlVector<Eigen::Vector2f> intersect_with_line(
    const Line & line, const autoware::common::types::bool8_t closest_point_only) const override;

  /// \brief gets the point on the line at the input scale
  /// \param[in] scale the scalar qty `t` representing the scaling factor for the line in
  /// equation :
  /// \f$(p = p^ + t.d)\f$ where \f$(p^)\f$ is the unit vector for starting point of the line and
  /// \f$(d)\f$ is the direction unit vector. if \f$(||d|| == 1)\f$ then the scale becomes a
  /// signed distance along the line
  /// \return returns the point on the line
  Eigen::Vector2f get_point(const autoware::common::types::float32_t scale) const;

  /// \brief gets the starting point of the line
  /// \return returns the starting point
  inline const Eigen::Vector2f & starting_point() const
  {
    return m_start;
  }

  /// \brief gets the end point of the line
  /// \return returns the end point
  inline const Eigen::Vector2f & end_point() const
  {
    return m_end;
  }

  /// \brief gets the line direction
  /// \return returns the line direction
  inline const Eigen::Vector2f & direction() const
  {
    return m_line_direction;
  }

  /// \brief gets the line length
  /// \return returns the line length
  inline const autoware::common::types::float32_t & length() const
  {
    return m_line_length;
  }

private:
  /// starting point of the line \f$(x_s,y_s)\f$
  Eigen::Vector2f m_start{Eigen::Vector2f::Zero()};
  /// ending point of the line \f$(x_e,y_e)\f$
  Eigen::Vector2f m_end{Eigen::Vector2f::Zero()};
  /// line length
  autoware::common::types::float32_t m_line_length{};
  /// vector representing direction of line
  Eigen::Vector2f m_line_direction{Eigen::Vector2f::Zero()};
};

/// \brief This is the class which represents the cars as rectangles in 2D and defines the function
/// intersect_with_line which gives the intersection points of the rectangle with the line
class TRACKING_TEST_FRAMEWORK_PUBLIC Rectangle : public Shape
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// \brief constructor
  Rectangle(
    const Eigen::Vector2f & center, const Eigen::Vector2f & size,
    const autoware::common::types::float32_t orientation_degrees);

  /// \brief Method to get intersection points with rectangle and the input line
  /// \param[in] line the Line object
  /// \param[in] closest_point_only the boolean to determine if closest intersection to be
  /// returned or all
  /// \return returns the intersection points
  EigenStlVector<Eigen::Vector2f> intersect_with_line(
    const Line & line, const autoware::common::types::bool8_t closest_point_only) const override;

private:
  /// center of the rectangle \f$(x_r,y_r)\f$
  Eigen::Vector2f m_center{Eigen::Vector2f::Zero()};
  /// size of the rectangle \f$(l ,b)\f$
  Eigen::Vector2f m_size{Eigen::Vector2f::Zero()};
  /// orientation of the rectangle in radians
  autoware::common::types::float32_t m_orientation_radians{};
  /// array representing the four borders of the rectangle
  std::array<Line, 4> m_borders{};
  /// array representing the four corner points of the rectangle
  std::array<Eigen::Vector2f, 4> m_corners{};
};

/// \brief This is the class which represents the pedestrians as circles in 2D and defines the
/// function intersect_with_line which gives the intersection points of circle with a line
class TRACKING_TEST_FRAMEWORK_PUBLIC Circle : public Shape
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// \brief constructor
  Circle(const Eigen::Vector2f & center, const autoware::common::types::float32_t radius);

  /// \brief Method to get intersection points of circle and the line
  /// \param[in] line the Line object
  /// \param[in] closest_point_only the boolean to determine if closest intersection to be
  /// returned or all
  /// \return returns the intersection points
  EigenStlVector<Eigen::Vector2f> intersect_with_line(
    const Line & line, const autoware::common::types::bool8_t closest_point_only) const override;

private:
  /// center of the circle \f$(x_c,y_c)\f$
  Eigen::Vector2f m_center{Eigen::Vector2f::Zero()};
  /// radius of the circle (r)
  autoware::common::types::float32_t m_radius{};
};

}  // namespace tracking_test_framework
}  // namespace autoware
#endif  // TRACKING_TEST_FRAMEWORK__SHAPES_HPP_
