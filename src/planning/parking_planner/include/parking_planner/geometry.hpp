// Copyright 2020 Embotech AG, Zurich, Switzerland. All rights reserved.
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

// This file defines 2D points, halfplanes and polyhedra as templates. The templates
// work both for concrete numerical types such as float and double as well as for
// "symbol" types such as CasADi variables. The advantage of this is that they can
// be used both when formulating constraints for an optimization problem as well
// as for numerical evaluation outside of such formulations.

#ifndef PARKING_PLANNER__GEOMETRY_HPP_
#define PARKING_PLANNER__GEOMETRY_HPP_

#include <vector>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace autoware
{

namespace motion
{

namespace planning
{

namespace parking_planner
{

// -- Point2D -----------------------------------------------------------------
/// \brief Class describing a point in two-dimensional space
template<typename T>
class Point2D
{
public:
  /// \brief No-argument constructor
  Point2D() noexcept
  {
    this->m_coord = std::pair<T, T>(T{}, T{});
  }

  /// \brief Scalar type Constructor
  Point2D(T x, T y) noexcept
  {
    this->m_coord = std::pair<T, T>(x, y);
  }

  /// \brief Check for equality of two points
  bool operator==(const Point2D<T> & other) const noexcept
  {
    return this->m_coord == other.m_coord;
  }

  /// \brief Add two points
  Point2D<T> operator+(const Point2D<T> & other) const noexcept
  {
    return Point2D<T>(
      this->m_coord.first + other.m_coord.first,
      this->m_coord.second + other.m_coord.second);
  }

  /// \brief Subtract one point from the other
  Point2D<T> operator-(const Point2D<T> & other) const noexcept
  {
    return Point2D<T>(
      this->m_coord.first - other.m_coord.first,
      this->m_coord.second - other.m_coord.second);
  }

  /// \brief Divide by a constant
  Point2D<T> operator/(const T dividend) const
  {
    if (dividend == T{}) {
      throw std::domain_error{"Divide by zero encountered"};
    }

    return Point2D<T>(
      this->m_coord.first / dividend,
      this->m_coord.second / dividend);
  }

  /// \brief Multiply by something
  Point2D<T> operator*(const T multiplier) const noexcept
  {
    return Point2D<T>(
      this->m_coord.first * multiplier,
      this->m_coord.second * multiplier);
  }

  /// \brief Compute the 2-norm of a point (as in, the distance to the origin)
  T norm2() const noexcept
  {
    return sqrt(this->dot(*this));
  }

  /// \brief Perform inner-product
  T dot(const Point2D<T> & other) const noexcept
  {
    return this->m_coord.first * other.m_coord.first +
           this->m_coord.second * other.m_coord.second;
  }

  /// \brief Rotate around the origin
  void rotate(const T angle) noexcept
  {
    T cos_angle = cos(angle);
    T sin_angle = sin(angle);
    T x = this->m_coord.first;
    T y = this->m_coord.second;
    this->m_coord.first = x * cos_angle - y * sin_angle;
    this->m_coord.second = x * sin_angle + y * cos_angle;
  }

  /// \brief Get the coordinates
  const std::pair<T, T> & get_coord() const noexcept
  {
    return this->m_coord;
  }

private:
  /// \brief Less-equal (only used for sorting)
  bool operator<=(const Point2D<T> & other) const noexcept
  {
    return this->m_coord <= other.m_coord;
  }

  /// \brief Less (only used for sorting)
  bool operator<(const Point2D<T> & other) const noexcept
  {
    return this->m_coord <= other.m_coord;
  }

  /// \brief Greater-equal (only used for sorting)
  bool operator>=(const Point2D<T> & other) const noexcept
  {
    return this->m_coord <= other.m_coord;
  }

  /// \brief Greater (only used for sorting)
  bool operator>(const Point2D<T> & other) const noexcept
  {
    return this->m_coord <= other.m_coord;
  }

  std::pair<T, T> m_coord;
};

// -- Halfplane2D -------------------------------------------------------------
/// \brief Class describing a halfplane in two-dimensional space
template<typename T>
class Halfplane2D
{
public:
  Halfplane2D(const Point2D<T> & coefficients, T right_hand_side)
  {
    this->m_coefficients = coefficients;
    this->m_right_hand_side = right_hand_side;
    this->normalize();
  }

  /// \brief Check if a given point is in the halfplane or not
  template<typename S = T, std::enable_if_t<std::is_arithmetic<S>::value, int> = 0>
  bool contains_point(const Point2D<T> & point) const noexcept
  {
    return this->m_coefficients.dot(point) <= this->m_right_hand_side;
  }

  /// \brief Get the coefficients of the left-hand side of the halfplane equation
  const Point2D<T> & get_coefficients() const noexcept
  {
    return this->m_coefficients;
  }

  /// \brief Get the right-hand side constant of the halfplane equation
  const T & get_right_hand_side() const noexcept
  {
    return this->m_right_hand_side;
  }

  /// \brief Turn this halfplane into a vector of scalars
  std::vector<T> serialize() const
  {
    std::vector<T> serialized{};
    serialized.reserve(internal_scalars_number);
    const auto & coordinates = m_coefficients.get_coord();
    serialized.push_back(coordinates.first);
    serialized.push_back(coordinates.second);
    serialized.push_back(m_right_hand_side);
    return serialized;
  }

  /// \brief Query how many scalars are involved in serializing this halfplane
  static constexpr std::size_t get_serialized_length() noexcept
  {
    return internal_scalars_number;
  }

private:
  /// Number of internal scalars, update this if that number changes
  static constexpr std::size_t internal_scalars_number = 3;

  Point2D<T> m_coefficients;
  T m_right_hand_side;

  // \brief  This template may be used for concrete number types as well as for symbolic
  //         variable objects. In the case of symbolic variable objects, normalization
  //         is not desired, and the norm check makes no sense anyway. The enable_if_t
  //         feature gating here turns the normalization on only if there is a concrete
  //         numeric type. If not, then the variant below that does nothing is used.
  template<typename S = T, std::enable_if_t<std::is_arithmetic<S>::value, int> = 0>
  void normalize()
  {
    S two_norm = this->m_coefficients.norm2();
    if (two_norm < std::numeric_limits<S>::epsilon()) {
      throw std::domain_error{"Divide by zero encountered"};
    }
    this->m_coefficients = this->m_coefficients / two_norm;
    this->m_right_hand_side = this->m_right_hand_side / two_norm;
  }

  // This case is used for symbolic template parameters T
  template<typename S = T, std::enable_if_t<!std::is_arithmetic<S>::value, int> = 0>
  void normalize() noexcept
  {
  }
};


// -- Polytope2D --------------------------------------------------------------
/// \brief Class describing a polytope in two-dimensional space
template<typename T>
class Polytope2D
{
public:
  /// \brief Construct a polyhedron
  /// \param[in] vertices vector of vertices, assumed to be "ordered anti-clockwise"
  ///            with respect to their "center of mass"
  explicit Polytope2D(const std::vector<Point2D<T>> & vertices) noexcept
  {
    this->m_vertices = vertices;
    this->update_halfplanes_from_vertices();
  }

  /// \brief Rotate and shift the polytope
  void rotate_and_shift(
    const T rotation_angle,
    const Point2D<T> & rotation_center,
    const Point2D<T> & shift_vector)
  {
    for (auto & v : this->m_vertices) {
      v = v - rotation_center;
      v.rotate(rotation_angle);
      v = v + rotation_center;
      v = v + shift_vector;
    }
    this->update_halfplanes_from_vertices();
  }

  /// \brief Check if polytope contains a point
  bool contains_point(const Point2D<T> & point) const noexcept
  {
    for (const auto & halfplane : this->m_halfplanes) {
      if (!halfplane.contains_point(point)) {
        return false;
      }
    }
    return true;
  }

  /// \brief Check if polytope intersects with another
  bool intersects_with(const Polytope2D<T> & other) const noexcept
  {
    for (const auto & halfplane : other.m_halfplanes) {
      bool contains_any_point = false;
      for (const auto & vertex : this->m_vertices) {
        if (halfplane.contains_point(vertex)) {
          contains_any_point = true;
        }
      }
      if (!contains_any_point) {
        return false;
      }
    }

    for (const auto & halfplane : this->m_halfplanes) {
      bool contains_any_point = false;
      for (const auto & vertex : other.m_vertices) {
        if (halfplane.contains_point(vertex)) {
          contains_any_point = true;
        }
      }
      if (!contains_any_point) {
        return false;
      }
    }

    return true;
  }

  /// \brief Getter for halfplanes
  const std::vector<Halfplane2D<T>> & get_halfplanes() const noexcept
  {
    return this->m_halfplanes;
  }

  /// \brief Getter for vertices
  const std::vector<Point2D<T>> & get_vertices() const noexcept
  {
    return this->m_vertices;
  }

private:
  void update_vertices_from_halfplanes() noexcept
  {
    this->m_vertices.clear();
    const auto it_begin = this->m_halfplanes.begin();
    const auto it_end = this->m_halfplanes.end();
    auto it_last = it_end;
    for (auto it_current = it_begin; it_current != it_end; ++it_current) {
      const Point2D<T> & coeff_last = it_last->get_coefficients();
      const T & rhs_last = it_last->get_right_hand_side();
      const Point2D<T> & coeff_current = it_current->get_coefficients();
      const T & rhs_current = it_current->get_right_hand_side();
      T x = coeff_current.get_coord().second * rhs_last -
        coeff_last.get_coord().second * rhs_current;
      T y = -coeff_last.get_coord().first * rhs_last +
        coeff_current.get_coord().first * rhs_current;
      this->m_vertices.push_back(Point2D<T>(x, y));
      it_last = it_current;
    }
  }

  void update_halfplanes_from_vertices() noexcept
  {
    this->m_halfplanes.clear();
    const auto it_begin = this->m_vertices.begin();
    auto it_last = it_begin;
    const auto it_end = this->m_vertices.end();
    if (it_begin != it_end) {
      for (auto it = it_begin + 1; it != it_end; ++it) {
        const Point2D<T> & vertex_last = *it_last;
        const Point2D<T> & vertex_current = *it;
        Point2D<T> diff_vector = vertex_current - vertex_last;
        Point2D<T> norm_vector = Point2D<T>(
          diff_vector.get_coord().second,
          -diff_vector.get_coord().first);
        this->m_halfplanes.push_back(Halfplane2D<T>(norm_vector, norm_vector.dot(vertex_current)));
        it_last = it;
      }
      const Point2D<T> & vertex_last = *it_last;
      const Point2D<T> & vertex_current = *it_begin;
      Point2D<T> diff_vector = vertex_current - vertex_last;
      Point2D<T> norm_vector = Point2D<T>(
        diff_vector.get_coord().second,
        -diff_vector.get_coord().first);
      this->m_halfplanes.push_back(Halfplane2D<T>(norm_vector, norm_vector.dot(vertex_current)));
    }
  }

  // These representations should be kept consistent, i.e. whenever one is
  // set the other needs to be updated. This is done by all the methods at
  // this point, but should further methods be added, they also need to take
  // care of this. The two helper methods above can be used for this.

  /// Vertex representation of the polytope
  std::vector<Point2D<T>> m_vertices;

  /// Halfplane representation of the polytope
  std::vector<Halfplane2D<T>> m_halfplanes;
};


}  // namespace parking_planner
}  // namespace planning
}  // namespace motion
}  // namespace autoware

#endif  // PARKING_PLANNER__GEOMETRY_HPP_
