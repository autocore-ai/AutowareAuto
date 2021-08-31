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

// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef TRACKING__PROJECTION_HPP_
#define TRACKING__PROJECTION_HPP_

#include <autoware_auto_msgs/msg/shape.hpp>
#include <common/types.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry/convex_hull.hpp>
#include <geometry/common_2d.hpp>
#include <Eigen/Geometry>
#include <geometry/interval.hpp>
#include <experimental/optional>
#include <tracking/visibility_control.hpp>
#include <list>
#include <algorithm>
#include <vector>

namespace autoware
{
namespace perception
{
namespace tracking
{
struct TRACKING_PUBLIC Projection
{
  using Point = geometry_msgs::msg::Point32;
  std::list<Point> shape{};
};

/// \brief Camera intrinsic parameters
struct TRACKING_PUBLIC CameraIntrinsics
{
  std::size_t width{0U};
  std::size_t height{0U};
  float32_t fx{0.0F};
  float32_t fy{0.0F};
  float32_t ox{0.0F};
  float32_t oy{0.0F};
  float32_t skew{0.0F};
};

/// \brief This model represents a camera in 3D space and can project 3D shapes into an image
// frame.
class TRACKING_PUBLIC CameraModel
{
public:
  using float32_t = common::types::float32_t;
  using Interval = common::geometry::Interval<float32_t>;
  using Point = geometry_msgs::msg::Point32;
  using EigPoint = Eigen::Vector3f;

  /// \brief Cnstructor
  /// \param intrinsics Camera intrinsics
  explicit CameraModel(
    const CameraIntrinsics & intrinsics
  );

  /// \brief Bring 3D points in ego frame to the camera frame and project onto the image plane.
  /// All 3D points in a shape (vertices on the bottom face and the top face) are projected onto
  // the image of the camera model. The resulting list of points are then outlined using the
  // `convex_hull` algorithm and then the outline of points are returned.
  /// `K * p_3d = p_2d * depth` where K is the projection matrix.
  /// \param points 3D set of points in the camera frame.
  /// \return List of points on the camera frame that outline the given 3D object.
  std::experimental::optional<Projection>
  project(const std::vector<Point> & points) const;

private:
  /// \brief Project a 3D point and return the value if the projection is valid (in fron of the
  // camera)
  std::experimental::optional<EigPoint> project_point(const EigPoint & pt_3d) const;

  Eigen::Matrix3f m_intrinsics;
  Interval m_height_interval;
  Interval m_width_interval;
  std::list<Point> m_corners;
};

}  // namespace tracking
}  // namespace perception
}  // namespace autoware


#endif  // TRACKING__PROJECTION_HPP_
