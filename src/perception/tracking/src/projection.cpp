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

#include <tracking/projection.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <autoware_auto_tf2/tf2_autoware_auto_msgs.hpp>
#include <geometry/intersection.hpp>
#include <geometry/common_2d.hpp>
#include <algorithm>

namespace autoware
{
namespace perception
{
namespace tracking
{
CameraModel::CameraModel(
  const CameraIntrinsics & intrinsics,
  const geometry_msgs::msg::Transform & tf_camera_from_ego
)
: CameraModel(intrinsics, tf2::transformToEigen(tf_camera_from_ego).cast<float32_t>()) {}

CameraModel::CameraModel(
  const CameraIntrinsics & intrinsics,
  const Eigen::Transform<float32_t, 3U, Eigen::Affine> & tf_camera_from_ego
)
: m_height_interval{-static_cast<float32_t>(intrinsics.height) / 2.0F,
    static_cast<float32_t>(intrinsics.height) / 2.0F},
  m_width_interval{-static_cast<float32_t>(intrinsics.width) / 2.0F,
    static_cast<float32_t>(intrinsics.width) / 2.0F},
  m_corners{
    Point{}.set__x(Interval::min(m_width_interval))
    .set__y(Interval::max(m_height_interval)).set__z(0.0F),
    Point{}.set__x(Interval::max(m_width_interval))
    .set__y(Interval::max(m_height_interval)).set__z(0.0F),
    Point{}.set__x(Interval::max(m_width_interval))
    .set__y(Interval::min(m_height_interval)).set__z(0.0F),
    Point{}.set__x(Interval::min(m_width_interval))
    .set__y(Interval::min(m_height_interval)).set__z(0.0F)
  }
{
  Eigen::Matrix3f intrinsic_matrix{};
  intrinsic_matrix <<
    intrinsics.fx, intrinsics.skew, intrinsics.ox,
    0.0, intrinsics.fy, intrinsics.oy,
    0.0, 0.0, 1.0;
  m_projector = intrinsic_matrix * tf_camera_from_ego;
}

std::experimental::optional<CameraModel::EigPoint>
CameraModel::project_point(const EigPoint & pt_3d) const
{
  // `m_projector * p_3d = p_2d * depth`
  const auto pt_2d = m_projector * pt_3d;
  const auto depth = pt_2d.z();
  // Only accept points are in front of the camera lens
  if (depth > 0.0F) {
    return pt_2d / depth;
  }
  return std::experimental::nullopt;
}

std::experimental::optional<Projection> CameraModel::project(
  const autoware_auto_msgs::msg::Shape & shape) const
{
  Projection result;
  const auto & points_3d = shape.polygon.points;
  auto & points2d = result.shape;

  const auto project_and_append = [&points2d, this](auto x, auto y, auto z) {
      const auto projected_pt = project_point(Eigen::Vector3f{x, y, z});
      if (projected_pt) {
        points2d.emplace_back(
          Point{}.set__x(projected_pt->x()).
          set__y(projected_pt->y()).set__z(projected_pt->z()));
      }
    };

  for (const auto & pt : points_3d) {
    project_and_append(pt.x, pt.y, pt.z);
    project_and_append(pt.x, pt.y, pt.z + shape.height);
  }

  // Outline the shape of the projected points in the image
  const auto & end_of_shape_it = common::geometry::convex_hull(points2d);
  // Discard the internal points of the shape
  points2d.resize(static_cast<uint32_t>(std::distance(points2d.cbegin(), end_of_shape_it)));

  result.shape = common::geometry::convex_polygon_intersection2d(m_corners, points2d);
  return is_projection_valid(result) ?
         std::experimental::make_optional(result) :
         std::experimental::nullopt;
}

bool CameraModel::is_projection_valid(const Projection & projection) const noexcept
{
  return projection.shape.size() >= 3U;
}


}  // namespace tracking
}  // namespace perception
}  // namespace autoware
