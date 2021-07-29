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

#include <gtest/gtest.h>
#include <tracking/projection.hpp>
#include <autoware_auto_msgs/msg/shape.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <algorithm>
#include <set>
#include <limits>

using Shape = autoware_auto_msgs::msg::Shape;
using Polygon = Shape::_polygon_type;
using Point32 = geometry_msgs::msg::Point32;
using CameraModel = autoware::perception::tracking::CameraModel;
using CameraIntrinsics = autoware::perception::tracking::CameraIntrinsics;
using Projection = autoware::perception::tracking::Projection;
using autoware::common::geometry::point_adapter::x_;
using autoware::common::geometry::point_adapter::y_;
using autoware::common::geometry::point_adapter::z_;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

class PrismProjectionTest : public testing::Test
{
public:
  PrismProjectionTest()
  {
    auto & outline = rectangular_prism.polygon.points;

    outline.emplace_back(
      geometry_msgs::build<Point32>().
      x(half_width).y(half_length).z(distance_from_origin));
    outline.emplace_back(
      geometry_msgs::build<Point32>().
      x(half_width).y(-half_length).z(distance_from_origin));
    outline.emplace_back(
      geometry_msgs::build<Point32>().
      x(-half_width).y(half_length).z(distance_from_origin));
    outline.emplace_back(
      geometry_msgs::build<Point32>().
      x(-half_width).y(-half_length).z(distance_from_origin));

    rectangular_prism.set__height(height);
  }
  float32_t half_length{1.0F};
  float32_t half_width{1.0F};
  float32_t height{4.0F};

  float32_t distance_from_origin{8.0F};
  Shape rectangular_prism;
};

void compare_shapes(
  const geometry_msgs::msg::Polygon & prism_face,
  const autoware::perception::tracking::Projection & projection,
  const CameraIntrinsics & intrinsics)
{
  for (auto i = 0U; i < prism_face.points.size(); ++i) {
    const auto & pt_3d = prism_face.points[i];
    const auto expected_projected_x =
      (pt_3d.x * intrinsics.fx + pt_3d.y * intrinsics.skew + pt_3d.z * intrinsics.ox) / pt_3d.z;
    const auto expected_projected_y =
      (pt_3d.y * intrinsics.fy + pt_3d.z * intrinsics.oy) / pt_3d.z;

    const auto projected_pt_it = std::find_if(
      projection.shape.begin(), projection.shape.end(),
      [expected_projected_x, expected_projected_y](const auto & pt) {
        constexpr auto eps = std::numeric_limits<float32_t>::epsilon();
        return autoware::common::helper_functions::comparisons::abs_eq(
          (x_(pt) - expected_projected_x), 0.0F, eps) &&
        autoware::common::helper_functions::comparisons::abs_eq(
          (y_(pt) - expected_projected_y), 0.0F, eps);
      });
    EXPECT_NE(projected_pt_it, projection.shape.end());
  }
}

/// \brief Project the prism's bottom face to the image plane. Since the prism and the camera
/// are on the same coordinate frame, manually projected points of the prism's bottom face should
// coincide with the output of the camera model's projection.

/*
 *
•••••••••
••      ••
• •     • •
•  •    •  •
•   •   •   •
••••••••••••••
 •   •   •   •
  •  •    •  •
   • •     • •
    ••      ••
     •••••••••

    \   /
      • Camera

 * */
TEST_F(PrismProjectionTest, camera_frame_projection_test) {
  CameraIntrinsics intrinsics{100, 100, 1.0F, 1.0F};
  geometry_msgs::msg::Transform identity{};
  identity.rotation.set__w(1.0);
  CameraModel model{intrinsics, identity};
  auto projection = model.project(rectangular_prism);
  compare_shapes(rectangular_prism.polygon, projection, intrinsics);
}

/// \brief Project the prism's face on positive X direciton to the image plane of a camera that
// is on the right of the prism.
/*
•••••••••
••      ••
• •     • •
•  •    •  •
•   •   •   •
••••••••••••••
 •   •   •   •        \
  •  •    •  •          • Camera
   • •     • •        /
    ••      ••
     •••••••••
 * */
TEST_F(PrismProjectionTest, transformed_camera_frame_test) {
  CameraIntrinsics intrinsics{100, 100, 0.6F, 1.5F, 0.01F, 0.01F, 0.005F};
  // Define where the camera is with respect to the origin
  Eigen::Transform<float32_t, 3U, Eigen::Affine> tf_ego_from_camera;
  tf_ego_from_camera.setIdentity();
  // Translate the camera to view the prism from the side. It's placed to align with the center
  // of the prism
  tf_ego_from_camera.translate(
    Eigen::Vector3f{
    distance_from_origin, 0.0F, distance_from_origin + height / 2.F});
  // The Z axis is looking up, the camera should be rotated over the Y axis to view the prism.
  tf_ego_from_camera.rotate(
    Eigen::AngleAxisf(autoware::common::types::PI / 2.0F, -Eigen::Vector3f::UnitY()));
  // Invert the transform as the camera model requires ego->camera transform to bring points into
  // the camera frame before projecting.
  const auto tf_camera_from_ego = tf_ego_from_camera.inverse();
  const auto transform =
    tf2::eigenToTransform(tf_camera_from_ego.cast<float64_t>()).transform;

  CameraModel model{intrinsics, transform};
  auto projection = model.project(rectangular_prism);

  // Manually construct the side face of the rectangle on the camera frame.
  geometry_msgs::msg::Polygon x_facade_on_camera_frame;
  const auto x_facade_depth = distance_from_origin - half_width;
  const auto half_height = 0.5F * height;
  x_facade_on_camera_frame.points.push_back(
    Point32{}.set__x(half_height).set__y(half_length).set__z(x_facade_depth));
  x_facade_on_camera_frame.points.push_back(
    Point32{}.set__x(half_height).set__y(-half_length).set__z(x_facade_depth));
  x_facade_on_camera_frame.points.push_back(
    Point32{}.set__x(-half_height).set__y(half_length).set__z(x_facade_depth));
  x_facade_on_camera_frame.points.push_back(
    Point32{}.set__x(-half_height).set__y(-half_length).set__z(x_facade_depth));

  compare_shapes(x_facade_on_camera_frame, projection, intrinsics);
}

/// \brief Test to validate that objects behind the camera are not captured.
TEST_F(PrismProjectionTest, behind_the_image_plane_test) {
  CameraIntrinsics intrinsics{100, 100, 1.0F, 1.0F};
  geometry_msgs::msg::Transform identity{};
  identity.rotation.set__w(1.0);
  CameraModel model{intrinsics, identity};
  std::transform(
    rectangular_prism.polygon.points.begin(), rectangular_prism.polygon.points.end(),
    rectangular_prism.polygon.points.begin(), [](Point32 pt) {
      pt.z = -pt.z;
      return pt;
    });
  auto projection = model.project(rectangular_prism);
  EXPECT_EQ(projection.shape.size(), 0U);
}

/// \brief Test to validate that points outside the FoV of the camera are clamped
TEST_F(PrismProjectionTest, out_of_plane_test) {
  #ifdef __aarch64__
  // Catastrophic cancellation makes this test fail on the arm release build. See #1241
  GTEST_SKIP();
  #endif
  CameraIntrinsics intrinsics{100, 100, 1.0F, 1.0F};
  geometry_msgs::msg::Transform identity{};
  identity.rotation.set__w(1.0);
  CameraModel model{intrinsics, identity};
  // Pull two of the corners of the rectangle to outside of the camera's FoV
  rectangular_prism.polygon.points[0U].x +=
    static_cast<float32_t>(intrinsics.width) * distance_from_origin * 1000.0F;
  rectangular_prism.polygon.points[1U].x +=
    static_cast<float32_t>(intrinsics.width) * distance_from_origin * 1000.0F;
  // Set the height to 0 to simplify the problem and not handle the rear corners of the prism.
  rectangular_prism.height = 0.0F;
  auto projection = model.project(rectangular_prism);

  const auto check_pt = [&projection](auto x, auto y) {
      const auto res_it = std::find_if(
        projection.shape.begin(), projection.shape.end(),
        [x, y](const auto & pt) {
          return autoware::common::helper_functions::comparisons::abs_eq(
            (x - pt.x()), 0.0F, std::numeric_limits<decltype(x)>::epsilon()) &&
          autoware::common::helper_functions::comparisons::abs_eq(
            (y - pt.y()), 0.0F, std::numeric_limits<decltype(y)>::epsilon());
        });
      if (res_it == projection.shape.end()) {
        std::stringstream projected_pts{};
        for (const auto & pt : projection.shape) {
          projected_pts << "(" << pt.x() << ", " << pt.y() << ") ";
        }
        FAIL() << "Point (" << x << ", " << y << ") is not found in projections:" << std::endl <<
          projected_pts.str();
      }
      ASSERT_NE(res_it, projection.shape.end());
      projection.shape.erase(res_it);
    };
  // Check that one of the projected points is clamped to the edge of the image plane as expected
  const auto depth = rectangular_prism.polygon.points.front().z;
  const auto y2d = (half_length * intrinsics.fy + depth * intrinsics.oy) / depth;
  check_pt(static_cast<float32_t>(intrinsics.width) / 2.0F, y2d);
  check_pt(static_cast<float32_t>(intrinsics.width) / 2.0F, -y2d);
  rectangular_prism.polygon.points.erase(
    rectangular_prism.polygon.points.begin(),
    rectangular_prism.polygon.points.begin() + 2U);

  compare_shapes(rectangular_prism.polygon, projection, intrinsics);
}
