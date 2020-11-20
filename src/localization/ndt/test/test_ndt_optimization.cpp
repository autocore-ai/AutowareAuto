// Copyright 2019 the Autoware Foundation
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

#include "test_ndt_optimization.hpp"
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <limits>
#include "common/types.hpp"

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

using autoware::localization::ndt::P2DNDTScan;
using autoware::localization::ndt::P2DNDTOptimizationProblem;
using autoware::localization::ndt::P2DNDTOptimizationConfig;
using autoware::localization::ndt::transform_adapters::pose_to_transform;

using P2DProblem = P2DNDTOptimizationProblem<autoware::localization::ndt::StaticNDTMap>;

OptTestParams::OptTestParams(
  float64_t x, float64_t y, float64_t z, float64_t ang_x, float64_t ang_y, float64_t ang_z,
  bool8_t large, bool8_t check_pcl)
: is_large{large}, check_pcl{check_pcl}
{
  // euler angles XYZ
  diff << x, y, z, ang_x, ang_y, ang_z;
}

class P2DOptimizationTest : public OptimizationTestContext, public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_EQ(m_dynamic_map.size(), m_static_map.size());
    for (const auto & pt_it : m_voxel_centers) {
      const auto static_vx = m_static_map.cell(pt_it.second)[0U];
      const auto dynamic_vx = m_dynamic_map.cell(pt_it.second)[0U];
      ASSERT_TRUE(static_vx.usable());
      ASSERT_TRUE(dynamic_vx.usable());

      ASSERT_TRUE(
        static_vx.centroid().isApprox(
          dynamic_vx.centroid(),
          std::numeric_limits<Real>::epsilon()));
      ASSERT_TRUE(
        static_vx.inverse_covariance().isApprox(
          dynamic_vx.inverse_covariance(),
          std::numeric_limits<Real>::epsilon() * 1e2));
    }
  }
};

class P2DOptimizationValidationTest : public P2DOptimizationTest,
  public ::testing::WithParamInterface<OptTestParams> {};

class P2DOptimizationNumericalTest : public P2DOptimizationTest,
  public ::testing::WithParamInterface<OptTestParams> {};

TEST_P(P2DOptimizationNumericalTest, numerical_analysis) {
  {
    // m_pc is also used as the map, so this scan perfectly aligns with the map
    P2DNDTScan matching_scan(m_downsampled_cloud, m_downsampled_cloud.width);
    P2DProblem problem{matching_scan, m_static_map, P2DNDTOptimizationConfig{0.55}};

    EigenPose<Real> pose = GetParam().diff;
    problem.evaluate(pose, autoware::common::optimization::ComputeMode{true, true, true});
    P2DProblem::Jacobian jacobian;
    P2DProblem::Hessian hessian;
    problem.jacobian(pose, jacobian);
    problem.hessian(pose, hessian);

    decltype(hessian) numerical_hessian;
    decltype(jacobian) numerical_jacobian;
    numerical_diff(problem, pose, numerical_jacobian, numerical_hessian);
    // Compare numerical and analytical values
    // Linearization error is higher on the hessian
    constexpr auto zero_eps = 1e-4;
    constexpr auto jacob_eps = 1e-6;
    constexpr auto hessian_eps = 1e-1;
    // Zero matrices cannot be compared by `isApprox(...)`. If the below checks fail, that means
    // both of them are zero, therefore equal.
    if (!jacobian.isZero(zero_eps) || !numerical_jacobian.isZero(zero_eps)) {
      EXPECT_TRUE(jacobian.isApprox(numerical_jacobian, jacob_eps));
    }
    if (!hessian.isZero(zero_eps) || numerical_hessian.isZero(zero_eps)) {
      EXPECT_TRUE(hessian.isApprox(numerical_hessian, hessian_eps));
    }
  }
}

TEST_P(P2DOptimizationValidationTest, sanity_test) {
  using PclCloud = pcl::PointCloud<pcl::PointXYZ>;
  constexpr auto translation_tol = 1e-2;
  constexpr auto rotation_tol = 1e-2;

  const auto & param = GetParam();
  EigenPose<Real> diff = param.diff;

  geometry_msgs::msg::TransformStamped diff_tf2;
  diff_tf2.header.frame_id = "custom";
  pose_to_transform(diff, diff_tf2.transform);

  // Ensure the scan won't be translated out of its voxel
  ASSERT_LT(diff(0), m_voxel_size.x);
  ASSERT_LT(diff(1), m_voxel_size.y);
  ASSERT_LT(diff(2), m_voxel_size.z);
  // translate the cloud by a small margin.
  auto translated_cloud = m_downsampled_cloud;
//  translate_pc(m_downsampled_cloud, translated_cloud, diff);
  tf2::doTransform(m_downsampled_cloud, translated_cloud, diff_tf2);
  ////////////// Solve using ndt optimization problem

  P2DProblem::DomainValue guess;
  decltype(guess) pcl_guess;
  guess.setZero();
  pcl_guess.setZero();
  const auto step_length = 0.12;
  const auto num_iters = 5U;
  {
    P2DNDTScan scan(translated_cloud, translated_cloud.width);
    P2DProblem problem{scan, m_static_map, P2DNDTOptimizationConfig{0.55}};
    // Solve the problem in 50 iterations:
    for (auto i = 0U; i < num_iters; ++i) {
      problem.evaluate(
        guess,
        autoware::common::optimization::ComputeMode{}.set_score().set_jacobian().set_hessian());
      P2DProblem::Jacobian jacobian;
      P2DProblem::Hessian hessian;
      problem.jacobian(guess, jacobian);
      problem.hessian(guess, hessian);
      // Solve for decent direction using newton method
      Eigen::JacobiSVD<Eigen::Matrix<float64_t, 6, 6>> sv(hessian,
        Eigen::ComputeFullU | Eigen::ComputeFullV);
      // Negative for maximization as opposed to minimization
      decltype(guess) delta = sv.solve(-jacobian);
      guess += step_length * delta;
    }
  }

  decltype(diff) neg_diff = -diff;
  is_pose_approx(guess, neg_diff, translation_tol, rotation_tol);

  std::cout << "The scan is transformed away from the map by: \n" << diff << std::endl;
  std::cout << "The estimated pose difference after " << num_iters << " iterations with " <<
    step_length << " step size: \n" << guess << std::endl;

  // Compare to the pcl result
  if (param.check_pcl) {
    PclCloud::Ptr scan_cloud{new PclCloud};
    PclCloud::Ptr map_cloud{new PclCloud};
    PclCloud::Ptr result_cloud{new PclCloud};
    *scan_cloud = from_pointcloud2(translated_cloud);
    *map_cloud = from_pointcloud2(m_pc);

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> pcl_ndt;
    Eigen::Transform<float32_t, 3, Eigen::Affine, Eigen::ColMajor> init_guess;
    init_guess.matrix().setIdentity();
    pcl_ndt.setInputSource(scan_cloud);
    pcl_ndt.setInputTarget(map_cloud);
    pcl_ndt.align(*result_cloud, init_guess.matrix());

    init_guess.matrix() = pcl_ndt.getFinalTransformation();
    pcl_guess.head(3) = init_guess.translation().cast<float64_t>();
    pcl_guess.tail(3) = init_guess.rotation().eulerAngles(0, 1, 2).cast<float64_t>();

    // Using high error tolerance since two different implementations may vary in result greatly.
    is_pose_approx(pcl_guess, neg_diff, 0.2, 0.1);

    std::cout << "PCL implementation's estimated pose difference: \n" << pcl_guess << std::endl;
  }
}

// TODO(yunus.caliskan): Make the sample scan more 'featureful' so that ndt can match
// and converge for larger drifts as well with a higher accuracy.
INSTANTIATE_TEST_CASE_P(
  sanity_test, P2DOptimizationValidationTest,
  ::testing::Values(
    // TODO(yunus.caliskan): Make voxel grid configuration similar to that of PCL
    // to get results that are more comparable.
    OptTestParams{0.0, 0.65, 0.0, 0.0, 0.0, 0.0, true, false},
    OptTestParams{0.7, 0.0, 0.7, 0.0, 0.0, 0.0, true, true},
    OptTestParams{0.0, 0.1, 0.1, 0.0, 3.14159265359 / 72.0, 0.0, false, false},
    OptTestParams{0.0, -0.2, 0.0, 0.0, 3.14159265359 / 72.0, 3.14159265359 / 72.0, false, false}
    // cppcheck-suppress syntaxError
  ), );

INSTANTIATE_TEST_CASE_P(
  numerical_analysis, P2DOptimizationNumericalTest,
  ::testing::Values(
    OptTestParams{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true, false},
    OptTestParams{0.5, 0.9, 0.1, 1.0, -3.1, 0.05, true, false},
    OptTestParams{2.5, -1.9, 0.1, -2.1, 0.1, 3.05, true, false}
    // cppcheck-suppress syntaxError
  ), );


////////////////////////////////////// Test function implementations

pcl::PointCloud<pcl::PointXYZ> from_pointcloud2(const sensor_msgs::msg::PointCloud2 & msg)
{
  pcl::PointCloud<pcl::PointXYZ> res{};
  sensor_msgs::PointCloud2ConstIterator<float32_t> x_it(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float32_t> y_it(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float32_t> z_it(msg, "z");

  while (x_it != x_it.end() &&
    y_it != y_it.end() &&
    z_it != z_it.end())
  {
    pcl::PointXYZ pt;
    pt.x = *x_it;
    pt.y = *y_it;
    pt.z = *z_it;
    ++x_it;
    ++y_it;
    ++z_it;
    res.push_back(pt);
  }
  return res;
}
