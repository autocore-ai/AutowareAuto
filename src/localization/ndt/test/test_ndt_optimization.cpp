// Copyright 2019 Apex.AI, Inc.
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

#include "test_ndt_optimization.hpp"
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace autoware
{
namespace localization
{
namespace ndt
{

struct OptTestParams
{
  OptTestParams(
    double x, double y, double z, double ang_x, double ang_y, double ang_z,
    bool large, bool check_pcl)
  : is_large{large}, check_pcl{check_pcl}
  {
    // euler angles XYZ
    diff << x, y, z, ang_x, ang_y, ang_z;
  }
  Pose<Real> diff;
  bool is_large;
  bool check_pcl;
};

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

      ASSERT_TRUE(static_vx.centroid().isApprox(dynamic_vx.centroid(),
        std::numeric_limits<Real>::epsilon()));
      ASSERT_TRUE(static_vx.covariance().isApprox(dynamic_vx.covariance(),
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
    P2DNDTOptimizationProblem problem{matching_scan, m_static_map, 0.55};

    Pose<Real> pose = GetParam().diff;
    problem.evaluate(pose, common::optimization::ComputeMode{true, true, true});
    P2DNDTOptimizationProblem::Jacobian jacobian;
    P2DNDTOptimizationProblem::Hessian hessian;
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
  constexpr double PI = 3.14159265359;
  const auto & param = GetParam();
  Pose<Real> diff = param.diff;

  Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> diff_eig_trans;
  diff_eig_trans.setIdentity();
  pose_to_transform(diff, diff_eig_trans);

  geometry_msgs::msg::TransformStamped diff_tf2;
  diff_tf2.header.frame_id = "custom";
  eig_to_tf2_trans(diff_eig_trans, diff_tf2.transform);

  // Ensure the scan won't be translated out of its voxel
  ASSERT_LT(diff(0), m_voxel_size.x);
  ASSERT_LT(diff(1), m_voxel_size.y);
  ASSERT_LT(diff(2), m_voxel_size.z);
  // translate the cloud by a small margin.
  auto translated_cloud = m_downsampled_cloud;
//  translate_pc(m_downsampled_cloud, translated_cloud, diff);
  tf2::doTransform(m_downsampled_cloud, translated_cloud, diff_tf2);
  ////////////// Solve using ndt optimization problem

  P2DNDTOptimizationProblem::DomainValue guess;
  decltype(guess) pcl_guess;
  guess.setZero();
  pcl_guess.setZero();
  const auto step_length = 0.05;
  const auto num_iters = 50U;
  {
    P2DNDTScan scan(translated_cloud, translated_cloud.width);
    P2DNDTOptimizationProblem problem{scan, m_static_map, 0.55};
    // Solve the problem in 50 iterations:
    for (auto i = 0U; i < num_iters; ++i) {
      problem.evaluate(guess,
        common::optimization::ComputeMode{}.set_score().set_jacobian().set_hessian());
      P2DNDTOptimizationProblem::Jacobian jacobian;
      P2DNDTOptimizationProblem::Hessian hessian;
      problem.jacobian(guess, jacobian);
      problem.hessian(guess, hessian);
      // Solve for decent direction using newton method
      Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> sv(hessian,
        Eigen::ComputeFullU | Eigen::ComputeFullV);
      // Negative for maximization as opposed to minimization
      decltype(guess) delta = sv.solve(-jacobian);
      guess += step_length * delta;
    }
  }

  // Check that the estimated translation has the same direction with the actual
  // required translation
  // (a and b are in same direction if `a x b = 0` and `a * b > 0`)
  auto pose_same_direction = [](const auto & p1, const auto & p2, double eps = 1e-2) {
      const Eigen::Vector3d t1 = p1.head(3);
      const Eigen::Vector3d r1 = p1.tail(3);
      const Eigen::Vector3d t2 = p2.head(3);
      const Eigen::Vector3d r2 = p2.tail(3);
      const Eigen::Vector3d cross_vec = t1.cross(t2);
      const double dot = t1.dot(t2);
      EXPECT_TRUE(cross_vec.isZero(eps));
      EXPECT_GT(dot, 0.0);
      if (!r1.isZero(eps) && !r2.isZero(eps)) {
        EXPECT_TRUE(r1.isApprox(r2, eps));
      }
    };

  // Compare translation results from algorithms
  const Eigen::Vector3d correct_trans = -diff.head(3);
  const Eigen::Vector3d estimated_trans = guess.head(3);
  if (!param.is_large) {
    EXPECT_TRUE(guess.isApprox(-diff, 0.1));
  } else {
    pose_same_direction(guess, -diff);
  }

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
    Eigen::Transform<float, 3, Eigen::Affine, Eigen::ColMajor> init_guess;
    init_guess.matrix().setIdentity();
    pcl_ndt.setInputSource(scan_cloud);
    pcl_ndt.setInputTarget(map_cloud);
    pcl_ndt.align(*result_cloud, init_guess.matrix());

    init_guess.matrix() = pcl_ndt.getFinalTransformation();
    pcl_guess.head(3) = init_guess.translation().cast<double>();
    pcl_guess.tail(3) = init_guess.rotation().eulerAngles(0, 1, 2).cast<double>();

    pose_same_direction(pcl_guess, -diff, 0.15);

    std::cout << "PCL implementation's estimated pose difference: \n" << pcl_guess << std::endl;
  }
}

// TODO(yunus.caliskan): Make the sample scan more 'featureful' so that ndt can match
// and converge for larger drifts as well with a higher accuracy.
INSTANTIATE_TEST_CASE_P(sanity_test, P2DOptimizationValidationTest,
  ::testing::Values(
    // TODO(yunus.caliskan): Make voxel grid configuration similar to that of PCL
    // to get results that are more comparable.
    OptTestParams{0.0, 0.65, 0.0, 0.0, 0.0, 0.0, true, false},
    OptTestParams{0.7, 0.0, 0.7, 0.0, 0.0, 0.0, true, true},
    OptTestParams{0.0, 0.1, 0.1, 0.0, 3.14159265359 / 72.0, 0.0, false, false},
    OptTestParams{0.0, -0.2, 0.0, 0.0, 3.14159265359 / 72.0, 3.14159265359 / 72.0, false, false}
));

INSTANTIATE_TEST_CASE_P(numerical_analysis, P2DOptimizationNumericalTest,
  ::testing::Values(
    OptTestParams{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true, false},
    OptTestParams{0.5, 0.9, 0.1, 1.0, -3.1, 0.05, true, false},
    OptTestParams{2.5, -1.9, 0.1, -2.1, 0.1, 3.05, true, false}
));


////////////////////////////////////// Test function implementations

pcl::PointCloud<pcl::PointXYZ> from_pointcloud2(const sensor_msgs::msg::PointCloud2 & msg)
{
  pcl::PointCloud<pcl::PointXYZ> res{};
  sensor_msgs::PointCloud2ConstIterator<float> x_it(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_it(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_it(msg, "z");

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


template<typename T>
void eig_to_tf2_trans(
  const Eigen::Transform<T, 3, Eigen::Affine, Eigen::ColMajor> & t_eig,
  geometry_msgs::msg::Transform & t_tf2)
{
  const auto & t = t_eig.translation();
  Eigen::Quaternion<T> r{t_eig.rotation()};
  t_tf2.translation.set__x(t(0)).set__y(t(1)).set__z(t(2));
  t_tf2.rotation.set__x(r.x()).set__y(r.y()).set__z(r.z()).set__w(r.w());
}

}  // namespace ndt
}  // namespace localization
}  // namespace autoware
