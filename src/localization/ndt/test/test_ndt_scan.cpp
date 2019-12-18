#include <ndt/ndt_scan.hpp>
#include <lidar_utils/point_cloud_utils.hpp>
#include "test_ndt_scan.hpp"
#include <gtest/gtest.h>

namespace autoware
{
namespace localization
{
namespace ndt
{
TEST_F(NDTScanTest, bad_input) {
  const auto capacity = 5U;
  ASSERT_LT(capacity, m_num_points);
  P2DNDTScan ndt_scan(capacity);
  EXPECT_THROW(ndt_scan.insert(m_pc), std::length_error);
}

TEST_F(NDTScanTest, basics) {
  ASSERT_EQ(m_pc.width, m_num_points);
  ASSERT_EQ(m_points.size(), m_num_points);

  P2DNDTScan ndt_scan(m_num_points);
  EXPECT_TRUE(ndt_scan.empty());
  EXPECT_NO_THROW(ndt_scan.insert(m_pc));

  EXPECT_EQ(ndt_scan.size(), m_num_points);
  EXPECT_FALSE(ndt_scan.empty());

  // since all points that are used in the form (x,y,z) where x = y = z. We can use a 1D
  // container for easier comparison. This is done to express the fact that the ndt scan does
  // not need to keep the order of its input.
  std::vector<uint32_t> reduced_reference_points;
  for (const auto & pt : m_points) {
    ASSERT_FLOAT_EQ(pt(0U), pt(1U));
    ASSERT_FLOAT_EQ(pt(0U), pt(2U));
    reduced_reference_points.push_back(static_cast<uint32_t>(pt(0U)));
  }

  std::vector<uint32_t> reduced_scan_points;
  for (const auto & pt : ndt_scan) {
    ASSERT_FLOAT_EQ(pt(0U), pt(1U));
    ASSERT_FLOAT_EQ(pt(0U), pt(2U));
    reduced_scan_points.push_back(static_cast<uint32_t>(pt(0U)));
  }

  std::sort(reduced_reference_points.begin(), reduced_reference_points.end());
  std::sort(reduced_scan_points.begin(), reduced_scan_points.end());

  EXPECT_EQ(reduced_scan_points, reduced_reference_points);

  // Adding a new point cloud overwrites the old one.
  EXPECT_NO_THROW(ndt_scan.insert(m_pc));
  EXPECT_EQ(ndt_scan.size(), m_num_points);

  ndt_scan.clear();
  EXPECT_TRUE(ndt_scan.empty());
  EXPECT_EQ(ndt_scan.size(), 0U);
};

}  // namespace ndt
}  // namespace localization
}  // namespace autoware
