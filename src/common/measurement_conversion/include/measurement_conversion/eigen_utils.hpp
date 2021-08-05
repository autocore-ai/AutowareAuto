// Copyright 2021 Apex.AI, Inc.
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

#ifndef MEASUREMENT_CONVERSION__EIGEN_UTILS_HPP_
#define MEASUREMENT_CONVERSION__EIGEN_UTILS_HPP_

#include <Eigen/Geometry>


namespace autoware
{
namespace common
{
namespace state_estimation
{

///
/// @brief      This class describes a data storage order.
///
enum class DataStorageOrder
{
  kRowMajor,
  kColumnMajor
};

namespace detail
{
///
/// @brief      A functor to compute an index in the data array.
///
class Index
{
public:
  ///
  /// @brief      Create an index functor.
  ///
  /// @param[in]  start_index    The start index at which to start the count.
  /// @param[in]  stride         The stride of the data, i.e., step to the next row or column.
  /// @param[in]  storage_order  How the data should be stored in memory.
  ///
  Index(
    const std::int32_t start_index,
    const std::int32_t stride,
    const DataStorageOrder storage_order)
  : m_start_index{start_index}, m_stride{stride}, m_storage_order{storage_order} {}

  ///
  /// @brief      Compute an index using the stored starting index, stride and storage order.
  ///
  /// @param[in]  row   Query row
  /// @param[in]  col   Query column
  ///
  /// @return     A 1D index in the underlying data array
  ///
  std::size_t operator()(const Eigen::Index row, const Eigen::Index col) const
  {
    switch (m_storage_order) {
      case DataStorageOrder::kRowMajor:
        return static_cast<std::size_t>(m_start_index + row * m_stride + col);
      case DataStorageOrder::kColumnMajor:
        return static_cast<std::size_t>(m_start_index + col * m_stride + row);
    }
    throw std::runtime_error("Unexpected storage order");
  }

private:
  std::int32_t m_start_index{};
  std::int32_t m_stride{};
  DataStorageOrder m_storage_order{};
};
}  // namespace detail

///
/// @brief      Downscale the isometry to a lower dimension if needed.
///
/// @param[in]  isometry              The isometry transform
///
/// @tparam     kStateDimensionality  Dimensionality of the space.
/// @tparam     FloatT                Type of scalar.
///
/// @return     Downscaled isometry.
///
template<std::int32_t kStateDimensionality, typename FloatT>
static constexpr Eigen::Transform<
  FloatT, kStateDimensionality, Eigen::TransformTraits::Isometry> downscale_isometry(
  const Eigen::Transform<FloatT, 3, Eigen::TransformTraits::Isometry> & isometry)
{
  static_assert(kStateDimensionality <= 3, "We only handle scaling the isometry down.");
  using Isometry = Eigen::Transform<
    FloatT, kStateDimensionality, Eigen::TransformTraits::Isometry>;
  Isometry result{Isometry::Identity()};
  result.linear() = isometry.rotation()
    .template block<kStateDimensionality, kStateDimensionality>(0, 0);
  result.translation() = isometry.translation().topRows(kStateDimensionality);
  return result;
}

///
/// @brief      Transform a given array to an Eigen matrix using the specified stride and a starting
///             index within this array.
///
/// @details    The function converts a given array into an Eigen matrix using the given starting
///             index and stride. The function will take the number of rows and columns from the
///             provided parameters and will iterate the given array as many times as needed to fill
///             all elements of the resulting matrix. It is assumed that there are no gaps between
///             the elements in a single row but that the columns are spaced `stride` apart.
///
/// @note       The storage order only refers to the storage order expected from the array, not from
///             the Eigen matrix. The Eigen matrices are expected to be column-major as per default.
///
/// @throws     std::runtime_error  if there is not enough elements in the array to perform a full
///                                 conversion of all the asked elements.
///
/// @param[in]  array          The given array (e.g. a covariance array from a message)
/// @param[in]  start_index    The start index of the first element in an array to be copied
/// @param[in]  stride         How big the step to the next row is in terms of indices in the array
/// @param[in]  storage_order  The storage order of the data in the input array (row-major for ROS)
///
/// @tparam     kRows          Number of rows in the resulting matrix
/// @tparam     kCols          Number of columns in the resulting matrix
/// @tparam     ScalarT        Scalar type (inferred)
/// @tparam     kSize          Size of the input array (inferred)
///
/// @return     An Eigen matrix that stores the requested data.
///
template<std::int32_t kRows, std::int32_t kCols, typename ScalarT, std::size_t kSize>
Eigen::Matrix<ScalarT, kRows, kCols> array_to_matrix(
  const std::array<ScalarT, kSize> & array,
  const std::int32_t start_index,
  const std::int32_t stride,
  const DataStorageOrder storage_order)
{
  using Mat = Eigen::Matrix<ScalarT, kRows, kCols>;
  const detail::Index index{start_index, stride, storage_order};
  Mat res{Mat::Zero()};
  const auto max_index = index(res.rows() - 1, res.cols() - 1);
  if (max_index >= array.size()) {
    throw std::runtime_error(
            "Trying to access out of bound memory at index " +
            std::to_string(max_index) + " of an array with size: " + std::to_string(array.size()));
  }

  for (auto col = 0; col < kCols; ++col) {
    for (auto row = 0; row < kRows; ++row) {
      res(row, col) = array[index(row, col)];
    }
  }
  return res;
}

///
/// @brief      Sets data in an array from a given Eigen matrix.
///
/// @note       The storage order only refers to the storage order expected from the array, not from
///             the Eigen matrix. The Eigen matrices are expected to be column-major as per default.
///
/// @param      array          The array for which the data is set
/// @param[in]  matrix         The matrix that has the data that is to be copied
/// @param[in]  start_index    The start index in the output data array
/// @param[in]  stride         The step that brings the index to the next row/column
/// @param[in]  storage_order  The storage order of the output data
///
/// @tparam     kRows          Number of rows in the input matrix, inferred by the compiler.
/// @tparam     kCols          Number of columns in the input matrix, inferred by the compiler.
/// @tparam     ScalarT        Type of underlying data elements, inferred by the compiler.
/// @tparam     kSize          Size of the input matrix, inferred by the compiler.
///
template<std::int32_t kRows, std::int32_t kCols, typename ScalarT, std::size_t kSize>
void set_from_matrix(
  std::array<ScalarT, kSize> & array,
  const Eigen::Matrix<ScalarT, kRows, kCols> & matrix,
  const std::int32_t start_index,
  const std::int32_t stride,
  const DataStorageOrder storage_order)
{
  const detail::Index index{start_index, stride, storage_order};
  const auto max_index = index(matrix.rows() - 1, matrix.cols() - 1);
  if (max_index >= array.size()) {
    throw std::runtime_error(
            "Trying to access out of bound memory at index " +
            std::to_string(max_index) + " of an array with size: " + std::to_string(array.size()));
  }
  for (auto col = 0; col < kCols; ++col) {
    for (auto row = 0; row < kRows; ++row) {
      array[index(row, col)] = matrix(row, col);
    }
  }
}


///
/// @brief      Get a slice of a given matrix.
///
/// @details    Given two sequences for rows and for columns generate a new matrix that contains
///             only rows and columns in those sequences.
///
/// @param[in]  matrix         The given matrix
/// @param[in]  sequence_rows  The sequence of rows to include in the slice
/// @param[in]  sequence_cols  The sequence of columns to include in the slice
///
/// @tparam     ScalarT        Type of underlying data
/// @tparam     kRows          Number of rows in a query matrix
/// @tparam     kCols          Number of columns in a query matrix
/// @tparam     kSeqRowsSize   Number of rows in the slice
/// @tparam     kSeqColsSize   Number of columns in the slice
///
/// @return     A matrix representing a slice of the original matrix.
///
template<
  typename ScalarT,
  std::int32_t kRows,
  std::int32_t kCols,
  std::size_t kSeqRowsSize,
  std::size_t kSeqColsSize>
Eigen::Matrix<
  ScalarT, static_cast<std::int32_t>(kSeqRowsSize), static_cast<std::int32_t>(kSeqColsSize)>
slice(
  const Eigen::Matrix<ScalarT, kRows, kCols> matrix,
  const std::array<Eigen::Index, kSeqRowsSize> & sequence_rows,
  const std::array<Eigen::Index, kSeqColsSize> & sequence_cols)
{
  using Mat = Eigen::Matrix<
    ScalarT, static_cast<std::int32_t>(kSeqRowsSize), static_cast<std::int32_t>(kSeqColsSize)>;
  Mat res{Mat::Zero()};
  for (auto col = 0UL; col < kSeqColsSize; ++col) {
    for (auto row = 0UL; row < kSeqRowsSize; ++row) {
      res(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(col)) =
        matrix(sequence_rows[row], sequence_cols[col]);
    }
  }
  return res;
}

}  // namespace state_estimation
}  // namespace common
}  // namespace autoware


#endif  // MEASUREMENT_CONVERSION__EIGEN_UTILS_HPP_
