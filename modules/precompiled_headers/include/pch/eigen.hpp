#pragma once

// only precompile eigen/geometry. everything else would be too much for our project
#define EIGEN_NO_STATIC_ASSERT
#include <Eigen/Geometry>

namespace Eigen {

// Eigen::Vector3d
template class Matrix<double, 3, 1>;

// Eigen::Vector3f
template class Matrix<float, 3, 1>;

// Eigen::Vector2d
template class Matrix<double, 2, 1>;

// Eigen::Vector2f
template class Matrix<float, 2, 1>;

template class Matrix<double, 3, 3, 0>;

template class Matrix<float, 3, 3, 0>;

// Instantiate the function with the longest compile time
// Results based on ClangBuildAnalyzer with Clang 14
template Translation<double, 2>::IsometryTransformType Translation<double, 2>::operator*(
    const RotationBase<Rotation2Dd, 2> &) const;

// Base type that is used in every template
using Matrix3x3d = Matrix<double, 3, 3, 0>;
template void
internal::generic_product_impl<Product<Inverse<Matrix3x3d>, Matrix3x3d>, Matrix3x3d>::evalTo<Matrix<double, 3, 3, 0>>(
    Matrix<double, 3, 3, 0> &dst, const Product<Inverse<Matrix<double, 3, 3, 0>>, Matrix<double, 3, 3, 0>, 0> &lhs,
    const Matrix<double, 3, 3, 0> &rhs);

template class Product<Product<Inverse<Matrix3x3d>, Matrix3x3d, 0>, Matrix3x3d, 0>;

};  // namespace Eigen