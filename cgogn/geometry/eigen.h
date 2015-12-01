#ifndef GEOMETRY_EIGEN_H
#define GEOMETRY_EIGEN_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

/* See Eigen documentation
 * http://eigen.tuxfamily.org/dox/group__QuickRefPage.html
 */

namespace Eigen {

template<typename SCALAR, int DIM>
using Vector = Matrix<SCALAR, DIM, 1>;

template<int DIM>
using Vectorf = Vector<float, DIM>;

template<int DIM>
using Vectord = Vector<double, DIM>;

// Common interface for the Eigen norm() method applied on Vectors 3D or 4D
// Limit the computation to the first 3 coordinates with a fixed block selector
//template <typename SCALAR, int DIM>
//SCALAR norm3D(const Eigen::Vector<SCALAR, DIM>& v)
//{
//	return v.template head<3>().norm();
//}

// Common interface for the Eigen norm() method applied on Vectors 3D or 4D
template <typename SCALAR>
SCALAR norm3D(const Eigen::Vector<SCALAR, 3>& v)
{
	return v.norm();
}

// Common interface for the Eigen norm() method applied on Vectors 3D or 4D
template <typename SCALAR>
SCALAR norm3D(const Eigen::Vector<SCALAR, 4>& v)
{
	return v.template head<3>().norm();
}

// Common interface for the Eigen cross() method applied on Vectors 3D or 4D
template <typename SCALAR>
Eigen::Vector<SCALAR, 3> crossProduct3D(const Eigen::Vector<SCALAR, 3>& v1,
										const Eigen::Vector<SCALAR, 3>& v2)
{
	return v1.cross(v2);
}

// Common interface for the Eigen cross() method applied on Vectors 3D or 4D
// Eigen::cross3 ignore the 4th coordinate of the given Vector4 and put 0.0 in the result
template <typename SCALAR>
Eigen::Vector<SCALAR, 4> crossProduct3D(const Eigen::Vector<SCALAR,4>& v1,
										const Eigen::Vector<SCALAR,4>& v2)
{
	return v1.cross3(v2);
}

}
#endif // GEOMETRY_EIGEN_H
