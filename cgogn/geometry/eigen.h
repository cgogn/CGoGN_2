#ifndef GEOMETRY_EIGEN_H
#define GEOMETRY_EIGEN_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
//#include <Eigen/eigen-master/unsupported/Eigen/AlignedVector3>

namespace Eigen {

template<typename SCALAR, int DIM>
using Vector = Matrix<SCALAR, DIM, 1>;

template<int DIM>
using Vectorf = Vector<float, DIM>;

template<int DIM>
using Vectord = Vector<double, DIM>;

}

#endif // GEOMETRY_EIGEN_H
