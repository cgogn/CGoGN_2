#ifndef GEOMETRY_GEOMETRY_H_
#define GEOMETRY_GEOMETRY_H_

#include <Eigen/Dense>

namespace cgogn {


// linear interpolation between 2 points
template <typename SCALAR, unsigned int DIM>
Eigen::Matrix<SCALAR,DIM,1> lerp(const Eigen::Matrix<SCALAR,DIM,1>& v1,
								 const Eigen::Matrix<SCALAR,DIM,1>& v2,
								 SCALAR s)
{
	return (1.0 - s) * v1 + s * v2 ;
}

template<typename SCALAR, unsigned int DIM>
using Vector = Eigen::Matrix<SCALAR, DIM, 1>;

// linear interpolation between 2 points version 2
template <typename SCALAR, unsigned int DIM>
Vector<SCALAR,DIM> lerp2(const Vector<SCALAR,DIM>& v1,
						 const Vector<SCALAR,DIM>& v2,
						 SCALAR s)
{
	return (1.0 - s) * v1 + s * v2 ;
}


// weighted barycenter of 2 points
template <typename VECT>
VECT barycenter(const VECT& v1, const VECT& v2, typename VECT::Scalar a, typename VECT::Scalar b)
{
	return a * v1 + b * v2 ;
}

} // namespace cgogn

#endif // GEOMETRY_GEOMETRY_H_
