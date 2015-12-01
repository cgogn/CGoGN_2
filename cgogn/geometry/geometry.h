#ifndef GEOMETRY_GEOMETRY_H_
#define GEOMETRY_GEOMETRY_H_

#include "eigen.h"

namespace cgogn {


// isobarycenter between 2 vectors
template <typename SCALAR, int DIM>
Eigen::Vector<SCALAR,DIM> isobarycenter(const Eigen::Vector<SCALAR,DIM>& v1,
										const Eigen::Vector<SCALAR,DIM>& v2)
{
	return (v1 + v2) / SCALAR(2);
}

// isobarycenter of 3 vectors
template <typename SCALAR, int DIM>
Eigen::Vector<SCALAR,DIM> isobarycenter(const Eigen::Vector<SCALAR,DIM>& v1,
									 const Eigen::Vector<SCALAR,DIM>& v2,
									 const Eigen::Vector<SCALAR,DIM>& v3,
									 SCALAR s1, SCALAR s2, SCALAR s3)
{
	return (v1 + v2 + v3) / SCALAR(3) ;
}

// linear interpolation between 2 points
template <typename SCALAR, int DIM>
Eigen::Vector<SCALAR,DIM> lerp(const Eigen::Vector<SCALAR,DIM>& v1,
							   const Eigen::Vector<SCALAR,DIM>& v2,
							   SCALAR s)
{
	return (SCALAR(1) - s) * v1 + s * v2;
}

// weighted barycenter between 2 vectors
template <typename SCALAR, int DIM>
Eigen::Vector<SCALAR,DIM> barycenter(const Eigen::Vector<SCALAR,DIM>& v1,
									 const Eigen::Vector<SCALAR,DIM>& v2,
									 SCALAR s1, SCALAR s2)
{
	return (s1 * v1 + s2 * v2) / (s1 + s2) ;
}

// weighted barycenter between 3 vectors
template <typename SCALAR, int DIM>
Eigen::Vector<SCALAR,DIM> barycenter(const Eigen::Vector<SCALAR,DIM>& v1,
									 const Eigen::Vector<SCALAR,DIM>& v2,
									 const Eigen::Vector<SCALAR,DIM>& v3,
									 SCALAR s1, SCALAR s2, SCALAR s3)
{
	return (s1 * v1 + s2 * v2 + s3 * v3) / (s1 + s2 + s3) ;
}

// normal of the plane spanned by 3 points in 3D
template <typename SCALAR>
Eigen::Vector<SCALAR,3> triangleNormal(const Eigen::Vector<SCALAR,3>& p1,
									   const Eigen::Vector<SCALAR,3>& p2,
									   const Eigen::Vector<SCALAR,3>& p3)
{
	return (p2 - p1).cross(p3 - p1) ;
}

// normal of the plane spanned by 3 points in 3D
template <typename SCALAR>
Eigen::Vector<SCALAR,4> triangleNormal(const Eigen::Vector<SCALAR,4>& p1,
									   const Eigen::Vector<SCALAR,4>& p2,
									   const Eigen::Vector<SCALAR,4>& p3)
{
	return (p2 - p1).cross3(p3 - p1) ;
}


} // namespace cgogn

#endif // GEOMETRY_GEOMETRY_H_
