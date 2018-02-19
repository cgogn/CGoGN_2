/*******************************************************************************
* CGoGN: Combinatorial and Geometric modeling with Generic N-dimensional Maps  *
* Copyright (C) 2015, IGG Group, ICube, University of Strasbourg, France       *
*                                                                              *
* This library is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU Lesser General Public License as published by the *
* Free Software Foundation; either version 2.1 of the License, or (at your     *
* option) any later version.                                                   *
*                                                                              *
* This library is distributed in the hope that it will be useful, but WITHOUT  *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License  *
* for more details.                                                            *
*                                                                              *
* You should have received a copy of the GNU Lesser General Public License     *
* along with this library; if not, write to the Free Software Foundation,      *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.           *
*                                                                              *
* Web site: http://cgogn.unistra.fr/                                           *
* Contact information: cgogn@unistra.fr                                        *
*                                                                              *
*******************************************************************************/

#ifndef CGOGN_GEOMETRY_TYPES_PLANE_3D_H_
#define CGOGN_GEOMETRY_TYPES_PLANE_3D_H_

#include <type_traits>
#include <array>

#include <cgogn/core/utils/numerics.h>

#include <cgogn/geometry/dll.h>
#include <cgogn/geometry/types/eigen.h>
#include <cgogn/geometry/types/geometry_traits.h>


namespace cgogn
{

namespace geometry
{

enum Orientation3D
{
	ON = 0,
	OVER,
	UNDER
};


// TODO specialize template function for Eigen::Vector3d

class CGOGN_GEOMETRY_API Plane3D
{
	template <typename VEC3>
	inline Eigen::Vector3d to_eigen(const VEC3& v) const
	{
		return Eigen::Vector3d(v[0],v[1],v[2]);
	}

public:

	using Vec = Eigen::Vector3d;
	using Scalar = double;
	using Self = Plane3D;

	static const bool eigen_make_aligned = std::is_same<Eigen::AlignedVector3<double>, Vec>::value;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(eigen_make_aligned)

	Plane3D();
	Plane3D(const Self&) = default;
	Self& operator=(const Self&) = default;

	// construct the plane from a normal vector and a scalar
	template <typename VEC3, typename SCALAR>
	inline Plane3D(const VEC3& normal, SCALAR d) :
		normal_(normal[0],normal[1],normal[2]),
		d_(d)
	{
		static_assert(is_dim_of<VEC3, 3>::value, "normal must be of dim 3");
		static_assert(std::is_arithmetic<SCALAR>::value, "d must be arithmetic");
		normal_.normalize();
	}

	// construct the plane with normal vector and going through p
	template <typename VEC3>
	inline Plane3D(const VEC3& normal, const VEC3& p) :
		normal_(normal[0],normal[1],normal[2])
	{
		static_assert(is_dim_of<VEC3, 3>::value, "normal and p must be of dim 3");
		normal_.normalize();
		d_ = -(to_eigen(p).dot(normal_));
	}

	// construct the plane going through p1, p2 and p3
	template <typename VEC3>
	inline Plane3D(const VEC3& p1, const VEC3& p2, const VEC3& p3)
	{
		static_assert(is_dim_of<VEC3, 3>::value, "p1 p2 p3 must be of dim 3");
		Vec q1 = to_eigen(p1);
		Vec u = to_eigen(p2)-q1;
		Vec v = to_eigen(p3)-q1;
		normal_ = u.cross(v);
		normal_.normalize();
		d_ = -(q1.dot(normal_));
	}

	/**********************************************/
	/*             UTILITY FUNCTIONS              */
	/**********************************************/

	// compute a point on the plane (-d*N)
	inline const Vec point() const
	{
		return normal_*d_;
	}

	// get the normal of the plane
	inline const Vec& normal() const
	{
		return normal_;
	}

	inline Scalar d() const
	{
		return d_;
	}

	// compute the distance between the plane and point p
	inline Scalar distance(const Vec& p) const
	{
		return normal_.dot(p) + d_;
	}

	// project the point p onto the plane
	template <typename VEC3>
	inline void project(VEC3& q) const
	{
		Vec p = to_eigen(q);
		Scalar d = -distance(p);

		if (!cgogn::almost_equal_relative(d,Scalar(0)))
		{
			p += normal_*d;
		}
		q = VEC3(p[0],p[1],p[2]);
	}

	// return on/over/under according to the side of the plane where point p is
	template <typename VEC3>
	inline Orientation3D orient(const VEC3& q) const
	{
		Vec p = to_eigen(q);
		const Scalar dist = distance(p);

		if (cgogn::almost_equal_relative(dist, Scalar(0)))
			return Orientation3D::ON;

		if (dist < -Scalar(0))
			return Orientation3D::UNDER;

		return Orientation3D::OVER;
	}

	static std::string cgogn_name_of_type();

private:

	Eigen::Vector3d normal_;
	double d_;
};

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_TYPES_PLANE_3D_H_
