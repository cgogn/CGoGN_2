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

#ifndef GEOMETRY_PLANE_3D_H_
#define GEOMETRY_PLANE_3D_H_

#include <type_traits>
#include <array>

#include <core/utils/name_types.h>
#include <core/utils/precision.h>

#include <geometry/dll.h>
#include <geometry/vec_operations.h>
#include <geometry/geometry_traits.h>

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

template<typename VEC_T>
class Plane3D{
public:
	using Vec = VEC_T;
	//	using Real = typename std::remove_reference<decltype(Vec()[0ul])>::type;
	using Scalar = typename vector_traits<Vec>::Scalar;
	using Self = Plane3D<Vec>;

	Plane3D() = default;
	Plane3D(const Self&) = default;
	Plane3D(Self&&) = default;
	Self& operator=(const Self&) = default;
	Self& operator=(Self&&) = default;

	// construct the plane from a normal vector and a scalar
	inline Plane3D(const Vec& normal, Scalar d) :
		normal_(normal),
		d_(d)
	{
		normalize(normal_);
	}

	// construct the plane with normal vector and going through p
	inline Plane3D(const Vec& normal, const Vec& p) :
		normal_(normal),
		d_(-(p*normal))
	{
		normalize(normal_);
	}

	// construct the plane going through p1, p2 and p3
	inline Plane3D(const Vec& p1, const Vec& p2, const Vec& p3)
	{
		normal_ = (p2-p1) ^(p3-p1);
		normalize(normal_);
		d_ = -(p1 * normal_);
	}

	/**********************************************/
	/*             UTILITY FUNCTIONS              */
	/**********************************************/

	// compute a point on the plane (-d*N)
	inline Vec point() const
	{
		return d_ * normal_;
	}

	// compute the distance between the plane and point p
	inline Scalar distance(const Vec& p) const
	{
		return normal_*p + d_;
	}

	// project the point p onto the plane
	inline void project(Vec& p) const
	{
		Scalar d = -distance(p);

		if (!cgogn::almost_equal_relative(d,Scalar(0)))
		{
			p += normal_*d;
		}
	}

	// return on/over/under according to the side of the plane where point p is
	inline Orientation3D orient(const Vec& p) const
	{
		const Scalar dist = distance(p);

		if (cgogn::almost_equal_relative(dist, Scalar(0)))
			return Orientation3D::ON;

		if (dist < -Scalar(0))
			return Orientation3D::UNDER;

		return Orientation3D::OVER;
	}

public:
	static std::string CGoGNnameOfType()
	{
		vector_traits<Vec>::SIZE;
		return std::string("geometry::Plane3D<") + name_of_type(Vec()) + std::string(">");
	}
private:
	Vec normal_;
	Scalar d_;
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(GEOMETRY_PLANE_3D_CPP_))
extern template class CGOGN_GEOMETRY_API Plane3D<Eigen::Vector3d>;
extern template class CGOGN_GEOMETRY_API Plane3D<std::array<double,3>>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(GEOMETRY_PLANE_3D_CPP_))

} // namespace geometry
} // namespace cgogn

#endif // GEOMETRY_PLANE_3D_H_
