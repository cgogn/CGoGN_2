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

#ifndef CGOGN_GEOMETRY_TYPES_QUADRIC_H_
#define CGOGN_GEOMETRY_TYPES_QUADRIC_H_

#include <cgogn/core/utils/numerics.h>

#include <cgogn/geometry/dll.h>
#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/geometry/types/plane_3d.h>

namespace cgogn
{

namespace geometry
{

template <typename Scalar>
class Quadric
{
public:

	using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
	using Vec4 = Eigen::Matrix<Scalar, 4, 1>;
	using Vec4d = Eigen::Vector4d;
	using Self = Quadric<Scalar>;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	inline Quadric()
	{
		matrix_.setZero();
	}

	inline Quadric(const Vec3& p1, const Vec3& p2, const Vec3& p3)
	{
		Plane3D<Vec3> plane(p1, p2, p3);
		const Vec3& n = plane.normal();
		Vec4d p = Vec4d(n[0], n[1], n[2], plane.d());
		matrix_ = p * p.transpose();
	}

	Quadric(const Self& q)
	{
		matrix_ = q.matrix_;
	}

	inline void zero()
	{
		matrix_.setZero();
	}

	Self& operator=(const Self& q)
	{
		matrix_ = q.matrix_;
		return *this;
	}

	Quadric& operator+=(const Self& q)
	{
		matrix_ += q.matrix_;
		return *this;
	}

	Scalar operator()(const Vec3& v)
	{
		return (*this)(Vec4(v[0], v[1], v[2], Scalar(1)));
	}

//	Scalar operator()(const Vec4& v)
//	{
//		Vec4 Av = matrix_ * v;
//		Scalar res = v.transpose() * Av;
//		return res;
//	}

	Scalar operator()(const Vec4& v)
	{
		Vec4d vd = v.template cast<Vec4d::Scalar>();
		Vec4d Av = matrix_ * vd;
		Vec4d::Scalar res = vd.transpose() * Av;
		return Scalar(res);
	}

	static std::string cgogn_name_of_type()
	{
		return std::string("cgogn::geometry::Quadric<") + name_of_type(Scalar()) + std::string(">");
	}

	inline friend std::ostream& operator<<(std::ostream& out, const Self&)
	{
		return out;
	}

	inline friend std::istream& operator>> (std::istream& in, const Self&)
	{
		return in;
	}

private:

	Eigen::Matrix4d matrix_;
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_GEOMETRY_TYPES_QUADRIC_CPP_))
extern template class CGOGN_GEOMETRY_API Quadric<float32>;
extern template class CGOGN_GEOMETRY_API Quadric<float64>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_GEOMETRY_TYPES_QUADRIC_CPP_))

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_TYPES_QUADRIC_H_
