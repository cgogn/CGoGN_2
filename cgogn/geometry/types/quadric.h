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

#include <cgogn/geometry/cgogn_geometry_export.h>
#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/geometry/types/plane_3d.h>

namespace cgogn
{

namespace geometry
{

class Quadric
{
public:

	using Self = Quadric;

//	using Vec3f = Eigen::Vector3f;
	using Vec3d = Eigen::Vector3d;
//	using Vec4f = Eigen::Vector4f;
	using Vec4d = Eigen::Vector4d;

	using Matrix4d = Eigen::Matrix4d;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	inline Quadric()
	{
		matrix_.setZero();
	}

	template <typename VEC3>
	inline Quadric(const VEC3& p1, const VEC3& p2, const VEC3& p3)
	{
		Plane3D plane(p1, p2, p3);
		const Vec3d& n = plane.normal();
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

	template <typename VEC3>
	auto operator()(const VEC3& v)
		-> typename std::enable_if<is_dim_of<VEC3, 3>::value && std::is_same<ScalarOf<VEC3>, double>::value, double>::type
	{
		return (*this)(Vec4d(v[0], v[1], v[2], 1.));
	}

	template <typename VEC3>
	auto operator()(const VEC3& v)
		-> typename std::enable_if<is_dim_of<VEC3, 3>::value && !std::is_same<ScalarOf<VEC3>, double>::value, ScalarOf<VEC3>>::type
	{
		using Scalar = ScalarOf<VEC3>;

		return Scalar((*this)(Vec4d(double(v[0]), double(v[1]), double(v[2]), 1.)));
	}

	inline double operator()(const Vec4d& v)
	{
		return v.transpose() * matrix_ * v;
	}

	template <typename VEC4>
	auto operator()(const VEC4& v)
		-> typename std::enable_if<is_dim_of<VEC4, 4>::value && !std::is_same<VEC4, Vec4d>::value, typename vector_traits<VEC4>::Scalar>::type
	{
		using Scalar = typename vector_traits<VEC4>::Scalar;

		Vec4d vd;
		vd[0] = double(v[0]);
		vd[1] = double(v[1]);
		vd[2] = double(v[2]);
		vd[3] = double(v[3]);
		double res = vd.transpose() * matrix_ * vd;
		return Scalar(res);
	}

	template <typename VEC3>
	auto optimized(VEC3& v)
		-> typename std::enable_if<is_dim_of<VEC3, 3>::value, bool>::type
	{
		using Scalar = ScalarOf<VEC3>;

		Vec4d hv;
		bool b = optimized(hv);
		if (b)
		{
			v[0] = Scalar(hv[0]);
			v[1] = Scalar(hv[1]);
			v[2] = Scalar(hv[2]);
		}
		return b;
	}

	template <typename VEC4>
	auto optimized(VEC4& v)
		-> typename std::enable_if<is_dim_of<VEC4, 4>::value && std::is_same<VEC4, Vec4d>::value, bool>::type
	{
		Matrix4d m(matrix_);
		for (uint32 i = 0; i < 3; ++i) m(3,i) = 0.;
		m(3,3) = 1.;
		Matrix4d inverse;
		double determinant;
		bool invertible;
		m.computeInverseAndDetWithCheck(inverse, determinant, invertible);
		if (invertible)
			v = inverse * Vec4d(0.,0.,0.,1.);
		return invertible;
	}

	template <typename VEC4>
	auto optimized(VEC4& v)
		-> typename std::enable_if<(vector_traits<VEC4>::SIZE ==4) && !std::is_same<VEC4, Vec4d>::value, bool>::type
	{
		using Scalar = typename vector_traits<VEC4>::Scalar;

		Matrix4d m(matrix_);
		for (uint32 i = 0; i < 3; ++i) m(3,i) = 0.;
		m(3,3) = 1.;
		Matrix4d inverse;
		double determinant;
		bool invertible;
		m.computeInverseAndDetWithCheck(inverse, determinant, invertible);
		if (invertible)
		{
			Vec4d vd = inverse * Vec4d(0.,0.,0.,1.);
			v[0] = Scalar(vd[0]);
			v[1] = Scalar(vd[1]);
			v[2] = Scalar(vd[2]);
			v[3] = Scalar(vd[3]);
		}
		return invertible;
	}

	static std::string cgogn_name_of_type()
	{
		return std::string("cgogn::geometry::Quadric");
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

	Matrix4d matrix_;
};

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_TYPES_QUADRIC_H_
