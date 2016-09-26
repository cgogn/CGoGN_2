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

#include <cgogn/geometry/types/eigen.h>
#include <cgogn/geometry/types/vec.h>
#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/geometry/functions/intersection.h>

#include <gtest/gtest.h>

using StdArrayf = cgogn::geometry::Vec_T<std::array<float,3>>;
using StdArrayd = cgogn::geometry::Vec_T<std::array<double,3>>;
using EigenVec3f = Eigen::Vector3f;
using EigenVec3d = Eigen::Vector3d;
using VecTypes = testing::Types<StdArrayf, EigenVec3f, StdArrayd ,EigenVec3d>;

template <typename Vec_T>
class Intesection_TEST : public testing::Test
{
};


TYPED_TEST_CASE(Intesection_TEST, VecTypes );

TYPED_TEST(Intesection_TEST, intersection_ray_triangle)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;
	TypeParam p0(Scalar(1), Scalar(1), Scalar(96.1));
	TypeParam p1(Scalar(5), Scalar(1), Scalar(92.3));
	TypeParam p2(Scalar(3), Scalar(5), Scalar(94.2));

	TypeParam A0(Scalar(3), Scalar(3), Scalar(0));
	TypeParam D0(Scalar(0.001), Scalar(0.001), Scalar(1.0));

	TypeParam A1(Scalar(3), Scalar(1), Scalar(0));
	TypeParam D1(Scalar(0), Scalar(0), Scalar(1.0));
	TypeParam A2(Scalar(5), Scalar(1), Scalar(0));
	TypeParam A3(Scalar(9), Scalar(5), Scalar(0));

	EXPECT_TRUE(cgogn::geometry::intersection_ray_triangle(A0,D0,p0,p1,p2));
	EXPECT_TRUE(cgogn::geometry::intersection_ray_triangle(A1,D1,p0,p1,p2));
	EXPECT_TRUE(cgogn::geometry::intersection_ray_triangle(A2,D1,p0,p1,p2));
	EXPECT_FALSE(cgogn::geometry::intersection_ray_triangle(A3,D0,p0,p1,p2));

}

//TYPED_TEST(Intesection_TEST, intersection_sphere_segment)
//{
//	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;
//	TypeParam c(Scalar(1), Scalar(1), Scalar(1));
//	Scalar r(3);
//	Scalar a;

//	TypeParam p1(Scalar(0), Scalar(0), Scalar(0));
//	TypeParam p2(Scalar(2), Scalar(2), Scalar(2));
//	EXPECT_FALSE(cgogn::geometry::intersection_sphere_segment(c,r,p1,p2,a));

//	TypeParam p3(Scalar(-5), Scalar(-4), Scalar(-2));
//	TypeParam p4(Scalar(7), Scalar(6), Scalar(4));
//	EXPECT_TRUE(cgogn::geometry::intersection_sphere_segment(c,r,p3,p4,a));

//	p3 = TypeParam(Scalar(0), Scalar(0), Scalar(0));
//	EXPECT_TRUE(cgogn::geometry::intersection_sphere_segment(c,r,p3,p4,a));

//	p3 = TypeParam(Scalar(1), Scalar(1), Scalar(1));
//	p4 = TypeParam(Scalar(1), Scalar(1), Scalar(7));
//	cgogn::geometry::intersection_sphere_segment(c,r,p3,p4,a);
//	EXPECT_TRUE(cgogn::almost_equal_absolute(a,Scalar(0.5)));
//}




TYPED_TEST(Intesection_TEST, intersection_line_plane)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;
	TypeParam a(Scalar(-1), Scalar(-1), Scalar(-3));
	TypeParam v(Scalar(3), Scalar(0), Scalar(0));
	TypeParam p_p(Scalar(2), Scalar(2), Scalar(1));
	TypeParam p_d(Scalar(5), Scalar(1), Scalar(3));

	EXPECT_TRUE(cgogn::geometry::intersection_line_plane(a,v,p_p,p_d));

	p_d = TypeParam(Scalar(0), Scalar(0), Scalar(2));
	EXPECT_FALSE(cgogn::geometry::intersection_line_plane(a,v,p_p,p_d));

}

