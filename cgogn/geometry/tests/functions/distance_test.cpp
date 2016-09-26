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

#include <cgogn/core/utils/numerics.h>
#include <cgogn/geometry/types/eigen.h>
#include <cgogn/geometry/types/vec.h>
#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/geometry/functions/distance.h>

#include <gtest/gtest.h>

using StdArrayf = cgogn::geometry::Vec_T<std::array<float,3>>;
using StdArrayd = cgogn::geometry::Vec_T<std::array<double,3>>;
using EigenVec3f = Eigen::Vector3f;
using EigenVec3d = Eigen::Vector3d;
using VecTypes = testing::Types<StdArrayf, EigenVec3f, StdArrayd ,EigenVec3d>;

template <typename Vec_T>
class Distance_TEST : public testing::Test
{};

TYPED_TEST_CASE(Distance_TEST, VecTypes );


TYPED_TEST(Distance_TEST, PointLineDistance)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;
	TypeParam A(Scalar(-4), Scalar(-4), Scalar(-4));
	TypeParam B(Scalar(3), Scalar(3), Scalar(3));

	TypeParam P0(Scalar(1), Scalar(1), Scalar(1));
	TypeParam P1(Scalar(20), Scalar(20), Scalar(20));
	TypeParam P2(Scalar(1), Scalar(1), Scalar(0));

	EXPECT_DOUBLE_EQ(cgogn::geometry::squared_distance_line_point(A,B,P0), Scalar(0));
	EXPECT_DOUBLE_EQ(cgogn::geometry::squared_distance_line_point(A,B,P1), Scalar(0));

	const Scalar tolerence = std::is_same<Scalar,double>::value ? Scalar(1e-8) : Scalar(1e-4f);
	EXPECT_NEAR(cgogn::geometry::squared_distance_line_point(A,B,P2), Scalar(2.0/3.0), tolerence);



}


TYPED_TEST(Distance_TEST, squared_distance_line_seg)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;
	TypeParam A(Scalar(-4), Scalar(-4), Scalar(-4));
	TypeParam B(Scalar(3), Scalar(3), Scalar(3));

	TypeParam P0(Scalar(1), Scalar(1), Scalar(2));
	TypeParam Q0(Scalar(1), Scalar(1), Scalar(0));
	EXPECT_DOUBLE_EQ(cgogn::geometry::squared_distance_line_seg(A,B,P0,Q0), Scalar(0));

	TypeParam P1(Scalar(20), Scalar(20), Scalar(20));
	TypeParam Q1(Scalar(22), Scalar(21), Scalar(23));
	EXPECT_DOUBLE_EQ(cgogn::geometry::squared_distance_line_seg(A,B,P1,Q1), Scalar(0));


	A = TypeParam(Scalar(0), Scalar(0), Scalar(-4));
	B = TypeParam(Scalar(0), Scalar(0), Scalar(3));

	TypeParam P2(Scalar(0), Scalar(3), Scalar(0));
	TypeParam Q2(Scalar(0), Scalar(3), Scalar(0));

	const Scalar tolerence = std::is_same<Scalar,double>::value ? Scalar(1e-8) : Scalar(1e-4f);
	EXPECT_NEAR(cgogn::geometry::squared_distance_line_seg(A,B,P2,Q2), Scalar(3*3), tolerence);

}
