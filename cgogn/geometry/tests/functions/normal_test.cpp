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
#include <cgogn/geometry/functions/normal.h>
#include <cgogn/geometry/types/geometry_traits.h>

#include <gtest/gtest.h>

using StdArrayf = cgogn::geometry::Vec_T<std::array<float,3>>;
using StdArrayd = cgogn::geometry::Vec_T<std::array<double,3>>;
using EigenVec3f = Eigen::Vector3f;
using EigenVec3d = Eigen::Vector3d;
using VecTypes = testing::Types<StdArrayf, EigenVec3f, StdArrayd ,EigenVec3d>;

template <typename Vec_T>
class Normal_TEST : public testing::Test {};

TYPED_TEST_CASE(Normal_TEST, VecTypes );

TYPED_TEST(Normal_TEST, TriangleNormal)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;

	const Scalar tolerence = std::is_same<Scalar,double>::value ? Scalar(1e-8) : Scalar(1e-4f);
	TypeParam p0(Scalar(1), Scalar(3), Scalar(-5));
	TypeParam p1(Scalar(7), Scalar(-4), Scalar(0.1f));
	TypeParam p2(Scalar(-15), Scalar(-2), Scalar(15));;

	TypeParam n = cgogn::geometry::normal(p0,p1,p2);

	cgogn::almost_equal_relative(n.dot(p1-p0), Scalar(0));
	//		EXPECT_TRUE(cgogn::almost_equal_relative(n.dot(p1-p0),0.)); // is false !
	EXPECT_TRUE(cgogn::almost_equal_absolute(n.dot(p1-p0), Scalar(0), tolerence));
	EXPECT_TRUE(cgogn::almost_equal_relative(n.dot(p2-p0), Scalar(0)));
	EXPECT_TRUE(cgogn::almost_equal_relative(n.dot(p2-p1), Scalar(0)));
	//		EXPECT_DOUBLE_EQ(n.dot(p1-p0), 0.); // is false !
	EXPECT_NEAR(n.dot(p1-p0), Scalar(0), tolerence);
	EXPECT_DOUBLE_EQ(n.dot(p2-p0), Scalar(0));
	EXPECT_DOUBLE_EQ(n.dot(p2-p1), Scalar(0));
}
