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
#include <cgogn/geometry/types/aabb.h>
#include <cgogn/geometry/types/geometry_traits.h>

#include <gtest/gtest.h>

using namespace cgogn::numerics;

using StdArrayf = cgogn::geometry::Vec_T<std::array<float32,3>>;
using StdArrayd = cgogn::geometry::Vec_T<std::array<float64,3>>;
using EigenVec3f = Eigen::Vector3f;
using EigenVec3d = Eigen::Vector3d;
using VecTypes = testing::Types<StdArrayf, EigenVec3f, StdArrayd ,EigenVec3d>;

template <typename Vec_T>
class AABB_TEST : public testing::Test
{
protected :
	cgogn::geometry::AABB<Vec_T> bb_;
};

TYPED_TEST_CASE(AABB_TEST, VecTypes );

TEST(AABB_TEST, NameOfType)
{
	EXPECT_EQ(cgogn::name_of_type(cgogn::geometry::AABB<StdArrayf>()), "cgogn::geometry::AABB<cgogn::geometry::Vec_T<std::array<float32,3>>>");
	EXPECT_EQ(cgogn::name_of_type(cgogn::geometry::AABB<EigenVec3f>()), "cgogn::geometry::AABB<Eigen::Matrix<float,3,1,0,3,1>>");
	EXPECT_EQ(cgogn::name_of_type(cgogn::geometry::AABB<StdArrayd>()), "cgogn::geometry::AABB<cgogn::geometry::Vec_T<std::array<float64,3>>>");
	EXPECT_EQ(cgogn::name_of_type(cgogn::geometry::AABB<EigenVec3d>()), "cgogn::geometry::AABB<Eigen::Matrix<double,3,1,0,3,1>>");
}

TYPED_TEST(AABB_TEST, Basics)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;

	this->bb_.add_point(TypeParam({Scalar(0.5f), Scalar(0.4f), Scalar(0.3f)}));
	this->bb_.add_point(TypeParam({Scalar(-1), Scalar(-2), Scalar(-3)}));
	this->bb_.add_point(TypeParam({Scalar(1), Scalar(2), Scalar(3)}));

	EXPECT_EQ(this->bb_.min(), TypeParam({Scalar(-1), Scalar(-2), Scalar(-3)}));
	EXPECT_EQ(this->bb_.max(), TypeParam({Scalar(1), Scalar(2), Scalar(3)}));
	EXPECT_EQ(this->bb_.max_size(), Scalar(6));
	EXPECT_EQ(this->bb_.min_size(), Scalar(2));
	EXPECT_TRUE(cgogn::almost_equal_relative(this->bb_.diag_size(), std::sqrt(Scalar(2*2+4*4+6*6))));
	EXPECT_EQ(this->bb_.center(), TypeParam({Scalar(0), Scalar(0), Scalar(0)}));
}

TYPED_TEST(AABB_TEST, testing)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;

	this->bb_.add_point(TypeParam({Scalar(0.5f), Scalar(0.4f), Scalar(0.3f)}));
	this->bb_.add_point(TypeParam({Scalar(-1), Scalar(-2), Scalar(-3)}));
	this->bb_.add_point(TypeParam({Scalar(1), Scalar(2), Scalar(3)}));

	EXPECT_TRUE(this->bb_.contains(TypeParam({Scalar(1), Scalar(1), Scalar(1)})));

	cgogn::geometry::AABB<TypeParam> bb2;
	bb2.add_point(TypeParam({Scalar(0), Scalar(0), Scalar(0)}));
	bb2.add_point(TypeParam({Scalar(4), Scalar(5), Scalar(2)}));

	EXPECT_TRUE(this->bb_.intersects(bb2));

	cgogn::geometry::AABB<TypeParam> bb3;
	bb3.add_point(TypeParam({Scalar(0), Scalar(0), Scalar(0)}));
	bb3.add_point(TypeParam({Scalar(1), Scalar(1), Scalar(1)}));

	EXPECT_TRUE(this->bb_.contains(bb3));

//	EXPECT_TRUE(this->bb_.ray_intersect(TypeParam({Scalar(-9), Scalar(-9), Scalar(-9)}), TypeParam({Scalar(1), Scalar(1), Scalar(1)})));
//	EXPECT_FALSE(this->bb_.ray_intersect(TypeParam({Scalar(-9), Scalar(-9), Scalar(-9)}), TypeParam({Scalar(1), Scalar(-1), Scalar(0)})));
}
