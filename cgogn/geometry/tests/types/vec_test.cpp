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
#include <cgogn/geometry/types/vec.h>
#include <cgogn/geometry/types/eigen.h>
#include <cgogn/geometry/types/geometry_traits.h>

#include <gtest/gtest.h>

using namespace cgogn::numerics;

using StdArrayf = cgogn::geometry::Vec_T<std::array<float32,3>>;
using StdArrayd = cgogn::geometry::Vec_T<std::array<float64,3>>;
using EigenVec3f = Eigen::Vector3f;
using EigenVec3d = Eigen::Vector3d;
using VecTypes = testing::Types<StdArrayf, EigenVec3f, StdArrayd ,EigenVec3d>;

template <typename Vec_T>
class VEC_OP_TEST : public testing::Test
{
};

TYPED_TEST_CASE(VEC_OP_TEST, VecTypes );

TEST(VEC_OP_TEST, CGOGN_Typename)
{
	EXPECT_EQ(cgogn::name_of_type(StdArrayf()),"cgogn::geometry::Vec_T<std::array<float32,3>>");
	EXPECT_EQ(cgogn::name_of_type(EigenVec3f()), "Eigen::Matrix<float,3,1,0,3,1>");
	EXPECT_EQ(cgogn::name_of_type(StdArrayd()),"cgogn::geometry::Vec_T<std::array<float64,3>>");
	EXPECT_EQ(cgogn::name_of_type(EigenVec3d()), "Eigen::Matrix<double,3,1,0,3,1>");
}

TYPED_TEST(VEC_OP_TEST, Constructor)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;

	const Scalar zero(0);
	const TypeParam vec1{zero, zero, zero};
	EXPECT_EQ(vec1[0],zero);
	EXPECT_EQ(vec1[1],zero);
	EXPECT_EQ(vec1[2],zero);
}

TYPED_TEST(VEC_OP_TEST, CopyConstructor)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;

	const TypeParam vec1a = {Scalar(1), Scalar(2), Scalar(3)};
	TypeParam vec1b(vec1a);
	EXPECT_EQ(vec1a[0], vec1b[0]);
	EXPECT_EQ(vec1a[1], vec1b[1]);
	EXPECT_EQ(vec1a[2], vec1b[2]);
}

TYPED_TEST(VEC_OP_TEST, AssignConstructor)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;

	const TypeParam vec1a = {Scalar(1), Scalar(2), Scalar(3)};
	TypeParam vec1b;
	vec1b = vec1a;
	EXPECT_EQ(vec1a[0], vec1b[0]);
	EXPECT_EQ(vec1a[1], vec1b[1]);
	EXPECT_EQ(vec1a[2], vec1b[2]);
}

TYPED_TEST(VEC_OP_TEST, UnaryMinus)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;

	const TypeParam vec1a = {Scalar(1), Scalar(2), Scalar(3)};
	const TypeParam vec1b = -vec1a;
	EXPECT_EQ(vec1a[0], -vec1b[0]);
	EXPECT_EQ(vec1a[1], -vec1b[1]);
	EXPECT_EQ(vec1a[2], -vec1b[2]);
}

TYPED_TEST(VEC_OP_TEST, PlusAssign)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;

	const TypeParam a = {Scalar(1), Scalar(2), Scalar(3)};
	TypeParam b = {Scalar(7), Scalar(5), Scalar(9)};
	b += a;
	EXPECT_EQ(b[0], 8);
	EXPECT_EQ(b[1], 7);
	EXPECT_EQ(b[2], 12);
}

TYPED_TEST(VEC_OP_TEST, MinusAssign)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;

	const TypeParam a = {Scalar(-1), Scalar(-2), Scalar(-3)};
	TypeParam b = {Scalar(7), Scalar(5), Scalar(9)};
	b -= a;
	EXPECT_EQ(b[0], 8);
	EXPECT_EQ(b[1], 7);
	EXPECT_EQ(b[2], 12);
}

TYPED_TEST(VEC_OP_TEST, MultAssign)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;

	TypeParam a = {Scalar(7), Scalar(5), Scalar(9)};
	a *= 2;
	EXPECT_EQ(a[0], 14);
	EXPECT_EQ(a[1], 10);
	EXPECT_EQ(a[2], 18);
}

TYPED_TEST(VEC_OP_TEST, DivAssign)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;

	TypeParam a = {Scalar(2), Scalar(4), Scalar(6)};
	a /= 2;
	EXPECT_EQ(a[0], 1);
	EXPECT_EQ(a[1], 2);
	EXPECT_EQ(a[2], 3);
}

TYPED_TEST(VEC_OP_TEST, Plus)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;

	const TypeParam a = {Scalar(1), Scalar(2), Scalar(3)};
	const TypeParam b = {Scalar(7), Scalar(5), Scalar(9)};
	const TypeParam c = a + b;
	EXPECT_EQ(c[0], a[0] + b[0]);
	EXPECT_EQ(c[1], a[1] + b[1]);
	EXPECT_EQ(c[2], a[2] + b[2]);
}

TYPED_TEST(VEC_OP_TEST, Minus)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;

	const TypeParam a = {Scalar(1), Scalar(2), Scalar(3)};
	const TypeParam b = {Scalar(7), Scalar(5), Scalar(9)};
	const TypeParam c = a - b;
	EXPECT_EQ(c[0], a[0] - b[0]);
	EXPECT_EQ(c[1], a[1] - b[1]);
	EXPECT_EQ(c[2], a[2] - b[2]);
}

TYPED_TEST(VEC_OP_TEST, MultScalar)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;

	TypeParam a = {Scalar(1), Scalar(2), Scalar(3)};
	const TypeParam c = a*2;
	const TypeParam d = 2*a;
	EXPECT_EQ(c[0], 2 * a[0]);
	EXPECT_EQ(c[1], 2 * a[1]);
	EXPECT_EQ(c[2], 2 * a[2]);
	EXPECT_EQ(d[0], 2 * a[0]);
	EXPECT_EQ(d[1], 2 * a[1]);
	EXPECT_EQ(d[2], 2 * a[2]);
}

TYPED_TEST(VEC_OP_TEST, Norm2)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;

	const TypeParam a = {Scalar(1), Scalar(2), Scalar(3)};
	EXPECT_EQ(a.squaredNorm(), 14.);
}

TYPED_TEST(VEC_OP_TEST, Norm)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;

	const TypeParam a = {Scalar(3), Scalar(-4), Scalar(0)};
	EXPECT_EQ(a.norm(), 5.);
}

TYPED_TEST(VEC_OP_TEST, Normalize)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;

	TypeParam a = {Scalar(3), Scalar(-4), Scalar(0)};
	a.normalize();
	EXPECT_DOUBLE_EQ(a[0], Scalar(3)/Scalar(5));
	EXPECT_DOUBLE_EQ(a[1], Scalar(-4)/Scalar(5));
	EXPECT_DOUBLE_EQ(a[2], Scalar(0));
	EXPECT_DOUBLE_EQ(a.norm(), Scalar(1));
}

TYPED_TEST(VEC_OP_TEST, DotProduct)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;

	TypeParam a = {Scalar(3), Scalar(-4), Scalar(10)};
	TypeParam b = {Scalar(-1), Scalar(5), Scalar(2)};
	EXPECT_EQ(a.dot(b), -3);
	EXPECT_EQ(b.dot(a), -3);
}

TYPED_TEST(VEC_OP_TEST, CrossProduct)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;

	const TypeParam a = {Scalar(1), Scalar(2), Scalar(3)};
	const TypeParam b = {Scalar(3), Scalar(2), Scalar(1)};
	const TypeParam c = a.cross(b);
	EXPECT_EQ(c[0], -4.);
	EXPECT_EQ(c[1], 8.);
	EXPECT_EQ(c[2], -4.);
}

TYPED_TEST(VEC_OP_TEST, Equality)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;

	const TypeParam a = {Scalar(1), Scalar(2), Scalar(3)};
	const TypeParam b= {Scalar(1), Scalar(2), Scalar(3)};
	EXPECT_TRUE(a == b);
}

TYPED_TEST(VEC_OP_TEST, DivScalar)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;

	const TypeParam a = {Scalar(2), Scalar(4), Scalar(6)};
	const TypeParam c = a/2;
	EXPECT_EQ(c[0], 1);
	EXPECT_EQ(c[1], 2);
	EXPECT_EQ(c[2], 3);
}
