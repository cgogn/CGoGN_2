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

#include <geometry/types/vec.h>
#include <geometry/types/eigen.h>

#include <gtest/gtest.h>


using StdArray = cgogn::geometry::Vec_T<std::array<double,3>>;
using EigenVec3d = Eigen::Vector3d;

TEST(VEC_OP_TEST, CGOGN_Typename)
{
	EXPECT_EQ(cgogn::name_of_type(StdArray()),"cgogn::geometry::Vec_T<std::array<double,3>>");
	EXPECT_EQ(cgogn::name_of_type(EigenVec3d()), "Eigen::Matrix<double,3,1,0,3,1>");
}

TEST(VEC_OP_TEST, Constructor)
{
	StdArray vec1{0.,0.,0.};
	EigenVec3d vec2 = {0.,0.,0.};
	EXPECT_EQ(vec1[0],0);
	EXPECT_EQ(vec1[1],0);
	EXPECT_EQ(vec1[2],0);
	EXPECT_EQ(vec2[0],0);
	EXPECT_EQ(vec2[1],0);
	EXPECT_EQ(vec2[2],0);
}

TEST(VEC_OP_TEST, CopyConstructor)
{
	StdArray vec1a = {1.,2., 3.};
	StdArray vec1b(vec1a);
	EigenVec3d vec2a = {1.,2., 3.};
	EigenVec3d vec2b(vec2a);
	EXPECT_EQ(vec1a[0], vec1b[0]);
	EXPECT_EQ(vec1a[1], vec1b[1]);
	EXPECT_EQ(vec1a[2], vec1b[2]);
	EXPECT_EQ(vec2a[0], vec2b[0]);
	EXPECT_EQ(vec2a[1], vec2b[1]);
	EXPECT_EQ(vec2a[2], vec2b[2]);
}

TEST(VEC_OP_TEST, AssignConstructor)
{
	StdArray vec1a = {1.,2., 3.};
	StdArray vec1b;
	vec1b = vec1a;
	EigenVec3d vec2a = {1.,2., 3.};
	EigenVec3d vec2b;
	vec2b = vec2a;

	EXPECT_EQ(vec1a[0], vec1b[0]);
	EXPECT_EQ(vec1a[1], vec1b[1]);
	EXPECT_EQ(vec1a[2], vec1b[2]);
	EXPECT_EQ(vec2a[0], vec2b[0]);
	EXPECT_EQ(vec2a[1], vec2b[1]);
	EXPECT_EQ(vec2a[2], vec2b[2]);
}

TEST(VEC_OP_TEST, UnaryMinus)
{
	StdArray vec1a = {1.,2., 3.};
	StdArray vec1b = -vec1a;

	EigenVec3d vec2a = {1.,2., 3.};
	EigenVec3d vec2b = -vec2a;

	EXPECT_EQ(vec1a[0], -vec1b[0]);
	EXPECT_EQ(vec1a[1], -vec1b[1]);
	EXPECT_EQ(vec1a[2], -vec1b[2]);
	EXPECT_EQ(vec2a[0], -vec2b[0]);
	EXPECT_EQ(vec2a[1], -vec2b[1]);
	EXPECT_EQ(vec2a[2], -vec2b[2]);
}

TEST(VEC_OP_TEST, PlusAssign)
{
	{
		StdArray a = {1.,2., 3.};
		StdArray b = {1.,2., 3.};
		b += a;
		EXPECT_EQ(b[0], 2);
		EXPECT_EQ(b[1], 4);
		EXPECT_EQ(b[2], 6);
	}

	{
		EigenVec3d a = {1.,2., 3.};
		EigenVec3d b = {1.,2., 3.};
		b += a;
		EXPECT_EQ(b[0], 2);
		EXPECT_EQ(b[1], 4);
		EXPECT_EQ(b[2], 6);
	}
}

TEST(VEC_OP_TEST, MinusAssign)
{
	{
		StdArray a = {-1.,-2., -3.};
		StdArray b = {1.,2., 3.};
		b -= a;
		EXPECT_EQ(b[0], 2);
		EXPECT_EQ(b[1], 4);
		EXPECT_EQ(b[2], 6);
	}

	{
		EigenVec3d a = {-1.,-2., -3.};
		EigenVec3d b = {1.,2., 3.};
		b -= a;
		EXPECT_EQ(b[0], 2);
		EXPECT_EQ(b[1], 4);
		EXPECT_EQ(b[2], 6);
	}
}

TEST(VEC_OP_TEST, MultAssign)
{
	{
		StdArray a = {1.,2., 3.};
		a *= 2;
		EXPECT_EQ(a[0], 2);
		EXPECT_EQ(a[1], 4);
		EXPECT_EQ(a[2], 6);
	}

	{
		EigenVec3d a = {1.,2., 3.};
		a *= 2;
		EXPECT_EQ(a[0], 2);
		EXPECT_EQ(a[1], 4);
		EXPECT_EQ(a[2], 6);
	}
}

TEST(VEC_OP_TEST, DivAssign)
{
	{
		StdArray a{2.,4., 6.};
		a /= 2;
		EXPECT_EQ(a[0], 1);
		EXPECT_EQ(a[1], 2);
		EXPECT_EQ(a[2], 3);
	}

	{
		EigenVec3d a{2.,4., 6.};
		a /= 2;
		EXPECT_EQ(a[0], 1);
		EXPECT_EQ(a[1], 2);
		EXPECT_EQ(a[2], 3);
	}
}

TEST(VEC_OP_TEST, Plus)
{
	{
		StdArray a = {1.,2., 3.};
		StdArray b = {1.,2., 3.};
		StdArray c = a+b;
		EXPECT_EQ(c[0], 2);
		EXPECT_EQ(c[1], 4);
		EXPECT_EQ(c[2], 6);
	}

	{
		EigenVec3d a = {1.,2., 3.};
		EigenVec3d b = {1.,2., 3.};
		EigenVec3d c = a+b;
		EXPECT_EQ(c[0], 2);
		EXPECT_EQ(c[1], 4);
		EXPECT_EQ(c[2], 6);
	}
}

TEST(VEC_OP_TEST, Minus)
{
	{
		StdArray a = {1.,2., 3.};
		StdArray b = {-1.,-2., -3.};
		StdArray c = a-b;
		EXPECT_EQ(c[0], 2);
		EXPECT_EQ(c[1], 4);
		EXPECT_EQ(c[2], 6);
	}

	{
		EigenVec3d a = {1.,2., 3.};
		EigenVec3d b = {-1.,-2., -3.};
		EigenVec3d c = a-b;
		EXPECT_EQ(c[0], 2);
		EXPECT_EQ(c[1], 4);
		EXPECT_EQ(c[2], 6);
	}
}

TEST(VEC_OP_TEST, MultScalar)
{
	{
		StdArray a = {1.,2., 3.};
		StdArray c = a*2;
		StdArray d = 2*a;
		EXPECT_EQ(c[0], 2);
		EXPECT_EQ(c[1], 4);
		EXPECT_EQ(c[2], 6);
		EXPECT_EQ(d[0], 2);
		EXPECT_EQ(d[1], 4);
		EXPECT_EQ(d[2], 6);
	}

	{
		EigenVec3d a = {1.,2., 3.};
		EigenVec3d c = a*2;
		EigenVec3d d = 2*a;
		EXPECT_EQ(c[0], 2);
		EXPECT_EQ(c[1], 4);
		EXPECT_EQ(c[2], 6);
		EXPECT_EQ(d[0], 2);
		EXPECT_EQ(d[1], 4);
		EXPECT_EQ(d[2], 6);
	}
}

TEST(VEC_OP_TEST, Norm2)
{
	StdArray vec1a = {1.,2., 3.};
	EigenVec3d vec2a = {1.,2., 3.};

	EXPECT_EQ(vec1a.squaredNorm(), 14.);
	EXPECT_EQ(vec2a.squaredNorm(), 14.);
}

TEST(VEC_OP_TEST, Norm)
{
	{
	StdArray a = {3.,-4., 0.};
	EXPECT_EQ(a.norm(), 5.);
	}

	{
	EigenVec3d a = {3.,-4., 0.};
	EXPECT_EQ(a.norm(), 5.);
	}
}

TEST(VEC_OP_TEST, Normalize)
{
	{
	StdArray a = {3.,-4., 0.};
	a.normalize();
	EXPECT_DOUBLE_EQ(a[0], 3./5.);
	EXPECT_DOUBLE_EQ(a[1], -4./5.);
	EXPECT_DOUBLE_EQ(a[2], 0.);
	EXPECT_DOUBLE_EQ(a.norm(), 1.);
	}

	{
	EigenVec3d a = {3.,-4., 0.};
	a.normalize();
	EXPECT_DOUBLE_EQ(a[0], 3./5.);
	EXPECT_DOUBLE_EQ(a[1], -4./5.);
	EXPECT_DOUBLE_EQ(a[2], 0.);
	EXPECT_DOUBLE_EQ(a.norm(), 1.);
	}
}

TEST(VEC_OP_TEST, DotProduct)
{
	{
	StdArray a = {1.,-2., 3.};
	StdArray b = {1.,-2., 3.};
	EXPECT_EQ(a.dot(b), 14.);
	EXPECT_EQ(b.dot(a), 14.);
	}

	{
	StdArray a = {1.,2., -3.};
	StdArray b = {1.,2., -3.};
	EXPECT_EQ(a.dot(b), 14.);
	EXPECT_EQ(b.dot(a), 14.);
	}
}

TEST(VEC_OP_TEST, CrossProduct)
{
	{
	StdArray a = {1.,2., 3.};
	StdArray b = {3.,2., 1.};
	StdArray c = a.cross(b);
	EXPECT_EQ(c[0], -4.);
	EXPECT_EQ(c[1], 8.);
	EXPECT_EQ(c[2], -4.);
	}

	{
	EigenVec3d a = {1.,2., 3.};
	EigenVec3d b = {3.,2., 1.};
	EigenVec3d c = a.cross(b);
	EXPECT_EQ(c[0], -4.);
	EXPECT_EQ(c[1], 8.);
	EXPECT_EQ(c[2], -4.);
	}
}

