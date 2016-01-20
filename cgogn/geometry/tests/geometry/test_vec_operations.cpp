#include <geometry/vec_operations.h>
#include <gtest/gtest.h>
#include <iostream>

using Vec1 = std::array<double,3>;
using Vec2 = Eigen::Vector3d;

TEST(VEC_OP_TEST, Constructor)
{
	Vec1 vec1 = {0.,0.,0.};
	Vec2 vec2 = {0.,0.,0.};
	EXPECT_EQ(vec1[0],0);
	EXPECT_EQ(vec1[1],0);
	EXPECT_EQ(vec1[2],0);
	EXPECT_EQ(vec2[0],0);
	EXPECT_EQ(vec2[1],0);
	EXPECT_EQ(vec2[2],0);
}

TEST(VEC_OP_TEST, CopyConstructor)
{
	Vec1 vec1a = {1.,2., 3.};
	Vec1 vec1b(vec1a);
	Vec2 vec2a = {1.,2., 3.};
	Vec2 vec2b(vec2a);
	EXPECT_EQ(vec1a[0], vec1b[0]);
	EXPECT_EQ(vec1a[1], vec1b[1]);
	EXPECT_EQ(vec1a[2], vec1b[2]);
	EXPECT_EQ(vec2a[0], vec2b[0]);
	EXPECT_EQ(vec2a[1], vec2b[1]);
	EXPECT_EQ(vec2a[2], vec2b[2]);
}

TEST(VEC_OP_TEST, AssignConstructor)
{
	Vec1 vec1a = {1.,2., 3.};
	Vec1 vec1b;
	vec1b = vec1a;
	Vec2 vec2a = {1.,2., 3.};
	Vec2 vec2b;
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
	using cgogn::geometry::operator -;
	Vec1 vec1a = {1.,2., 3.};
	Vec1 vec1b = -vec1a;

	Vec2 vec2a = {1.,2., 3.};
	Vec2 vec2b = -vec2a;

	EXPECT_EQ(vec1a[0], -vec1b[0]);
	EXPECT_EQ(vec1a[1], -vec1b[1]);
	EXPECT_EQ(vec1a[2], -vec1b[2]);
	EXPECT_EQ(vec2a[0], -vec2b[0]);
	EXPECT_EQ(vec2a[1], -vec2b[1]);
	EXPECT_EQ(vec2a[2], -vec2b[2]);
}

TEST(VEC_OP_TEST, PlusAssign)
{
	using cgogn::geometry::operator +=;

	{
		Vec1 a = {1.,2., 3.};
		Vec1 b = {1.,2., 3.};
		b += a;
		EXPECT_EQ(b[0], 2);
		EXPECT_EQ(b[1], 4);
		EXPECT_EQ(b[2], 6);
	}

	{
		Vec2 a = {1.,2., 3.};
		Vec2 b = {1.,2., 3.};
		b += a;
		EXPECT_EQ(b[0], 2);
		EXPECT_EQ(b[1], 4);
		EXPECT_EQ(b[2], 6);
	}
}

TEST(VEC_OP_TEST, MinusAssign)
{
	using cgogn::geometry::operator -=;

	{
		Vec1 a = {-1.,-2., -3.};
		Vec1 b = {1.,2., 3.};
		b -= a;
		EXPECT_EQ(b[0], 2);
		EXPECT_EQ(b[1], 4);
		EXPECT_EQ(b[2], 6);
	}

	{
		Vec2 a = {-1.,-2., -3.};
		Vec2 b = {1.,2., 3.};
		b -= a;
		EXPECT_EQ(b[0], 2);
		EXPECT_EQ(b[1], 4);
		EXPECT_EQ(b[2], 6);
	}
}

TEST(VEC_OP_TEST, Plus)
{
	using cgogn::geometry::operator +;

	{
		Vec1 a = {1.,2., 3.};
		Vec1 b = {1.,2., 3.};
		Vec1 c = a+b;
		EXPECT_EQ(c[0], 2);
		EXPECT_EQ(c[1], 4);
		EXPECT_EQ(c[2], 6);
	}

	{
		Vec2 a = {1.,2., 3.};
		Vec2 b = {1.,2., 3.};
		Vec2 c = a+b;
		EXPECT_EQ(c[0], 2);
		EXPECT_EQ(c[1], 4);
		EXPECT_EQ(c[2], 6);
	}
}

TEST(VEC_OP_TEST, Minus)
{
	using cgogn::geometry::operator -;

	{
		Vec1 a = {1.,2., 3.};
		Vec1 b = {-1.,-2., -3.};
		Vec1 c = a-b;
		EXPECT_EQ(c[0], 2);
		EXPECT_EQ(c[1], 4);
		EXPECT_EQ(c[2], 6);
	}

	{
		Vec2 a = {1.,2., 3.};
		Vec2 b = {-1.,-2., -3.};
		Vec2 c = a-b;
		EXPECT_EQ(c[0], 2);
		EXPECT_EQ(c[1], 4);
		EXPECT_EQ(c[2], 6);
	}
}

TEST(VEC_OP_TEST, MultScalar)
{
	using cgogn::geometry::operator *;

	{
		Vec1 a = {1.,2., 3.};
		Vec1 c = a*2;
		Vec1 d = 2*a;
		EXPECT_EQ(c[0], 2);
		EXPECT_EQ(c[1], 4);
		EXPECT_EQ(c[2], 6);
		EXPECT_EQ(d[0], 2);
		EXPECT_EQ(d[1], 4);
		EXPECT_EQ(d[2], 6);
	}

	{
		Vec2 a = {1.,2., 3.};
		Vec2 c = a*2;
		Vec2 d = 2*a;
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
	Vec1 vec1a = {1.,2., 3.};
	Vec2 vec2a = {1.,2., 3.};

	EXPECT_EQ(cgogn::geometry::norm2(vec1a), 14.);
	EXPECT_EQ(cgogn::geometry::norm2(vec2a), 14.);
}

TEST(VEC_OP_TEST, Norm)
{
	{
	Vec1 a = {3.,-4., 0.};
	EXPECT_EQ(cgogn::geometry::norm(a), 5.);
	}

	{
	Vec2 a = {3.,-4., 0.};
	EXPECT_EQ(cgogn::geometry::norm(a), 5.);
	}
}

TEST(VEC_OP_TEST, Normalize)
{
	{
	Vec1 a = {3.,-4., 0.};
	double norm = cgogn::geometry::normalize(a);
	EXPECT_EQ(norm, 5);
	EXPECT_DOUBLE_EQ(a[0], 3./5.);
	EXPECT_DOUBLE_EQ(a[1], -4./5.);
	EXPECT_DOUBLE_EQ(a[2], 0.);
	EXPECT_DOUBLE_EQ(cgogn::geometry::norm(a), 1.);
	}

	{
	Vec2 a = {3.,-4., 0.};
	double norm = cgogn::geometry::normalize(a);
	EXPECT_EQ(norm, 5);
	EXPECT_DOUBLE_EQ(a[0], 3./5.);
	EXPECT_DOUBLE_EQ(a[1], -4./5.);
	EXPECT_DOUBLE_EQ(a[2], 0.);
	EXPECT_DOUBLE_EQ(cgogn::geometry::norm(a), 1.);
	}
}

TEST(VEC_OP_TEST, DotProduct)
{
	using cgogn::geometry::operator *;
	{
	Vec1 a = {1.,-2., 3.};
	Vec1 b = {1.,-2., 3.};
	EXPECT_EQ(a*b, 14.);
	}

	{
	Vec1 a = {1.,2., -3.};
	Vec1 b = {1.,2., -3.};
	EXPECT_EQ(b*a, 14.);
	}
}

TEST(VEC_OP_TEST, CrossProduct)
{
	using cgogn::geometry::operator ^;
	{
	Vec1 a = {1.,2., 3.};
	Vec1 b = {3.,2., 1.};
	Vec1 c = a^b;
	EXPECT_EQ(c[0], -4.);
	EXPECT_EQ(c[1], 8.);
	EXPECT_EQ(c[2], -4.);
	}

	{
	Vec2 a = {1.,2., 3.};
	Vec2 b = {3.,2., 1.};
	Vec2 c = a^b;
	EXPECT_EQ(c[0], -4.);
	EXPECT_EQ(c[1], 8.);
	EXPECT_EQ(c[2], -4.);
	}
}

