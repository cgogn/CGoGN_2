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

#include <gtest/gtest.h>

#include <core/basic/dart.h>

namespace cgogn
{

TEST(DartTest, DefaultConstructor)
{
	Dart d;
	EXPECT_EQ(std::numeric_limits<unsigned int>::max(), d.index);
}

TEST(DartTest, Constructor)
{
	Dart d(10u);
	EXPECT_EQ(10u, d.index);
}

TEST(DartTest, OutOfLimitConstructor)
{
	Dart d1(std::numeric_limits<unsigned int>::max());
	Dart d2;
	EXPECT_EQ(d1.index, d2.index);
}

TEST(DartTest, CopyConstructor)
{
	Dart d(20u);
	Dart dcopy(d);
	EXPECT_EQ(d.index, dcopy.index);
}

TEST(DartTest, IsNil)
{
	Dart d;
	EXPECT_TRUE(d.is_nil());
}

TEST(DartTest, Assignation)
{
	Dart d1(10u);
	Dart d2;
	d2 = d1;
	EXPECT_EQ(d2.index, d1.index);
}

TEST(DartTest, Equality)
{
	Dart d1(10u);
	Dart d2(10u);
	EXPECT_EQ(d2.index, d1.index);
}

TEST(DartTest, Difference)
{
	Dart d1(10u);
	Dart d2(100u);
	EXPECT_EQ(10u, d1.index);
	EXPECT_EQ(100u, d2.index);
}

TEST(DartTest, PrintingOut)
{
	Dart d(10u);
	std::ostringstream s;
	s << "d=" << d;
	EXPECT_EQ(0, strcmp(s.str().c_str(), "d=10"));
}

TEST(DartTest, ReadingIn)
{
	Dart d;
	std::istringstream s("10");
	s >> d;

	EXPECT_EQ(10u, d.index);
}

} // namespace cgogn
