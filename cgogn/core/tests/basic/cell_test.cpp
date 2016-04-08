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

#include <cgogn/core/basic/cell.h>

namespace cgogn
{
uint32_t toto;
std::uint32_t tutu;

const Dart dglobal(10u);
const Dart dmax(std::numeric_limits<uint32>::max());


TEST(CellTest, DefaultConstructor)
{
	Cell<Orbit::DART> c;
	Dart d = c.dart;
	EXPECT_EQ(std::numeric_limits<uint32>::max(), d.index);
}

TEST(CellTest, Constructor)
{
	Cell<Orbit::DART> c(dglobal);
	Dart d = c.dart;
	EXPECT_EQ(10u, d.index);
}

TEST(CellTest, OutOfLimitConstructor)
{
	Cell<Orbit::DART> c1(dmax);
	Dart d1 = c1.dart;
	Cell<Orbit::DART> c2;
	Dart d2 = c2.dart;
	EXPECT_EQ(d1.index, d2.index);
}

TEST(CellTest, CopyConstructor)
{
	Cell<Orbit::DART> c(dglobal);
	Dart d = c.dart;
	Cell<Orbit::DART> ccopy(c);
	Dart dcopy = ccopy.dart;
	EXPECT_EQ(d.index, dcopy.index);
}

TEST(CellTest, IsValid)
{
	Cell<Orbit::DART> c(dglobal);
	EXPECT_TRUE(c.is_valid());
}

TEST(CellTest, Assignation)
{
	Cell<Orbit::DART> c1(dglobal);
	Cell<Orbit::DART> c2;
	c2 = c1;

	Dart d2 = c2.dart;

	EXPECT_EQ(d2.index, dglobal.index);
}

TEST(CellTest, PrintingOut)
{
	Cell<Orbit::DART> c(dglobal);
	std::ostringstream s;
	s << "c=" << c;
	EXPECT_EQ(0, strcmp(s.str().c_str(), "c=10"));
}

TEST(CellTest, ReadingIn)
{
	Cell<Orbit::DART> c;
	std::istringstream s("10");
	s >> c;

	Dart d = c.dart;

	EXPECT_EQ(10u, d.index);
}

} // namespace cgogn
