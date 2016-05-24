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

#include <cgogn/core/basic/dart.h>

namespace cgogn
{

class DartTest : public ::testing::Test
{

public:

	DartTest() : d10a_(10u), d10b_(10u), d20a_(20u),
		dMax_(std::numeric_limits<uint32>::max()) {}

	// virtual void TearDown() {}

	const Dart dNil_;
	const Dart d10a_;
	const Dart d10b_;
	const Dart d20a_;
	const Dart dMax_;
};

TEST_F(DartTest, DefaultConstructor)
{
	EXPECT_EQ(std::numeric_limits<uint32>::max(), dNil_.index);
}

TEST_F(DartTest, Constructor)
{
	EXPECT_EQ(10u, d10a_.index);
	EXPECT_EQ(dNil_.index, dMax_.index);
}

TEST_F(DartTest, CopyConstructor)
{
	Dart d1(d10a_);
	Dart d2(dNil_);
	EXPECT_EQ(d1.index, d10a_.index);
	EXPECT_EQ(d2.index, dNil_.index);
}

TEST_F(DartTest, IsNil)
{
	EXPECT_TRUE(dNil_.is_nil());
	EXPECT_TRUE(dMax_.is_nil());
	EXPECT_FALSE(d10a_.is_nil());
}

TEST_F(DartTest, Assignation)
{
	Dart d1 = d10a_;
	Dart d2 = dNil_;
	EXPECT_EQ(d1.index, d10a_.index);
	EXPECT_EQ(d2.index, dNil_.index);
}

TEST_F(DartTest, Equality)
{
	EXPECT_TRUE(d10a_ == d10a_);
	EXPECT_TRUE(d10a_ == d10b_);
	EXPECT_TRUE(dNil_ == dMax_);
	EXPECT_FALSE(d10a_ == dNil_);
	EXPECT_FALSE(d10a_ == d20a_);
}

TEST_F(DartTest, Difference)
{
	EXPECT_TRUE(d10a_ != d20a_);
	EXPECT_TRUE(d10a_ != dNil_);
	EXPECT_FALSE(d10a_ != d10a_);
	EXPECT_FALSE(d10a_ != d10b_);
}

TEST_F(DartTest, PrintingOut)
{
	std::ostringstream s;
	s << d10a_;
	EXPECT_STREQ(s.str().c_str(), "10");
	std::ostringstream t;
	t << dNil_;
	EXPECT_STREQ(t.str().c_str(), "4294967295");
}

TEST_F(DartTest, ReadingIn)
{
	Dart d;
	std::istringstream s("10");
	s >> d;
	EXPECT_TRUE(d == d10a_);
	Dart e;
	std::istringstream t("4294967295");
	t >> e;
	EXPECT_TRUE(e == dNil_);
}

} // namespace cgogn
