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

#include <cgogn/core/utils/name_types.h>
#include <cgogn/core/basic/cell.h>
#include <gtest/gtest.h>


using namespace cgogn::numerics;

TEST(NameTypesTest, NumTypes)
{
	EXPECT_EQ(cgogn::name_of_type(bool()), "bool");
	EXPECT_EQ(cgogn::name_of_type(char()), "char");
	EXPECT_EQ(cgogn::name_of_type(int8()), "int8");
	EXPECT_EQ(cgogn::name_of_type(uint8()), "uint8");
	EXPECT_EQ(cgogn::name_of_type(wchar_t()), "wchar_t");

#if _MSC_VER == 1800 // VS2013
	EXPECT_EQ(cgogn::name_of_type(char16_t()), "uint16");
	EXPECT_EQ(cgogn::name_of_type(char32_t()), "uint32");
#else
	EXPECT_EQ(cgogn::name_of_type(char16_t()), "char16_t");
	EXPECT_EQ(cgogn::name_of_type(char32_t()), "char32_t");
#endif // VS2013

	EXPECT_EQ(cgogn::name_of_type(int16()), "int16");
	EXPECT_EQ(cgogn::name_of_type(uint16()), "uint16");

	EXPECT_EQ(cgogn::name_of_type(int32()), "int32");
	EXPECT_EQ(cgogn::name_of_type(uint32()), "uint32");

	EXPECT_EQ(cgogn::name_of_type(int64()), "int64");
	EXPECT_EQ(cgogn::name_of_type(uint64()), "uint64");

	EXPECT_EQ(cgogn::name_of_type(float()), "float32");
	EXPECT_EQ(cgogn::name_of_type(double()), "float64");

	EXPECT_EQ(cgogn::name_of_type(std::string()), "std::basic_string<char>");
	EXPECT_EQ(cgogn::name_of_type(std::vector<float64>()), "std::vector<float64>");
	EXPECT_EQ(cgogn::name_of_type(std::vector<std::list<float64>>()), "std::vector<std::list<float64>>");
	EXPECT_EQ(cgogn::name_of_type(std::array<float64,3>()), "std::array<float64,3>");

	EXPECT_EQ(cgogn::name_of_type(cgogn::Dart()), "cgogn::Dart");
	EXPECT_EQ(cgogn::name_of_type(cgogn::Cell<cgogn::Orbit::PHI1>()), "cgogn::Cell<cgogn::Orbit::PHI1>");
}
