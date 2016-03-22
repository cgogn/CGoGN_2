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

#include <core/utils/name_types.h>
#include <core/basic/cell.h>
#include <gtest/gtest.h>

using namespace cgogn::numerics;

TEST(NameTypesTest, NumTypes)
{
	using llint = long long;
	using ullint = unsigned long long;

	EXPECT_EQ(cgogn::name_of_type(bool()), "bool");
	EXPECT_EQ(cgogn::name_of_type(char()), "char");
	EXPECT_EQ(cgogn::name_of_type(int8()), "signed char");
	EXPECT_EQ(cgogn::name_of_type(uint8()), "unsigned char");
	EXPECT_EQ(cgogn::name_of_type(wchar_t()), "wchar_t");

#if _MSC_VER == 1800 // VS2013
	EXPECT_EQ(cgogn::name_of_type(char16_t()), "unsigned short");
	EXPECT_EQ(cgogn::name_of_type(char32_t()), "unsigned int");
#else
	EXPECT_EQ(cgogn::name_of_type(char16_t()), "char16_t");
	EXPECT_EQ(cgogn::name_of_type(char32_t()), "char32_t");
#endif // VS2013

	EXPECT_EQ(cgogn::name_of_type(int16()), "short");
	EXPECT_EQ(cgogn::name_of_type(uint16()), "unsigned short");
	EXPECT_EQ(cgogn::name_of_type(int32()), "int");

	EXPECT_EQ(cgogn::name_of_type(uint32()), "unsigned int");

// TODO A TIRER AU CLAIR !!
#ifdef _MSC_VER
	EXPECT_EQ(cgogn::name_of_type(int64()), "long long");
	EXPECT_EQ(cgogn::name_of_type(uint64()), "unsigned long long");
#else
	EXPECT_EQ(cgogn::name_of_type(int64()), "long");
	EXPECT_EQ(cgogn::name_of_type(uint64()), "unsigned long");
#endif


	EXPECT_EQ(cgogn::name_of_type(llint()), "long long");
	EXPECT_EQ(cgogn::name_of_type(ullint()), "unsigned long long");

	EXPECT_EQ(cgogn::name_of_type(float()), "float");
	EXPECT_EQ(cgogn::name_of_type(double()), "double");

	EXPECT_EQ(cgogn::name_of_type(std::string()), "std::basic_string<char>");
	EXPECT_EQ(cgogn::name_of_type(std::vector<double>()), "std::vector<double>");
	EXPECT_EQ(cgogn::name_of_type(std::vector<std::list<double>>()), "std::vector<std::list<double>>");
	EXPECT_EQ(cgogn::name_of_type(std::array<double,3>()), "std::array<double,3>");

	EXPECT_EQ(cgogn::name_of_type(cgogn::Dart()), "cgogn::Dart");
	EXPECT_EQ(cgogn::name_of_type(cgogn::Cell<cgogn::Orbit::PHI1>()), "cgogn::Cell<cgogn::Orbit::PHI1>");
}
