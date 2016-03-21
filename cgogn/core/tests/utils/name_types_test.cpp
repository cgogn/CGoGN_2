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

TEST(NameTypesTest, NumTypes)
{

	using signed_char = signed char;
	using unsigned_char = unsigned char;
	using unsigned_short = unsigned short;
	using uint = cgogn::uint32;
	using ulint = unsigned long;
	using llint = long long;
	using ullint = unsigned long long;

	EXPECT_EQ(cgogn::name_of_type(bool()), "bool");
	EXPECT_EQ(cgogn::name_of_type(char()), "char");
	EXPECT_EQ(cgogn::name_of_type(signed_char()), "signed char");
	EXPECT_EQ(cgogn::name_of_type(unsigned_char()), "unsigned char");
	EXPECT_EQ(cgogn::name_of_type(wchar_t()), "wchar_t");

#if _MSC_VER == 1800 // VS2013
	EXPECT_EQ(cgogn::name_of_type(char16_t()), "unsigned short");
	EXPECT_EQ(cgogn::name_of_type(char32_t()), "uint32");
#else
	EXPECT_EQ(cgogn::name_of_type(char16_t()), "char16_t");
	EXPECT_EQ(cgogn::name_of_type(char32_t()), "char32_t");
#endif // VS2013

	EXPECT_EQ(cgogn::name_of_type(short()), "short");
	EXPECT_EQ(cgogn::name_of_type(unsigned_short()), "unsigned short");
	EXPECT_EQ(cgogn::name_of_type(int()), "int");

	EXPECT_EQ(cgogn::name_of_type(uint()), "uint32");
	EXPECT_EQ(cgogn::name_of_type(long()), "long");
	EXPECT_EQ(cgogn::name_of_type(ulint()), "unsigned long");
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
