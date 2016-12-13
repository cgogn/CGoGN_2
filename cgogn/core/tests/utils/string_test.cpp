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
#include <cgogn/core/utils/string.h>




TEST(StringTest, to_upper)
{
	const std::string a("a");
	const std::string foobar("FoObAR");

	EXPECT_EQ(cgogn::to_upper(a), std::string("A"));
	EXPECT_EQ(cgogn::to_upper(foobar), std::string("FOOBAR"));
}

TEST(StringTest, to_lower)
{
	const std::string a("A");
	const std::string foobar("FoObAR");

	EXPECT_EQ(cgogn::to_lower(a), std::string("a"));
	EXPECT_EQ(cgogn::to_lower(foobar), std::string("foobar"));
}

TEST(StringTest, extension)
{
	const std::string f1("file_with_no_extension");
	const std::string f2("file_with_one_extension.vtk");
	const std::string f3("file_with_two_extensions.tar.gz");
	const std::string f4("tricky_file.");

	EXPECT_TRUE(cgogn::extension(f1).empty());
	EXPECT_EQ(cgogn::extension(f2), std::string("vtk"));
	EXPECT_EQ(cgogn::extension(f3), std::string("gz"));
	EXPECT_TRUE(cgogn::extension(f4).empty());
}

TEST(StringTest, remove_extension)
{
	const std::string f1("file_with_no_extension");
	const std::string f2("file_with_one_extension.vtk");
	const std::string f3("file_with_two_extensions.tar.gz");
	const std::string f4("tricky_file.");

	EXPECT_EQ(cgogn::remove_extension(f1), f1);
	EXPECT_EQ(cgogn::remove_extension(f2), std::string("file_with_one_extension"));
	EXPECT_EQ(cgogn::remove_extension(f3), std::string("file_with_two_extensions.tar"));
	EXPECT_EQ(cgogn::remove_extension(f4), std::string("tricky_file."));
}

TEST(StringTest, i_equals)
{
	const std::string f1("MyFile.tar.gz");
	EXPECT_TRUE(cgogn::i_equals(f1, std::string("myfiLE.taR.GZ")));
	EXPECT_FALSE(cgogn::i_equals(f1, std::string("myfiLE.taR.GZz")));
}
