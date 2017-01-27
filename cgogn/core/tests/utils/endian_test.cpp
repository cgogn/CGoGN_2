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
#include <cgogn/core/utils/endian.h>
#include <array>


using namespace cgogn::numerics;

TEST(EndianTest, swap16)
{
	const uint16 n(0x0401); // n = 1025
	const uint16 swapped = cgogn::swap_endianness(n);
	const uint16 res(0x0104);
	EXPECT_EQ(swapped, res);
}

TEST(EndianTest, swap32)
{
	const uint32 n(0x00020804 ); // n = 133124
	const uint32 swapped = cgogn::swap_endianness(n);
	const uint32 res(0x04080200);
	EXPECT_EQ(swapped, res);
}

TEST(EndianTest, swap64)
{
	const uint64 n(0x0000010000040880); // n =  1 099 511 892 096
	const uint64 swapped = cgogn::swap_endianness(n);
	const uint64 res(0x8008040000010000);
	EXPECT_EQ(swapped, res);
}

TEST(EndianTest, swap_array)
{
	std::array<uint16, 3> arr16;
	std::array<uint32, 3> arr32;
	std::array<uint64, 3> arr64;
	arr16.fill(0x0401);
	arr32.fill(0x00020804);
	arr64.fill(0x0000010000040880);

	const auto& swapped16 = cgogn::swap_endianness(arr16);
	const auto& swapped32 = cgogn::swap_endianness(arr32);
	const auto& swapped64 = cgogn::swap_endianness(arr64);

	const uint16 res16(0x0104);
	const uint32 res32(0x04080200);
	const uint64 res64(0x8008040000010000);

	EXPECT_EQ(swapped16[0], res16);
	EXPECT_EQ(swapped16[1], swapped16[0]);
	EXPECT_EQ(swapped16[2], swapped16[0]);

	EXPECT_EQ(swapped32[0], res32);
	EXPECT_EQ(swapped32[1], swapped32[0]);
	EXPECT_EQ(swapped32[2], swapped32[0]);

	EXPECT_EQ(swapped64[0], res64);
	EXPECT_EQ(swapped64[1], swapped64[0]);
	EXPECT_EQ(swapped64[2], swapped64[0]);
}
