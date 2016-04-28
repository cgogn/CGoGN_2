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

#include <cgogn/core/container/chunk_array_container.h>

namespace cgogn
{

class ChunkArrayContainerTest : public ::testing::Test
{
//protected:
public:

//	ChunkArrayContainer<16u,uint32> ca_cont_;

	ChunkArrayContainerTest()
	{}

	// 
	void SetUp()
	{

	}

	void testAddAttribute()
	{
		// avec un grand nombre de type
	}
};

// Test
TEST_F(ChunkArrayContainerTest, testAddAttribute)
{
	this->testAddAttribute();
}

TEST_F(ChunkArrayContainerTest, testRemove)
{
	ChunkArrayContainer<16u,uint32> ca_cont;

	for (uint32 i=0; i<40; ++i)
		ca_cont.insert_lines<1>();

	EXPECT_EQ(ca_cont.size(),40);

	ca_cont.remove_lines<1>(3);
	ca_cont.remove_lines<1>(19);
	ca_cont.remove_lines<1>(37);

	EXPECT_EQ(ca_cont.size(),37);

	uint32 i1 = ca_cont.insert_lines<1>();
	uint32 i2 = ca_cont.insert_lines<1>();
	uint32 i3 = ca_cont.insert_lines<1>();

	EXPECT_EQ(ca_cont.size(),40);

	EXPECT_EQ(i1,37);
	EXPECT_EQ(i2,19);
	EXPECT_EQ(i3,3);
}

TEST_F(ChunkArrayContainerTest, test_compact)
{
	using DATA = uint32;

	ChunkArrayContainer<16u,uint32> ca_cont;


	ChunkArray<16u,DATA>* indices = ca_cont.add_attribute<DATA>("indices");

	for (uint32 i=0; i<20; ++i)
	{
		ca_cont.insert_lines<1>();
		indices->operator [](i) = i;
	}

	ca_cont.remove_lines<1>(0);
	ca_cont.remove_lines<1>(18);
	ca_cont.remove_lines<1>(2);
	ca_cont.remove_lines<1>(17);
	ca_cont.remove_lines<1>(3);
	ca_cont.remove_lines<1>(15);
	ca_cont.remove_lines<1>(5);
	ca_cont.remove_lines<1>(6);
	ca_cont.remove_lines<1>(7);
	ca_cont.remove_lines<1>(9);
	ca_cont.remove_lines<1>(10);
	ca_cont.remove_lines<1>(11);
	ca_cont.remove_lines<1>(13);
	ca_cont.remove_lines<1>(14);

	EXPECT_EQ(ca_cont.size(),6);

	std::vector<uint32> old_new = ca_cont.compact<1>();
	EXPECT_EQ(old_new.size(),20);

	EXPECT_EQ(ca_cont.size(),6);

//	for (uint32 i=ca_cont.begin(); i!=ca_cont.end(); ca_cont.next(i))
//		std::cout << i << " => "<<indices->operator [](i)<< std::endl;

	std::vector<DATA> after;
	for (uint32 i=ca_cont.begin(); i!=ca_cont.end(); ca_cont.next(i))
		after.push_back(indices->operator [](i));

	auto contains = [&] (DATA x) -> bool { return std::find(after.begin(),after.end(),x) != after.end(); };

	EXPECT_TRUE( contains(1) );
	EXPECT_TRUE( contains(4) );
	EXPECT_TRUE( contains(8) );
	EXPECT_TRUE( contains(12) );
	EXPECT_TRUE( contains(16) );
	EXPECT_TRUE( contains(19) );

//	uint32 i=0;
//	for(uint32 x: old_new)
//		std::cout << i++ << " : "<< x << std::endl;

}

TEST_F(ChunkArrayContainerTest, test_compact_tri)
{
	using DATA = uint32;

	ChunkArrayContainer<16u,uint32> ca_cont;


	ChunkArray<16u,DATA>* indices = ca_cont.add_attribute<DATA>("indices");

	for (uint32 i=0; i<10; ++i)
	{
		ca_cont.insert_lines<3>();
		indices->operator [](i) = i;
	}

	for (uint32 i=0; i<30; ++i)
	{
		indices->operator [](i) = i;
	}


	ca_cont.remove_lines<3>(0);
	ca_cont.remove_lines<3>(8*3);
	ca_cont.remove_lines<3>(2*3);
	ca_cont.remove_lines<3>(6*3);
	ca_cont.remove_lines<3>(4*3);

	EXPECT_EQ(ca_cont.size(),15);

	std::vector<uint32> old_new = ca_cont.compact<3>();
	EXPECT_EQ(old_new.size(),30);

	EXPECT_EQ(ca_cont.size(),15);

//	for (uint32 i=ca_cont.begin(); i!=ca_cont.end(); ca_cont.next(i))
//		std::cout << i << " => "<<indices->operator [](i)<< std::endl;

	std::vector<DATA> after;
	for (uint32 i=ca_cont.begin(); i!=ca_cont.end(); ca_cont.next(i))
		after.push_back(indices->operator [](i));

	auto contains = [&] (DATA x) -> bool { return std::find(after.begin(),after.end(),x) != after.end(); };

	EXPECT_TRUE( contains(3) );
	EXPECT_TRUE( contains(4) );
	EXPECT_TRUE( contains(5) );
	EXPECT_TRUE( contains(9) );
	EXPECT_TRUE( contains(10) );
	EXPECT_TRUE( contains(11) );
	EXPECT_TRUE( contains(15) );
	EXPECT_TRUE( contains(16) );
	EXPECT_TRUE( contains(17) );
	EXPECT_TRUE( contains(21) );
	EXPECT_TRUE( contains(22) );
	EXPECT_TRUE( contains(23) );
	EXPECT_TRUE( contains(27) );
	EXPECT_TRUE( contains(28) );
	EXPECT_TRUE( contains(29) );

//	uint32 i=0;
//	for(uint32 x: old_new)
//		std::cout << i++ << " : "<< x << std::endl;

}




} // namespace cgogn
