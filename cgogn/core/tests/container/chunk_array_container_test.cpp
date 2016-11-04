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

	using ChunkArrayContainer = cgogn::ChunkArrayContainer<16u,uint32> ;
	template <class T> using ChunkArray = cgogn::ChunkArray<16u, T>;

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
	ChunkArrayContainer ca_cont;

	for (uint32 i=0; i<40; ++i)
		ca_cont.insert_lines<1>();

	EXPECT_EQ(ca_cont.size(),40u);

	ca_cont.remove_lines<1>(3);
	ca_cont.remove_lines<1>(19);
	ca_cont.remove_lines<1>(37);

	EXPECT_EQ(ca_cont.size(),37u);

	uint32 i1 = ca_cont.insert_lines<1>();
	uint32 i2 = ca_cont.insert_lines<1>();
	uint32 i3 = ca_cont.insert_lines<1>();

	EXPECT_EQ(ca_cont.size(),40u);

	EXPECT_EQ(i1,37u);
	EXPECT_EQ(i2,19u);
	EXPECT_EQ(i3,3u);
}

TEST_F(ChunkArrayContainerTest, test_compact)
{
	using DATA = uint32;
	ChunkArrayContainer ca_cont;
	ChunkArray<DATA>* indices = ca_cont.add_chunk_array<DATA>("indices");

	for (uint32 i=0; i<20; ++i)
	{
		ca_cont.insert_lines<1>();
		indices->operator[](i) = i;
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

	EXPECT_EQ(ca_cont.size(),6u);

	std::vector<uint32> old_new = ca_cont.compact<1>();
	EXPECT_EQ(old_new.size(),20u);

	EXPECT_EQ(ca_cont.size(),6u);

//	for (uint32 i=ca_cont.begin(); i!=ca_cont.end(); ca_cont.next(i))
//		std::cout << i << " => "<<indices->operator[](i)<< std::endl;

	std::vector<DATA> after;
	for (uint32 i = ca_cont.begin(); i != ca_cont.end(); ca_cont.next(i))
		after.push_back(indices->operator[](i));

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

	ChunkArrayContainer ca_cont;

	ChunkArray<DATA>* indices = ca_cont.add_chunk_array<DATA>("indices");

	for (uint32 i = 0; i < 10; ++i)
	{
		ca_cont.insert_lines<3>();
		indices->operator[](i) = i;
	}

	for (uint32 i = 0; i < 30; ++i)
	{
		indices->operator[](i) = i;
	}

	ca_cont.remove_lines<3>(0);
	ca_cont.remove_lines<3>(8*3);
	ca_cont.remove_lines<3>(2*3);
	ca_cont.remove_lines<3>(6*3);
	ca_cont.remove_lines<3>(4*3);

	EXPECT_EQ(ca_cont.size(),15u);

	std::vector<uint32> old_new = ca_cont.compact<3>();
	EXPECT_EQ(old_new.size(),30u);

	EXPECT_EQ(ca_cont.size(),15u);

//	for (uint32 i=ca_cont.begin(); i!=ca_cont.end(); ca_cont.next(i))
//		std::cout << i << " => "<<indices->operator[](i)<< std::endl;

	std::vector<DATA> after;
	for (uint32 i=ca_cont.begin(); i!=ca_cont.end(); ca_cont.next(i))
		after.push_back(indices->operator[](i));

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

TEST_F(ChunkArrayContainerTest, test_merge)
{
	using VEC3F = std::array<float32,3>;
	ChunkArrayContainer::ChunkArrayFactory::register_known_types();

	ChunkArrayContainer ca_cont;
	ChunkArray<uint32>* data_i = ca_cont.add_chunk_array<uint32>("indices");
	ChunkArray<float32>* data_f = ca_cont.add_chunk_array<float32>("data_f");

	ChunkArrayContainer ca_cont2;
	ChunkArray<VEC3F>* data2_v = ca_cont2.add_chunk_array<VEC3F>("data_v");
	ChunkArray<uint16>* data2_i16 = ca_cont2.add_chunk_array<uint16>("indices");

	// test impossible merge
	bool ok = ca_cont.check_before_merge(ca_cont2);
	EXPECT_FALSE(ok);

	// correct attribute
	ca_cont2.remove_chunk_array(data2_i16);
	ChunkArray<uint32>* data2_i = ca_cont2.add_chunk_array<uint32>("indices");

	// filling
	for (uint32 i=0; i<10; ++i)
	{
		ca_cont.insert_lines<1>();
		data_i->operator[](i) = i;
		data_f->operator[](i) = 0.01f*i;
	}
	ca_cont.remove_lines<1>(2);
	ca_cont.remove_lines<1>(4);
	ca_cont.remove_lines<1>(7);

	for (uint32 i=0; i<10; ++i)
	{
		ca_cont2.insert_lines<1>();
		data2_i->operator[](i) = 100+i;
		float32 x = 100.0f+0.01f*i;
		data2_v->operator[](i) = {{x,x,x}};
	}
	ca_cont2.remove_lines<1>(3);
	ca_cont2.remove_lines<1>(6);
	ca_cont2.remove_lines<1>(9);


	//testing
	ok = ca_cont.check_before_merge(ca_cont2);
	EXPECT_TRUE(ok);

	if (!ok)
		return;

	std::vector<uint32> old_new = ca_cont.merge<1>(ca_cont2);
	EXPECT_EQ(old_new.size(),9u);
	EXPECT_EQ(ca_cont.size(),14u);

//	std::cout << "=============================" << std::endl;
//	ChunkArray<VEC3F>* data_v = ca_cont.get_attribute<VEC3F>("data_v");
//	for (uint32 i=ca_cont.begin(); i!=ca_cont.end(); ca_cont.next(i))
//	{
//		std::cout << i << " => ";
//		std::cout << data_i->operator[](i) <<" / ";
//		std::cout << data_f->operator[](i) <<" / ";
//		const VEC3F& X= data_v->operator[](i);
//		std::cout << X[0]<< "," << X[1]<< "," << X[2];
//		std::cout << std::endl;
//	}
//	std::cout << "=============================" << std::endl;
//	uint32 i=0;
//	for(uint32 x: old_new)
//		std::cout << i++ << " : "<< x << std::endl;

	// check contains of result
	std::vector<int32> after;
	for (uint32 i=ca_cont.begin(); i!=ca_cont.end(); ca_cont.next(i))
		after.push_back(data_i->operator[](i));

	auto contains = [&] (uint32 x) -> bool { return std::find(after.begin(),after.end(),x) != after.end(); };

	EXPECT_EQ(after.size(),14u);
	EXPECT_TRUE( contains(0) );
	EXPECT_TRUE( contains(1) );
	EXPECT_TRUE( contains(3) );
	EXPECT_TRUE( contains(5) );
	EXPECT_TRUE( contains(6) );
	EXPECT_TRUE( contains(8) );
	EXPECT_TRUE( contains(9) );
	EXPECT_TRUE( contains(100) );
	EXPECT_TRUE( contains(101) );
	EXPECT_TRUE( contains(102) );
	EXPECT_TRUE( contains(104) );
	EXPECT_TRUE( contains(105) );
	EXPECT_TRUE( contains(107) );
	EXPECT_TRUE( contains(108) );
}

TEST_F(ChunkArrayContainerTest, test_merge_tri)
{
	using VEC3F = std::array<float32,3>;

	ChunkArrayContainer::ChunkArrayFactory::register_known_types();

	ChunkArrayContainer ca_cont;
	ChunkArray<uint32>* data_i = ca_cont.add_chunk_array<uint32>("indices");
	ChunkArray<float32>* data_f = ca_cont.add_chunk_array<float32>("data_f");

	ChunkArrayContainer ca_cont2;
	ChunkArray<VEC3F>* data2_v = ca_cont2.add_chunk_array<VEC3F>("data_v");
	ChunkArray<uint32>* data2_i = ca_cont2.add_chunk_array<uint32>("indices");
	ChunkArray<bool>* data2_b = ca_cont2.add_chunk_array<bool>("booleens");

	for (uint32 i=0; i<3; ++i)
		ca_cont.insert_lines<3>();

	for (uint32 i=0; i<9; ++i)
	{
		data_i->operator[](i) = i;
		data_f->operator[](i) = 0.01f*i;
	}

	ca_cont.remove_lines<3>(4);

	for (uint32 i=0; i<3; ++i)
		ca_cont2.insert_lines<3>();

	for (uint32 i=0; i<9; ++i)
	{
		data2_i->operator[](i) = 100+i;
		float32 x = 100.0f+0.01f*i;
		data2_v->operator[](i) = {{x,x,x}};
		data2_b->operator[](i) = true;
	}

	ca_cont2.remove_lines<3>(5);

	bool ok = ca_cont.check_before_merge(ca_cont2);
	EXPECT_TRUE(ok);

	if (!ok)
		return;

	std::vector<uint32> old_new = ca_cont.merge<3>(ca_cont2);

	EXPECT_EQ(old_new.size(),9u);
	EXPECT_EQ(ca_cont.size(),12u);

	ChunkArray<bool>* data_b = ca_cont.get_chunk_array<bool>("booleens");

	//	ChunkArray<VEC3F>* data_v = ca_cont.get_attribute<VEC3F>("data_v");
//	std::cout << "=============================" << std::boolalpha<<std::endl;
//	for (uint32 i=ca_cont.begin(); i!=ca_cont.end(); ca_cont.next(i))
//	{
//		std::cout << i << " => ";
//		std::cout << data_i->operator[](i) <<" / ";
//		std::cout << data_f->operator[](i) <<" / ";
//		const VEC3F& X= data_v->operator[](i);
//		std::cout << X[0]<< "," << X[1]<< "," << X[2] << " / " << data_b->operator[](i);
//		std::cout << std::endl;
//	}
//	std::cout << "=============================" << std::endl;
//	uint32 i=0;
//	for(uint32 x: old_new)
//		std::cout << i++ << " : "<< x << std::endl;



	// check contains of result
	std::vector<int32> after;
	uint32 nb_true=0u;
	for (uint32 i=ca_cont.begin(); i!=ca_cont.end(); ca_cont.next(i))
	{
		after.push_back(data_i->operator[](i));
		if (data_b->operator[](i))
			nb_true++;
	}

	auto contains = [&] (uint32 x) -> bool { return std::find(after.begin(),after.end(),x) != after.end(); };

	EXPECT_EQ(after.size(),12u);
	EXPECT_EQ(nb_true,6u);
	EXPECT_TRUE( contains(0) );
	EXPECT_TRUE( contains(1) );
	EXPECT_TRUE( contains(2) );
	EXPECT_TRUE( contains(6) );
	EXPECT_TRUE( contains(7) );
	EXPECT_TRUE( contains(8) );
	EXPECT_TRUE( contains(100) );
	EXPECT_TRUE( contains(101) );
	EXPECT_TRUE( contains(102) );
	EXPECT_TRUE( contains(106) );
	EXPECT_TRUE( contains(107) );
	EXPECT_TRUE( contains(108) );

}



} // namespace cgogn
