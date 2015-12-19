#include <gtest/gtest.h>

#include <core/container/chunk_array_container.h>

namespace cgogn
{

class ChunkArrayContainerTest : public ::testing::Test
{

protected:
	ChunkArrayContainerTest() 
	{}

	// 
	void SetUp()
	{

	}

	void testAddAttribute()
	{
		std::cout << "test" << std::endl;
		//avec un grand nombre de type
		//
	}

};

// Test
TEST_F(ChunkArrayContainerTest, testAddAttribute)
{
	this->testAddAttribute();
}

} //end namespace cgogn