
#include "core/map/map1.h"

using namespace cgogn;



struct My_Data_Traits
{
	static const int CHUNK_SIZE=64;
};



int test1()
{
	// typedef for short writing
	typedef Map1<My_Data_Traits> MAP1;

	// declare a map
	MAP1 map;

	// add an attribute on vertex of map with
	MAP1::VertexAttributeHandler<float> ah = map.addAttribute<float,MAP1::VERTEX>("floats");

	// get ChunkArrayContainer -> get ChunkArray -> fill
	ChunkArrayContainer<My_Data_Traits::CHUNK_SIZE, unsigned int>& container = map.getAttributeContainer(VERTEX2);
	ChunkArray<My_Data_Traits::CHUNK_SIZE,float>* att = container.getAttribute<float>("floats");
	for (int i=0;i<10;++i)
		container.insertLines<1>();
	for(unsigned int i=container.begin(); i!=container.end(); container.next(i))
		(*att)[i] = 3.0f + 0.1f*float(i);

	// access with index
	std::cout << ah[0] << std::endl;

	// traverse container with for range
	for (float f:ah)
		std::cout << f << std::endl;

	// equivalent to
	for (MAP1::VertexAttributeHandler<float>::iterator it = ah.begin(); it != ah.end(); ++it)
		std::cout << *it << std::endl;

	return 0;
}




int main()
{
	test1();
	return 0;
}

