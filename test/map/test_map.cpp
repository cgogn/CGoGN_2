
#include "core/map/map1.h"
#include "core/map/map2.h"

using namespace cgogn;


struct My_Data_Traits
{
	static const int CHUNK_SIZE = 64;
};


// typedef for short writing
typedef Map1<My_Data_Traits> MAP1;

// typedef for short writing
typedef Map2<My_Data_Traits> MAP2;




void fonc_const(const MAP1::VertexAttributeHandler<float>& ah)
{
	for (const float& f:ah)
	{
		std::cout << f << std::endl;
	}

	// equivalent to
	for (MAP1::VertexAttributeHandler<float>::const_iterator it = ah.begin(); it != ah.end(); ++it)
			std::cout << *it << std::endl;
}

void fonc_non_const(MAP1::VertexAttributeHandler<float>& ah)
{
	for (float& f:ah)
	{
		f *= 2.0f;
		std::cout << f << std::endl;
	}

	// equivalent to
	for (MAP1::VertexAttributeHandler<float>::iterator it = ah.begin(); it != ah.end(); ++it)
	{
		*it /= 2.0f;
	}
}


int test1(MAP1& map)
{
	// add an attribute on vertex of map with
	MAP1::VertexAttributeHandler<float> ah = map.addAttribute<float, MAP1::VERTEX>("floats");

	// get ChunkArrayContainer -> get ChunkArray -> fill
	ChunkArrayContainer<My_Data_Traits::CHUNK_SIZE, unsigned int>& container = map.getAttributeContainer(VERTEX1);
	ChunkArray<My_Data_Traits::CHUNK_SIZE,float>* att = container.getAttribute<float>("floats");
	for (int i=0;i<10;++i)
		container.insertLines<1>();
	for(unsigned int i=container.begin(); i!=container.end(); container.next(i))
		(*att)[i] = 3.0f + 0.1f*float(i);

	// access with index
	std::cout << ah[0] << std::endl;

	fonc_non_const(ah);

	fonc_const(ah);

//	// traverse container with for range
//	for (float f:ah)
//		std::cout << f << std::endl;

	return 0;
}



int main()
{
    // declare a map
    MAP1 map;
    MAP2 map2;
    test1(map);
	return 0;
}
