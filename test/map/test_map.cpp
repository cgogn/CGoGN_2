
#include "core/map/map1.h"

using namespace cgogn;



struct My_Data_Traits
{
	static const int CHUNK_SIZE=64;
};



int test1()
{
	typedef Map1<My_Data_Traits> MAP1;

	MAP1 map;

	MAP1::VertexAttributeHandler<float> ah = map.addAttribute<float,MAP1::VERTEX>("floats");

	ChunkArrayContainer<My_Data_Traits::CHUNK_SIZE, unsigned int>& container = map.getAttributeContainer(VERTEX2);
	ChunkArray<My_Data_Traits::CHUNK_SIZE,float>* att = container.getAttribute<float>("floats");
	for (int i=0;i<10;++i)
		container.insertLines<1>();
	for(unsigned int i=container.begin(); i!=container.end(); container.next(i))
		(*att)[i] = 3.0f + 0.1f*float(i);

	for (float f:ah)
		std::cout << f << std::endl;

	return 0;
}




int main()
{
	test1();
	return 0;
}

