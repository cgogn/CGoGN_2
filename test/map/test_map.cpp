
#include <core/map/map1.h>
#include <core/map/map2.h>

#include <core/basic/dart_marker.h>
#include <core/basic/cell_marker.h>

#include <core/traversal/global.h>

using namespace cgogn;


struct My_Data_Traits
{
	static const unsigned int CHUNK_SIZE = 64;
};


// typedefs for short writing
typedef Map1<My_Data_Traits> MAP1;
typedef Map2<My_Data_Traits> MAP2;


void fonc_const(const MAP1::VertexAttributeHandler<float>& ah);
void fonc_non_const(MAP1::VertexAttributeHandler<float>& ah);
int test1(MAP1& map);


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
	MAP1::VertexAttributeHandler<float> ah = map.add_attribute<float, MAP1::VERTEX>("floats");

	std::vector<unsigned int>* uib = cgogn::get_uint_buffers()->get_buffer();
	uib->push_back(3);
	cgogn::get_uint_buffers()->release_buffer(uib);

	Dart d1 = map.add_cycle(3);

	DartMarker<MAP1> dm(map);
	CellMarker<MAP1, MAP1::VERTEX> cm(map);

	dm.mark(d1);

	std::cout << "Darts :" << std::endl;
	for (Dart dit : map)
	{
		std::cout << dit << std::endl;
	}
	std::cout << "End Darts" << std::endl;

	std::cout << "Vertices :" << std::endl;
	for (MAP1::Vertex v : vertices<FORCE_CELL_MARKING>(map))
	{
		std::cout << v << std::endl;
	}
	std::cout << "End Vertices" << std::endl;

	// get ChunkArrayContainer -> get ChunkArray -> fill
	ChunkArrayContainer<My_Data_Traits::CHUNK_SIZE, unsigned int>& container = map.get_attribute_container(VERTEX1);
	ChunkArray<My_Data_Traits::CHUNK_SIZE,float>* att = container.get_attribute<float>("floats");
	for (unsigned int i = 0; i < 10; ++i)
		container.insert_lines<1>();
	for(unsigned int i = container.begin(); i != container.end(); container.next(i))
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
	cgogn::thread_start();
	MAP1 map1;
	MAP2 map2;
	test1(map1);
	cgogn::thread_stop();
	return 0;
}
