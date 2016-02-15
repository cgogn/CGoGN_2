
#include <core/cmap/cmap1.h>
#include <core/cmap/cmap2.h>
#include <core/cmap/cmap3.h>

#include <core/basic/dart_marker.h>
#include <core/basic/cell_marker.h>

using namespace cgogn;


using Map1 = CMap1<DefaultMapTraits>;
using Map2 = CMap2<DefaultMapTraits>;
using Map3 = CMap3<DefaultMapTraits>;


template <typename MAP>
void fonc_const(const typename MAP::template VertexAttributeHandler<float>& ah);

template <typename MAP>
void fonc_non_const(typename MAP::template VertexAttributeHandler<float>& ah);

template <typename MAP>
int test1(MAP& map);


template <typename MAP>
void fonc_const(const typename MAP::template VertexAttributeHandler<float>& ah)
{
	for (const float& f : ah)
	{
		std::cout << f << std::endl;
	}

	// equivalent to
	for (typename MAP::template VertexAttributeHandler<float>::const_iterator it = ah.begin(); it != ah.end(); ++it)
		std::cout << *it << std::endl;
}

template <typename MAP>
void fonc_non_const(typename MAP::template VertexAttributeHandler<float>& ah)
{
	for (float& f : ah)
	{
		f *= 2.0f;
		std::cout << f << std::endl;
	}

	// equivalent to
	for (typename MAP::template VertexAttributeHandler<float>::iterator it = ah.begin(); it != ah.end(); ++it)
	{
		*it /= 2.0f;
	}
}

template <typename MAP>
int test1(MAP& map)
{
	// add an attribute on vertex of map with
	typename MAP::template VertexAttributeHandler<float> ah = map.template add_attribute<float, MAP::VERTEX>("floats");

	typename MAP::template FaceAttributeHandler<float> ahf = map.template add_attribute<float, MAP::FACE>("floats");

	// get attribute and change type (dangerous!)
	typename MAP::template VertexAttributeHandler<int> ahf2 = map.template get_attribute_force_type<int,float, MAP::VERTEX>("floats");

	map.remove_attribute(ahf);
	std::cout << "ahf valid : " << std::boolalpha << ahf.is_valid() << std::endl;

	std::vector<unsigned int>* uib = cgogn::get_uint_buffers()->get_buffer();
	uib->push_back(3);
	cgogn::get_uint_buffers()->release_buffer(uib);


	Dart d1 = map.add_face(3);

	// get cell buffer typed
//	std::vector<typename MAP::Vertex>* vert_b = cgogn::get_dart_buffers()->get_cell_buffer<typename MAP::Vertex>();

	std::vector<Dart>* vertdb = cgogn::get_dart_buffers()->get_buffer();
	std::vector<typename MAP::Vertex>* vert_b = reinterpret_cast< std::vector<typename MAP::Vertex>* >(vertdb);


	vert_b->push_back(d1);
	vert_b->push_back(typename MAP::Vertex(d1));

	cgogn::get_dart_buffers()->release_cell_buffer(vertdb);
//	cgogn::get_dart_buffers()->release_cell_buffer(vert_b);

	DartMarker<MAP> dm(map);
	CellMarker<MAP, MAP::VERTEX> cm(map);

	dm.mark(d1);

	std::cout << "Darts :" << std::endl;
	for (Dart dit : map)
	{
		std::cout << dit << std::endl;
	}
	std::cout << "End Darts" << std::endl;

	std::cout << "Vertices :" << std::endl;
	map.template foreach_cell<MAP::VERTEX>([&] (typename MAP::Vertex v)
	{
		std::cout << v << std::endl;
		ah[v] = 2.0f;
	});
	std::cout << "End Vertices" << std::endl;

//	map.foreach_adjacent_vertex_through_edge(d1, [&] (typename MAP::Vertex v)
//	{
//		ah[v] = 4.0f;
//	});

	// get ChunkArrayContainer -> get ChunkArray -> fill
//	typename MAP::template ChunkArrayContainer<unsigned int>& container = map.get_attribute_container(MAP::VERTEX);
//	typename MAP::template ChunkArray<float>* att = container.template get_attribute<float>("floats");
//	for (unsigned int i = 0; i < 10; ++i)
//		container.template insert_lines<1>();
	for (auto& v : ah)
		v = 3.0f;

	// access with index
	std::cout << ah[0] << std::endl;

	fonc_non_const<MAP>(ah);
	fonc_const<MAP>(ah);

	//	// traverse container with for range
	//	for (float f:ah)
	//		std::cout << f << std::endl;

	return 0;
}

int main()
{
	Map1 map1;
	Map2 map2;
//	Map3 map3;
	test1(map1);
	test1(map2);
//	test1(map3);

	return 0;
}
