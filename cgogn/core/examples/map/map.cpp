
#include <core/cmap/cmap1.h>
#include <core/cmap/cmap2.h>
#include <core/cmap/cmap3.h>

#include <core/basic/dart_marker.h>
#include <core/basic/cell_marker.h>

using namespace cgogn;
using namespace cgogn::numerics;

using Map1 = CMap1<DefaultMapTraits>;
using Map2 = CMap2<DefaultMapTraits>;
using Map3 = CMap3<DefaultMapTraits>;


template <typename MAP>
void fonc_const(const typename MAP::template VertexAttributeHandler<float32>& ah);

template <typename MAP>
void fonc_non_const(typename MAP::template VertexAttributeHandler<float32>& ah);

template <typename MAP>
int test1(MAP& map);


template <typename MAP>
void fonc_const(const typename MAP::template VertexAttributeHandler<float32>& ah)
{
	for (const float32& f : ah)
	{
		std::cout << f << std::endl;
	}

	// equivalent to
	for (typename MAP::template VertexAttributeHandler<float32>::const_iterator it = ah.begin(); it != ah.end(); ++it)
		std::cout << *it << std::endl;
}

template <typename MAP>
void fonc_non_const(typename MAP::template VertexAttributeHandler<float32>& ah)
{
	for (float32& f : ah)
	{
		f *= 2.0f;
		std::cout << f << std::endl;
	}

	// equivalent to
	for (typename MAP::template VertexAttributeHandler<float32>::iterator it = ah.begin(); it != ah.end(); ++it)
	{
		*it /= 2.0f;
	}
}

template <typename MAP>
int test1(MAP& map)
{
	using Vertex = typename MAP::Vertex;
	using Face = typename MAP::Face;

	// add an attribute on vertex of map with
	typename MAP::template VertexAttributeHandler<float32> ah = map.template add_attribute<float32, Vertex::ORBIT>("floats");

	typename MAP::template FaceAttributeHandler<float32> ahf = map.template add_attribute<float32, Face::ORBIT>("floats");

	// get attribute and change type (dangerous!)
	typename MAP::template VertexAttributeHandler<int32> ahf2 = map.template get_attribute_force_type<int32,float32, Vertex::ORBIT>("floats");

	map.remove_attribute(ahf);
	std::cout << "ahf valid : " << std::boolalpha << ahf.is_valid() << std::endl;

	std::vector<uint32>* uib = cgogn::get_uint_buffers()->get_buffer();
	uib->push_back(3);
	cgogn::get_uint_buffers()->release_buffer(uib);


	Dart d1 = map.add_face(3).dart;

	// get cell buffer typed
//	std::vector<typename MAP::Vertex>* vert_b = cgogn::get_dart_buffers()->get_cell_buffer<typename MAP::Vertex>();

	std::vector<Dart>* vertdb = cgogn::get_dart_buffers()->get_buffer();
	std::vector<typename MAP::Vertex>* vert_b = reinterpret_cast< std::vector<typename MAP::Vertex>* >(vertdb);

	vert_b->push_back(typename MAP::Vertex(d1));
	vert_b->push_back(typename MAP::Vertex(d1));

	cgogn::get_dart_buffers()->release_cell_buffer(vertdb);
//	cgogn::get_dart_buffers()->release_cell_buffer(vert_b);

	DartMarker<MAP> dm(map);
	CellMarker<MAP, MAP::Vertex::ORBIT> cm(map);

	dm.mark(d1);

	std::cout << "Darts :" << std::endl;
	map.foreach_dart([] (Dart d) { std::cout << d << std::endl; });
	std::cout << "End Darts" << std::endl;

	std::cout << "Vertices :" << std::endl;
	map.foreach_cell([&] (Vertex v)
	{
		std::cout << v << std::endl;
		ah[v] = 2.0f;
	});
	std::cout << "End Vertices" << std::endl;

	// the method foreach_adjacent_vertex_through_edge is not well defined for a MAP1
//	map.foreach_adjacent_vertex_through_edge(d1, [&] (typename MAP::Vertex v)
//	{
//		ah[v] = 4.0f;
//	});

	// get ChunkArrayContainer -> get ChunkArray -> fill
//	typename MAP::template ChunkArrayContainer<uint32>& container = map.get_attribute_container(MAP::Vertex);
//	typename MAP::template ChunkArray<float32>* att = container.template get_attribute<float32>("floats");
//	for (uint32 i = 0; i < 10; ++i)
//		container.template insert_lines<1>();
	for (auto& v : ah)
		v = 3.0f;

	// access with index
	std::cout << ah[0] << std::endl;

	fonc_non_const<MAP>(ah);
	fonc_const<MAP>(ah);

	//	// traverse container with for range
	//	for (float32 f:ah)
	//		std::cout << f << std::endl;

	return 0;
}

int main()
{
	Map1 map1;
	Map2 map2;
//	Map3 map3;
//	test1(map1);
	test1(map2);
//	test1(map3);

	return 0;
}
