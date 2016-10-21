
#include <cgogn/core/cmap/cmap1.h>
#include <cgogn/core/cmap/cmap2.h>
#include <cgogn/core/cmap/cmap3.h>

#include <cgogn/core/basic/dart_marker.h>
#include <cgogn/core/basic/cell_marker.h>

using namespace cgogn;
using namespace cgogn::numerics;


template <typename MAP>
void fonc_const(const typename MAP::template VertexAttribute<float32>& ah);

template <typename MAP>
void fonc_non_const(typename MAP::template VertexAttribute<float32>& ah);

template <typename MAP>
int test1(MAP& map);


template <typename MAP>
void fonc_const(const typename MAP::template VertexAttribute<float32>& ah)
{
	for (const float32& f : ah)
	{
		cgogn_log_info("example_map") << f;
	}

	// equivalent to
	for (typename MAP::template VertexAttribute<float32>::const_iterator it = ah.begin(); it != ah.end(); ++it)
		cgogn_log_info("example_map") << *it;
}

template <typename MAP>
void fonc_non_const(typename MAP::template VertexAttribute<float32>& ah)
{
	for (float32& f : ah)
	{
		f *= 2.0f;
		cgogn_log_info("example_map") << f;
	}

	// equivalent to
	for (typename MAP::template VertexAttribute<float32>::iterator it = ah.begin(); it != ah.end(); ++it)
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
	typename MAP::template VertexAttribute<float32> ah = map.template add_attribute<float32, Vertex>("floats");

	typename MAP::template FaceAttribute<float32> ahf = map.template add_attribute<float32, Face>("floats");

	// get attribute and change type (dangerous!)
	typename MAP::template VertexAttribute<int32> ahf2 = map.template get_attribute_force_type<int32,float32, Vertex::ORBIT>("floats");

	map.remove_attribute(ahf);
	cgogn_log_info("example_map") << "ahf valid : " << std::boolalpha << ahf.is_valid();

	std::vector<uint32>* uib = cgogn::uint_buffers()->buffer();
	uib->push_back(3);
	cgogn::uint_buffers()->release_buffer(uib);

	Dart d1 = map.add_face(3).dart;

	// get cell buffer typed
	std::vector<typename MAP::Vertex>* vertb = cgogn::dart_buffers()->cell_buffer<typename MAP::Vertex>();

//	std::vector<Dart>* vertdb = cgogn::dart_buffers()->buffer();
//	std::vector<typename MAP::Vertex>* vert_b = reinterpret_cast< std::vector<typename MAP::Vertex>* >(vertdb);

	vertb->push_back(typename MAP::Vertex(d1));
	vertb->push_back(typename MAP::Vertex(d1));

	cgogn::dart_buffers()->release_cell_buffer(vertb);

	DartMarker<MAP> dm(map);
	CellMarker<MAP, MAP::Vertex::ORBIT> cm(map);

	dm.mark(d1);

	cgogn_log_info("example_map") << "Darts :";
	map.foreach_dart([] (Dart d) { cgogn_log_info("example_map") << d; });
	cgogn_log_info("example_map")<< "End Darts";

	cgogn_log_info("example_map") << "Vertices :";
	map.foreach_cell([&] (Vertex v)
	{
		cgogn_log_info("example_map") << v;
		ah[v] = 2.0f;
	});
	cgogn_log_info("example_map") << "End Vertices";

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
	cgogn_log_info("example_map") << ah[0];

	fonc_non_const<MAP>(ah);
	fonc_const<MAP>(ah);

	//	// traverse container with for range
	//	for (float32 f:ah)
	//		cgogn_log_info("example_map") << f;

	return 0;
}

int main()
{
	CMap1 map1;
	CMap2 map2;
//	CMap3 map3;
//	test1(map1);
	test1(map2);
//	test1(map3);

	return 0;
}
