
#include <core/map/cmap1.h>
#include <core/map/cmap2.h>

#include <core/basic/dart_marker.h>
#include <core/basic/cell_marker.h>

#include <core/iterator/cell_iterators.h>

using namespace cgogn;


template <typename MAP>
void fonc_const(const typename MAP::template VertexAttributeHandler<float>& ah);

template <typename MAP>
void fonc_non_const(typename MAP::template VertexAttributeHandler<float>& ah);

template <typename MAP>
int test1(MAP& map);


template <typename MAP>
void fonc_const(const typename MAP::template VertexAttributeHandler<float>& ah)
{
	for (const float& f:ah)
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
	for (float& f:ah)
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
	typename MAP::template VertexAttributeHandler<float> ah =
			map.template add_attribute<float, MAP::VERTEX>("floats");

	// get ChunkArrayContainer -> get ChunkArray -> fill
	typename MAP::template ChunkArrayContainer<unsigned int>& container =
			map.get_attribute_container(MAP::VERTEX);
	typename MAP::template ChunkArray<float>* att =
			container.template get_attribute<float>("floats");
	for (unsigned int i = 0; i < 10; ++i)
		container.template insert_lines<1>();
	for(unsigned int i = container.begin(); i != container.end(); container.next(i))
		(*att)[i] = 3.0f + 0.1f*float(i);

	// access with index
	std::cout << ah[0] << std::endl;

	fonc_non_const<MAP>(ah);
	fonc_const<MAP>(ah);

	//	// traverse container with for range
	//	for (float f:ah)
	//		std::cout << f << std::endl;

	return 0;
}

template <typename MAP>
int test2(MAP& map)
{
	// add an attribute on vertex of map with
	typename MAP::template VertexAttributeHandler<float> ah =
			map.template get_attribute<float, MAP::VERTEX>("floats");

	std::vector<unsigned int>* uib = cgogn::get_uint_buffers()->get_buffer();
	uib->push_back(3);
	cgogn::get_uint_buffers()->release_buffer(uib);

	Dart d1 = map.add_face(3);

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

	return 0;
}

template <typename MAP>
int test3(MAP& map)
{

	Dart d1 = map.add_face(3);

	std::cout << "Faces :" ;
	CellIterator<MAP,MAP::FACE> tf(map, d1);

	for (Dart d : tf)
	{
		std::cout << std::endl;
		std::cout << d;
		for (Dart e : tf)
		{
			std::cout << " - " << e ;
		}
	}
	std::cout << std::endl;
	std::cout << "End Faces" << std::endl;

	std::cout << "Vertices :" ;
	CellIterator<MAP,MAP::VERTEX> tv(map, d1);
	for (Dart d : tv)
	{
		std::cout << " - " << d ;
	}
	std::cout << std::endl;
	std::cout << "End Vertices" << std::endl;

	std::cout << "Volume :" ;
	CellIterator<MAP,MAP::VOLUME> tw(map, d1);
	for (Dart d : tw)
	{
		std::cout << " - " << d ;
	}
	std::cout << std::endl;
	std::cout << "End Volume" << std::endl;

	std::cout << "Autre :" ;
	CellIterator<MAP,cgogn::NB_ORBITS> to(map, d1);
	for (Dart d : to)
	{
		std::cout << " - " << d ;
	}
	std::cout << std::endl;
	std::cout << "End Autre" << std::endl;
	return 0;
}

int main()
{
	cgogn::thread_start();
	CMap1 map1;
	CMap2 map2;
	test1(map1);
	test1(map2);
	test2(map1);
	test2(map2);
	test3(map1);
	test3(map2);
	cgogn::thread_stop();
	return 0;
}
