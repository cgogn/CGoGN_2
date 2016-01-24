#include <core/mrcmap/mrcmap2.h>


using namespace cgogn;

using MRMap2 = MRCMap2PrimalRegular<DefaultMapTraits>;


template <typename MAP>
void print_map(MAP& map)
{
	// std::cout << map.get_nb_cell<MRMap2::VERTEX> << std::endl;
}

int main()
{
	MRMap2 map;

	std::cout << "max level: " << map.get_maximum_level() << std::endl;
	
	MRMap2::Face f1 = map.add_face(3);
	MRMap2::Face f2 = map.add_face(3);
	map.sew_faces(f1, f2);

	std::cout << "Vertices :" << std::endl;
	map.template foreach_cell<MRMap2::VERTEX>([&] (typename MRMap2::Vertex v)
	{
		std::cout << v << std::endl;
	});
	std::cout << "End Vertices" << std::endl;


	std::cout << "-> add level" << std::endl;
	map.add_level_back();

	std::cout << "max level: " << map.get_maximum_level() << std::endl;

	return 0;
}
