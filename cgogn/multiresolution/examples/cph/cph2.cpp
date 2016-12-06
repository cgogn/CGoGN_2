
#include <cgogn/multiresolution/cph/ihcmap2_regular.h>
#include <cgogn/multiresolution/mra/lerp_triquad_mra.h>
#include <cgogn/geometry/types/eigen.h>


using namespace cgogn;

using IHMap2 = IHCMap2Regular;

using Vec3 = Eigen::Vector3d;

template<typename T>
using VertexAttributeHandler = IHMap2::VertexAttribute<T>;


int main()
{	
	IHMap2 map;
	VertexAttributeHandler<Vec3> position = map.get_attribute<Vec3, IHMap2::Vertex::ORBIT>("position");

	LerpTriQuadMRAnalysis<IHMap2, Vec3> lerp(map, position);

	map.add_face(4);
	std::cout << "before add level Faces :" << std::endl;
	map.foreach_cell([&] (IHMap2::Face v)
	{
		std::cout << v << std::endl;
	});
	std::cout << "End Faces" << std::endl;


	{
		lerp.add_level();
		lerp.synthesis();

		std::cout << "after add level Faces :" << std::endl;
		map.foreach_cell([&] (IHMap2::Face f)
		{
			std::cout << f << " | " << map.get_dart_level(f.dart) << std::endl;
		});
		std::cout << "End Faces" << std::endl;


		unsigned int cur = map.get_current_level();

		std::cout << "current level = " << cur << std::endl;
		map.set_current_level(cur - 1);

		std::cout << "after add level Faces :" << std::endl;
		map.foreach_cell([&] (IHMap2::Face f)
		{
			std::cout << f << " | " << map.get_dart_level(f.dart) << std::endl;
		});
		std::cout << "End Vertices" << std::endl;
	}

	/*

	{
		map.add_mixed_level();

		std::cout << "after add level Faces :" << std::endl;
		map.template foreach_cell<IHMap2::FACE>([&] (typename IHMap2::Face f)
		{
			std::cout << f << " | " << map.get_dart_level(f.dart) << std::endl;
		});
		std::cout << "End Faces" << std::endl;


		unsigned int cur = map.get_current_level();

		std::cout << "current level = " << cur << std::endl;
		map.set_current_level(cur - 1);

		std::cout << "after add level Faces :" << std::endl;
		map.template foreach_cell<IHMap2::FACE>([&] (typename IHMap2::Face f)
		{
			std::cout << f << " | " << map.get_dart_level(f.dart) << std::endl;
		});
		std::cout << "End Vertices" << std::endl;
	}

	{
		map.add_mixed_level();

		std::cout << "after add level Faces :" << std::endl;
		map.template foreach_cell<IHMap2::FACE>([&] (typename IHMap2::Face f)
		{
			std::cout << f << " | " << map.get_dart_level(f.dart) << std::endl;
		});
		std::cout << "End Faces" << std::endl;


		unsigned int cur = map.get_current_level();

		std::cout << "current level = " << cur << std::endl;
		map.set_current_level(cur - 1);

		std::cout << "after add level Faces :" << std::endl;
		map.template foreach_cell<IHMap2::FACE>([&] (typename IHMap2::Face f)
		{
			std::cout << f << " | " << map.get_dart_level(f.dart) << std::endl;
		});
		std::cout << "End Vertices" << std::endl;
	}
	*/

	return 0;
}
