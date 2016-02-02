
#include <core/cph/ihcmap2_regular.h>

using namespace cgogn;

using IHMap2 = IHCMap2Regular<DefaultMapTraits>;


template<typename T>
using VertexAttributeHandler = IHMap2::VertexAttributeHandler<T>;


int main()
{	
     IHMap2 map;

	// VertexAttributeHandler<int> vertex_position_ = map.get_attribute<int, IHMap2::VERTEX>("position");

    map.add_face(4);

    std::cout << "before add level Faces :" << std::endl;
    map.template foreach_cell<IHMap2::FACE>([&] (typename IHMap2::Face v)
	{
		std::cout << v << std::endl;
	});
    std::cout << "End Faces" << std::endl;


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

	return 0;
}
