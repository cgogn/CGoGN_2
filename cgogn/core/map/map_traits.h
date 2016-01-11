#ifndef CORE_MAP_MAP_TRAITS_H_
#define CORE_MAP_MAP_TRAITS_H_

#include <array>

namespace cgogn
{

struct DefaultMapTraits
{
	static const unsigned int CHUNK_SIZE = 4096;
	using Real = double;
	using Vec3 = std::array<Real, 3>;
};

}

#endif // CORE_MAP_MAP_TRAITS_H_
