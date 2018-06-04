/*******************************************************************************
* CGoGN: Combinatorial and Geometric modeling with Generic N-dimensional Maps  *
* Copyright (C) 2015, IGG Group, ICube, University of Strasbourg, France       *
*                                                                              *
* This library is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU Lesser General Public License as published by the *
* Free Software Foundation; either version 2.1 of the License, or (at your     *
* option) any later version.                                                   *
*                                                                              *
* This library is distributed in the hope that it will be useful, but WITHOUT  *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License  *
* for more details.                                                            *
*                                                                              *
* You should have received a copy of the GNU Lesser General Public License     *
* along with this library; if not, write to the Free Software Foundation,      *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.           *
*                                                                              *
* Web site: http://cgogn.unistra.fr/                                           *
* Contact information: cgogn@unistra.fr                                        *
*                                                                              *
*******************************************************************************/

#ifndef CGOGN_CORE_CMAP_CMAP0_BUILDER_H_
#define CGOGN_CORE_CMAP_CMAP0_BUILDER_H_

#include <cgogn/core/cmap/map_base.h>

namespace cgogn
{

template <typename MAP0>
class CMap0Builder_T
{
    static_assert(MAP0::DIMENSION == 0, "CMap0Builder_T works only with 0D Maps.");

public:

    using Self = CMap0Builder_T<MAP0>;
    using Map0 = MAP0;
    using CDart = typename Map0::CDart;
    using Vertex = typename Map0::Vertex;

    template <typename T>
    using ChunkArrayContainer = typename Map0::template ChunkArrayContainer<T>;

    template <typename T>
    using ChunkArray = typename Map0::template ChunkArray<T>;

    inline CMap0Builder_T(Map0& map) : map_(map) {}
    CGOGN_NOT_COPYABLE_NOR_MOVABLE(CMap0Builder_T);
    inline ~CMap0Builder_T() {}

public:
    template <Orbit ORBIT>
    inline void create_embedding()
    {
        map_.template create_embedding<ORBIT>();
    }

    template <Orbit ORBIT>
    inline ChunkArrayContainer<uint32>& attribute_container()
    {
        return map_.template non_const_attribute_container<ORBIT>();
    }

    template <class CellType>
    inline void set_embedding(Dart d, uint32 emb)
    {
        map_.template set_embedding<CellType>(d, emb);
    }

    template <class CellType, Orbit ORBIT>
    inline void set_orbit_embedding(Cell<ORBIT> c, uint32 emb)
    {
        map_.template set_orbit_embedding<CellType>(c, emb);
    }

    template <class CellType>
    inline uint32 new_orbit_embedding(CellType c)
    {
        return map_.new_orbit_embedding(c);
    }

    inline Dart add_topology_element()
    {
            return map_.add_topology_element();
    }

private:

    Map0& map_;
};

} //end namespace cgogn

#endif // CGOGN_CORE_CMAP_CMAP0_BUILDER_H_
