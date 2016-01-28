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

#ifndef CORE_CMAP_SANITY_CHECKS_H_
#define CORE_CMAP_SANITY_CHECKS_H_

#include <core/cmap/map_base.h>

namespace cgogn
{

	/**
	 * \brief Tests if all orbits of \p ORBIT are uniquely embedded
	 * \details This is a surjectivity test from the 
	 * embedding indices to the attributes tables.
	 * 
	 * \tparam ORBIT [description]
	 * \return [description]
	 */
	template <Orbit ORBIT, typename MAP>
	bool is_embedding_surjective(MAP& map)
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		cgogn_message_assert(map.template is_orbit_embedded<ORBIT>(), "Invalid parameter: orbit not embedded");

		typename MAP::template AttributeHandler<unsigned int, ORBIT> counter = map.template add_attribute<unsigned int, ORBIT>("tmp_counter");
		for (unsigned int& elt : counter) elt = 0;

		//count the number of cells
		//pointing to an attribute line
		map.template foreach_cell<ORBIT, FORCE_DART_MARKING>([&counter] (Cell<ORBIT> c) { counter[c]++; });

		bool result = true;
		unsigned int i = 0;
		for (unsigned int& elt : counter)
		{
			// if there is more or less than one cell pointing to
			// one attribute line
			if(elt != 1)
			{
				result = false;
				std::cout << "Orbit #" << i << " is not surjective (" << elt << " embeddings)." << std::endl;
			}
			++i;
		}	

		map.remove_attribute(counter);

		return result;
	}

	/**
	 * \brief Tests if all attributes are uniquely indexed by an 
	 * orbit of \p ORBIT
	 * \details This is a injectivity test from the 
	 * the attributes tables to the embedding indices.
	 * 
	 * \tparam ORBIT [description]
	 * \return [description]
	 */
	template <Orbit ORBIT, typename MAP>
	bool is_embedding_injective(MAP& map)
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
        cgogn_message_assert(map.template is_orbit_embedded<ORBIT>(), "Invalid parameter: orbit not embedded");

        typename MAP::template AttributeHandler<unsigned int, ORBIT> counter = map.template add_attribute<unsigned int, ORBIT>("tmp_counter");
        for (unsigned int& i : counter) i = 0;

        bool result = true;
        map.template foreach_cell<ORBIT, FORCE_DART_MARKING>([&] (Cell<ORBIT> c)
        {
            if (counter[c] > 0)
            {
                result = false;
                std::cout << "Index #" << map.get_embedding(c) << " of orbit " << orbit_name(ORBIT) << " is referenced by more than one orbit" << std::endl;
            }
            counter[c]++;
        });

        map.remove_attribute(counter);
        return result;
	}

	template <Orbit ORBIT, typename MAP>
	bool is_embedding_bijective(MAP& map)
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		cgogn_message_assert(map.template is_orbit_embedded<ORBIT>(), "Invalid parameter: orbit not embedded");

		return is_embedding_surjective<ORBIT>(map) && is_embedding_injective<ORBIT>(map);
	}

    /**
     * \brief Tests if all orbits of type \p ORBIT are well embedded
     * \details An orbit is well embedded if all its darts
     * have the same embedding
     *
     * \tparam ORBIT [description]
     * \return [description]
     */
    template <Orbit ORBIT, typename MAP>
    bool is_well_embedded(MAP& map)
    {
        bool result = true;

        map.template foreach_cell<ORBIT>([&] (Cell<ORBIT> c)
        {
            result = map.template is_well_embedded(c);
        });

        return result;
    }

} // namespace cgogn

#endif // CORE_CMAP_SANITY_CHECKS_H_