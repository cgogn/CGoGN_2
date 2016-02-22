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

#ifndef CORE_CMAP_SANITY_CHECK_H_
#define CORE_CMAP_SANITY_CHECK_H_

namespace cgogn
{

/**
 * \brief Tests if all \p ORBIT orbits are well embedded
 * \details An orbit is well embedded if all its darts
 * have the same embedding (index)
 *
 * \tparam ORBIT [description]
 * \return [description]
 */
template <Orbit ORBIT, typename MAP>
bool is_well_embedded(const MAP& map)
{
	bool result = true;
	map.foreach_cell([&] (Cell<ORBIT> c)
	{
		result = map.template is_well_embedded<ORBIT>(c);
	});
	return result;
}

/**
 * \brief Tests if each \p ORBIT orbit of the map has a unique index in the \p ORBIT attribute container
 * \details This is a injectivity test from the darts embedding to the attribute indices
 *
 * \tparam ORBIT [description]
 * \return [description]
 */
template <Orbit ORBIT, typename MAP>
bool is_orbit_embedding_unique(MAP& map)
{
	static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
	cgogn_message_assert(map.template is_orbit_embedded<ORBIT>(), "Invalid parameter: orbit not embedded");

	typename MAP::template AttributeHandler<unsigned int, ORBIT> counter = map.template add_attribute<unsigned int, ORBIT>("__tmp_counter");
	for (unsigned int& i : counter) i = 0;

	bool result = true;
	map.template foreach_cell<FORCE_DART_MARKING>([&] (Cell<ORBIT> c)
	{
		if (counter[c] > 0)
		{
			result = false;
			std::cout << "Index #" << map.get_embedding(c) << " of attribute container of orbit " << orbit_name(ORBIT) << " is referenced by more than one orbit" << std::endl;
		}
		counter[c]++;
	});

	map.remove_attribute(counter);
	return result;
}

/**
 * \brief Tests if each index in the \p ORBIT attribute container is referenced the good number of times,
 * \details The number of references to an index in the \p ORBIT attribute container should be equal to the
 *  number of darts that are embedded on this index
 *
 * \tparam ORBIT [description]
 * \return [description]
 */
template <Orbit ORBIT, typename MAP>
bool is_container_well_referenced(MAP& map)
{
	static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
	cgogn_message_assert(map.template is_orbit_embedded<ORBIT>(), "Invalid parameter: orbit not embedded");

	typename MAP::template AttributeHandler<unsigned int, ORBIT> counter = map.template add_attribute<unsigned int, ORBIT>("__tmp_counter");

	const MAP& const_map = static_cast<const MAP&>(map);
	const typename MAP::template ChunkArrayContainer<unsigned int>& container = const_map.template get_attribute_container<ORBIT>();

	// a counter is initialized to 0 for each "used" index of the container
	for (unsigned int i = container.begin(), end = container.end(); i != end; container.next(i))
		counter[i] = 0;

	// for each dart of the map, the counter corresponding to its embedding index is incremented
	map.foreach_dart([&] (Dart d) { counter[map.template get_embedding<ORBIT>(d)]++; });

	bool result = true;
	for (unsigned int i = container.begin(), end = container.end(); i != end; container.next(i))
	{
		unsigned int nb_refs = container.get_nb_refs(i);
		if (nb_refs == 1)
		{
			result = false;
			std::cout << "Index #" << i << " has 1 ref (considered used) but is not referenced by any dart" << std::endl;
		}
		if (nb_refs != counter[i] + 1)
		{
			result = false;
			std::cout << "Index #" << i << " has " << nb_refs <<  " refs but is referenced by " << counter[i] << " darts (nb_refs should be equal to " << counter[i] + 1 << ")" << std::endl;
		}
	}

	map.remove_attribute(counter);
	return result;
}

} // namespace cgogn

#endif // CORE_CMAP_SANITY_CHECK_H_
