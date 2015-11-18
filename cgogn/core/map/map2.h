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

#ifndef CORE_MAP_MAP2_H_
#define CORE_MAP_MAP2_H_

#include <core/map/map1.h>
#include <core/basic/dart_marker.h>

namespace cgogn
{

template <typename DATA_TRAITS>
class Map2 : public Map1<DATA_TRAITS>
{
public:

	typedef Map1<DATA_TRAITS> Inherit;

	static const unsigned int VERTEX = VERTEX2;
	static const unsigned int EDGE   = EDGE2;
	static const unsigned int FACE   = FACE2;
	static const unsigned int VOLUME = VOLUME3;

	template<typename T>
	using VertexAttributeHandler = cgogn::AttributeHandler<DATA_TRAITS, T, VERTEX>;

	template<typename T>
	using EdgeAttributeHandler = cgogn::AttributeHandler<DATA_TRAITS, T, EDGE>;

	template<typename T>
	using FaceAttributeHandler = cgogn::AttributeHandler<DATA_TRAITS, T, FACE>;

protected:

	void init()
	{
		ChunkArray<DATA_TRAITS::CHUNK_SIZE, Dart>* phi2 = this->topology_.template addAttribute<Dart>("phi2");
		this->topo_relations_.push_back(phi2);
	}

	/*******************************************************************************
	 * Low-level topological operations
	 *******************************************************************************/

	/**
	 * \brief Link dart d with dart e by an involution
	 * @param d,e the darts to link
	 *	- Before: d->d and e->e
	 *	- After:  d->e and e->d
	 */
	void phi2sew(Dart d, Dart e)
	{
		cgogn_assert(phi2(d) == d);
		cgogn_assert(phi2(e) == e);
		(*(this->topo_relations_[2]))[d.index] = e;
		(*(this->topo_relations_[2]))[e.index] = d;
	}

	/**
	 * \brief Unlink the current dart by an involution
	 * @param d the dart to unlink
	 * - Before: d->e and e->d
	 * - After:  d->d and e->e
	 */
	void phi2unsew(Dart d)
	{
		Dart e = phi2(d) ;
		(*(this->topo_relations_[2]))[d.index] = d;
		(*(this->topo_relations_[2]))[e.index] = e;
	}

public:

	Map2() : Inherit()
	{
		init();
	}

	~Map2() override
	{}

	/*******************************************************************************
	 * Basic topological operations
	 *******************************************************************************/

	/**
	 * \brief phi2
	 * @param d
	 * @return
	 */
	Dart phi2(Dart d) const
	{
		// phi2 first topo relation
		return (*(this->topo_relations_[2]))[d.index];
	}

	/*******************************************************************************
	 * Orbits traversal
	 *******************************************************************************/

	template <typename FUNC>
	inline void foreach_dart_of_vertex(Dart d, const FUNC& f) const
	{
		Dart dNext = d;
		do
		{
			f(dNext);
			dNext = phi2(this->phi_1(dNext));
		} while (dNext != d);
	}

	template <typename FUNC>
	inline void foreach_dart_of_edge(Dart d, const FUNC& f) const
	{
		f(d);
		f(phi2(d));
	}

	template <typename FUNC>
	inline void foreach_dart_of_face(Dart d, const FUNC& f) const
	{
		Inherit::foreach_dart_of_face(d, f);
	}

	template <typename FUNC>
	void foreach_dart_of_volume(Dart d, const FUNC& f) const
	{
		DartMarkerStore<Map2> marker(*this); // get a marker

		std::vector<Dart>* visited_faces = dart_buffers_thread->getBuffer();

		visited_faces->push_back(d); // Start with the face of d

		// For every face added to the list
		for(unsigned int i = 0; i < visited_faces->size(); ++i)
		{
			if (!marker.isMarked((*visited_faces)[i]))	// Face has not been visited yet
			{
				// Apply functor to the darts of the face
				Map2::foreach_dart_of_face((*visited_faces)[i], f);

				// mark visited darts (current face)
				// and add non visited adjacent faces to the list of face
				Dart e = (*visited_faces)[i] ;
				do
				{
					marker.mark(e);				// Mark
					Dart adj = phi2(e);			// Get adjacent face
					if (!marker.isMarked(adj))
						visited_faces->push_back(adj);	// Add it
					e = this->phi1(e);
				} while(e != (*visited_faces)[i]);
			}
		}

		dart_buffers_thread->releaseBuffer(visited_faces);
	}

	template <unsigned int ORBIT, typename FUNC>
	void foreach_dart_of_orbit(Cell<ORBIT> c, const FUNC& f) const
	{
		switch(ORBIT)
		{
			case Map2::VERTEX: f(c); break;
			case Map2::EDGE:   foreach_dart_of_edge(c, f); break;
			case Map2::FACE:   foreach_dart_of_face(c, f); break;
			case Map2::VOLUME: foreach_dart_of_volume(c, f); break;
			default: cgogn_assert_not_reached("Cells of this dimension are not handled"); break;
		}
	}
};

} // namespace cgogn

#endif // CORE_MAP_MAP2_H_
