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
	typedef Map2<DATA_TRAITS> Self;

	static const unsigned int VERTEX = VERTEX2;
	static const unsigned int EDGE   = EDGE2;
	static const unsigned int FACE   = FACE2;
	static const unsigned int VOLUME = VOLUME3;

	typedef Cell<Self::VERTEX> Vertex;
	typedef Cell<Self::EDGE> Edge;
	typedef Cell<Self::FACE> Face;
	typedef Cell<Self::VOLUME> Volume;

	template<typename T>
	using ChunkArray =  typename Inherit::template ChunkArray<T>;
	template<typename T>
	using ChunkArrayContainer =  typename Inherit::template ChunkArrayContainer<T>;

	template<typename T, unsigned int ORBIT>
	using AttributeHandler = typename Inherit::template AttributeHandler<T, ORBIT>;

	template<typename T>
	using VertexAttributeHandler = AttributeHandler<T, Self::VERTEX>;

	template<typename T>
	using EdgeAttributeHandler = AttributeHandler<T, Self::EDGE>;

	template<typename T>
	using FaceAttributeHandler = AttributeHandler<T, Self::FACE>;
protected:

	ChunkArray<Dart>* phi2_;

	void init()
	{
		phi2_ = this->topology_.template add_attribute<Dart>("phi2");
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
	void phi2_sew(Dart d, Dart e)
	{
		cgogn_assert(phi2(d) == d);
		cgogn_assert(phi2(e) == e);
		(*phi2_)[d.index] = e;
		(*phi2_)[e.index] = d;
	}

	/**
	 * \brief Unlink the current dart by an involution
	 * @param d the dart to unlink
	 * - Before: d->e and e->d
	 * - After:  d->d and e->e
	 */
	void phi2_unsew(Dart d)
	{
		Dart e = phi2(d) ;
		(*phi2_)[d.index] = d;
		(*phi2_)[e.index] = e;
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
		return (*phi2_)[d.index];
	}

	/**
	* \brief add a Dart in the map
	* @return the new Dart
	*/
	inline Dart add_dart()
	{
		Dart d = Inherit::add_dart();

		(*phi2_)[d.index] = d;

		return d;
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

		std::vector<Dart>* visited_faces = cgogn::getDartBuffers()->get_buffer();

		visited_faces->push_back(d); // Start with the face of d

		// For every face added to the list
		for(unsigned int i = 0; i < visited_faces->size(); ++i)
		{
			if (!marker.is_marked((*visited_faces)[i]))	// Face has not been visited yet
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
					if (!marker.is_marked(adj))
						visited_faces->push_back(adj);	// Add it
					e = this->phi1(e);
				} while (e != (*visited_faces)[i]);
			}
		}

		cgogn::getDartBuffers()->release_buffer(visited_faces);
	}

	template <unsigned int ORBIT, typename FUNC>
	inline void foreach_dart_of_orbit(Cell<ORBIT> c, const FUNC& f) const
	{
		switch (ORBIT)
		{
			case VERTEX1: Inherit::foreach_dart_of_vertex(c, f); break;
			case VERTEX2: foreach_dart_of_vertex(c, f); break;
			case EDGE2:   foreach_dart_of_edge(c, f); break;
			case FACE2:   foreach_dart_of_face(c, f); break;
			case VOLUME3: foreach_dart_of_volume(c, f); break;
			default:      cgogn_assert_not_reached("Cells of this dimension are not handled"); break;
		}
	}

	template <typename FUNC>
	inline void foreach_dart_of_orbit(Dart d, unsigned int orbit, const FUNC& f) const
	{
		switch (orbit)
		{
			case VERTEX1: Inherit::foreach_dart_of_vertex(d, f); break;
			case VERTEX2: foreach_dart_of_vertex(d, f); break;
			case EDGE2:   foreach_dart_of_edge(d, f); break;
			case FACE2:   foreach_dart_of_face(d, f); break;
			case VOLUME3: foreach_dart_of_volume(d, f); break;
			default:      cgogn_assert_not_reached("Cells of this dimension are not handled"); break;
		}
	}

	/*******************************************************************************
	 * Embedding management
	 *******************************************************************************/

	template <unsigned int ORBIT>
	inline void set_orbit_embedding(Cell<ORBIT> c, unsigned int emb)
	{
		foreach_dart_of_orbit(c, [this, emb] (Dart d) {	this->set_embedding<ORBIT>(d, emb); });
	}

	inline void set_orbit_embedding(Dart d, unsigned int orbit, unsigned int emb)
	{
		foreach_dart_of_orbit(d, orbit, [this, orbit, emb] (Dart it) { this->set_embedding(it, orbit, emb); });
	}

protected:

	void init_orbit_embedding(unsigned int orbit) override
	{
		cgogn_message_assert(this->attributes_[orbit].size() == 0, "init_orbit_embedding : container is not empty");

		switch (orbit)
		{
			case VERTEX1:
				for (Dart d : cells<VERTEX1, FORCE_DART_MARKING>(*this))
				{
					unsigned int idx = this->attributes_[orbit].template insert_lines<1>();
					this->attributes_[orbit].init_markers_of_line(idx);
					set_orbit_embedding(d, orbit, idx);
				}
				break;
			case VERTEX2:
				for (Dart d : cells<VERTEX2, FORCE_DART_MARKING>(*this))
				{
					unsigned int idx = this->attributes_[orbit].template insert_lines<1>();
					this->attributes_[orbit].init_markers_of_line(idx);
					set_orbit_embedding(d, orbit, idx);
				}
				break;
			case EDGE2:
				for (Dart d : cells<EDGE2, FORCE_DART_MARKING>(*this))
				{
					unsigned int idx = this->attributes_[orbit].template insert_lines<1>();
					this->attributes_[orbit].init_markers_of_line(idx);
					set_orbit_embedding(d, orbit, idx);
				}
				break;
			case FACE2:
				for (Dart d : cells<FACE2, FORCE_DART_MARKING>(*this))
				{
					unsigned int idx = this->attributes_[orbit].template insert_lines<1>();
					this->attributes_[orbit].init_markers_of_line(idx);
					set_orbit_embedding(d, orbit, idx);
				}
				break;
			case VOLUME3:
				for (Dart d : cells<VOLUME3, FORCE_DART_MARKING>(*this))
				{
					unsigned int idx = this->attributes_[orbit].template insert_lines<1>();
					this->attributes_[orbit].init_markers_of_line(idx);
					set_orbit_embedding(d, orbit, idx);
				}
				break;
			default:
				cgogn_assert_not_reached("Cells of this dimension are not handled"); break;
				break;
		}
	}
};

} // namespace cgogn

#endif // CORE_MAP_MAP2_H_
