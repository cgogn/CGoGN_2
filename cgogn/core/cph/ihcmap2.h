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

#ifndef CORE_CPH_IHCMAP2_H_
#define CORE_CPH_IHCMAP2_H_

#include <cgogn/core/map/cmap2.h>
#include <cgogn/core/cph/attribute_handler_cph.h>

namespace cgogn
{

template <typename DATA_TRAITS, typename TOPO_TRAITS>
class IHCMap2_T : protected CMap2_T<DATA_TRAITS, TOPO_TRAITS>
{
public:

	typedef CMap2_T<DATA_TRAITS, TOPO_TRAITS> Inherit;
	typedef IHCMap2_T<DATA_TRAITS, TOPO_TRAITS> Self;

	friend typename Self::Inherit;
	friend typename Inherit::Inherit;

	template <typename MAP> friend class cgogn::DartMarkerT;

	static const Orbit DART = Orbit::DART;
	static const Orbit VERTEX = Orbit::PHI21;
	static const Orbit EDGE   = Orbit::PHI2;
	static const Orbit FACE   = Orbit::PHI1;
	static const Orbit VOLUME = Orbit::PHI1_PHI2;

	typedef Cell<Self::VERTEX> Vertex;
	typedef Cell<Self::EDGE> Edge;
	typedef Cell<Self::FACE> Face;
	typedef Cell<Self::VOLUME> Volume;

	template<typename T, Orbit ORBIT>
	using AttributeHandler = typename AttributeHandlerCPH<T, ORBIT>;
	template<typename T>
	using DartAttributeHandler = AttributeHandlerCPH<T, Self::DART>;
	template<typename T>
	using VertexAttributeHandler = AttributeHandlerCPH<T, Self::VERTEX>;
	template<typename T>
	using EdgeAttributeHandler = AttributeHandlerCPH<T, Self::EDGE>;
	template<typename T>
	using FaceAttributeHandler = AttributeHandlerCPH<T, Self::FACE>;
	template<typename T>
	using VolumeAttributeHandler = AttributeHandlerCPH<T, Self::VOLUME>;

	using DartMarker = typename Inherit::DartMarker;
	using DartMarkerStore = typename Inherit::DartMarkerStore;

private:
	unsigned int current_level_;
	unsigned int maximum_level_;
	unsigned int id_count_;

	DartAttribute<unsigned int> dart_level_ ;
	DartAttribute<unsigned int> edge_id_ ;

public:
	IHCMap2_T() : Inherit()
	{
		init();
	}

	~IHCMap2_T() override
	{}

	IHCMap2_T(Self const&) = delete;
	IHCMap2_T(Self &&) = delete;
	Self& operator=(Self const&) = delete;
	Self& operator=(Self &&) = delete;


	inline void init_implicit_properties()
	{
		//initEdgeId() ;

		//init each edge Id at 0
		for(Dart d : Inherit)
		{
			set_edge_id(d, 0);
		}

		// for(unsigned int orbit = 0u; orbit < Orbit::NB_ORBITS; ++orbit)
		// {
		// 	if(m_nextLevelCell[orbit] != NULL)
		// 	{
		// 		AttributeContainer& cellCont = m_attribs[orbit] ;
		// 		for(unsigned int i = cellCont.begin(); i < cellCont.end(); cellCont.next(i))
		// 			m_nextLevelCell[orbit]->operator[](i) = EMBNULL ;
		// 	}
		// }
	}
	
	/*******************************************************************************
	 * Basic topological operations
	 *******************************************************************************/

	inline Dart phi1(Dart d) const
	{
		cgogn_debug_assert(get_dart_level(d) <= get_current_level(), "Access to a dart introduced after current level") ;
	
		bool finished = false ;
		unsigned int edge_id = get_edge_id(d) ;
		Dart it = d ;
		do
		{
			it = Inherit::phi1(it) ;
			if(get_dart_level(it) <= get_current_level())
				finished = true ;
			else
			{
				while(get_edge_id(it) != edge_id)
					it = Inherit::phi1(Inherit::phi2(it)) ;
			}
		} while(!finished) ;
		return it ;
	}

	inline Dart phi_1(Dart d) const
	{
		cgogn_debug_assert(get_dart_level(d) <= get_current_level(), "Access to a dart introduced after current level") ;

		bool finished = false ;
		Dart it = Inherit::phi_1(d) ;
		unsigned int edge_id = get_edge_id(d) ;
		do
		{
			if(get_dart_level(it) <= get_current_level())
				finished = true ;
			else
			{
				it = Inherit::phi_1(it) ;
				while(get_edge_id(it) != edge_id)
					it = Inherit::phi_1(Inherit::phi2(it)) ;
			}
		} while(!finished) ;
		return it ;
	}

	/**
	 * \brief phi2
	 * @param d
	 * @return phi2(d)
	 */
	inline Dart phi2(Dart d) const
	{
		cgogn_debug_assert(get_dart_level(d) <= get_current_level(), "Access to a dart introduced after current level") ;

		if(Inherit::phi2(d) == d)
			return d;
		return Inherit::phi2(Inherit::phi_1(phi1(d)));
	}

protected:

	/**
	* \brief add a Dart in the map
	* @return the new Dart
	*/
	inline Dart add_dart()
	{
		Dart d = Inherit::add_dart();
		set_dart_level(d, get_current_level());
		if(get_current_level() > get_maximum_level()) // update max level
			set_maximum_level(get_current_level()); // if needed
		
		return d ;
	}

protected:

	/*******************************************************************************
	 * Orbits traversal
	 *******************************************************************************/

	template <typename FUNC>
	inline void foreach_dart_of_vertex(Dart d, const FUNC& f) const
	{
		Dart it = d;
		do
		{
			f(it);
			it = phi2(phi_1(it));
		} while (it != d);
	}

	template <typename FUNC>
	void foreach_dart_of_volume(Dart d, const FUNC& f) const
	{
		DartMarkerStore marker(*this);

		std::vector<Dart>* visited_faces = cgogn::get_dart_buffers()->get_buffer();
		visited_faces->push_back(d); // Start with the face of d

		// For every face added to the list
		for(unsigned int i = 0; i < visited_faces->size(); ++i)
		{
			if (!marker.is_marked((*visited_faces)[i]))	// Face has not been visited yet
			{
				// mark visited darts (current face)
				// and add non visited adjacent faces to the list of face
				Dart e = (*visited_faces)[i] ;
				do
				{
					f(e); // apply the function to the darts of the face
					marker.mark(e);				// Mark
					Dart adj = phi2(e);			// Get adjacent face
					if (!marker.is_marked(adj))
						visited_faces->push_back(adj);	// Add it
					e = phi1(e);
				} while (e != (*visited_faces)[i]);
			}
		}

		cgogn::get_dart_buffers()->release_buffer(visited_faces);
	}

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_dart_of_orbit(Cell<ORBIT> c, const FUNC& f) const
	{
		switch (ORBIT)
		{
			case Orbit::DART: Inherit::foreach_dart_of_orbit(c, f); break;
			case Orbit::PHI1: Inherit::foreach_dart_of_orbit(c, f); break;
			case Orbit::PHI2: f(c.dart); f(phi2(c.dart)); break;
			case Orbit::PHI1_PHI2: foreach_dart_of_volume(c, f); break;
			case Orbit::PHI21: foreach_dart_of_vertex(c, f); break;
			case Orbit::PHI2_PHI3:
			case Orbit::PHI1_PHI3:
			case Orbit::PHI21_PHI31:
			default: cgogn_assert_not_reached("Cells of this dimension are not handled"); break;
		}
	}

public:
	/***************************************************
	 *              LEVELS MANAGEMENT                  *
	 ***************************************************/

	inline unsigned int get_current_level() const
	{
		return current_level_ ;
	}

	inline void set_current_level(unsigned int l)
	{
		current_level_ = l ;
	}

	inline void inc_current_level() 
	{
		cgogn_debug_assert(get_current_level() < get_maximum_level(), "incCurrentLevel : already at maximum resolution level");
		++current_level_ ;
	}

	inline void dec_current_level()
	{
		cgogn_debug_assert(get_current_level() > 0u, "decCurrentLevel : already at minimum resolution level");
		--current_level_ ;
	}

	inline unsigned int get_maximum_level() const
	{
		return maximum_level_ ;
	}

	inline void set_maximum_level(unsigned int l)
	{
		maximum_level_ = l;
	}

	inline unsigned int get_dart_level(Dart d) const
	{
		return dart_level_[d] ;
	}

	inline void set_dart_level(Dart d, unsigned int l)
	{
		dart_level_[d] = l ;
	}

	/***************************************************
	 *             EDGE ID MANAGEMENT                  *
	 ***************************************************/

	/**
	 * Give a new unique id to all the edges of the map
	 */
	inline void init_edge_ids()
	{
		id_count_ = 0;
		DartMarker marker(*this);

		for (Dart d : Inherit)
		{
			if(!marker.is_marked(d))
			{
				marker.mark_orbit(Edge(d)) ;
				edge_id_[d] = id_count_ ;
				edge_id_[Inherit::phi2(d)] = id_count_++ ;
			}
		}
	}

	/**
	 * Return the next available edge id
	 */
	inline unsigned int get_new_edge_id() const
	{
		return id_count_++ ;
	}

	inline unsigned int get_edge_id(Dart d) const
	{
		return edge_id_[d] ;
	}

	inline void set_edge_id(Dart d, unsigned int i)
	{
		edge_id_[d] = i ;
	}

	inline unsigned int get_tri_refinement_edge_id(Dart d) const
	{
		unsigned int d_id = get_edge_id(phi_1(d));
		unsigned int e_id = get_edge_id(phi1(d));

		unsigned int id = d_id + e_id;

		if(id == 0u)
			return 1u;
		else if(id == 1u)
			return 2u;
		else if(id == 2u)
		{
			if(d_id == e_id)
				return 0u;
			else
				return 1u;
		}

		//else if(id == 3)
		return 0u;
	}

	inline unsigned int get_quad_refinement_edge_id(Dart d) const
	{
		unsigned int e_id = get_edge_id(phi1(d));

		if(e_id == 0u)
			return 1u;

		//else if(e_id == 1)
		return 0u;
	}

};

template <typename DataTraits>
struct IHCMap2TopoTraits
{
	static const int PRIM_SIZE = 1;
	typedef IHCMap2_T<DataTraits, IHCMap2TopoTraits<DataTraits>> CONCRETE;
};

struct IHCMap2DataTraits
{
	static const unsigned int CHUNK_SIZE = 4096;
};

using IHCMap2 = IHCMap2_T<IHCMap2DataTraits, IHCMap2TopoTraits<IHCMap2DataTraits>>;

} // namespace cgogn

#endif // CORE_CPH_IHCMAP2_H_