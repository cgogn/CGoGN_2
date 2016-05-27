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

#ifndef CGOGN_MULTIRESOLUTION_CPH_IHCMAP2_H_
#define CGOGN_MULTIRESOLUTION_CPH_IHCMAP2_H_

#include <cgogn/core/cmap/cmap2.h>
#include <cgogn/multiresolution/cph/cph2.h>

namespace cgogn
{

template <typename MAP_TRAITS, typename MAP_TYPE>
class IHCMap2_T : public CMap2_T<MAP_TRAITS, MAP_TYPE>, public CPH2<MAP_TRAITS>
{
public:

	static const int32 PRIM_SIZE = 1;

	using MapTraits = MAP_TRAITS;
	using MapType = MAP_TYPE;
	using Inherit_CMAP = CMap2_T<MAP_TRAITS, MAP_TYPE>;
	using Inherit_CPH = CPH2<MAP_TRAITS>;
	using Self = IHCMap2_T<MAP_TRAITS, MAP_TYPE>;

	friend class MapBase<MAP_TRAITS, MAP_TYPE>;
	friend class DartMarker_T<Self>;
	friend class cgogn::DartMarkerStore<Self>;

	using CDart		= typename Inherit_CMAP::CDart;
	using Vertex	= typename Inherit_CMAP::Vertex;
	using Edge		= typename Inherit_CMAP::Edge;
	using Face		= typename Inherit_CMAP::Face;
	using Volume	= typename Inherit_CMAP::Volume;


	template <typename T>
	using ChunkArray =  typename Inherit_CMAP::template ChunkArray<T>;
	template <typename T>
	using ChunkArrayContainer =  typename Inherit_CMAP::template ChunkArrayContainer<T>;

	template <typename T, Orbit ORBIT>
	using Attribute = typename Inherit_CMAP::template Attribute<T, ORBIT>;
	template <typename T>
	using DartAttribute = Attribute<T, Orbit::DART>;
	template <typename T>
	using VertexAttribute = Attribute<T, Orbit::PHI21>;
	template <typename T>
	using EdgeAttribute = Attribute<T, Orbit::PHI2>;
	template <typename T>
	using FaceAttribute = Attribute<T, Orbit::PHI1>;
	template <typename T>
	using VolumeAttribute = Attribute<T, Orbit::PHI1_PHI2>;

	using DartMarker = typename cgogn::DartMarker<Self>;
	using DartMarkerStore = typename cgogn::DartMarkerStore<Self>;

	ChunkArray<uint32>* next_level_cell[NB_ORBITS];

protected:

	inline void init()
	{}

public:

	IHCMap2_T() : Inherit_CMAP(), Inherit_CPH(this->topology_)
	{
		init();
	}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(IHCMap2_T);

	~IHCMap2_T() override
	{}

	/*******************************************************************************
	 * Low-level topological operations
	 *******************************************************************************/

protected:

	/**
	* \brief Init an newly added dart.
	* The dart is added to the current level of resolution
	*/
	inline void init_dart(Dart d)
	{
		Inherit_CMAP::init_dart(d);

		Inherit_CPH::inc_nb_darts();
		Inherit_CPH::set_edge_id(d, 0);
		Inherit_CPH::set_dart_level(d, Inherit_CPH::get_current_level());
	}

	/*******************************************************************************
	 * Basic topological operations
	 *******************************************************************************/

public:

	inline Dart phi1(Dart d) const
	{
		cgogn_message_assert(Inherit_CPH::get_dart_level(d) <= Inherit_CPH::get_current_level(),
							 "Access to a dart introduced after current level") ;

		bool finished = false ;
		uint32 edge_id = Inherit_CPH::get_edge_id(d) ;
		Dart it = d ;
		do
		{
			it = Inherit_CMAP::phi1(it) ;
			if(Inherit_CPH::get_dart_level(it) <= Inherit_CPH::get_current_level())
				finished = true ;
			else
			{
				while(Inherit_CPH::get_edge_id(it) != edge_id)
					it = Inherit_CMAP::phi1(Inherit_CMAP::phi2(it)) ;
			}
		} while(!finished) ;
		return it ;
	}

	inline Dart phi_1(Dart d) const
	{
		cgogn_message_assert(Inherit_CPH::get_dart_level(d) <= Inherit_CPH::get_current_level(), "Access to a dart introduced after current level") ;

		bool finished = false ;
		Dart it = Inherit_CMAP::phi_1(d) ;
		uint32 edge_id = Inherit_CPH::get_edge_id(d) ;
		do
		{
			if(Inherit_CPH::get_dart_level(it) <= Inherit_CPH::get_current_level())
				finished = true ;
			else
			{
				it = Inherit_CMAP::phi_1(it) ;
				while(Inherit_CPH::get_edge_id(it) != edge_id)
					it = Inherit_CMAP::phi_1(Inherit_CMAP::phi2(it)) ;
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
		cgogn_message_assert(Inherit_CPH::get_dart_level(d) <= Inherit_CPH::get_current_level(), "Access to a dart introduced after current level") ;

		if(Inherit_CMAP::phi2(d) == d)
			return d;
		return Inherit_CMAP::phi2(Inherit_CMAP::phi_1(phi1(d)));
	}

	/*******************************************************************************
	 * High-level embedded and topological operations
	 *******************************************************************************/

//	template <Orbit ORBIT>
//	inline uint32 get_embedding_cph(Cell<ORBIT> c) const
//	{
//		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
//		cgogn_message_assert(Inherit::is_embedded<Cell<ORBIT>>(), "Invalid parameter: orbit not embedded");

//		uint32 nb_steps = Inherit::get_current_level() - Inherit::get_dart_level(c.dart);
//		uint32 index = Inherit::get_embedding(c);

//		uint32 step = 0;
//		while(step < nb_steps)
//		{
//			step++;
//			uint32 next = next_level_cell_[ORBIT]->operator[](index);
//			//index = next;
//			if(next != INVALID_INDEX) index = next;
//			else break;
//		}

//		return index;
//	}

	/*!
	 * \brief Add a face in the map.
	 * \param size : the number of edges in the built face
	 * \return A dart of the built face
	 * If the map has DART, VERTEX, EDGE, FACE or VOLUME attributes,
	 * the inserted darts are automatically embedded on new attribute elements.
	 * Actually a FACE attribute is created, if needed, for the new face.
	 */
	Face add_face(uint32 size)
	{
		Face f(this->add_face_topo(size));

		if (this->template is_embedded<CDart>())
			foreach_dart_of_orbit(f, [this] (Dart d)
			{
				this->new_orbit_embedding(CDart(d));
			});

		if (this->template is_embedded<Vertex>())
			foreach_dart_of_orbit(f, [this] (Dart v)
			{
				this->new_orbit_embedding(Vertex(v));
			});

		if (this->template is_embedded<Edge>())
		{
			cgogn_assert_not_reached("Not implemented");
		}
		
		if (this->template is_embedded<Face>())
		{
			cgogn_assert_not_reached("Not implemented");
		}

		if (this->template is_embedded<Volume>())
		{
			cgogn_assert_not_reached("Not implemented");
		}

		return f;
	}

	inline uint32 face_degree(Dart d)
	{
		uint32 count = 0 ;
		Dart it = d ;
		do
		{
			++count ;
			it = phi1(it) ;
		} while (it != d) ;
		return count ;
	}

	/*******************************************************************************
	 * Orbits traversal
	 *******************************************************************************/

protected:

	template <typename FUNC>
	inline void foreach_dart_of_PHI1(Dart d, const FUNC& f) const
	{
		Dart it = d;
		do
		{
			f(it);
			it = phi1(it);
		} while (it != d);
	}

	template <typename FUNC>
	inline void foreach_dart_of_PHI2(Dart d, const FUNC& f) const
	{
		f(d);
		Dart d2 = phi2(d);
		if (d2 != d)
			f(d2);
	}

	template <typename FUNC>
	void foreach_dart_of_PHI1_PHI2(Dart d, const FUNC& f) const
	{
		DartMarkerStore marker(*this);

		std::vector<Dart>* visited_faces = cgogn::dart_buffers()->buffer();
		visited_faces->push_back(d); // Start with the face of d

		// For every face added to the list
		for(uint32 i = 0; i < visited_faces->size(); ++i)
		{
			Dart e = (*visited_faces)[i] ;
			if (!marker.is_marked(e))	// Face has not been visited yet
			{
				// mark visited darts (current face)
				// and add non visited adjacent faces to the list of face
				Dart it = e;
				do
				{
					f(it); // apply the function to the darts of the face
					marker.mark(it);				// Mark
					Dart adj = phi2(it);			// Get adjacent face
					if (!marker.is_marked(adj))
						visited_faces->push_back(adj);	// Add it
					it = phi1(it);
				} while (it != e);
			}
		}

		cgogn::dart_buffers()->release_buffer(visited_faces);
	}

public:
	template <Orbit ORBIT, typename FUNC>
	inline void foreach_dart_of_orbit(Cell<ORBIT> c, const FUNC& f) const
	{
		static_assert(ORBIT == Orbit::DART || ORBIT == Orbit::PHI1 ||
					  ORBIT == Orbit::PHI2 || ORBIT == Orbit::PHI1_PHI2 || ORBIT == Orbit::PHI21,
					  "Orbit not supported in a CMap2");
		switch (ORBIT)
		{
			case Orbit::DART: f(c.dart); break;
			case Orbit::PHI1: foreach_dart_of_PHI1(c.dart, f); break;
			case Orbit::PHI2: foreach_dart_of_PHI2(c.dart, f); break;
			case Orbit::PHI1_PHI2: foreach_dart_of_PHI1_PHI2(c.dart, f); break;
			case Orbit::PHI21: this->foreach_dart_of_PHI21(c.dart, f); break;
			case Orbit::PHI2_PHI3:
			case Orbit::PHI1_PHI3:
			default: cgogn_assert_not_reached("Cells of this dimension are not handled"); break;
		}
	}

};

template <typename MAP_TRAITS>
struct IHCMap2Type
{
	using TYPE = IHCMap2_T<MAP_TRAITS, IHCMap2Type<MAP_TRAITS>>;
};

template <typename MAP_TRAITS>
using IHCMap2 = IHCMap2_T<MAP_TRAITS, IHCMap2Type<MAP_TRAITS>>;

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_MULTIRESOLUTION_CPH_IHCMAP2_CPP_))
extern template class CGOGN_MULTIRESOLUTION_API IHCMap2_T<DefaultMapTraits, IHCMap2Type<DefaultMapTraits>>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_MULTIRESOLUTION_CPH_IHCMAP2_CPP_))

} // namespace cgogn

#endif // CGOGN_MULTIRESOLUTION_CPH_IHCMAP2_H_
