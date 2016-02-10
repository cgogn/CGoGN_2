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

#ifndef MULTIRESOLUTION_CPH_IHCMAP2_H_
#define MULTIRESOLUTION_CPH_IHCMAP2_H_

#include <core/cmap/cmap2.h>
#include <multiresolution/cph/cph2.h>
//#include <multiresolution/cph/attribute_handler_cph.h>

namespace cgogn
{

template <typename CONTAINER, typename MAP>
class ContainerCPHBrowser : public ContainerBrowser
{
	const CONTAINER& cac_;
	const MAP* map_;

public:
	ContainerCPHBrowser(const CONTAINER& cac, const MAP* map) : cac_(cac), map_(map) {}
	virtual unsigned int begin() const { return cac_.real_begin(); }
	virtual unsigned int end() const { return cac_.real_end(); }
	virtual void next(unsigned int &it)  const
	{
		cac_.real_next(it);
		if(map_->get_dart_level(Dart(it)) > map_->get_current_level())
			it = cac_.real_end();
	}
	virtual void next_primitive(unsigned int &it, unsigned int primSz) const { cac_.real_next_primitive(it,primSz); }
	virtual void enable() {}
	virtual void disable() {}
	virtual ~ContainerCPHBrowser() {}
};

template <typename DATA_TRAITS, typename MAP_TYPE>
class IHCMap2_T : public CMap2_T<DATA_TRAITS, MAP_TYPE>, public CPH2<DATA_TRAITS>
{
public:

	typedef CMap2_T<DATA_TRAITS, MAP_TYPE> Inherit_CMAP;
	typedef CPH2<DATA_TRAITS> Inherit_CPH;
	typedef IHCMap2_T<DATA_TRAITS, MAP_TYPE> Self;

	friend typename Self::Inherit_CMAP;
	//	friend typename Inherit::Inherit;

	friend class DartMarker_T<Self>;

	static const Orbit DART = Orbit::DART;
	static const Orbit VERTEX = Inherit_CMAP::VERTEX;
	static const Orbit EDGE   = Inherit_CMAP::EDGE;
	static const Orbit FACE   = Inherit_CMAP::FACE;
	static const Orbit VOLUME = Inherit_CMAP::VOLUME;

	using Vertex = typename Inherit_CMAP::Vertex;
	using Edge = typename Inherit_CMAP::Edge;
	using Face = typename Inherit_CMAP::Face;
	using Volume = typename Inherit_CMAP::Volume;

	template <typename T>
	using ChunkArray =  typename Inherit_CMAP::template ChunkArray<T>;
	template <typename T>
	using ChunkArrayContainer =  typename Inherit_CMAP::template ChunkArrayContainer<T>;

	template<typename T, Orbit ORBIT>
	using AttributeHandler = typename Inherit_CMAP::template AttributeHandler<T, ORBIT>;
	template<typename T>
	using DartAttributeHandler = AttributeHandler<T, Self::DART>;
	template<typename T>
	using VertexAttributeHandler = AttributeHandler<T, Self::VERTEX>;
	template<typename T>
	using EdgeAttributeHandler = AttributeHandler<T, Self::EDGE>;
	template<typename T>
	using FaceAttributeHandler = AttributeHandler<T, Self::FACE>;
	template<typename T>
	using VolumeAttributeHandler = AttributeHandler<T, Self::VOLUME>;

//	template<typename T>
//	using VertexAttributeHandlerCPH = AttributeHandlerCPH<Self, DATA_TRAITS, T, Self::VERTEX>;

	using DartMarker = typename cgogn::DartMarker<Self>;
	using DartMarkerStore = typename cgogn::DartMarkerStore<Self>;

	ChunkArray<unsigned int>* next_level_cell[NB_ORBITS];

protected:
	ContainerCPHBrowser<ChunkArrayContainer<unsigned char>, Self>* cph_browser;

	inline void init()
	{
		cph_browser = new ContainerCPHBrowser<ChunkArrayContainer<unsigned char>, Self>(this->topology_, this);
		this->topology_.set_current_browser(cph_browser);

		// Inherit_CPH::new_level_darts();
	}

public:
	IHCMap2_T() : Inherit_CMAP(), Inherit_CPH(this->topology_)
	{
		init();
	}

	~IHCMap2_T() override
	{}

	IHCMap2_T(Self const&) = delete;
	IHCMap2_T(Self &&) = delete;
	Self& operator=(Self const&) = delete;
	Self& operator=(Self &&) = delete;

public:
	/*******************************************************************************
	 * Basic topological operations
	 *******************************************************************************/

	inline Dart phi1(Dart d) const
	{
		cgogn_message_assert(Inherit_CPH::get_dart_level(d) <= Inherit_CPH::get_current_level(), "Access to a dart introduced after current level") ;

		bool finished = false ;
		unsigned int edge_id = Inherit_CPH::get_edge_id(d) ;
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
		unsigned int edge_id = Inherit_CPH::get_edge_id(d) ;
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

	/**
	* \brief add a Dart in the map
	* @return the new Dart
	*/
	inline Dart add_dart()
	{
		Dart d = Inherit_CMAP::add_dart();

		Inherit_CPH::set_edge_id(d, 0);
		Inherit_CPH::set_dart_level(d, Inherit_CPH::get_current_level());

		// update max level if needed
		if(Inherit_CPH::get_current_level() > Inherit_CPH::get_maximum_level())
		{
			Inherit_CPH::set_maximum_level(Inherit_CPH::get_current_level());
			// Inherit_CPH::new_level_darts();
		}

		//		Inherit_CPH::inc_nb_darts(get_current_level());

		return d ;
	}

//	template <Orbit ORBIT>
//	inline unsigned int get_embedding_cph(Cell<ORBIT> c) const
//	{
//		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
//		cgogn_message_assert(Inherit::is_orbit_embedded<ORBIT>(), "Invalid parameter: orbit not embedded");

//		unsigned int nb_steps = Inherit::get_current_level() - Inherit::get_dart_level(c.dart);
//		unsigned int index = Inherit::get_embedding(c);

//		unsigned int step = 0;
//		while(step < nb_steps)
//		{
//			step++;
//			unsigned int next = next_level_cell_[ORBIT]->operator[](index);
//			//index = next;
//			if(next != EMBNULL) index = next;
//			else break;
//		}

//		return index;
//	}

protected:

	/*******************************************************************************
	 * Orbits traversal
	 *******************************************************************************/

	template <typename FUNC>
	inline void foreach_dart_of_DART(Dart d, const FUNC& f) const
	{
		f(d);
	}

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
	inline void foreach_dart_of_PHI21(Dart d, const FUNC& f) const
	{
		Dart it = d;
		do
		{
			f(it);
			it = Inherit_CMAP::phi2(Inherit_CMAP::phi_1(it));
		} while (it != d);
	}

	template <typename FUNC>
	void foreach_dart_of_PHI1_PHI2(Dart d, const FUNC& f) const
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

public:
	template <Orbit ORBIT, typename FUNC>
	inline void foreach_dart_of_orbit(Cell<ORBIT> c, const FUNC& f) const
	{
		static_assert(ORBIT == Orbit::DART || ORBIT == Orbit::PHI1 ||
					  ORBIT == Orbit::PHI2 || ORBIT == Orbit::PHI1_PHI2 || ORBIT == Orbit::PHI21,
					  "Orbit not supported in a CMap2");
		switch (ORBIT)
		{
			case Orbit::DART: foreach_dart_of_DART(c, f); break;
			case Orbit::PHI1: foreach_dart_of_PHI1(c, f); break;
			case Orbit::PHI2: foreach_dart_of_PHI2(c, f); break;
			case Orbit::PHI1_PHI2: foreach_dart_of_PHI1_PHI2(c, f); break;
			case Orbit::PHI21: foreach_dart_of_PHI21(c, f); break;
			case Orbit::PHI2_PHI3:
			case Orbit::PHI1_PHI3:
			default: cgogn_assert_not_reached("Cells of this dimension are not handled"); break;
		}
	}

public:

	inline unsigned int face_degree(Dart d)
	{
		unsigned int count = 0 ;
		Dart it = d ;
		do
		{
			++count ;
			it = phi1(it) ;
		} while (it != d) ;
		return count ;
	}

//	template <typename MAP, typename T, Orbit ORBIT>
//	inline AttributeHandlerCPH<MAP, T, ORBIT> add_attribute(const std::string& attribute_name = "")
//	{
//		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");

//	}

//	template <typename MAP, typename T, Orbit ORBIT>
//	inline bool remove_attribute(AttributeHandlerCPH<MAP, T, ORBIT>& ah)
//	{
//		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");

//	}
};


template <typename MAP_TRAITS>
struct IHCMap2Type
{
	typedef IHCMap2_T<MAP_TRAITS, IHCMap2Type<MAP_TRAITS>> TYPE;
};

template <typename MAP_TRAITS>
using IHCMap2 = IHCMap2_T<MAP_TRAITS, IHCMap2Type<MAP_TRAITS>>;

} // namespace cgogn

#endif // MULTIRESOLUTION_CPH_IHCMAP2_H_

