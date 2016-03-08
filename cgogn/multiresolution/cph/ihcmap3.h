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

#ifndef MULTIRESOLUTION_CPH_IHCMAP3_H_
#define MULTIRESOLUTION_CPH_IHCMAP3_H_

#include <core/cmap/cmap3.h>
#include <multiresolution/cph/cph3.h>

namespace cgogn
{



template <typename DATA_TRAITS, typename MAP_TYPE>
class IHCMap3_T :public CMap3_T<DATA_TRAITS, MAP_TYPE>, public CPH3<DATA_TRAITS> // Can't use virtual inheritance because of the use of the CRTP
{
public:

	using Inherit_CMAP = CMap3_T<DATA_TRAITS, MAP_TYPE>;
	using Inherit_CPH = CPH3<DATA_TRAITS>;
	using Self = IHCMap3_T<DATA_TRAITS, MAP_TYPE>;

	friend typename Self::Inherit_CMAP;
	//	friend typename Inherit::Inherit;

	friend class DartMarker_T<Self>;

	static const Orbit DART = Orbit::DART;
	static const Orbit VERTEX = Orbit::PHI21;
	static const Orbit EDGE   = Orbit::PHI2;
	static const Orbit FACE   = Orbit::PHI1;
	static const Orbit VOLUME = Orbit::PHI1_PHI2;

	using Vertex = Cell<Self::VERTEX>;
	using Edge = Cell<Self::EDGE>;
	using Face = Cell<Self::FACE>;
	using Volume = Cell<Self::VOLUME>;

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
	using DartMarker = typename cgogn::DartMarker<Self>;
	using DartMarkerStore = typename cgogn::DartMarkerStore<Self>;

	ChunkArray<unsigned int>* next_level_cell[NB_ORBITS];

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
		ContainerCPHBrowser& operator=(const ContainerCPHBrowser&) = delete;
	};

protected:
	ContainerCPHBrowser<ChunkArrayContainer<unsigned char>, Self>* cph_browser;

	inline void init()
	{
		cph_browser = new ContainerCPHBrowser<ChunkArrayContainer<unsigned char>, Self>(this->topology_, this);
		this->topology_.set_current_browser(cph_browser);
	}

public:
	IHCMap3_T() : Inherit_CMAP(), Inherit_CPH(this->topology_)
	{
		init();
	}

	~IHCMap3_T() override
	{}

	IHCMap3_T(Self const&) = delete;
	IHCMap3_T(Self &&) = delete;
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
					it = Inherit_CMAP::phi1(phi2bis(it)) ;
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
					it = Inherit_CMAP::phi_1(phi2bis(it)) ;
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

		return Inherit_CMAP::phi2(Inherit_CMAP::phi_1(phi1(d)));
	}

	inline Dart phi3(Dart d) const
	{
		cgogn_message_assert(Inherit_CPH::get_dart_level(d) <= Inherit_CPH::get_current_level(), "Access to a dart introduced after current level") ;

		if(Inherit_CMAP::phi3(d) == d);
		return d;

		return Inherit_CMAP::phi3(Inherit_CMAP::phi_1(phi1(d)));
	}

	/**
		* \brief add a Dart in the map
		* @return the new Dart
		*/
	inline void init_dart(Dart d)
	{
		Inherit_CMAP::init_dart(d);

		Inherit_CPH::inc_nb_darts();
		Inherit_CPH::set_edge_id(d, 0);
		Inherit_CPH::set_face_id(d, 0);
		Inherit_CPH::set_dart_level(d, Inherit_CPH::get_current_level());

		// update max level if needed
		if(Inherit_CPH::get_current_level() > Inherit_CPH::get_maximum_level())
		{
			Inherit_CPH::set_maximum_level(Inherit_CPH::get_current_level());
		}
	}

protected:

	inline Dart phi2bis(Dart d) const
	{
		unsigned int face_id = Inherit_CPH::get_face_id(d);
		Dart it = d;

		it = Inherit_CMAP::phi2(it);

		if(Inherit_CPH::get_face_id(it) == face_id)
			return it;
		else
		{
			do
			{
				it = Inherit_CMAP::phi2(Inherit_CMAP::phi3(it));
			}
			while(Inherit_CPH::get_face_id(it) != face_id);

			return it;
		}
	}

	/*******************************************************************************
		 * Orbits traversal
		 *******************************************************************************/

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

	template <typename FUNC>
	inline void foreach_dart_of_PHI21_PHI31(Dart d, const FUNC& f) const
	{
		DartMarkerStore marker(*this);
		const std::vector<Dart>* marked_darts = marker.get_marked_darts();

		marker.mark(d);
		for(unsigned int i = 0; i < marked_darts->size(); ++i)
		{
			f((*marked_darts)[i]);

			Dart d2 = phi2((*marked_darts)[i]);
			Dart d21 = phi1(d2); // turn in volume
			Dart d23 = phi3(d2); // change volume
			if(!marker.is_marked(d21))
				marker.mark(d21);
			if(!marker.is_marked(d23))
				marker.mark(d23);
		}
	}

	template <typename FUNC>
	inline void foreach_dart_of_PHI2_PHI3(Dart d, const FUNC& f) const
	{
		Dart it = d;
		do
		{
			f(it);
			it = phi2(it);
			f(it);
			it = phi3(it);
		} while (it != d);
	}

	template <typename FUNC>
	inline void foreach_dart_of_PHI23(Dart d, const FUNC& f) const
	{
		Dart it = d;
		do
		{
			f(it);
			it = phi3(phi2(it));
		} while (it != d);
	}

	template <typename FUNC>
	inline void foreach_dart_of_PHI1_PHI3(Dart d, const FUNC& f) const
	{
		foreach_dart_of_PHI1(d, [&] (Dart fd)
		{
			f(fd);
			f(phi3(fd));
		});
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
			case Orbit::PHI1: foreach_dart_of_PHI1(c, f); break;
			case Orbit::PHI2: foreach_dart_of_PHI2(c, f); break;
			case Orbit::PHI1_PHI2: foreach_dart_of_PHI1_PHI2(c, f); break;
			case Orbit::PHI1_PHI3: foreach_dart_of_PHI1_PHI3(c, f); break;
			case Orbit::PHI2_PHI3: foreach_dart_of_PHI2_PHI3(c, f); break;
			case Orbit::PHI21: foreach_dart_of_PHI21(c, f); break;
			case Orbit::PHI21_PHI31: foreach_dart_of_PHI21_PHI31(c, f); break;
			default: cgogn_assert_not_reached("Cells of this dimension are not handled"); break;
		}
	}

};

template <typename MAP_TRAITS>
struct IHCMap3Type
{
	using TYPE = IHCMap3_T<MAP_TRAITS, IHCMap3Type<MAP_TRAITS>>;
};

template <typename MAP_TRAITS>
using IHCMap3 = IHCMap3_T<MAP_TRAITS, IHCMap3Type<MAP_TRAITS>>;

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(MULTIRESOLUTION_CPH_IHCMAP3_CPP_))
extern template class CGOGN_MULTIRESOLUTION_API IHCMap3_T<DefaultMapTraits, IHCMap3Type<DefaultMapTraits>>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(MULTIRESOLUTION_CPH_IHCMAP3_CPP_))

} // namespace cgogn


#endif // MULTIRESOLUTION_CPH_IHCMAP3_H_
