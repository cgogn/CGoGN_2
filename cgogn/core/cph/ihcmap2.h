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


#include <core/cmap/cmap2.h>
// #include <core/cph/attribute_handler_cph.h>

namespace cgogn
{

<<<<<<< HEAD
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
class IHCMap2_T : public CMap2_T<DATA_TRAITS, MAP_TYPE>
{
public:

	typedef CMap2_T<DATA_TRAITS, MAP_TYPE> Inherit;
	typedef IHCMap2_T<DATA_TRAITS, MAP_TYPE> Self;

	friend typename Self::Inherit;
	friend typename Inherit::Inherit;


	friend class DartMarker_T<Self>;

	static const Orbit DART = Orbit::DART;
	static const Orbit VERTEX = Orbit::PHI21;
	static const Orbit EDGE   = Orbit::PHI2;
	static const Orbit FACE   = Orbit::PHI1;
	static const Orbit VOLUME = Orbit::PHI1_PHI2;

	typedef Cell<Self::VERTEX> Vertex;
	typedef Cell<Self::EDGE> Edge;
	typedef Cell<Self::FACE> Face;
	typedef Cell<Self::VOLUME> Volume;

	template <typename T>
	using ChunkArray =  typename Inherit::template ChunkArray<T>;
    template <typename T>
    using ChunkArrayContainer =  typename Inherit::template ChunkArrayContainer<T>;

	template<typename T, Orbit ORBIT>
	using AttributeHandler = typename Inherit::template AttributeHandler<T, ORBIT>;
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

protected:
	unsigned int current_level_;
	unsigned int maximum_level_;
	unsigned int id_count_;

	// DartAttributeHandler<unsigned int> dart_level_ ;
	// DartAttributeHandler<unsigned int> edge_id_ ;
    ChunkArray<unsigned int>* dart_level_;
    ChunkArray<unsigned int>* edge_id_;

    ContainerCPHBrowser<ChunkArrayContainer<unsigned char>, Self>* cph_browser;

	std::vector<unsigned int> nb_darts_per_level;

	inline void init()
	{
        dart_level_ = this->topology_.template add_attribute<unsigned int>("dartLevel") ;
        edge_id_ = this->topology_.template add_attribute<unsigned int>("edgeId");

        cph_browser = new ContainerCPHBrowser<ChunkArrayContainer<unsigned char>, Self>(this->topology_, this);

        this->topology_.set_current_browser(cph_browser);
	}

public:
    IHCMap2_T() : Inherit(),
        current_level_(0),
        maximum_level_(0),
        id_count_(0)
	{
		init();        
	}

	~IHCMap2_T() override
	{
        this->topology_.template remove_attribute(dart_level_);
        this->topology_.template remove_attribute(edge_id_);
	}

	IHCMap2_T(Self const&) = delete;
	IHCMap2_T(Self &&) = delete;
	Self& operator=(Self const&) = delete;
	Self& operator=(Self &&) = delete;

	/*******************************************************************************
	 * Basic topological operations
	 *******************************************************************************/

	inline Dart phi1(Dart d) const
	{
        cgogn_message_assert(get_dart_level(d) <= get_current_level(), "Access to a dart introduced after current level") ;
	
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
        cgogn_message_assert(get_dart_level(d) <= get_current_level(), "Access to a dart introduced after current level") ;

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
		cgogn_message_assert(get_dart_level(d) <= get_current_level(), "Access to a dart introduced after current level") ;

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

		set_edge_id(d, 0);
		set_dart_level(d, get_current_level());

		// update max level if needed
		if(get_current_level() > get_maximum_level()) 
		{
			set_maximum_level(get_current_level()); 
			nb_darts_per_level.push_back(0);
		}

//		inc_nb_darts(get_current_level());

		
		return d ;
	}

	inline Vertex cut_edge(Edge e)
	{
		Dart dd = phi2(e);
		unsigned int e_id = get_edge_id(e) ;

		Vertex v = Inherit::cut_edge(e);
		
		set_edge_id(phi1(e), e_id) ;
		set_edge_id(phi1(dd), e_id) ;
		
		return v;
	}

	inline void split_face(Face d, Face e)
	{
		Inherit::split_face(d, e);

		foreach_dart_of_orbit(d, [this, e_id] (Dart dit) { set_edge_id(dit, e_id); });
		foreach_dart_of_orbit(e, [this, e_id] (Dart dit) { set_edge_id(dit, e_id); });
	}

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
            it = Inherit::phi2(Inherit::phi_1(it));
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
		cgogn_message_assert(get_current_level() < get_maximum_level(), "incCurrentLevel : already at maximum resolution level");
		++current_level_ ;
	}

	inline void dec_current_level()
    {
		cgogn_message_assert(get_current_level() > 0u, "decCurrentLevel : already at minimum resolution level");
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
        return (*dart_level_)[d.index] ;
	}

	inline void set_dart_level(Dart d, unsigned int l)
	{
        (*dart_level_)[d.index] = l ;
	}

	/***************************************************
	 *             EDGE ID MANAGEMENT                  *
	 ***************************************************/

	/**
	 * Return the next available edge id
	 */
	inline unsigned int get_new_edge_id()
	{
		return id_count_++ ;
	}

	inline unsigned int get_edge_id(Dart d) const
	{
        return (*edge_id_)[d.index] ;
	}

	inline void set_edge_id(Dart d, unsigned int i)
	{
        (*edge_id_)[d.index] = i ;
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

	inline void inc_nb_darts(unsigned int level)
	{
		nb_darts_per_level[level]++;
	}
};

template <typename MAP_TRAITS>
struct IHCMap2Type
{
	typedef IHCMap2_T<MAP_TRAITS, IHCMap2Type<MAP_TRAITS>> TYPE;
};

template <typename MAP_TRAITS>
using IHCMap2 = IHCMap2_T<MAP_TRAITS, IHCMap2Type<MAP_TRAITS>>;

} // namespace cgogn

#endif // CORE_CPH_IHCMAP2_H_

