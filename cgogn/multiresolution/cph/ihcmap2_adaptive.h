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

#ifndef CGOGN_MULTIRESOLUTION_CPH_IHCMAP2_ADAPTIVE_H_
#define CGOGN_MULTIRESOLUTION_CPH_IHCMAP2_ADAPTIVE_H_

#include <cgogn/multiresolution/cph/ihcmap2.h>

namespace cgogn
{

template <typename MAP_TYPE>
class IHCMap2Adaptive_T : public IHCMap2_T<MAP_TYPE>
{
public:

	static const int PRIM_SIZE = 1;

	using MapType = MAP_TYPE;
	using Inherit = IHCMap2_T<MAP_TYPE>;
	using Self = IHCMap2Adaptive_T<MAP_TYPE>;

	friend class MapBase<MAP_TYPE>;

	using Vertex = typename Inherit::Vertex;
	using Edge = typename Inherit::Edge;
	using Face = typename Inherit::Face;
	using Volume = typename Inherit::Volume;

	IHCMap2Adaptive_T() : Inherit() {}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(IHCMap2Adaptive_T);

	~IHCMap2Adaptive_T() override {}

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
		Inherit::init_dart(d);
	}

	/***************************************************
	 *               CELLS INFORMATION                 *
	 ***************************************************/

public:

	/**
	 * Return the level of the edge of d in the current level map
	 * \details The level of an edge is the maximum of the levels of
	 * its darts. As phi1(d) and phi2(d) are from the same level we can
	 * optimize by checking phi1(d) instead of phi2(d)
	 */
	uint32 edge_level(Dart d)
	{
		cgogn_message_assert(Inherit::get_dart_level(d) <= Inherit::get_current_level(),
							 "Access to a dart introduced after current level");
		return std::max(Inherit::get_dart_level(d),Inherit::get_dart_level(Inherit::phi1(d)));
	}

	/**
	 * Return the level of the face of d in the current level map
	 * \details The level of a face is the minimum of the levels of its edges
	 * but a specific treatment has to be done in the particular case of a
	 * face with all neighboring faces are regularly subdivided
	 * but not the face itself
	 */
	uint32 face_level(Dart d)
	{
		cgogn_message_assert(Inherit::get_dart_level(d) <= Inherit::get_current_level(),
							 "Access to a dart introduced after current level");

		if(Inherit::get_current_level() == 0)
			return 0;

		Dart it = d;
		Dart old = it;
		uint32 l_old = Inherit::get_dart_level(old);
		uint32 fLevel = edge_level(it);
		do
		{
			it = Inherit::phi1(it);
			uint32 dl = Inherit::get_dart_level(it);

			// compute the oldest dart of the face in the same time
			if(dl < l_old)
			{
				old = it;
				l_old = dl;
			}
			uint32 l = edge_level(it);
			fLevel = l < fLevel ? l : fLevel;
		} while(it != d);

		uint32 cur = Inherit::get_current_level();
		Inherit::set_current_level(fLevel);

		uint32 nbSubd = 0;
		it = old;
		uint32 eId = Inherit::get_edge_id(old);
		do
		{
			++nbSubd;
			it = Inherit::phi1(it);
		} while(Inherit::get_edge_id(it) == eId);

		while(nbSubd > 1)
		{
			nbSubd /= 2;
			--fLevel;
		}

		Inherit::set_current_level(cur) ;
		return fLevel ;
	}

	/**
	 * Given the face of d in the current level map,
	 * return a level 0 dart of its origin face
	 */
	Dart face_origin(Dart d)
	{
		cgogn_message_assert(Inherit::get_dart_level(d) <= Inherit::get_current_level(),
							 "Access to a dart introduced after current level");

		uint32 cur = Inherit::get_current_level();
		Dart p = d;
		uint32 pLevel = Inherit::get_dart_level(p);
		while(pLevel > 0)
		{
			p = face_oldest_dart(p);
			pLevel = Inherit::get_dart_level(p);
			Inherit::set_current_level(pLevel);
		}
		Inherit::set_current_level(cur);
		return p;
	}

	/**
	 * Return the oldest dart of the face of d in the current level map
	 */
	Dart face_oldest_dart(Dart d)
	{
		cgogn_message_assert(Inherit::get_dart_level(d) <= Inherit::get_current_level(),
							 "Access to a dart introduced after current level");

		Dart it = d ;
		Dart oldest = it ;
		uint32 l_old = Inherit::get_dart_level(oldest);
		do
		{
			uint32 l = Inherit::get_dart_level(it);
			if(l == 0)
				return it;

			if(l < l_old)
				//		if(l < l_old || (l == l_old && it < oldest))
			{
				oldest = it;
				l_old = l;
			}
			it = Inherit::phi1(it);
		} while(it != d);

		return oldest;
	}

	/**
	 * Return true if the edge of d in the current level map
	 * has already been subdivided to the next level
	 * As before testing phi2(d) or phi1(d) is the same
	 */
	bool edge_is_subdivided(Dart d)
	{
		cgogn_message_assert(Inherit::get_dart_level(d) <= Inherit::get_current_level(),
							 "Access to a dart introduced after current level");

		if(Inherit::get_current_level() == Inherit::get_maximum_level())
			return false ;

		Dart d1 = Inherit::phi1(d);
		uint32 cur = Inherit::get_current_level();
		Inherit::set_current_level(cur + 1);
		Dart d1_l = Inherit::phi1(d);
		Inherit::set_current_level(cur);
		if(d1 != d1_l)
			return true;
		else
			return false;
	}

	/**
	 * Return true if the edge of d in the current level map
	 * is subdivided to the next level,
	 * none of its resulting edges is in turn subdivided to the next level
	 * and the middle vertex is of degree 2
	 */
	bool edge_can_be_coarsened(Dart d)
	{
		cgogn_message_assert(Inherit::get_dart_level(d) <= Inherit::get_current_level(),
							 "Access to a dart introduced after current level");

		bool subd = false ;
		bool subdOnce = true ;
		bool degree2 = false ;
		if(edge_is_subdivided(d))
		{
			subd = true ;
			Dart d2 = Inherit::phi2(d) ;
			uint32 cur = Inherit::get_current_level();
			Inherit::set_current_level(cur + 1);
			if(this->degree(typename Inherit::Vertex(Inherit::phi1(d))) == 2)
			{
				degree2 = true ;
				if(edge_is_subdivided(d) || edge_is_subdivided(d2))
					subdOnce = false ;
			}
			Inherit::set_current_level(cur);

		}
		return subd && degree2 && subdOnce ;
	}

	/**
	 * Return true if the face of d in the current level map
	 * has already been subdivided to the next level
	 */
	bool face_is_subdivided(Dart d)
	{
		cgogn_message_assert(Inherit::get_dart_level(d) <= Inherit::get_current_level(),
							 "Access to a dart introduced after current level");

		uint32 fLevel = face_level(d) ;
		if(fLevel <= Inherit::get_current_level())
			return false ;

		bool subd = false ;

		uint32 cur = Inherit::get_current_level();
		Inherit::set_current_level(cur + 1);
		if(Inherit::get_dart_level(Inherit::phi1(d)) == Inherit::get_current_level()
				&& Inherit::get_edge_id(Inherit::phi1(d)) != Inherit::get_edge_id(d))
			subd = true;
		Inherit::set_current_level(cur);

		return subd;
	}

	/**
	 * Return true if the face of d in the current level map
	 * is subdivided to the next level
	 * and none of its resulting faces is in turn subdivided to the next level
	 * \details
	 * A face whose level in the current level map is lower than the current
	 * level can not be subdivided to higher levels
	 */
	bool face_is_subdivided_once(Dart d)
	{
		cgogn_message_assert(Inherit::get_dart_level(d) <= Inherit::get_current_level(),
							 "Access to a dart introduced after current level");

		uint32 fLevel = face_level(d);
		if(fLevel < Inherit::get_current_level())
			return false;

		uint32 degree = 0 ;
		bool subd = false ;
		bool subdOnce = true ;
		Dart fit = d ;
		do
		{
			uint32 cur = Inherit::get_current_level();
			Inherit::set_current_level(cur + 1);
			if(Inherit::get_dart_level(Inherit::phi1(fit)) == Inherit::get_current_level()
					&& Inherit::get_edge_id(Inherit::phi1(fit)) != Inherit::get_edge_id(fit))
			{
				subd = true ;
				uint32 cur2 = Inherit::get_current_level();
				Inherit::set_current_level(cur2 + 1);
				if(Inherit::get_dart_level(Inherit::phi1(fit)) == Inherit::get_current_level()
						&& Inherit::get_edge_id(this->phi1(fit)) != Inherit::get_edge_id(fit))
					subdOnce = false ;
				Inherit::set_current_level(cur2);
			}
			Inherit::set_current_level(cur);
			++degree ;
			fit = Inherit::phi1(fit) ;
		} while(subd && subdOnce && fit != d) ;

		if(degree == 3 && subd)
		{
			uint32 cur = Inherit::get_current_level();
			Inherit::set_current_level(cur + 1);
			Dart cf = Inherit::phi2(Inherit::phi1(d)) ;
			uint32 cur2 = Inherit::get_current_level();
			Inherit::set_current_level(cur2 + 1);
			if(Inherit::get_dart_level(Inherit::phi1(cf)) == Inherit::get_current_level()
					&& Inherit::get_edge_id(Inherit::phi1(cf)) != Inherit::get_edge_id(cf))
				subdOnce = false ;
			Inherit::set_current_level(cur2);
			Inherit::set_current_level(cur);
		}

		return subd && subdOnce ;
	}

protected:
	inline Vertex cut_edge_update_emb(Dart /*e*/, Dart /*e2*/, Dart /*nd*/)
	{
		CGOGN_CHECK_CONCRETE_TYPE;
		cgogn_log_error("IHCMap2Adaptive_T::cut_edge_update_emb") << "Method is not implemented yet.";
		return Vertex();
	}

	inline void cut_face_update_emb(Dart /*e*/, Dart /*e2*/)
	{
		CGOGN_CHECK_CONCRETE_TYPE;
		cgogn_log_error("IHCMap2Adaptive_T::cut_face_update_emb") << "Method is not implemented yet.";
	}

	/***************************************************
	 *               SUBDIVISION                       *
	 ***************************************************/

	/**
	 * subdivide the edge of d to the next level
	 */
	void subdivide_edge(Dart d)
	{
		cgogn_message_assert(Inherit::get_dart_level(d) <= Inherit::get_current_level(),
							 "subdivideEdge : called with a dart inserted after current level") ;
		cgogn_message_assert(!edge_is_subdivided(d), "Trying to subdivide an already subdivided edge");

		uint32 eLevel = edge_level(d);

		uint32 cur = Inherit::get_current_level();
		Inherit::set_current_level(eLevel);

		Dart dd = Inherit::phi2(d);

		Inherit::set_current_level(eLevel + 1);

		this->cut_edge_topo(d);
		uint32 eId = Inherit::get_edge_id(d);
		Inherit::set_edge_id(Inherit::phi1(d), eId);
		Inherit::set_edge_id(Inherit::phi1(dd), eId);

		//		if(is_embedded<Vertex>())
		//		{
		//            (*edgeVertexFunctor)(Inherit::phi1(d));
		//		}

		//	    //quid des autres cellules ?
		//	    (*edgeEdgeFunctor)(Inherit::phi1(d));
		//		(*edgeEdgeFunctor)(d);


		Inherit::set_current_level(cur);
	}

	/**
	 * coarsen the edge of d from the next level
	 */
	void coarsen_edge(Dart d)
	{
		cgogn_message_assert(Inherit::get_dart_level(d) <= Inherit::get_current_level(),
							 "coarsenEdge : called with a dart inserted after current level");
		cgogn_message_assert(edge_can_be_coarsened(d), "Trying to coarsen an edge that can not be coarsened");


		uint32 cur = Inherit::get_current_level();
		//	Dart e = Inherit::phi2(d);
		Inherit::set_current_level(cur + 1);
		//	uint32 dl = Inherit::get_dart_level(e);
		//	Inherit::set_dart_level(Inherit::phi1(e), dl);
		//	Inherit::collapseEdge(e);
		this->merge_adjacent_edges_topo(d);
		Inherit::set_current_level(cur);
	}

public:
	/**
	 * subdivide the face of d to the next level
	 */
	uint32 subdivide_face(Dart d, bool triQuad = true, bool OneLevelDifference = true)
	{
		cgogn_message_assert(Inherit::get_dart_level(d) <= Inherit::get_current_level(),
							 "coarsenEdge : called with a dart inserted after current level");
		cgogn_message_assert(!face_is_subdivided(d), "Trying to coarsen an edge that can not be coarsened");

		uint32 fLevel = face_level(d);
		Dart old = face_oldest_dart(d);

		uint32 cur = Inherit::get_current_level();
		Inherit::set_current_level(fLevel);		// go to the level of the face to subdivide its edges

		uint32 degree = 0;
		Dart it = old;
		do
		{
			++degree;						// compute the degree of the face

			if(OneLevelDifference)
			{
				Dart nf = Inherit::phi2(it);
				if(face_level(nf) == fLevel - 1)	// check if neighboring faces have to be subdivided first
					subdivide_face(nf,triQuad);
			}

			if(!edge_is_subdivided(it))
				subdivide_edge(it);			// and cut the edges (if they are not already)
			it = Inherit::phi1(it);
		} while(it != old);

		Inherit::set_current_level(fLevel + 1);	// go to the next level to perform face subdivision

		if((degree == 3) && triQuad)					// if subdividing a triangle
		{
			Dart dd = Inherit::phi1(old);
			Dart e = Inherit::phi1(dd);
			//            (*vertexVertexFunctor)(e) ;

			e = Inherit::phi1(e);
			this->cut_face_topo(dd,e);

			uint32 id = Inherit::get_tri_refinement_edge_id(Inherit::phi_1(Inherit::phi_1(dd)), Inherit::phi1(Inherit::phi_1(dd)));
			Inherit::set_edge_id(Inherit::phi_1(dd), id);		// set the edge id of the inserted
			Inherit::set_edge_id(Inherit::phi_1(e), id);		// edge to the next available id

			dd = e ;
			e = Inherit::phi1(dd);
			//            (*vertexVertexFunctor)(e);
			e = Inherit::phi1(e);
			this->cut_face_topo(dd,e);
			id = Inherit::get_tri_refinement_edge_id(Inherit::phi_1(Inherit::phi_1(dd)), Inherit::phi1(Inherit::phi_1(dd)));
			Inherit::set_edge_id(Inherit::phi_1(dd), id);
			Inherit::set_edge_id(Inherit::phi_1(e), id);

			dd = e ;
			e = Inherit::phi1(dd);
			//            (*vertexVertexFunctor)(e);
			e = Inherit::phi1(e);
			this->cut_face_topo(dd,e);
			id = Inherit::get_tri_refinement_edge_id(Inherit::phi_1(Inherit::phi_1(dd)), Inherit::phi1(Inherit::phi_1(dd)));
			Inherit::set_edge_id(Inherit::phi_1(dd), id);
			Inherit::set_edge_id(Inherit::phi_1(e), id);
		}
		else											// if subdividing a polygonal face
		{
			Dart dd = Inherit::phi1(old);
			Dart next = this->phi1(dd);
			//            (*vertexVertexFunctor)(next);
			next = Inherit::phi1(next);
			this->cut_face_topo(dd,next); // insert a first edge
			Dart ne = Inherit::phi2(Inherit::phi_1(dd));
			Dart ne2 = Inherit::phi2(ne);
			this->cut_edge_topo(ne); // cut the new edge to insert the central vertex

			uint32 id = Inherit::get_quad_refinement_edge_id(Inherit::phi1(Inherit::phi2(ne)));
			Inherit::set_edge_id(ne, id);
			Inherit::set_edge_id(Inherit::phi2(ne), id);			// set the edge id of the inserted

			id = Inherit::get_quad_refinement_edge_id(Inherit::phi1(ne2));
			Inherit::set_edge_id(ne2, id);					// edges to the next available ids
			Inherit::set_edge_id(Inherit::phi2(ne2), id);


			dd = Inherit::phi1(next);
			//            (*vertexVertexFunctor)(dd);
			dd = Inherit::phi1(dd);
			while(dd != ne)								// turn around the face and insert new edges
			{											// linked to the central vertex
				this->cut_face_topo(Inherit::phi1(ne), dd);
				Dart nne = Inherit::phi2(Inherit::phi_1(dd));

				id = Inherit::get_quad_refinement_edge_id(Inherit::phi1(Inherit::phi2(nne)));
				Inherit::set_edge_id(nne, id) ;
				Inherit::set_edge_id(Inherit::phi2(nne), id);

				dd = Inherit::phi1(dd);
				//                (*vertexVertexFunctor)(dd);
				dd = Inherit::phi1(dd);
			}

			//            (*faceVertexFunctor)(Inherit::phi1(ne));
		}

		Inherit::set_current_level(cur);

		return fLevel ;
	}

	/**
	 * coarsen the face of d from the next level
	 */
	void coarsen_face(Dart d)
	{
		cgogn_message_assert(Inherit::get_dart_level(d) <= Inherit::get_current_level(),
							 "coarsenEdge : called with a dart inserted after current level");
		cgogn_message_assert(face_is_subdivided_once(d), "Trying to coarsen an edge that can not be coarsened");

		uint32 cur = Inherit::get_current_level();

		uint32 degree = 0;
		Dart fit = d;
		do
		{
			++degree;
			fit = Inherit::phi1(fit);
		} while(fit != d);

		if(degree == 3)
		{
			fit = d ;
			do
			{
				Inherit::set_current_level(cur + 1);
				Dart innerEdge = Inherit::phi1(fit);
				Inherit::set_current_level(Inherit::get_maximum_level());
				this->merge_incident_faces_topo(innerEdge);
				Inherit::set_current_level(cur);
				fit = this->phi1(fit);
			} while(fit != d);
		}
		else
		{
			Inherit::set_current_level(cur + 1);
			Dart centralV = Inherit::phi1(Inherit::phi1(d));
			Inherit::set_current_level(Inherit::get_maximum_level());
// TODO			this->delete_vertex(centralV);
			Inherit::set_current_level(cur);
		}

		fit = d ;
		do
		{
			if(edge_can_be_coarsened(fit))
				coarsen_edge(fit);
			fit = Inherit::phi1(fit);
		} while(fit != d);
	}
};

struct IHCMap2AdaptiveType
{
	using TYPE = IHCMap2Adaptive_T<IHCMap2AdaptiveType>;
};

using IHCMap2Adaptive = IHCMap2Adaptive_T<IHCMap2AdaptiveType>;

} // namespace cgogn

#endif // CGOGN_MULTIRESOLUTION_CPH_IHCMAP2_ADAPTIVE_H_
