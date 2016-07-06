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

#ifndef CGOGN_CORE_CMAP_CMAP3_TETRA_H_
#define CGOGN_CORE_CMAP_CMAP3_TETRA_H_

#include <cgogn/core/cmap/map_base.h>

namespace cgogn
{

// forward declaration of CMap3TetraBuilder_T
template <typename MAP_TRAITS> class CMap3TetraBuilder_T;

template <typename MAP_TRAITS, typename MAP_TYPE>
class CMap3Tetra_T : public MapBase<MAP_TRAITS, MAP_TYPE>
{
public:
	static const uint8 DIMENSION = 3;
	static const uint8 PRIM_SIZE = 12;

	using MapTraits = MAP_TRAITS;
	using MapType = MAP_TYPE;
	using Inherit = MapBase<MAP_TRAITS, MAP_TYPE>;
	using Self = CMap3Tetra_T<MAP_TRAITS, MAP_TYPE>;

	using Builder = CMap3TetraBuilder_T<MapTraits>;


	friend class MapBase<MAP_TRAITS, MAP_TYPE>;
	friend class CMap3TetraBuilder_T<MapTraits>;
	friend class DartMarker_T<Self>;
	friend class cgogn::DartMarkerStore<Self>;

	using CDart		= Cell<Orbit::DART>;
	using Vertex2	= Cell<Orbit::PHI21>;
	using Vertex	= Cell<Orbit::PHI21_PHI31>;
	using Edge2		= Cell<Orbit::PHI2>;
	using Edge		= Cell<Orbit::PHI2_PHI3>;
	using Face2		= Cell<Orbit::PHI1>;
	using Face		= Cell<Orbit::PHI1_PHI3>;
	using Volume	= Cell<Orbit::PHI1_PHI2>;


	using Boundary  = Face;
	using ConnectedComponent = Cell<Orbit::PHI1_PHI2_PHI3>;

	template <typename T>
	using ChunkArray =  typename Inherit::template ChunkArray<T>;
	template <typename T>
	using ChunkArrayContainer =  typename Inherit::template ChunkArrayContainer<T>;

	template <typename T, Orbit ORBIT>
	using Attribute = typename Inherit::template Attribute<T, ORBIT>;
	template <typename T>
	using VertexAttribute = Attribute<T, Vertex::ORBIT>;
	template <typename T>
	using EdgeAttribute = Attribute<T, Edge::ORBIT>;
	template <typename T>
	using FaceAttribute = Attribute<T, Face::ORBIT>;
	template <typename T>
	using VolumeAttribute = Attribute<T, Volume::ORBIT>;

	using DartMarker = typename cgogn::DartMarker<Self>;
	using DartMarkerStore = typename cgogn::DartMarkerStore<Self>;

	template <Orbit ORBIT>
	using CellMarker = typename cgogn::CellMarker<Self, ORBIT>;

protected:

	ChunkArray<Dart>* phi3_;

	inline void init()
	{
		phi3_ = this->topology_.template add_chunk_array<Dart>("phi3");
	}

public:

	CMap3Tetra_T() : Inherit()
	{
		init();
	}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CMap3Tetra_T);

	~CMap3Tetra_T() override
	{}

	/*!
	 * \brief Check the integrity of embedding information
	 */
	inline bool check_embedding_integrity()
	{
		bool result = true;

		if (this->template is_embedded<CDart>())
			result = result && this->template is_well_embedded<CDart>();

		if (this->template is_embedded<Vertex2>())
			result = result && this->template is_well_embedded<Vertex2>();

		if (this->template is_embedded<Vertex>())
			result = result && this->template is_well_embedded<Vertex>();

		if (this->template is_embedded<Edge2>())
			result = result && this->template is_well_embedded<Edge2>();

		if (this->template is_embedded<Edge>())
			result = result && this->template is_well_embedded<Edge>();

		if (this->template is_embedded<Face2>())
			result = result && this->template is_well_embedded<Face2>();

		if (this->template is_embedded<Face>())
			result = result && this->template is_well_embedded<Face>();

		if (this->template is_embedded<Volume>())
			result = result && this->template is_well_embedded<Volume>();

		return result;
	}

	/*******************************************************************************
	 * Low-level topological operations
	 *******************************************************************************/

protected:

	/**
	 * \brief Init newly added dart.
	 * The dart is defined as a fixed point for PHI2.
	 */
	inline void init_dart(Dart d)
	{
		(*phi3_)[d.index] = d;
	}

	/**
	 * @brief Check the integrity of a dart
	 * @param d the dart to check
	 * @return true if the integrity constraints are locally statisfied
	 * PHI2 should be an involution without fixed point
	 */
	inline bool check_integrity(Dart d) const
	{
		bool integrity2d = (phi1(phi_1(d)) == d &&
				phi_1(phi1(d)) == d &&
				phi2(phi2(d)) == d &&
				phi2(d) != d);
		return integrity2d &&
				phi3(phi3(d)) == d &&
				phi3(d) != d &&
				phi3(this->phi1(phi3(this->phi1(d)))) == d;

	}

	/**
	 * @brief check boundary integrity
	 * @return always true (boundary could is not set of separated faces)
	 */
	inline bool check_boundary_integrity(Dart) const
	{
		return true;
	}

	/**
	 * \brief Link dart d with dart e by the phi2 involution
	 * @param d,e the darts to link
	 *	- Before: d->d and e->e
	 *	- After:  d->e and e->d
	 */
	inline void phi3_sew(Dart d, Dart e)
	{
		cgogn_assert(phi3(d) == d);
		cgogn_assert(phi3(e) == e);
		(*phi3_)[d.index] = e;
		(*phi3_)[e.index] = d;
	}

	/**
	 * \brief Remove the phi3 link between the current dart and its linked dart
	 * @param d the dart to unlink
	 * - Before: d->e and e->d
	 * - After:  d->d and e->e
	 */
	inline void phi3_unsew(Dart d)
	{
		Dart e = phi3(d);
		(*phi3_)[d.index] = d;
		(*phi3_)[e.index] = e;
	}

	/*******************************************************************************
	 * Basic topological operations
	 *******************************************************************************/

public:

	/*!
	 * \brief phi1
	 * @param d
	 * @return phi1(d)
	 */
	inline Dart phi1(Dart d) const
	{
		switch(d.index%3)
		{
		case 0: return Dart(d.index+1);break;
		case 1: return Dart(d.index+1);break;
		}
		return Dart(d.index-2);
	}

	/*!
	 * \brief phi_1
	 * @param d
	 * @return phi_1(d)
	 */
	Dart phi_1(Dart d) const
	{
		switch(d.index%3)
		{
		case 2: return Dart(d.index-1);break;
		case 1: return Dart(d.index-1);break;
		}
		return Dart(d.index+2);
	}

	/*!
	 * \brief phi11
	 * @param d
	 * @return phi1(phi1(d))
	 */
	inline Dart phi11(Dart d) const
	{
		return phi_1(d);
	}

	/*!
	 * \brief phi1
	 * @param d
	 * @return phi1(d)
	 */
	inline Dart phi2(Dart d) const
	{
		return Dart(d.index + MapGen::tetra_phi2[d.index%12]);
	}

	/**
	 * \brief phi3
	 * @param d
	 * @return phi3(d)
	 */
	inline Dart phi3(Dart d) const
	{
		return (*phi3_)[d.index];
	}


	/**
	 * \brief Composition of PHI calls
	 * @param d
	 * @return The result of successive applications of PHI1 and PHI2 on d.
	 * The template parameter contains a sequence (Base10 encoded) of PHI indeices.
	 * If N=0 the identity is used.
	 */
	template <uint64 N>
	inline Dart phi(Dart d) const
	{
		static_assert((N % 10) <= 3, "Composition of PHI: invalid index");

		if (N%100 == 11)
			return phi11(phi< N/100 >(d));

		switch(N % 10)
		{
			case 1 : return phi1(phi<N / 10>(d)) ;
			case 2 : return phi2(phi<N / 10>(d)) ;
			case 3 : return phi3(phi<N / 10>(d)) ;
			default : return d ;
		}
	}

	/*******************************************************************************
	 * High-level embedded and topological operations
	 *******************************************************************************/

protected:

	/**
	 * @brief Add a tetrahedron with fixed point phi3
	 * @return
	 */
	inline Dart add_tetra_topo_fp()
	{
		Dart d = this->add_topology_element(); // in fact insert PRIM_SIZE darts
		// no need to set phi1 & phi2
		return d;
	}

	/**
	 * @brief remove a tetrahedron (12 darts)
	 * @param d
	 */
	inline void remove_tetra_topo_fp(Dart d)
	{
		this->remove_topology_element(d); // in fact remove PRIM_SIZE darts
	}

	/**
	 * \brief Add a tetra in the map
	 * \return A dart of the built tetra.
	 * Two 2-tetra ( phi3-f.p.) are built. The first one is the returned volume,
	 * the second is a boundary vol that closes the map.
	 */
	Dart add_tetra_topo()
	{
		Dart d = add_tetra_topo_fp();
		Dart e = add_tetra_topo_fp();

		uint32 di = d.index;
		uint32 ei = e.index;

		phi3_sew(Dart(di),Dart(ei + 0)); ++di;
		phi3_sew(Dart(di),Dart(ei + 2)); ++di;
		phi3_sew(Dart(di),Dart(ei + 1)); ++di;
		phi3_sew(Dart(di),Dart(ei + 3)); ++di;
		phi3_sew(Dart(di),Dart(ei + 5)); ++di;
		phi3_sew(Dart(di),Dart(ei + 4)); ++di;
		phi3_sew(Dart(di),Dart(ei + 9)); ++di;
		phi3_sew(Dart(di),Dart(ei +11)); ++di;
		phi3_sew(Dart(di),Dart(ei +10)); ++di;
		phi3_sew(Dart(di),Dart(ei + 6)); ++di;
		phi3_sew(Dart(di),Dart(ei + 8)); ++di;
		phi3_sew(Dart(di),Dart(ei + 7)); ++di;

		for (uint32 k=0; k<12; ++k)
			this->set_boundary(Dart(ei++), true);

		return d;
	}

public:

	/**
	 * \brief Add a tetra in the map.
	 * \return A dart of the built tetra.
	 * Two 2-tetra ( phi3-f.p.) are built. The first one is the returned volume,
	 * the second is a boundary vol that closes the map.
	 */
	Volume add_tetra()
	{
		Volume vol(add_tetra_topo());

		if (this->template is_embedded<CDart>())
		{
			foreach_dart_of_orbit(vol, [this] (Dart d)
			{
				this->new_orbit_embedding(CDart(d));
			});
		}

		if (this->template is_embedded<Vertex>())
		{
			foreach_incident_vertex(vol, [this] (Vertex v)
			{
				this->new_orbit_embedding(v);
			});
		}

		if (this->template is_embedded<Edge>())
		{
			foreach_incident_edge(vol, [this] (Edge e)
			{
				this->new_orbit_embedding(e);
			});
		}

		if (this->template is_embedded<Face>())
		{
			foreach_incident_face(vol, [this] (Face f)
			{
				this->new_orbit_embedding(f);
			});
		}

		if (this->template is_embedded<Volume>())
			this->new_orbit_embedding(vol);

		return vol;
	}


public:

	inline uint32 degree(Vertex2 v) const
	{
		return this->nb_darts_of_orbit(v);
	}

	inline uint32 degree(Vertex v) const
	{
		uint32 result = 0;
		foreach_incident_edge(v, [&result] (Edge) { ++result; });
		return result;
	}

	inline uint32 codegree(Edge2) const
	{
		return 2;
	}

	inline uint32 degree(Edge2 e) const
	{
		if (this->is_boundary(e.dart) || this->is_boundary(phi2(e.dart)))
			return 1;
		else
			return 2;
	}

	inline uint32 codegree(Edge) const
	{
		return 2;
	}

	inline uint32 degree(Edge e) const
	{
		uint32 result = 0;
		foreach_incident_face(e, [&result] (Face) { ++result; });
		return result;
	}

	inline uint32 codegree(Face2) const
	{
		return 3;
	}

	inline uint32 degree(Face2) const
	{
		return 1;
	}

	inline bool has_codegree(Face2, uint32 codegree) const
	{
		return codegree == 3;
	}


	inline uint32 codegree(Face) const
	{
		return 3;
	}

	inline bool has_codegree(Face, uint32 codegree) const
	{
		return codegree == 3;
	}

	inline uint32 degree(Face f) const
	{
		if (this->is_boundary(f.dart) || this->is_boundary(phi3(f.dart)))
			return 1;
		else
			return 2;
	}

	inline uint32 codegree(Volume) const
	{
		return 4;
	}

	/*******************************************************************************
	 * Boundary information
	 *******************************************************************************/

	bool is_adjacent_to_boundary(Boundary c)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		bool result = false;
		foreach_dart_of_orbit_until(c, [this, &result] (Dart d)
		{
			if (this->is_boundary(phi3(d))) { result = true; return false; }
			return true;
		});
		return result;
	}

	/*******************************************************************************
	 * Orbits traversal
	 *******************************************************************************/

protected:

	template <typename FUNC>
	inline void foreach_dart_of_PHI1(Dart d, const FUNC& f) const
	{
		uint32 first = (d.index/3)*3;
		f(Dart(first++));
		f(Dart(first++));
		f(Dart(first));
	}

	template <typename FUNC>
	inline void foreach_dart_of_PHI2(Dart d, const FUNC& f) const
	{
		f(d);
		f(phi2(d));
	}

	template <typename FUNC>
	inline void foreach_dart_of_PHI21(Dart d, const FUNC& f) const
	{
		uint32 first = (d.index/12)*12;
		switch(d.index - first)
		{
			case 0:
			case 4:
			case 9:
				f(Dart(first+0));
				f(Dart(first+4));
				f(Dart(first+9));
				break;
			case 1:
			case 3:
			case 7:
				f(Dart(first+1));
				f(Dart(first+3));
				f(Dart(first+7));
				break;
			case 2:
			case 6:
			case 10:
				f(Dart(first+2));
				f(Dart(first+6));
				f(Dart(first+10));
				break;
			case 5:
			case 8:
			case 11:
				f(Dart(first+5));
				f(Dart(first+8));
				f(Dart(first+11));
				break;
			default:
				break;
		}

//		Dart it = d;
//		f(it);
//		it = phi2(phi_1(it));
//		f(it);
//		it = phi2(phi_1(it));
//		f(it);

	}

	template <typename FUNC>
	void foreach_dart_of_PHI1_PHI2(Dart d, const FUNC& f) const
	{
		uint32 first = (d.index/12)*12;
		f(Dart(first++));
		f(Dart(first++));
		f(Dart(first++));
		f(Dart(first++));
		f(Dart(first++));
		f(Dart(first++));
		f(Dart(first++));
		f(Dart(first++));
		f(Dart(first++));
		f(Dart(first++));
		f(Dart(first++));
		f(Dart(first++));
	}

	// foreach with phi3

	template <typename FUNC>
	inline void foreach_dart_of_PHI21_PHI31(Dart d, const FUNC& f) const
	{
		DartMarkerStore marker(*this);
		const std::vector<Dart>* marked_darts = marker.marked_darts();

		marker.mark(d);
		for(uint32 i = 0; i < marked_darts->size(); ++i)
		{
			const Dart curr_dart = marked_darts->operator[](i);
			if ( !(this->is_boundary(curr_dart) && this->is_boundary(phi3(curr_dart))) )
				f(curr_dart);

			const Dart d_1 = this->phi_1(curr_dart);
			const Dart d2_1 = this->phi2(d_1); // turn in volume
			const Dart d3_1 = phi3(d_1); // change volume

			if(!marker.is_marked(d2_1))
				marker.mark(d2_1);
			if(!marker.is_marked(d3_1))
				marker.mark(d3_1);
		}
	}

	template <typename FUNC>
	inline void foreach_dart_of_PHI2_PHI3(Dart d, const FUNC& f) const
	{
		Dart it = d;
		do
		{
			if ( !(this->is_boundary(it) && this->is_boundary(phi3(it))) )
				f(it);
			it = this->phi2(it);
			if ( !(this->is_boundary(it) && this->is_boundary(phi3(it))) )
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
			if ( !(this->is_boundary(it) && this->is_boundary(phi3(it))) )
				f(it);
			it = phi3(phi2(it));
		} while (it != d);
	}

	template <typename FUNC>
	inline void foreach_dart_of_PHI1_PHI3(Dart d, const FUNC& f) const
	{
		this->foreach_dart_of_PHI1(d, [&] (Dart fd)
		{
			f(fd);
			f(phi3(fd));
		});
	}

	template <typename FUNC>
	void foreach_dart_of_PHI1_PHI2_PHI3(Dart d, const FUNC& f) const
	{
		DartMarkerStore marker(*this);

		std::vector<Dart>* visited_vols = cgogn::dart_buffers()->buffer();
		visited_vols->push_back(d); // Start with the face of d

		// For every vol added to the list
		for(uint32 i = 0; i < visited_vols->size(); ++i)
		{
			uint32 first = (((*visited_vols)[i]).index/12)*12;
			Dart dv = Dart(first);

			if (!marker.is_marked(dv)) // is volum not already marked
			{
				for(int nf=0; nf<4; ++nf) // foreach face2
				{
					// for dart of face
					dv = Dart(first++);
					f(dv);
					marker.mark(dv);
					// for second dart of face
					dv = Dart(first++);
					f(dv);
					marker.mark(dv);
					// for third dart of face
					dv = Dart(first++);
					f(dv);
					marker.mark(dv);
					// mark neighbour volume for visiting
					Dart ev = phi3(dv);
					if (!marker.is_marked(ev))
						visited_vols->push_back(ev);
				}
			}
		}
		cgogn::dart_buffers()->release_buffer(visited_vols);
	}




	template <Orbit ORBIT, typename FUNC>
	inline void foreach_dart_of_orbit(Cell<ORBIT> c, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Dart), "Wrong function parameter type");

		static_assert(ORBIT == Orbit::DART || ORBIT == Orbit::PHI1 || ORBIT == Orbit::PHI2 ||
					  ORBIT == Orbit::PHI1_PHI2 || ORBIT == Orbit::PHI21 ||
					  ORBIT == Orbit::PHI1_PHI3 || ORBIT == Orbit::PHI2_PHI3 ||
					  ORBIT == Orbit::PHI21_PHI31 || ORBIT == Orbit::PHI1_PHI2_PHI3,
					  "Orbit not supported in a CMap3Tetra");

		switch (ORBIT)
		{
			case Orbit::DART: f(c.dart); break;
			case Orbit::PHI1: foreach_dart_of_PHI1(c.dart, f); break;
			case Orbit::PHI2: foreach_dart_of_PHI2(c.dart, f); break;
			case Orbit::PHI1_PHI2: foreach_dart_of_PHI1_PHI2(c.dart, f); break;
			case Orbit::PHI21: foreach_dart_of_PHI21(c.dart, f); break;
			case Orbit::PHI2_PHI3:foreach_dart_of_PHI2_PHI3(c.dart, f); break;
			case Orbit::PHI1_PHI3:foreach_dart_of_PHI1_PHI3(c.dart, f); break;
			case Orbit::PHI21_PHI31:foreach_dart_of_PHI21_PHI31(c.dart, f); break;
			case Orbit::PHI1_PHI2_PHI3:foreach_dart_of_PHI1_PHI2_PHI3(c.dart, f); break;
			default: cgogn_assert_not_reached("Orbit not supported in a CMap3Tetra"); break;
		}
	}



	template <typename FUNC>
	inline void foreach_dart_of_PHI1_until(Dart d, const FUNC& f) const
	{

		uint32 first = (d.index/3)*3;
		if (!f(Dart(first++))) return;
		if (!f(Dart(first++))) return;
		if (!f(Dart(first))) return;
	}

	template <typename FUNC>
	inline void foreach_dart_of_PHI2_until(Dart d, const FUNC& f) const
	{
		if (f(d))
			f(phi2(d));
	}

	template <typename FUNC>
	inline void foreach_dart_of_PHI21_until(Dart d, const FUNC& f) const
	{
		uint32 first = (d.index/12)*12;
		switch(d.index - first)
		{
			case 0:
			case 4:
			case 9:
				if (!f(Dart(first+0))) return;
				if (!f(Dart(first+4))) return;
				if (!f(Dart(first+9))) return;
				break;
			case 1:
			case 3:
			case 7:
				if (!f(Dart(first+1))) return;
				if (!f(Dart(first+3))) return;
				if (!f(Dart(first+7))) return;
				break;
			case 2:
			case 6:
			case 10:
				if (!f(Dart(first+2))) return;
				if (!f(Dart(first+6))) return;
				if (!f(Dart(first+10))) return;
				break;
			case 5:
			case 8:
			case 11:
				if (!f(Dart(first+5))) return;
				if (!f(Dart(first+8))) return;
				if (!f(Dart(first+11))) return;
				break;
			default:
				break;
		}
	}

	template <typename FUNC>
	void foreach_dart_of_PHI1_PHI2_until(Dart d, const FUNC& f) const
	{
		uint32 first = (d.index/12)*12;
		if (!f(Dart(first++))) return;
		if (!f(Dart(first++))) return;
		if (!f(Dart(first++))) return;
		if (!f(Dart(first++))) return;
		if (!f(Dart(first++))) return;
		if (!f(Dart(first++))) return;
		if (!f(Dart(first++))) return;
		if (!f(Dart(first++))) return;
		if (!f(Dart(first++))) return;
		if (!f(Dart(first++))) return;
		if (!f(Dart(first++))) return;
		if (!f(Dart(first))) return;
	}


	// foreach_until with phi3

	template <typename FUNC>
	inline void foreach_dart_of_PHI21_PHI31_until(Dart d, const FUNC& f) const
	{
		DartMarkerStore marker(*this);
		const std::vector<Dart>* marked_darts = marker.marked_darts();

		marker.mark(d);
		for(uint32 i = 0; i < marked_darts->size(); ++i)
		{
			const Dart curr_dart = marked_darts->operator[](i);

			if ( !(this->is_boundary(curr_dart) && this->is_boundary(phi3(curr_dart))) )
				if (!f(curr_dart))
					break;

			const Dart d_1 = this->phi_1(curr_dart);
			const Dart d2_1 = this->phi2(d_1); // turn in volume
			const Dart d3_1 = phi3(d_1); // change volume

			if(!marker.is_marked(d2_1))
				marker.mark(d2_1);
			if(!marker.is_marked(d3_1))
				marker.mark(d3_1);
		}
	}

	template <typename FUNC>
	inline void foreach_dart_of_PHI2_PHI3_until(Dart d, const FUNC& f) const
	{
		Dart it = d;
		do
		{
			if ( !(this->is_boundary(it) && this->is_boundary(phi3(it))) )
				if (!f(it))
					break;
			it = this->phi2(it);

			if ( !(this->is_boundary(it) && this->is_boundary(phi3(it))) )
				if (!f(it))
					break;
			it = phi3(it);
		} while (it != d);
	}

	template <typename FUNC>
	inline void foreach_dart_of_PHI23_until(Dart d, const FUNC& f) const
	{
		Dart it = d;
		do
		{
			if ( !(this->is_boundary(it) && this->is_boundary(phi3(it))) )
				if (!f(it))
					break;
			it = phi3(this->phi2(it));
		} while (it != d);
	}

	template <typename FUNC>
	inline void foreach_dart_of_PHI1_PHI3_until(Dart d, const FUNC& f) const
	{
		this->foreach_dart_of_PHI1_until(d, [&] (Dart fd) -> bool
		{
			if (f(fd))
				return f(phi3(fd));
			return false;
		});
	}

	template <typename FUNC>
	void foreach_dart_of_PHI1_PHI2_PHI3_until(Dart d, const FUNC& f) const
	{
		DartMarkerStore marker(*this);

		std::vector<Dart>* visited_vols = cgogn::dart_buffers()->buffer();
		visited_vols->push_back(d); // Start with the face of d

		// For every vol added to the list
		for(uint32 i = 0; i < visited_vols->size(); ++i)
		{
			uint32 first = (((*visited_vols)[i]).index/12)*12;
			Dart dv = Dart(first);

			if (!marker.is_marked(dv)) // is volum not already marked
			{
				for(int nf=0; nf<4; ++nf) // foreach face2
				{
					// for dart of face
					dv = Dart(first++);
					if (!f(dv)) break;
					marker.mark(dv);
					// for second dart of face
					dv = Dart(first++);
					if (!f(dv)) break;
					marker.mark(dv);
					// for third dart of face
					dv = Dart(first++);
					if (!f(dv)) break;
					marker.mark(dv);
					// mark neighbour volume for visiting
					Dart ev = phi3(dv);
					if (!marker.is_marked(ev))
						visited_vols->push_back(ev);
				}
			}
		}
		cgogn::dart_buffers()->release_buffer(visited_vols);
	}




	template <Orbit ORBIT, typename FUNC>
	inline void foreach_dart_of_orbit_until(Cell<ORBIT> c, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Dart), "Wrong function parameter type");
		static_assert(check_func_return_type(FUNC, bool), "Wrong function return type");

		static_assert(ORBIT == Orbit::DART || ORBIT == Orbit::PHI1 || ORBIT == Orbit::PHI2 ||
					  ORBIT == Orbit::PHI1_PHI2 || ORBIT == Orbit::PHI21 ||
					  ORBIT == Orbit::PHI1_PHI3 || ORBIT == Orbit::PHI2_PHI3 ||
					  ORBIT == Orbit::PHI21_PHI31 || ORBIT == Orbit::PHI1_PHI2_PHI3,
					  "Orbit not supported in a CMap3Tetra");

		switch (ORBIT)
		{
			case Orbit::DART: f(c.dart); break;
			case Orbit::PHI1: foreach_dart_of_PHI1_until(c.dart, f); break;
			case Orbit::PHI2: foreach_dart_of_PHI2_until(c.dart, f); break;
			case Orbit::PHI1_PHI2: foreach_dart_of_PHI1_PHI2_until(c.dart, f); break;
			case Orbit::PHI21: foreach_dart_of_PHI21_until(c.dart, f); break;
			case Orbit::PHI2_PHI3:foreach_dart_of_PHI2_PHI3_until(c.dart, f); break;
			case Orbit::PHI1_PHI3:foreach_dart_of_PHI1_PHI3_until(c.dart, f); break;
			case Orbit::PHI21_PHI31:foreach_dart_of_PHI21_PHI31_until(c.dart, f); break;
			case Orbit::PHI1_PHI2_PHI3:foreach_dart_of_PHI1_PHI2_PHI3_until(c.dart, f); break;
			default: cgogn_assert_not_reached("Orbit not supported in a CMap3Tetra"); break;
		}
	}




	/*******************************************************************************
	 * Incidence traversal (of dim 2)
	 *******************************************************************************/

public:

	template <typename FUNC>
	inline void foreach_incident_edge(Vertex2 v, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Edge2), "Wrong function cell parameter type");
		foreach_dart_of_orbit(v, [&] (Dart d)
		{
			func(Edge2(d));
		});
	}

	template <typename FUNC>
	inline void foreach_incident_face(Vertex2 v, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Face2), "Wrong function cell parameter type");
		foreach_dart_of_orbit(v, [this, &func] (Dart d)
		{
			func(Face2(d));
		});
	}

	template <typename FUNC>
	inline void foreach_incident_volume(Vertex2 v, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Volume), "Wrong function cell parameter type");
		func(Volume(v.dart));
	}

	template <typename FUNC>
	inline void foreach_incident_vertex(Edge2 e, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Vertex2), "Wrong function cell parameter type");
		foreach_dart_of_orbit(e, [&func] (Dart d)
		{
			func(Vertex2(d));
		});
	}

	template <typename FUNC>
	inline void foreach_incident_face(Edge2 e, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Face2), "Wrong function cell parameter type");
		foreach_dart_of_orbit(e, [this, &func] (Dart d)
		{
			func(Face2(d));
		});
	}

	template <typename FUNC>
	inline void foreach_incident_volume(Edge2 e, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Volume), "Wrong function cell parameter type");
		func(Volume(e.dart));
	}

	template <typename FUNC>
	inline void foreach_incident_vertex(Face2 f, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Vertex2), "Wrong function cell parameter type");
		foreach_dart_of_orbit(f, [&func] (Dart d)
		{
			func(Vertex2(d));
		});
	}

	template <typename FUNC>
	inline void foreach_incident_edge(Face2 f, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Edge2), "Wrong function cell parameter type");
		foreach_dart_of_orbit(f, [&func] (Dart d)
		{
			func(Edge2(d));
		});
	}

	template <typename FUNC>
	inline void foreach_incident_volume(Face2 f, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Volume), "Wrong function cell parameter type");
		func(Volume(f.dart));
	}

	template <typename FUNC>
	inline void foreach_incident_vertex2(Volume w, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Vertex2), "Wrong function cell parameter type");
		uint32 first = (w.dart.index/12)*12;
		func(Vertex2(Dart(first)));
		func(Vertex2(Dart(first+1)));
		func(Vertex2(Dart(first+2)));
		func(Vertex2(Dart(first+5)));
	}

	template <typename FUNC>
	inline void foreach_incident_edge2(Volume w, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Edge2), "Wrong function cell parameter type");
		uint32 first = (w.dart.index/12)*12;
		func(Edge2(Dart(first)));
		func(Edge2(Dart(first+1)));
		func(Edge2(Dart(first+2)));
		func(Edge2(Dart(first+4)));
		func(Edge2(Dart(first+7)));
		func(Edge2(Dart(first+10)));
	}

	template <typename FUNC>
	inline void foreach_incident_face2(Volume w, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Face2), "Wrong function cell parameter type");
		uint32 first = (w.dart.index/12)*12;
		func(Face2(Dart(first)));// 0
		first += 3;
		func(Face2(Dart(first)));// 3
		first += 3;
		func(Face2(Dart(first)));// 6
		first += 3;
		func(Face2(Dart(first))); // 9
	}

	/*******************************************************************************
	 * Adjacence traversal (of dim 2)
	 *******************************************************************************/

	template <typename FUNC>
	inline void foreach_adjacent_vertex_through_edge(Vertex2 v, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Vertex2), "Wrong function cell parameter type");
		foreach_dart_of_orbit(v, [this, &f] (Dart d)
		{
			f(Vertex2(phi2(d)));
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_vertex_through_face(Vertex2 v, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Vertex2), "Wrong function cell parameter type");
		foreach_dart_of_orbit(v, [this, &f] (Dart vd)
		{
			Dart vd1 = phi1(vd);
			foreach_dart_of_orbit(Face2(vd), [&f, vd, vd1] (Dart fd)
			{
				// skip Vertex v itself and its first successor around current face
				if (fd != vd && fd != vd1)
					f(Vertex2(fd));
			});
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_edge_through_vertex(Edge2 e, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Edge2), "Wrong function cell parameter type");
		foreach_dart_of_orbit(e, [&f, this] (Dart ed)
		{
			foreach_dart_of_orbit(Vertex2(ed), [&, ed] (Dart vd)
			{
					// skip Edge e itself
					if (vd != ed)
						f(Edge2(vd));
			});
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_edge_through_face(Edge2 e, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Edge2), "Wrong function cell parameter type");
		foreach_dart_of_orbit(e, [&f, this] (Dart ed)
		{
			if (!this->is_boundary(ed))
			{
				foreach_dart_of_orbit(Face2(ed), [&f, ed] (Dart fd)
				{
					// skip Edge e itself
					if (fd != ed)
						f(Edge2(fd));
				});
			}
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_face_through_vertex(Face2 f, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Face2), "Wrong function cell parameter type");
		foreach_dart_of_orbit(f, [this, &func] (Dart fd)
		{
			Dart fd1 = phi2(phi_1(fd));
			foreach_dart_of_orbit(Vertex2(fd), [this, &func, fd, fd1] (Dart vd)
			{
				// skip Face f itself and its first successor around current vertex
				if (vd != fd && vd != fd1)
					func(Face2(vd));
			});
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_face_through_edge(Face2 f, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Face2), "Wrong function cell parameter type");
		foreach_dart_of_orbit(f, [this, &func] (Dart d)
		{
			const Dart d2 = phi2(d);
			func(Face(d2));
		});
	}


	/*******************************************************************************
	 * Incidence traversal
	 *******************************************************************************/

	template <typename FUNC>
	inline void foreach_incident_edge(Vertex v, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Edge), "Wrong function cell parameter type");
		DartMarkerStore marker(*this);
		foreach_dart_of_orbit(v, [&] (Dart d)
		{
			if (!marker.is_marked(d))
			{
				foreach_dart_of_PHI23(d, [&marker] (Dart dd) { marker.mark(dd); });
				func(Edge(d));
			}
		});
	}

	template <typename FUNC>
	inline void foreach_incident_face(Vertex v, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Face), "Wrong function cell parameter type");
		DartMarkerStore marker(*this);
		foreach_dart_of_orbit(v, [&] (Dart d)
		{
			if (!marker.is_marked(d))
			{
				marker.mark(d);
				marker.mark(this->phi1(phi3(d)));
				func(Face(d));
			}
		});
	}

	template <typename FUNC>
	inline void foreach_incident_volume(Vertex v, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Volume), "Wrong function cell parameter type");
		DartMarkerStore marker(*this);
		foreach_dart_of_orbit(v, [&] (Dart d)
		{
			if (!marker.is_marked(d) && !this->is_boundary(d))
			{
				marker.mark_orbit(Vertex2(d));
				func(Volume(d));
			}
		});
	}

	template <typename FUNC>
	inline void foreach_incident_vertex(Edge e, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Vertex), "Wrong function cell parameter type");
		f(Vertex(e.dart));
		f(Vertex(this->phi2(e.dart)));
	}

	template <typename FUNC>
	inline void foreach_incident_face(Edge e, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Face), "Wrong function cell parameter type");
		foreach_dart_of_PHI23(e.dart, [&func] (Dart d) { func(Face(d)); });
	}

	template <typename FUNC>
	inline void foreach_incident_volume(Edge e, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Volume), "Wrong function cell parameter type");
		foreach_dart_of_PHI23(e.dart, [this, &func] (Dart d)
		{
			if (!this->is_boundary(d))
				func(Volume(d));
		});
	}

	template <typename FUNC>
	inline void foreach_incident_vertex(Face f, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Vertex), "Wrong function cell parameter type");
		foreach_dart_of_orbit(Face2(f.dart), [&func] (Dart v) { func(Vertex(v)); });
	}


	template <typename FUNC>
	inline void foreach_incident_edge(Face f, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Edge), "Wrong function cell parameter type");
		foreach_dart_of_orbit(Face2(f.dart), [&func] (Dart e) { func(Edge(e)); });
	}

	template <typename FUNC>
	inline void foreach_incident_volume(Face f, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Volume), "Wrong function cell parameter type");
		if (!this->is_boundary(f.dart))
			func(Volume(f.dart));
		const Dart d3 = phi3(f.dart);
		if (!this->is_boundary(d3))
			func(Volume(d3));
	}

	template <typename FUNC>
	inline void foreach_incident_vertex(Volume v, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Vertex), "Wrong function cell parameter type");
		foreach_incident_vertex2(v, [&func] (Vertex2 ve)
		{
			func(Vertex(ve.dart));
		});
	}

	template <typename FUNC>
	inline void foreach_incident_edge(Volume v, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Edge), "Wrong function cell parameter type");
		foreach_incident_edge2(v, [&func] (Edge2 e)
		{
			func(Edge(e.dart));
		});
	}

	template <typename FUNC>
	inline void foreach_incident_face(Volume v, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Face), "Wrong function cell parameter type");

		foreach_incident_face2(v, [&func] (Face2 f)
		{
			func(Face(f.dart));
		});


//		DartMarkerStore marker(*this);
//		foreach_dart_of_orbit(v, [&] (Dart d)
//		{
//			if (!marker.is_marked(d))
//			{
//				marker.mark_orbit(Face2(d));
//				func(Face(d));
//			}
//		});
	}



	/*******************************************************************************
	 * Adjacence traversal (dim 3)
	 *******************************************************************************/

	template <typename FUNC>
	inline void foreach_adjacent_vertex_through_edge(Vertex v, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Vertex), "Wrong function cell parameter type");
		foreach_incident_edge(v, [&] (Edge e)
		{
			func(Vertex(this->phi2(e.dart)));
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_vertex_through_face(Vertex v, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Vertex), "Wrong function cell parameter type");
		DartMarker marker_vertex(*this);
		marker_vertex.mark_orbit(v);
		foreach_incident_face(v, [&] (Face inc_face)
		{
			foreach_incident_vertex(inc_face, [&] (Vertex vertex_of_face)
			{
				if (!marker_vertex.is_marked(vertex_of_face.dart))
				{
					marker_vertex.mark_orbit(vertex_of_face);
					func(vertex_of_face);
				}
			});
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_vertex_through_volume(Vertex v, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Vertex), "Wrong function cell parameter type");
		DartMarker marker_vertex(*this);
		marker_vertex.mark_orbit(v);
		foreach_incident_volume(v, [&] (Volume inc_vol)
		{
			foreach_incident_vertex(inc_vol, [&] (Vertex inc_vert)
			{
				if (!marker_vertex.is_marked(inc_vert.dart))
				{
					marker_vertex.mark_orbit(inc_vert);
					func(inc_vert);
				}
			});
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_edge_through_vertex(Edge e, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Edge), "Wrong function cell parameter type");
		foreach_incident_vertex(e, [&] (Vertex iv)
		{
			foreach_incident_edge(iv, [&] (Edge ie)
			{
				if (ie.dart != iv.dart)
					func(ie);
			});
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_edge_through_face(Edge e, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Edge), "Wrong function cell parameter type");
		DartMarker marker_edge(*this);
		marker_edge.mark_orbit(e);
		foreach_incident_face(e, [&] (Face inc_face)
		{
			foreach_incident_edge(inc_face, [&] (Edge inc_edge)
			{
				if (!marker_edge.is_marked(inc_edge.dart))
				{
					marker_edge.mark_orbit(inc_edge);
					func(inc_edge);
				}
			});
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_edge_through_volume(Edge e, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Edge), "Wrong function cell parameter type");
		DartMarker marker_edge(*this);
		marker_edge.mark_orbit(e);
		foreach_incident_volume(e, [&] (Volume inc_vol)
		{
			foreach_incident_edge(inc_vol, [&] (Edge inc_edge)
			{
				if (!marker_edge.is_marked(inc_edge.dart))
				{
					marker_edge.mark_orbit(inc_edge);
					func(inc_edge);
				}
			});
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_face_through_vertex(Face f, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Face), "Wrong function cell parameter type");
		DartMarker marker_face(*this);
		marker_face.mark_orbit(f);
		foreach_incident_vertex(f, [&] (Vertex v)
		{
			foreach_incident_face(f, [&](Face inc_fac)
			{
				if (!marker_face.is_marked(inc_fac.dart))
				{
					marker_face.mark_orbit(inc_fac);
					func(inc_fac);
				}
			});
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_face_through_edge(Face f, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Face), "Wrong function cell parameter type");
		foreach_incident_edge(f, [&] (Edge ie)
		{
			foreach_incident_face(ie, [&] (Face iface)
			{
				if (iface.dart != ie.dart)
					func(iface);
			});
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_face_through_volume(Face f, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Face), "Wrong function cell parameter type");
		DartMarker marker_face(*this);
		marker_face.mark_orbit(f);
		if (!this->is_boundary(f.dart))
		{
			foreach_incident_face(Volume(f.dart), [&] (Face inc_face)
			{
				if (!marker_face.is_marked(inc_face.dart))
				{
					marker_face.mark_orbit((inc_face));
					func(inc_face);
				}
			});
		}
		const Dart d3 = phi3(f.dart);
		if (!this->is_boundary(d3))
		{
			foreach_incident_face(Volume(d3), [&] (Face inc_face)
			{
				if (!marker_face.is_marked(inc_face.dart))
				{
					marker_face.mark_orbit((inc_face));
					func(inc_face);
				}
			});
		}
	}

	template <typename FUNC>
	inline void foreach_adjacent_volume_through_vertex(Volume v, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Volume), "Wrong function cell parameter type");
		DartMarker marker_volume(*this);
		marker_volume.mark_orbit(v);
		foreach_incident_vertex(v, [&] (Vertex inc_vert)
		{
			foreach_incident_volume(inc_vert, [&](Volume inc_vol)
			{
				if (!marker_volume.is_marked(inc_vol.dart) && !this->is_boundary(inc_vol.dart))
				{
					marker_volume.mark_orbit(inc_vol);
					func(inc_vol);
				}
			});
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_volume_through_edge(Volume v, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Volume), "Wrong function cell parameter type");
		DartMarker marker_volume(*this);
		marker_volume.mark_orbit(v);
		foreach_incident_edge(v, [&] (Edge inc_edge)
		{
			foreach_incident_volume(inc_edge, [&] (Volume inc_vol)
			{
				if (!marker_volume.is_marked(inc_vol.dart) && !this->is_boundary(inc_vol.dart))
				{
					marker_volume.mark_orbit(inc_vol);
					func(inc_vol);
				}
			});
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_volume_through_face(Volume v, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Volume), "Wrong function cell parameter type");
		DartMarker marker_volume(*this);
		marker_volume.mark_orbit(v);
		foreach_incident_face(v, [&] (Edge inc_face)
		{
			foreach_incident_volume(inc_face, [&] (Volume inc_vol)
			{
				if (!marker_volume.is_marked(inc_vol.dart) && !this->is_boundary(inc_vol.dart))
				{
					marker_volume.mark_orbit(inc_vol);
					func(inc_vol);
				}
			});
		});
	}



	inline std::pair<Vertex, Vertex> vertices(Edge e)
	{
		return std::pair<Vertex, Vertex>(Vertex(e.dart), Vertex(this->phi1(e.dart)));
	}

	inline std::pair<Vertex2, Vertex2> vertices(Edge2 e)
	{
		return std::pair<Vertex2, Vertex2>(Vertex2(e.dart), Vertex2(this->phi1(e.dart)));
	}



protected:

#define FOR_ALL_ORBITS( CODE) {\
{static const Orbit orbit_const=DART; CODE }\
{static const Orbit orbit_const=PHI1; CODE }\
{static const Orbit orbit_const=PHI2; CODE }\
{static const Orbit orbit_const=PHI1_PHI2; CODE }\
{static const Orbit orbit_const=PHI21; CODE }}


	/**
	 * @brief check if embedding of map is also embedded in this (create if not). Used by merge method
	 * @param map
	 */
	void merge_check_embedding(const Self& map)
	{
		FOR_ALL_ORBITS
		(
			if (!this->template is_embedded<orbit_const>() && map.template is_embedded<orbit_const>())
				this->template create_embedding<orbit_const>();
		)
	}


	/**
	 * @brief ensure all cells (introduced while merging) are embedded.
	 * @param first index of first dart to scan
	 */
	void merge_finish_embedding(uint32 first)
	{
		FOR_ALL_ORBITS
		(
			if (this->template is_embedded<orbit_const>())
			{
				for (uint32 j=first; j!= this->topology_.end(); this->topology_.next(j))
				{
					if ( ((orbit_const != Boundary::ORBIT) && (orbit_const != DART)) || (!this->is_boundary(Dart(j))))
						if ((*this->embeddings_[orbit_const])[j] == INVALID_INDEX)
						{
							this->new_orbit_embedding(Cell<orbit_const>(Dart(j)));
						}
				}
			}
		)
	}

#undef FOR_ALL_ORBITS

};

template <typename MAP_TRAITS>
struct CMap3TetraType
{
	using TYPE = CMap3Tetra_T<MAP_TRAITS, CMap3TetraType<MAP_TRAITS>>;
};

template <typename MAP_TRAITS>
using CMap3Tetra = CMap3Tetra_T<MAP_TRAITS, CMap3TetraType<MAP_TRAITS>>;

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_CMAP_CMAP3_TETRA_CPP_))
extern template class CGOGN_CORE_API CMap3Tetra_T<DefaultMapTraits, CMap3TetraType<DefaultMapTraits>>;
extern template class CGOGN_CORE_API DartMarker<CMap3Tetra<DefaultMapTraits>>;
extern template class CGOGN_CORE_API DartMarkerStore<CMap3Tetra<DefaultMapTraits>>;
extern template class CGOGN_CORE_API DartMarkerNoUnmark<CMap3Tetra<DefaultMapTraits>>;
extern template class CGOGN_CORE_API CellMarker<CMap3Tetra<DefaultMapTraits>, CMap3Tetra<DefaultMapTraits>::Vertex::ORBIT>;
extern template class CGOGN_CORE_API CellMarker<CMap3Tetra<DefaultMapTraits>, CMap3Tetra<DefaultMapTraits>::Edge::ORBIT>;
extern template class CGOGN_CORE_API CellMarker<CMap3Tetra<DefaultMapTraits>, CMap3Tetra<DefaultMapTraits>::Face::ORBIT>;
extern template class CGOGN_CORE_API CellMarker<CMap3Tetra<DefaultMapTraits>, CMap3Tetra<DefaultMapTraits>::Volume::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap3Tetra<DefaultMapTraits>, CMap3Tetra<DefaultMapTraits>::Vertex::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap3Tetra<DefaultMapTraits>, CMap3Tetra<DefaultMapTraits>::Edge::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap3Tetra<DefaultMapTraits>, CMap3Tetra<DefaultMapTraits>::Face::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap3Tetra<DefaultMapTraits>, CMap3Tetra<DefaultMapTraits>::Volume::ORBIT>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_MAP_MAP2_CPP_))

} // namespace cgogn

#include <cgogn/core/cmap/cmap3_tetra_builder.h>

#endif // CGOGN_CORE_CMAP_CMAP3_TETRA_H_
