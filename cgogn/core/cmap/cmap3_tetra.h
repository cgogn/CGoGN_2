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
#include <cgogn/core/cmap/cmap3_builder.h>

namespace cgogn
{

template <typename MAP_TYPE>
class CMap3Tetra_T : public MapBase<MAP_TYPE>
{
public:

	static const uint8 DIMENSION = 3;
	static const uint8 PRIM_SIZE = 12;

	using MapType = MAP_TYPE;
	using Inherit = MapBase<MAP_TYPE>;
	using Self = CMap3Tetra_T<MAP_TYPE>;

	using Builder = CMap3Builder_T<Self>;

	friend class MapBase<MAP_TYPE>;
	friend class CMap3Builder_T<Self>;
	friend class DartMarker_T<Self>;
	friend class cgogn::DartMarkerStore<Self>;

	using CDart   = Cell<Orbit::DART>;
	using Vertex2 = Cell<Orbit::PHI21>;
	using Vertex  = Cell<Orbit::PHI21_PHI31>;
	using Edge2   = Cell<Orbit::PHI2>;
	using Edge    = Cell<Orbit::PHI2_PHI3>;
	using Face2   = Cell<Orbit::PHI1>;
	using Face    = Cell<Orbit::PHI1_PHI3>;
	using Volume  = Cell<Orbit::PHI1_PHI2>;

	using Boundary = Volume;
	using ConnectedComponent = Cell<Orbit::PHI1_PHI2_PHI3>;

	template <typename T>
	using ChunkArrayContainer = typename Inherit::template ChunkArrayContainer<T>;
	using typename Inherit::ChunkArrayGen;
	template <typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

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
	template <Orbit ORBIT>
	using CellMarkerNoUnmark = typename cgogn::CellMarkerNoUnmark<Self, ORBIT>;

	using CellCache = typename cgogn::CellCache<Self>;
	using QuickTraversor = typename cgogn::QuickTraversor<Self>;
	using BoundaryCache = typename cgogn::BoundaryCache<Self>;

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
		switch (d.index%3)
		{
			case 0: return Dart(d.index+1); break;
			case 1: return Dart(d.index+1); break;
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
		switch (d.index%3)
		{
			case 2: return Dart(d.index-1); break;
			case 1: return Dart(d.index-1); break;
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
		return Dart(d.index + MapBaseData::tetra_phi2[d.index%12u]);
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

	inline Dart add_pyramid_topo_fp(std::size_t size)
	{
		cgogn_message_assert(size == 3u, "Can create only tetra");
		if (size != 3)
		{
			cgogn_log_warning("add_pyramid_topo_fp") << "Attempt to create a volume which is not a tetrahedron in CMap3Tetra";
			return Dart();
		}
		return add_tetra_topo_fp();
	}

	inline Dart add_prism_topo_fp(std::size_t size)
	{
		unused_parameters(size);
		cgogn_message_assert(false, "Can create only tetra");
		cgogn_log_warning("add_prism_topo_fp") << "Attempt to create a volume which is not a tetrahedron in CMap3Tetra";
		return Dart();
	}

	inline Dart add_stamp_volume_topo_fp()
	{
		cgogn_message_assert(false, "Can create only tetra");
		cgogn_log_warning("add_stamp_volume_topo_fp") << "Attempt to create a volume which is not a tetrahedron in CMap3Tetra";
		return Dart();
	}

	/**
	 * @brief sew two volumes along a face
	 * The darts given in the Volume parameters must be part of Face2 that have
	 * a similar co-degree and whose darts are all phi3 fix points
	 * @param v1 first volume
	 * @param v2 second volume
	 */
	inline void sew_volumes_fp(Dart v1, Dart v2)
	{
		cgogn_message_assert(phi3(v1) == v1 &&
							 phi3(v2) == v2 &&
							 codegree(Face(v1)) == codegree(Face(v1)) &&
							 !this->same_orbit(Face2(v1), Face2(v2)), "CMap3Builder sew_volumes: preconditions not respected");

		Dart it1 = v1;
		Dart it2 = v2;
		const Dart begin = it1;
		do
		{
			phi3_sew(it1, it2);
			it1 = this->phi1(it1);
			it2 = this->phi_1(it2);
		} while (it1 != begin);
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

protected:

	/**
	 * @brief close_hole_topo closes the topological hole that contains Dart d (a fixed point of phi3 relation)
	 * @param d a dart incident to the hole
	 * @return a dart of the volume that closes the hole
	 */
	inline Dart close_hole_topo(Dart d, bool mark_boundary = false)
	{
		cgogn_message_assert(phi3(d) == d, "CMap3Tetra: close hole called on a dart that is not a phi3 fix point");

		DartMarkerStore fmarker(*this);
		DartMarkerStore boundary_marker(*this);

		std::vector<Dart> visited_faces; // Faces that are traversed
		visited_faces.reserve(1024u);

		visited_faces.push_back(d);		// Start with the face of d
		fmarker.mark_orbit(Face2(d));

		auto local_func = [&] (Dart f)
		{
			Dart e = phi3(this->phi2(f));
			bool found = false;
			do
			{
				if (phi3(e) == e)
				{
					found = true;
					if (!fmarker.is_marked(e))
					{
						visited_faces.push_back(e);
						fmarker.mark_orbit(Face2(e));
					}
				}
				else
				{
					if (boundary_marker.is_marked(e))
					{
						found = true;
						sew_volumes_fp(phi<32>(f), this->phi2(e));
					}
					else
						e = phi3(this->phi2(e));
				}
			} while (!found);
		};

		// For every face added to the list
		for(uint32 i = 0u; i < visited_faces.size(); ++i)
		{
			Dart f = visited_faces[i];

			const Dart tb = add_tetra_topo_fp();
			boundary_marker.mark_orbit(Volume(tb));
			sew_volumes_fp(tb, f);

			local_func(f);
			f = this->phi1(f);
			local_func(f);
			f = this->phi1(f);
			local_func(f);
		}

		if (mark_boundary)
		{
			for (Dart db: boundary_marker.marked_darts())
				this->set_boundary(db, true);
		}

		return phi3(d);
	}

	/**
	 * @brief close_map closes the map removing topological holes (only for import/creation)
	 * Add volumes to the map that close every existing hole.
	 * @return the number of closed holes
	 */
	inline uint32 close_map()
	{
		uint32 nb_holes = 0u;

		// Search the map for topological holes (fix points of phi3)
		std::vector<Dart>* fix_point_darts = dart_buffers()->buffer();
		this->foreach_dart([&] (Dart d)
		{
			if (phi3(d) == d)
				fix_point_darts->push_back(d);
		});

		for (Dart d : (*fix_point_darts))
		{
			if (phi3(d) == d)
			{
				close_hole_topo(d, true);
				++nb_holes;
			}
		}

		if (this->template is_embedded<Vertex>())
		{
			for (Dart d : (*fix_point_darts))
			{
				Dart e = phi3(d);
				this->template copy_embedding<Vertex>(this->phi2(e), d);
				e = this->phi1(e);
				this->template copy_embedding<Vertex>(e, d);
				this->template copy_embedding<Vertex>(phi<21>(e), d);
			}
		}

		if (this->template is_embedded<Edge>())
		{
			for (Dart d : (*fix_point_darts))
			{
				Dart e = phi3(d);
				this->template copy_embedding<Edge>(e, d);
				this->template copy_embedding<Edge>(this->phi2(e), d);
			}
		}

		if (this->template is_embedded<Face>())
		{
			for (Dart d : (*fix_point_darts))
				this->template copy_embedding<Face>(phi3(d), d);
		}

		dart_buffers()->release_buffer(fix_point_darts);

		return nb_holes;
	}

	/*******************************************************************************
	 * Connectivity information
	 *******************************************************************************/

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
		foreach_dart_of_orbit(c, [this, &result] (Dart d) -> bool
		{
			if (this->is_boundary(phi3(d))) { result = true; return false; }
			return true;
		});
		return result;
	}

	/*******************************************************************************
	 * Orbits traversal
	 *******************************************************************************/

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_dart_of_orbit(Cell<ORBIT> c, const FUNC& f) const
	{
		static_assert(is_func_parameter_same<FUNC, Dart>::value, "Wrong function parameter type");

		static_assert(ORBIT == Orbit::DART || ORBIT == Orbit::PHI1 || ORBIT == Orbit::PHI2 ||
					  ORBIT == Orbit::PHI21 || ORBIT == Orbit::PHI1_PHI2 ||
					  ORBIT == Orbit::PHI1_PHI3 || ORBIT == Orbit::PHI2_PHI3 ||
					  ORBIT == Orbit::PHI21_PHI31 || ORBIT == Orbit::PHI1_PHI2_PHI3,
					  "Orbit not supported in a CMap3Tetra");

		switch (ORBIT)
		{
			case Orbit::DART: f(c.dart); break;
			case Orbit::PHI1: foreach_dart_of_PHI1(c.dart, f); break;
			case Orbit::PHI2: foreach_dart_of_PHI2(c.dart, f); break;
			case Orbit::PHI21: foreach_dart_of_PHI21(c.dart, f); break;
			case Orbit::PHI1_PHI2: foreach_dart_of_PHI1_PHI2(c.dart, f); break;
			case Orbit::PHI2_PHI3:foreach_dart_of_PHI2_PHI3(c.dart, f); break;
			case Orbit::PHI1_PHI3:foreach_dart_of_PHI1_PHI3(c.dart, f); break;
			case Orbit::PHI21_PHI31:foreach_dart_of_PHI21_PHI31(c.dart, f); break;
			case Orbit::PHI1_PHI2_PHI3:foreach_dart_of_PHI1_PHI2_PHI3(c.dart, f); break;
			default: cgogn_assert_not_reached("Orbit not supported in a CMap3Tetra"); break;
		}
	}

protected:

	template <typename FUNC>
	inline void foreach_dart_of_PHI1(Dart d, const FUNC& f) const
	{
		uint32 first = (d.index/3)*3;
		if (!internal::void_to_true_binder(f, Dart(first++))) return;
		if (internal::void_to_true_binder(f, Dart(first++)))
			f(Dart(first));
	}

	template <typename FUNC>
	inline void foreach_dart_of_PHI2(Dart d, const FUNC& f) const
	{
		if (internal::void_to_true_binder(f, d))
			f(phi2(d));
	}

	template <typename FUNC>
	inline void foreach_dart_of_PHI21(Dart d, const FUNC& f) const
	{
		const uint32 first = (d.index/12)*12;
		switch(d.index - first)
		{
			case 0:
			case 4:
			case 9:
				if (!internal::void_to_true_binder(f, Dart(first+0))) return;
				if (internal::void_to_true_binder(f, Dart(first+4)))
					f(Dart(first+9));
				break;
			case 1:
			case 3:
			case 7:
				if (!internal::void_to_true_binder(f, Dart(first+1))) return;
				if (internal::void_to_true_binder(f, Dart(first+3)))
					f(Dart(first+7));
				break;
			case 2:
			case 6:
			case 10:
				if (!internal::void_to_true_binder(f, Dart(first+2))) return;
				if (internal::void_to_true_binder(f, Dart(first+6)))
					f(Dart(first+10));
				break;
			case 5:
			case 8:
			case 11:
				if (!internal::void_to_true_binder(f, Dart(first+5))) return;
				if (internal::void_to_true_binder(f, Dart(first+8)))
					f(Dart(first+11));
				break;
			default:
				break;
		}
	}

	template <typename FUNC>
	void foreach_dart_of_PHI1_PHI2(Dart d, const FUNC& f) const
	{
		uint32 first = (d.index/12)*12;
		if (!internal::void_to_true_binder(f, Dart(first++))) return;
		if (!internal::void_to_true_binder(f, Dart(first++))) return;
		if (!internal::void_to_true_binder(f, Dart(first++))) return;
		if (!internal::void_to_true_binder(f, Dart(first++))) return;
		if (!internal::void_to_true_binder(f, Dart(first++))) return;
		if (!internal::void_to_true_binder(f, Dart(first++))) return;
		if (!internal::void_to_true_binder(f, Dart(first++))) return;
		if (!internal::void_to_true_binder(f, Dart(first++))) return;
		if (!internal::void_to_true_binder(f, Dart(first++))) return;
		if (!internal::void_to_true_binder(f, Dart(first++))) return;
		if (internal::void_to_true_binder(f, Dart(first++)))
			f(Dart(first));
	}

	// foreach with phi3

	template <typename FUNC>
	inline void foreach_dart_of_PHI21_PHI31(Dart d, const FUNC& f) const
	{
		DartMarkerStore marker(*this);
		const std::vector<Dart>& marked_darts = marker.marked_darts();

		marker.mark(d);
		for(uint32 i = 0; i < marked_darts.size(); ++i)
		{
			const Dart curr_dart = marked_darts[i];

			if ( !(this->is_boundary(curr_dart) && this->is_boundary(phi3(curr_dart))) )
				if (!internal::void_to_true_binder(f, curr_dart))
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
	inline void foreach_dart_of_PHI2_PHI3(Dart d, const FUNC& f) const
	{
		Dart it = d;
		do
		{
			if ( !(this->is_boundary(it) && this->is_boundary(phi3(it))) )
				if (!internal::void_to_true_binder(f, it))
					break;
			it = this->phi2(it);

			if ( !(this->is_boundary(it) && this->is_boundary(phi3(it))) )
				if (!internal::void_to_true_binder(f, it))
					break;
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
				if (!internal::void_to_true_binder(f, it))
					break;
			it = phi3(this->phi2(it));
		} while (it != d);
	}

	template <typename FUNC>
	inline void foreach_dart_of_PHI1_PHI3(Dart d, const FUNC& f) const
	{
		this->foreach_dart_of_PHI1(d, [&] (Dart fd) -> bool
		{
			if (internal::void_to_true_binder(f, fd))
				return internal::void_to_true_binder(f, phi3(fd));
			return false;
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
					if (!internal::void_to_true_binder(f, dv)) break;
					marker.mark(dv);
					// for second dart of face
					dv = Dart(first++);
					if (!internal::void_to_true_binder(f, dv)) break;
					marker.mark(dv);
					// for third dart of face
					dv = Dart(first++);
					if (!internal::void_to_true_binder(f, dv)) break;
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

	/*******************************************************************************
	 * Incidence traversal (of dim 2)
	 *******************************************************************************/

public:

	template <typename FUNC>
	inline void foreach_incident_edge(Vertex2 v, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Edge2>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(v, [&] (Dart d) -> bool
		{
			return internal::void_to_true_binder(func, Edge2(d));
		});
	}

	template <typename FUNC>
	inline void foreach_incident_face(Vertex2 v, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Face2>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(v, [this, &func] (Dart d) -> bool
		{
			return internal::void_to_true_binder(func, Face2(d));
		});
	}

	template <typename FUNC>
	inline void foreach_incident_volume(Vertex2 v, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Volume>::value, "Wrong function cell parameter type");
		func(Volume(v.dart));
	}

	template <typename FUNC>
	inline void foreach_incident_vertex(Edge2 e, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Vertex2>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(e, [&func] (Dart d) -> bool
		{
			return internal::void_to_true_binder(func, Vertex2(d));
		});
	}

	template <typename FUNC>
	inline void foreach_incident_face(Edge2 e, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Face2>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(e, [this, &func] (Dart d) -> bool
		{
			return internal::void_to_true_binder(func, Face2(d));
		});
	}

	template <typename FUNC>
	inline void foreach_incident_volume(Edge2 e, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Volume>::value, "Wrong function cell parameter type");
		func(Volume(e.dart));
	}

	template <typename FUNC>
	inline void foreach_incident_vertex(Face2 f, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Vertex2>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(f, [&func] (Dart d) -> bool
		{
			return internal::void_to_true_binder(func, Vertex2(d));
		});
	}

	template <typename FUNC>
	inline void foreach_incident_edge(Face2 f, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Edge2>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(f, [&func] (Dart d) -> bool
		{
			return internal::void_to_true_binder(func, Edge2(d));
		});
	}

	template <typename FUNC>
	inline void foreach_incident_volume(Face2 f, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Volume>::value, "Wrong function cell parameter type");
		func(Volume(f.dart));
	}

	template <typename FUNC>
	inline void foreach_incident_vertex2(Volume w, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Vertex2>::value, "Wrong function cell parameter type");
		const uint32 first = (w.dart.index/12)*12;
		if (!internal::void_to_true_binder(func, Vertex2(Dart(first))))
				return;
		if (!internal::void_to_true_binder(func, Vertex2(Dart(first+1))))
				return;
		if (internal::void_to_true_binder(func, Vertex2(Dart(first+2))))
			func(Vertex2(Dart(first+5)));
	}

	template <typename FUNC>
	inline void foreach_incident_edge2(Volume w, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Edge2>::value, "Wrong function cell parameter type");
		const uint32 first = (w.dart.index/12)*12;
		if (!internal::void_to_true_binder(func, Edge2(Dart(first))))
			return;
		if (!internal::void_to_true_binder(func, Edge2(Dart(first+1))))
			return;
		if (!internal::void_to_true_binder(func, Edge2(Dart(first+2))))
			return;
		if (!internal::void_to_true_binder(func, Edge2(Dart(first+4))))
			return;
		if (internal::void_to_true_binder(func, Edge2(Dart(first+7))))
			func(Edge2(Dart(first+10)));
	}

	template <typename FUNC>
	inline void foreach_incident_face2(Volume w, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Face2>::value, "Wrong function cell parameter type");
		uint32 first = (w.dart.index/12)*12;
		if (!internal::void_to_true_binder(func, Face2(Dart(first)))) // 0
			return;
		first += 3;
		if (!internal::void_to_true_binder(func, Face2(Dart(first)))) // 3
			return;
		first += 3;
		if (!internal::void_to_true_binder(func, Face2(Dart(first)))) // 6
			return;
		first += 3;
		func(Face2(Dart(first))); // 9
	}

	/*******************************************************************************
	 * Adjacence traversal (of dim 2)
	 *******************************************************************************/

	template <typename FUNC>
	inline void foreach_adjacent_vertex_through_edge(Vertex2 v, const FUNC& f) const
	{
		static_assert(is_func_parameter_same<FUNC, Vertex2>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(v, [this, &f] (Dart d) -> bool
		{
			return internal::void_to_true_binder(f, Vertex2(phi2(d)));
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_vertex_through_face(Vertex2 v, const FUNC& f) const
	{
		static_assert(is_func_parameter_same<FUNC, Vertex2>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(v, [this, &f] (Dart vd) -> bool
		{
			bool res_nested_lambda = true;
			const Dart vd1 = phi1(vd);
			foreach_dart_of_orbit(Face2(vd), [&f, &res_nested_lambda, vd, vd1] (Dart fd) -> bool
			{
				// skip Vertex v itself and its first successor around current face
				if (fd != vd && fd != vd1)
					res_nested_lambda = internal::void_to_true_binder(f, Vertex2(fd));
				return res_nested_lambda;
			});
			return res_nested_lambda;
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_edge_through_vertex(Edge2 e, const FUNC& f) const
	{
		static_assert(is_func_parameter_same<FUNC, Edge2>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(e, [&f, this] (Dart ed) -> bool
		{
			bool res_nested_lambda = true;
			foreach_dart_of_orbit(Vertex2(ed), [&, ed] (Dart vd) -> bool
			{
					// skip Edge e itself
					if (vd != ed)
						res_nested_lambda = internal::void_to_true_binder(f, Edge2(vd));
					return res_nested_lambda;
			});
			return res_nested_lambda;
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_edge_through_face(Edge2 e, const FUNC& f) const
	{
		static_assert(is_func_parameter_same<FUNC, Edge2>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(e, [&f, this] (Dart ed) -> bool
		{
			bool res_nested_lambda = true;
			if (!this->is_boundary(ed))
			{
				foreach_dart_of_orbit(Face2(ed), [&f, &res_nested_lambda, ed] (Dart fd) -> bool
				{
					// skip Edge e itself
					if (fd != ed)
						res_nested_lambda = internal::void_to_true_binder(f, Edge2(fd));
					return res_nested_lambda;
				});
			}
			return res_nested_lambda;
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_face_through_vertex(Face2 f, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Face2>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(f, [this, &func] (Dart fd) -> bool
		{
			bool res_nested_lambda = true;
			const Dart fd1 = phi2(phi_1(fd));
			foreach_dart_of_orbit(Vertex2(fd), [this, &func, &res_nested_lambda, fd, fd1] (Dart vd) -> bool
			{
				// skip Face f itself and its first successor around current vertex
				if (vd != fd && vd != fd1)
					res_nested_lambda = internal::void_to_true_binder(func, Face2(vd));
				return res_nested_lambda;
			});
			return res_nested_lambda;
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_face_through_edge(Face2 f, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Face2>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(f, [this, &func] (Dart d) -> bool
		{
			return internal::void_to_true_binder(func, Face(phi2(d)));
		});
	}


	/*******************************************************************************
	 * Incidence traversal
	 *******************************************************************************/

	template <typename FUNC>
	inline void foreach_incident_edge(Vertex v, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Edge>::value, "Wrong function cell parameter type");
		DartMarkerStore marker(*this);
		foreach_dart_of_orbit(v, [&] (Dart d) -> bool
		{
			if (!marker.is_marked(d))
			{
				foreach_dart_of_PHI23(d, [&marker] (Dart dd) { marker.mark(dd); });
				return internal::void_to_true_binder(func, Edge(d));
			}
			return true;
		});
	}

	template <typename FUNC>
	inline void foreach_incident_face(Vertex v, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Face>::value, "Wrong function cell parameter type");
		DartMarkerStore marker(*this);
		foreach_dart_of_orbit(v, [&] (Dart d) -> bool
		{
			if (!marker.is_marked(d))
			{
				marker.mark(d);
				marker.mark(this->phi1(phi3(d)));
				return internal::void_to_true_binder(func, Face(d));
			}
			return true;
		});
	}

	template <typename FUNC>
	inline void foreach_incident_volume(Vertex v, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Volume>::value, "Wrong function cell parameter type");
		DartMarkerStore marker(*this);
		foreach_dart_of_orbit(v, [&] (Dart d) -> bool
		{
			if (!marker.is_marked(d) && !this->is_boundary(d))
			{
				marker.mark_orbit(Vertex2(d));
				return internal::void_to_true_binder(func, Volume(d));
			}
			return true;
		});
	}

	template <typename FUNC>
	inline void foreach_incident_vertex(Edge e, const FUNC& f) const
	{
		static_assert(is_func_parameter_same<FUNC, Vertex>::value, "Wrong function cell parameter type");
		if (internal::void_to_true_binder(f, Vertex(e.dart)))
			f(Vertex(this->phi2(e.dart)));
	}

	template <typename FUNC>
	inline void foreach_incident_face(Edge e, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Face>::value, "Wrong function cell parameter type");
		foreach_dart_of_PHI23(e.dart, [&func] (Dart d) -> bool{ return internal::void_to_true_binder(func, Face(d)); });
	}

	template <typename FUNC>
	inline void foreach_incident_volume(Edge e, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Volume>::value, "Wrong function cell parameter type");
		foreach_dart_of_PHI23(e.dart, [this, &func] (Dart d) -> bool
		{
			if (!this->is_boundary(d))
				return internal::void_to_true_binder(func, Volume(d));
			else
				return true;
		});
	}

	template <typename FUNC>
	inline void foreach_incident_vertex(Face f, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Vertex>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(Face2(f.dart), [&func] (Dart v) -> bool { return internal::void_to_true_binder(func, Vertex(v)); });
	}


	template <typename FUNC>
	inline void foreach_incident_edge(Face f, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Edge>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(Face2(f.dart), [&func] (Dart e) -> bool { return internal::void_to_true_binder(func, Edge(e)); });
	}

	template <typename FUNC>
	inline void foreach_incident_volume(Face f, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Volume>::value, "Wrong function cell parameter type");
		if (!this->is_boundary(f.dart))
			if (!internal::void_to_true_binder(func, Volume(f.dart)))
				return;
		const Dart d3 = phi3(f.dart);
		if (!this->is_boundary(d3))
			func(Volume(d3));
	}

	template <typename FUNC>
	inline void foreach_incident_vertex(Volume v, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Vertex>::value, "Wrong function cell parameter type");
		foreach_incident_vertex2(v, [&func] (Vertex2 ve) -> bool
		{
			return internal::void_to_true_binder(func, Vertex(ve.dart));
		});
	}

	template <typename FUNC>
	inline void foreach_incident_edge(Volume v, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Edge>::value, "Wrong function cell parameter type");
		foreach_incident_edge2(v, [&func] (Edge2 e) -> bool
		{
			return internal::void_to_true_binder(func, Edge(e.dart));
		});
	}

	template <typename FUNC>
	inline void foreach_incident_face(Volume v, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Face>::value, "Wrong function cell parameter type");

		foreach_incident_face2(v, [&func] (Face2 f) -> bool
		{
			return internal::void_to_true_binder(func, Face(f.dart));
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
		static_assert(is_func_parameter_same<FUNC, Vertex>::value, "Wrong function cell parameter type");
		foreach_incident_edge(v, [&] (Edge e) -> bool
		{
			return internal::void_to_true_binder(func, Vertex(this->phi2(e.dart)));
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_vertex_through_face(Vertex v, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Vertex>::value, "Wrong function cell parameter type");
		DartMarker marker_vertex(*this);
		marker_vertex.mark_orbit(v);
		foreach_incident_face(v, [&] (Face inc_face) -> bool
		{
			bool res_nested_lambda = true;
			foreach_incident_vertex(inc_face, [&] (Vertex vertex_of_face) -> bool
			{
				if (!marker_vertex.is_marked(vertex_of_face.dart))
				{
					marker_vertex.mark_orbit(vertex_of_face);
					res_nested_lambda = internal::void_to_true_binder(func, vertex_of_face);
				}
				return res_nested_lambda;
			});
			return res_nested_lambda;
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_vertex_through_volume(Vertex v, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Vertex>::value, "Wrong function cell parameter type");
		DartMarker marker_vertex(*this);
		marker_vertex.mark_orbit(v);
		foreach_incident_volume(v, [&] (Volume inc_vol) -> bool
		{
			bool res_nested_lambda = true;
			foreach_incident_vertex(inc_vol, [&] (Vertex inc_vert) -> bool
			{
				if (!marker_vertex.is_marked(inc_vert.dart))
				{
					marker_vertex.mark_orbit(inc_vert);
					res_nested_lambda = internal::void_to_true_binder(func, inc_vert);
				}
				return res_nested_lambda;
			});
			return res_nested_lambda;
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_edge_through_vertex(Edge e, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Edge>::value, "Wrong function cell parameter type");
		foreach_incident_vertex(e, [&] (Vertex iv) -> bool
		{
			bool res_nested_lambda = true;
			foreach_incident_edge(iv, [&] (Edge ie) -> bool
			{
				if (ie.dart != iv.dart)
					res_nested_lambda = internal::void_to_true_binder(func, ie);
				return res_nested_lambda;
			});
			return res_nested_lambda;
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_edge_through_face(Edge e, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Edge>::value, "Wrong function cell parameter type");
		DartMarker marker_edge(*this);
		marker_edge.mark_orbit(e);
		foreach_incident_face(e, [&] (Face inc_face) -> bool
		{
			bool res_nested_lambda = true;
			foreach_incident_edge(inc_face, [&] (Edge inc_edge) -> bool
			{
				if (!marker_edge.is_marked(inc_edge.dart))
				{
					marker_edge.mark_orbit(inc_edge);
					res_nested_lambda = internal::void_to_true_binder(func, inc_edge);
				}
				return res_nested_lambda;
			});
			return res_nested_lambda;
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_edge_through_volume(Edge e, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Edge>::value, "Wrong function cell parameter type");
		DartMarker marker_edge(*this);
		marker_edge.mark_orbit(e);
		foreach_incident_volume(e, [&] (Volume inc_vol) -> bool
		{
			bool res_nested_lambda = true;
			foreach_incident_edge(inc_vol, [&] (Edge inc_edge) -> bool
			{
				if (!marker_edge.is_marked(inc_edge.dart))
				{
					marker_edge.mark_orbit(inc_edge);
					res_nested_lambda = internal::void_to_true_binder(func, inc_edge);
				}
				return res_nested_lambda;
			});
			return res_nested_lambda;
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_face_through_vertex(Face f, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Face>::value, "Wrong function cell parameter type");
		DartMarker marker_face(*this);
		marker_face.mark_orbit(f);
		foreach_incident_vertex(f, [&] (Vertex v) -> bool
		{
			bool res_nested_lambda = true;
			foreach_incident_face(v, [&](Face inc_fac) -> bool
			{
				if (!marker_face.is_marked(inc_fac.dart))
				{
					marker_face.mark_orbit(inc_fac);
					res_nested_lambda = internal::void_to_true_binder(func, inc_fac);
				}
				return res_nested_lambda;
			});
			return res_nested_lambda;
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_face_through_edge(Face f, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Face>::value, "Wrong function cell parameter type");
		foreach_incident_edge(f, [&] (Edge ie) -> bool
		{
			bool res_nested_lambda = true;
			foreach_incident_face(ie, [&] (Face iface) -> bool
			{
				if (iface.dart != ie.dart)
					res_nested_lambda = internal::void_to_true_binder(func, iface);
				return res_nested_lambda;
			});
			return res_nested_lambda;
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_face_through_volume(Face f, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Face>::value, "Wrong function cell parameter type");
		DartMarker marker_face(*this);
		marker_face.mark_orbit(f);
		bool res1 = true;
		if (!this->is_boundary(f.dart))
		{
			foreach_incident_face(Volume(f.dart), [&] (Face inc_face) -> bool
			{
				if (!marker_face.is_marked(inc_face.dart))
				{
					marker_face.mark_orbit((inc_face));
					res1 = internal::void_to_true_binder(func, inc_face);
				}
				return res1;
			});
		}

		const Dart d3 = phi3(f.dart);
		if (!this->is_boundary(d3) && res1)
		{
			foreach_incident_face(Volume(d3), [&] (Face inc_face) -> bool
			{
				if (!marker_face.is_marked(inc_face.dart))
				{
					marker_face.mark_orbit((inc_face));
					return internal::void_to_true_binder(func, inc_face);
				}
				return true;
			});
		}
	}

	template <typename FUNC>
	inline void foreach_adjacent_volume_through_vertex(Volume v, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Volume>::value, "Wrong function cell parameter type");
		DartMarker marker_volume(*this);
		marker_volume.mark_orbit(v);
		foreach_incident_vertex(v, [&] (Vertex inc_vert) -> bool
		{
			bool res_nested_lambda = true;
			foreach_incident_volume(inc_vert, [&](Volume inc_vol) -> bool
			{
				if (!marker_volume.is_marked(inc_vol.dart) && !this->is_boundary(inc_vol.dart))
				{
					marker_volume.mark_orbit(inc_vol);
					res_nested_lambda = internal::void_to_true_binder(func, inc_vol);
				}
				return res_nested_lambda;
			});
			return res_nested_lambda;
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_volume_through_edge(Volume v, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Volume>::value, "Wrong function cell parameter type");
		DartMarker marker_volume(*this);
		marker_volume.mark_orbit(v);
		foreach_incident_edge(v, [&] (Edge inc_edge) -> bool
		{
			bool res_nested_lambda = true;
			foreach_incident_volume(inc_edge, [&] (Volume inc_vol) -> bool
			{
				if (!marker_volume.is_marked(inc_vol.dart) && !this->is_boundary(inc_vol.dart))
				{
					marker_volume.mark_orbit(inc_vol);
					res_nested_lambda = internal::void_to_true_binder(func, inc_vol);
				}
				return res_nested_lambda;
			});
			return res_nested_lambda;
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_volume_through_face(Volume v, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Volume>::value, "Wrong function cell parameter type");
		DartMarker marker_volume(*this);
		marker_volume.mark_orbit(v);
		foreach_incident_face(v, [&] (Edge inc_face) -> bool
		{
			bool res_nested_lambda = true;
			foreach_incident_volume(inc_face, [&] (Volume inc_vol) -> bool
			{
				if (!marker_volume.is_marked(inc_vol.dart) && !this->is_boundary(inc_vol.dart))
				{
					marker_volume.mark_orbit(inc_vol);
					res_nested_lambda = internal::void_to_true_binder(func, inc_vol);
				}
				return res_nested_lambda;
			});
			return res_nested_lambda;
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
	/**
	 * @brief check if embedding of map is also embedded in this (create if not). Used by merge method
	 * @param map
	 */
	void merge_check_embedding(const Self& map)
	{
		const static auto create_embedding = [=] (Self* map_ptr, Orbit orb)
		{
			switch (orb)
			{
				case Orbit::DART: map_ptr->template create_embedding<Orbit::DART>(); break;
				case Orbit::PHI1: map_ptr->template create_embedding<Orbit::PHI1>(); break;
				case Orbit::PHI2: map_ptr->template create_embedding<Orbit::PHI2>(); break;
				case Orbit::PHI21: map_ptr->template create_embedding<Orbit::PHI21>(); break;
				case Orbit::PHI1_PHI2: map_ptr->template create_embedding<Orbit::PHI1_PHI2>(); break;
				case Orbit::PHI1_PHI3: map_ptr->template create_embedding<Orbit::PHI1_PHI3>(); break;
				case Orbit::PHI2_PHI3: map_ptr->template create_embedding<Orbit::PHI2_PHI3>(); break;
				case Orbit::PHI21_PHI31: map_ptr->template create_embedding<Orbit::PHI21_PHI31>(); break;
				case Orbit::PHI1_PHI2_PHI3: map_ptr->template create_embedding<Orbit::PHI1_PHI2_PHI3>(); break;
				default: break;
			}
		};

		for (Orbit orb : { DART, PHI1, PHI2, PHI21, PHI1_PHI2, PHI1_PHI3, PHI2_PHI3, PHI21_PHI31, PHI1_PHI2_PHI3 })
			if (!this->is_embedded(orb) && map.is_embedded(orb))
				create_embedding(this, orb);
	}

	/**
	 * @brief ensure all cells (introduced while merging) are embedded.
	 * @param first index of first dart to scan
	 */
	void merge_finish_embedding(uint32 first)
	{
		const static auto new_orbit_embedding = [=] (Self* map, Dart d, cgogn::Orbit orb)
		{
			switch (orb)
			{
				case Orbit::DART: map->new_orbit_embedding(Cell<Orbit::DART>(d)); break;
				case Orbit::PHI1: map->new_orbit_embedding(Cell<Orbit::PHI1>(d)); break;
				case Orbit::PHI2: map->new_orbit_embedding(Cell<Orbit::PHI2>(d)); break;
				case Orbit::PHI21: map->new_orbit_embedding(Cell<Orbit::PHI21>(d)); break;
				case Orbit::PHI1_PHI2: map->new_orbit_embedding(Cell<Orbit::PHI1_PHI2>(d)); break;
				case Orbit::PHI1_PHI3: map->new_orbit_embedding(Cell<Orbit::PHI1_PHI3>(d)); break;
				case Orbit::PHI2_PHI3: map->new_orbit_embedding(Cell<Orbit::PHI2_PHI3>(d)); break;
				case Orbit::PHI21_PHI31: map->new_orbit_embedding(Cell<Orbit::PHI21_PHI31>(d)); break;
				case Orbit::PHI1_PHI2_PHI3: map->new_orbit_embedding(Cell<Orbit::PHI1_PHI2_PHI3>(d)); break;
				default: break;
			}
		};

		for (uint32 j = first, end = this->topology_.end(); j != end; this->topology_.next(j))
		{
			for (Orbit orb : { DART, PHI1, PHI2, PHI21, PHI1_PHI2, PHI1_PHI3, PHI2_PHI3, PHI21_PHI31, PHI1_PHI2_PHI3 })
			{
				if (this->is_embedded(orb))
				{
					if (!this->is_boundary(Dart(j)) && (*this->embeddings_[orb])[j] == INVALID_INDEX)
						new_orbit_embedding(this, Dart(j), orb);
				}
			}
		}
	}
};

struct CMap3TetraType
{
	using TYPE = CMap3Tetra_T<CMap3TetraType>;
};

using CMap3Tetra = CMap3Tetra_T<CMap3TetraType>;

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_CMAP_CMAP3_TETRA_CPP_))
extern template class CGOGN_CORE_API CMap3Builder_T<CMap3Tetra>;
extern template class CGOGN_CORE_API DartMarker<CMap3Tetra>;
extern template class CGOGN_CORE_API DartMarkerStore<CMap3Tetra>;
extern template class CGOGN_CORE_API DartMarkerNoUnmark<CMap3Tetra>;
extern template class CGOGN_CORE_API CellMarker<CMap3Tetra, CMap3Tetra::Vertex::ORBIT>;
extern template class CGOGN_CORE_API CellMarker<CMap3Tetra, CMap3Tetra::Edge::ORBIT>;
extern template class CGOGN_CORE_API CellMarker<CMap3Tetra, CMap3Tetra::Face::ORBIT>;
extern template class CGOGN_CORE_API CellMarker<CMap3Tetra, CMap3Tetra::Volume::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerNoUnmark<CMap3Tetra, CMap3Tetra::Vertex::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerNoUnmark<CMap3Tetra, CMap3Tetra::Edge::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerNoUnmark<CMap3Tetra, CMap3Tetra::Face::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerNoUnmark<CMap3Tetra, CMap3Tetra::Volume::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap3Tetra, CMap3Tetra::Vertex::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap3Tetra, CMap3Tetra::Edge::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap3Tetra, CMap3Tetra::Face::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap3Tetra, CMap3Tetra::Volume::ORBIT>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_MAP_MAP2_CPP_))

} // namespace cgogn

#endif // CGOGN_CORE_CMAP_CMAP3_TETRA_H_
