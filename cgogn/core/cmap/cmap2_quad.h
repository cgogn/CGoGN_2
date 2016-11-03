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

#ifndef CGOGN_CORE_CMAP_CMAP2_QUAD_H_
#define CGOGN_CORE_CMAP_CMAP2_QUAD_H_

#include <cgogn/core/cmap/map_base.h>
#include <cgogn/core/cmap/cmap2_builder.h>

namespace cgogn
{

template <typename MAP_TYPE>
class CMap2Quad_T : public MapBase<MAP_TYPE>
{
public:

	static const uint8 DIMENSION = 2;
	static const uint8 PRIM_SIZE = 4;

	using MapType = MAP_TYPE;
	using Inherit = MapBase<MAP_TYPE>;
	using Self = CMap2Quad_T<MAP_TYPE>;

	using Builder = CMap2Builder_T<Self>;

	friend class MapBase<MAP_TYPE>;
	friend class CMap2Builder_T<Self>;
	friend class DartMarker_T<Self>;
	friend class cgogn::DartMarkerStore<Self>;

	using CDart  = Cell<Orbit::DART>;
	using Vertex = Cell<Orbit::PHI21>;
	using Edge   = Cell<Orbit::PHI2>;
	using Face   = Cell<Orbit::PHI1>;
	using Volume = Cell<Orbit::PHI1_PHI2>;

	using Boundary = Face;
	using ConnectedComponent = Volume;

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

protected:

	ChunkArray<Dart>* phi2_;

	inline void init()
	{
		phi2_ = this->topology_.template add_chunk_array<Dart>("phi2");
	}

public:

	CMap2Quad_T() : Inherit()
	{
		init();
	}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CMap2Quad_T);

	~CMap2Quad_T() override
	{}

	/*!
	 * \brief Check the integrity of embedding information
	 */
	inline bool check_embedding_integrity()
	{
		bool result = true;

		if (this->template is_embedded<CDart>())
			result = result && this->template is_well_embedded<CDart>();

		if (this->template is_embedded<Vertex>())
			result = result && this->template is_well_embedded<Vertex>();

		if (this->template is_embedded<Edge>())
			result = result && this->template is_well_embedded<Edge>();

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
	 * \brief Init a newly added dart.
	 * The dart is defined as a fixed point for PHI2.
	 */
	inline void init_dart(Dart d)
	{
		(*phi2_)[d.index] = d;
	}

	/**
	 * @brief Check the integrity of a dart
	 * @param d the dart to check
	 * @return true if the integrity constraints are locally statisfied
	 * PHI2 should be an involution without fixed point (except for some boundary darts)
	 */
	inline bool check_integrity(Dart d) const
	{
		return phi1(phi_1(d)) == d && phi_1(phi1(d)) == d &&
				phi2(phi2(d)) == d &&
				( phi2(d) != d || ( phi2(d) == d && this->is_boundary(d)));
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
	inline void phi2_sew(Dart d, Dart e)
	{
		cgogn_assert(phi2(d) == d);
		cgogn_assert(phi2(e) == e);
		(*phi2_)[d.index] = e;
		(*phi2_)[e.index] = d;
	}

	/**
	 * \brief Remove the phi2 link between the current dart and its linked dart
	 * @param d the dart to unlink
	 * - Before: d->e and e->d
	 * - After:  d->d and e->e
	 */
	inline void phi2_unsew(Dart d)
	{
		Dart e = phi2(d);
		(*phi2_)[d.index] = d;
		(*phi2_)[e.index] = e;
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
		switch(d.index%4)
		{
		case 0: return Dart(d.index+1);break;
		case 1: return Dart(d.index+1);break;
		case 2: return Dart(d.index+1);break;
		}
		return Dart(d.index-3);
	}

	/*!
	 * \brief phi_1
	 * @param d
	 * @return phi_1(d)
	 */
	Dart phi_1(Dart d) const
	{
//		switch(d.index%4)
//		{
//		case 3: return Dart(d.index-1);break;
//		case 2: return Dart(d.index-1);break;
//		case 1: return Dart(d.index-1);break;
//		}
//		return Dart(d.index+3);

		switch(d.index%4)
		{
		case 1: return Dart(d.index-1);break;
		case 2: return Dart(d.index-1);break;
		case 3: return Dart(d.index-1);break;
		}
		return Dart(d.index+3);
	}

	/*!
	 * \brief phi11
	 * @param d
	 * @return phi1(phi1(d))
	 */
	inline Dart phi11(Dart d) const
	{
		switch(d.index%4)
		{
		case 0: return Dart(d.index+2);break;
		case 1: return Dart(d.index+2);break;
		case 2: return Dart(d.index-2);break;
		}
		return Dart(d.index-2);
	}

	/**
	 * \brief phi2
	 * @param d
	 * @return phi2(d)
	 */
	inline Dart phi2(Dart d) const
	{
		return (*phi2_)[d.index];
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
		static_assert((N % 10) <= 2, "Composition of PHI: invalid index");

		if (N%100 == 11)
			return phi11(phi< N/100 >(d));

		switch(N % 10)
		{
			case 1 : return phi1(phi<N / 10>(d)) ;
			case 2 : return phi2(phi<N / 10>(d)) ;
			default : return d ;
		}
	}

	/*******************************************************************************
	 * High-level embedded and topological operations
	 *******************************************************************************/

protected:

	/**
	 * @brief add a quad with fixed point phi2
	 * @return
	 */
	inline Dart add_quad_topo_fp()
	{
		Dart d = this->add_topology_element(); // this insert PRIM_SIZE darts
		// no need to set phi1
		return d;
	}

	inline Dart add_face_topo_fp(std::size_t size)
	{
		cgogn_message_assert(size == 4u, "Can create only quads");
		if (size != 4)
		{
			cgogn_log_warning("add_face_topo_fp") << "Attempt to create a face which is not a quad in CMap2Quad";
			return Dart();
		}
		return add_quad_topo_fp();
	}

	/**
	 * @brief remove a quad (4 darts)
	 * @param d
	 */
	inline void remove_face_topo_fp(Dart d)
	{
		this->remove_topology_element(d); // this remove PRIM_SIZE darts
	}

	/**
	 * \brief Add a quad in the map.
	 * \return A dart of the built face.
	 * Two 1-quad (f.p.) are built. The first one is the returned face,
	 * the second is a boundary face that closes the map.
	 */
	Dart add_quad_topo()
	{
		Dart d = add_quad_topo_fp();
		Dart e = add_quad_topo_fp();

		foreach_dart_of_PHI1(d, [&] (Dart it)
		{
			this->set_boundary(e, true);
			phi2_sew(it, e);
			e = phi_1(e);
		});

		return d;
	}

public:

	/**
	 * \brief Add a face (quad) in the map. Necessary function for import
	 * \param size : 4 or assert failed
	 * \return The built face
	 * If the map has Dart, Vertex, Edge, Face or Volume attributes,
	 * the inserted cells are automatically embedded on new attribute elements.
	 * More precisely :
	 *  - a Face attribute is created, if needed, for the new face.
	 */
	Face add_face(uint32 size)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		cgogn_message_assert(size == 4u, "Can create only quad");
		if (size != 4)
			cgogn_log_warning("add_face") << "Attempt to create a face which is not a quad";

		const Face f(add_quad_topo());

		if (this->template is_embedded<CDart>())
		{
			foreach_dart_of_orbit(f, [this] (Dart d)
			{
				this->new_orbit_embedding(CDart(d));
			});
		}

		if (this->template is_embedded<Vertex>())
		{
			foreach_dart_of_orbit(f, [this] (Dart d)
			{
				this->new_orbit_embedding(Vertex(d));
			});
		}

		if (this->template is_embedded<Edge>())
		{
			foreach_dart_of_orbit(f, [this] (Dart d)
			{
				this->new_orbit_embedding(Edge(d));
			});
		}

		if (this->template is_embedded<Face>())
			this->new_orbit_embedding(f);

		if (this->template is_embedded<Volume>())
			this->new_orbit_embedding(Volume(f.dart));

		return f;
	}

protected:

	inline Dart add_hexa_topo()
	{

		Dart f1 = add_quad_topo_fp();
		Dart f2 = add_quad_topo_fp();
		Dart f3 = add_quad_topo_fp();
		Dart f4 = add_quad_topo_fp();
		Dart f5 = add_quad_topo_fp();
		Dart f6 = add_quad_topo_fp();

		phi2_sew(f1,f2);
		f1 = phi1(f1);
		phi2_sew(f1,f3);
		f1 = phi1(f1);
		phi2_sew(f1,f4);
		f1 = phi1(f1);
		phi2_sew(f1,f5);
		f1 = phi1(f1);

		phi2_sew(phi1(f3),phi_1(f2));
		phi2_sew(phi1(f4),phi_1(f3));
		phi2_sew(phi1(f5),phi_1(f4));
		phi2_sew(phi1(f2),phi_1(f5));

		phi2_sew(phi11(f2),f6);
		f6 = phi1(f6);
		phi2_sew(phi11(f5),f6);
		f6 = phi1(f6);
		phi2_sew(phi11(f4),f6);
		f6 = phi1(f6);
		phi2_sew(phi11(f3),f6);
		f6 = phi1(f6);

		return f1;
	}

public:

	Volume add_hexa()
	{
		Volume vol(add_hexa_topo());

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
	 * @brief extrude a quad: replace by 5 faces. (topo version)
	 * @param f the quad face to extrude
	 */
	void extrude_quad_topo(Face f)
	{
		Dart d = f.dart;
		Dart ff1 = phi2(d);
		d = phi1(d);
		Dart ff2 = phi2(d);
		d = phi1(d);
		Dart ff3 = phi2(d);
		d = phi1(d);
		Dart ff4 = phi2(d);

#ifndef	NDEBUG
		phi2_unsew(ff1);
		phi2_unsew(ff2);
		phi2_unsew(ff3);
		phi2_unsew(ff4);
#endif

		Dart f1 = f.dart;
		Dart f2 = add_quad_topo_fp();
		Dart f3 = add_quad_topo_fp();
		Dart f4 = add_quad_topo_fp();
		Dart f5 = add_quad_topo_fp();

		phi2_sew(f1,f2);
		f1 = phi1(f1);
		phi2_sew(f1,f3);
		f1 = phi1(f1);
		phi2_sew(f1,f4);
		f1 = phi1(f1);
		phi2_sew(f1,f5);
		f1 = phi1(f1);

		phi2_sew(phi1(f3),phi_1(f2));
		phi2_sew(phi1(f4),phi_1(f3));
		phi2_sew(phi1(f5),phi_1(f4));
		phi2_sew(phi1(f2),phi_1(f5));

		phi2_sew(phi11(f2),ff1);
		phi2_sew(phi11(f3),ff2);
		phi2_sew(phi11(f4),ff3);
		phi2_sew(phi11(f5),ff4);
	}

public:

	/**
	 * @brief extrude a quad: replace by 5 quads.
	 * @param f the quad face
	 */
	Face extrude_quad(Face f)
	{
		extrude_quad_topo(f);

		if (this->template is_embedded<CDart>())
		{
			foreach_dart_of_orbit(f, [this] (Dart d)
			{
				// darts of f
				this->new_orbit_embedding(CDart(d));
				// darts of faces adjacent to f
				foreach_dart_of_orbit(Face(phi2(d)), [this] (Dart e)
				{
					this->new_orbit_embedding(CDart(e));
				});
			});
		}

		if (this->template is_embedded<Vertex>())
		{
			foreach_dart_of_orbit(f, [this] (Dart d)
			{
				this->new_orbit_embedding(Vertex(d));
				Dart v1 = phi_1(phi2(d));
				Dart v0 = phi2(phi_1(v1));
				this->template copy_embedding<Vertex>(v1, v0);
				Dart v2 = phi1(phi2(v1));
				this->template copy_embedding<Vertex>(v2, v0);
			});
		}

		if (this->template is_embedded<Edge>())
		{
			foreach_dart_of_orbit(f, [this] (Dart d)
			{
				this->new_orbit_embedding(Edge(d));
				Dart d1 = phi1(phi2(d));
				this->new_orbit_embedding(Edge(d1));
				d1 = phi1(d1);
				this->template copy_embedding<Edge>(d1, phi2(d1));
			});
		}

		if (this->template is_embedded<Face>())
		{
			this->new_orbit_embedding(f);
			foreach_adjacent_face_through_edge(f, [this] (Face fi)
			{
				this->new_orbit_embedding(fi);
			});
		}

		if (this->template is_embedded<Volume>())
		{
			uint32 emb = this->embedding(Volume(phi<2112>(f.dart)));
			this->template set_orbit_embedding<Volume>(f, emb);
			foreach_adjacent_face_through_edge(f, [this, emb] (Face fi)
			{
				this->template set_orbit_embedding<Volume>(fi, emb);
			});
		}

		return f;
	}

protected:

	/**
	 * @brief Close the topological hole that contains Dart d (a fixed point for PHI2).
	 * @param d : a dart of the hole
	 * @return a dart of on of the faces that closes the hole
	 */
	inline Dart close_hole_topo(Dart d)
	{
		cgogn_message_assert(phi2(d) == d, "CMap2Quad: close hole called on a dart that is not a phi2 fix point");

		Dart first = add_quad_topo_fp();	// First edge of the face that will fill the hole
		phi2_sew(d, first);				// 2-sew the new edge to the hole

		Dart prec_quad = first;
		Dart d_next = d;				// Turn around the hole
		Dart d_phi1;					// to complete the face
		do
		{
			do
			{
				d_phi1 = phi1(d_next); // Search and put in d_next
				d_next = phi2(d_phi1); // the next dart of the hole
			} while (d_next != d_phi1 && d_phi1 != d);

			if (d_phi1 != d)
			{
				Dart quad = add_quad_topo_fp();
				phi2_sew(d_next, quad);
				phi2_sew(phi_1(prec_quad), phi1(quad));
				prec_quad = quad;
			}
		} while (d_phi1 != d);

		phi2_sew(phi_1(prec_quad), phi1(first));

		return first;
	}

	/**
	 * @brief Close (not really) a hole with a set of quad.
	 * @return a face of the fan
	 */
	inline Face close_hole(Dart d)
	{
		//	const Face f(map_.close_hole_topo(d));
		Dart dh = close_hole_topo(d);

		Dart di = dh;

		do
		{
			Dart di0 = phi2(di);
			Dart di1 = phi1(di);

			if (this->template is_embedded<Vertex>())
			{
				this->template copy_embedding<Vertex>(di, phi1(di0));
				this->template copy_embedding<Vertex>(di1, di0);
			}

			if (this->template is_embedded<Edge>())
				this->template copy_embedding<Edge>(di, di0);

			if (this->template is_embedded<Volume>())
				this->template set_orbit_embedding<Volume>(Face(di), this->embedding(Volume(d)));

			di = phi<21>(di1);
		} while (di != dh);

		return Face(dh);
	}

	/**
	 * @brief close_map
	 * @return the number of holes (filled)
	 */
	inline uint32 close_map()
	{
		uint32 nb_holes = 0;

		std::vector<Dart>* fix_point_darts = cgogn::dart_buffers()->buffer();
		this->foreach_dart([&] (Dart d)
		{
			if (phi2(d) == d)
				fix_point_darts->push_back(d);
		});

		for (Dart d : (*fix_point_darts))
		{
			if (phi2(d) == d)
			{
				Face f = close_hole(d);
				Dart df = f.dart;
				do
				{
					this->boundary_mark(Face(df));
					df = phi<121>(df);
				} while (df != f.dart);
				++nb_holes;
			}
		}

		cgogn::dart_buffers()->release_buffer(fix_point_darts);

		return nb_holes;
	}

	/*******************************************************************************
	 * Connectivity information
	 *******************************************************************************/

public:

	inline uint32 degree(Vertex v) const
	{
		return this->nb_darts_of_orbit(v);
	}

	inline uint32 codegree(Edge e) const
	{
		if (phi1(e.dart) == e.dart)
			return 1;
		else
			return 2;
	}

	inline uint32 degree(Edge e) const
	{
		if (this->is_incident_to_boundary(e))
			return 1;
		else
			return 2;
	}

	inline uint32 codegree(Face) const
	{
		return 4;
	}

	inline uint32 degree(Face) const
	{
		return 1;
	}

	inline bool has_codegree(Face, uint32 codegree) const
	{
		return codegree == 4;
	}

	inline uint32 codegree(Volume v) const
	{
		uint32 result = 0;
		foreach_incident_face(v, [&result] (Face) { ++result; });
		return result;
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
			if (this->is_boundary(phi2(d))) { result = true; return false; }
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
		uint32 first = d.index & 0xfffffffc;
		f(Dart(first++));
		f(Dart(first++));
		f(Dart(first++));
		f(Dart(first));

//		Dart it = d;
//		do
//		{
//			f(it);
//			it = phi1(it);
//		} while (it != d);
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
		Dart it = d;
		do
		{
			if ( !(this->is_boundary(it) && this->is_boundary(phi2(it))) )
				f(it);
			it = phi2(phi_1(it));
		} while (it != d);
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
			Dart e = (*visited_faces)[i];
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

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_dart_of_orbit(Cell<ORBIT> c, const FUNC& f) const
	{
		static_assert(is_func_parameter_same<FUNC, Dart>::value, "Wrong function parameter type");
		static_assert(ORBIT == Orbit::DART || ORBIT == Orbit::PHI1 || ORBIT == Orbit::PHI2 ||
					  ORBIT == Orbit::PHI1_PHI2 || ORBIT == Orbit::PHI21,
					  "Orbit not supported in a CMap2");

		switch (ORBIT)
		{
			case Orbit::DART: f(c.dart); break;
			case Orbit::PHI1: foreach_dart_of_PHI1(c.dart, f); break;
			case Orbit::PHI2: foreach_dart_of_PHI2(c.dart, f); break;
			case Orbit::PHI21: foreach_dart_of_PHI21(c.dart, f); break;
			case Orbit::PHI1_PHI2: foreach_dart_of_PHI1_PHI2(c.dart, f); break;
			case Orbit::PHI2_PHI3:
			case Orbit::PHI1_PHI3:
			case Orbit::PHI21_PHI31:
			case Orbit::PHI1_PHI2_PHI3:
			default: cgogn_assert_not_reached("Orbit not supported in a CMap2"); break;
		}
	}

	template <typename FUNC>
	inline void foreach_dart_of_PHI1_until(Dart d, const FUNC& f) const
	{
		Dart it = d;
		do
		{
			if (!f(it))
				break;
			it = phi1(it);
		} while (it != d);
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
		Dart it = d;
		do
		{
			if ( !(this->is_boundary(it) && this->is_boundary(phi2(it))) )
				if (!f(it))
					break;
			it = phi2(phi_1(it));
		} while (it != d);
	}

	template <typename FUNC>
	void foreach_dart_of_PHI1_PHI2_until(Dart d, const FUNC& f) const
	{
		DartMarkerStore marker(*this);

		std::vector<Dart>* visited_faces = cgogn::dart_buffers()->buffer();
		visited_faces->push_back(d); // Start with the face of d

		// For every face added to the list
		for(uint32 i = 0; i < visited_faces->size(); ++i)
		{
			Dart e = (*visited_faces)[i];
			if (!marker.is_marked(e))	// Face has not been visited yet
			{
				// mark visited darts (current face)
				// and add non visited adjacent faces to the list of face
				Dart it = e;
				do
				{
					if (!f(it)) // apply the function to the darts of the face
					{
						cgogn::dart_buffers()->release_buffer(visited_faces);
						return;
					}
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

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_dart_of_orbit_until(Cell<ORBIT> c, const FUNC& f) const
	{
		static_assert(is_func_parameter_same<FUNC, Dart>::value, "Wrong function parameter type");
		static_assert(is_func_return_same<FUNC, bool>::value, "Wrong function return type");

		static_assert(ORBIT == Orbit::DART || ORBIT == Orbit::PHI1 || ORBIT == Orbit::PHI2 ||
					  ORBIT == Orbit::PHI1_PHI2 || ORBIT == Orbit::PHI21,
					  "Orbit not supported in a CMap2");

		switch (ORBIT)
		{
			case Orbit::DART: f(c.dart); break;
			case Orbit::PHI1: foreach_dart_of_PHI1_until(c.dart, f); break;
			case Orbit::PHI2: foreach_dart_of_PHI2_until(c.dart, f); break;
			case Orbit::PHI21: foreach_dart_of_PHI21_until(c.dart, f); break;
			case Orbit::PHI1_PHI2: foreach_dart_of_PHI1_PHI2_until(c.dart, f); break;
			case Orbit::PHI2_PHI3:
			case Orbit::PHI1_PHI3:
			case Orbit::PHI21_PHI31:
			case Orbit::PHI1_PHI2_PHI3:
			default: cgogn_assert_not_reached("Orbit not supported in a CMap2"); break;
		}
	}

	/*******************************************************************************
	 * Incidence traversal
	 *******************************************************************************/

public:

	template <typename FUNC>
	inline void foreach_incident_edge(Vertex v, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Edge>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(v, [&] (Dart d)
		{
				func(Edge(d));
		});
	}

	template <typename FUNC>
	inline void foreach_incident_face(Vertex v, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Face>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(v, [this, &func] (Dart d)
		{
			if (!this->is_boundary(d))
				func(Face(d));
		});
	}

	template <typename FUNC>
	inline void foreach_incident_volume(Vertex v, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Volume>::value, "Wrong function cell parameter type");
		func(Volume(v.dart));
	}

	template <typename FUNC>
	inline void foreach_incident_vertex(Edge e, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Vertex>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(e, [&func] (Dart d){ func(Vertex(d)); });
	}

	template <typename FUNC>
	inline void foreach_incident_face(Edge e, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Face>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(e, [this, &func] (Dart d)
		{
			if (!this->is_boundary(d))
				func(Face(d));
		});
	}

	template <typename FUNC>
	inline void foreach_incident_volume(Edge e, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Volume>::value, "Wrong function cell parameter type");
		func(Volume(e.dart));
	}

	template <typename FUNC>
	inline void foreach_incident_vertex(Face f, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Vertex>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(f, [&func] (Dart d) { func(Vertex(d)); });
	}

	template <typename FUNC>
	inline void foreach_incident_edge(Face f, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Edge>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(f, [&func] (Dart d) { func(Edge(d)); });
	}

	template <typename FUNC>
	inline void foreach_incident_volume(Face f, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Volume>::value, "Wrong function cell parameter type");
		func(Volume(f.dart));
	}

	template <typename FUNC>
	inline void foreach_incident_vertex(Volume w, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Vertex>::value, "Wrong function cell parameter type");
		DartMarkerStore marker(*this);
		foreach_dart_of_orbit(w, [&] (Dart d)
		{
			if (!marker.is_marked(d))
			{
				marker.mark_orbit(Vertex(d));
				func(Vertex(d));
			}
		});
	}

	template <typename FUNC>
	inline void foreach_incident_edge(Volume w, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Edge>::value, "Wrong function cell parameter type");
		DartMarkerStore marker(*this);
		foreach_dart_of_orbit(w, [&] (Dart d)
		{
			if (!marker.is_marked(d))
			{
				marker.mark_orbit(Edge(d));
				func(Edge(d));
			}
		});
	}

	template <typename FUNC>
	inline void foreach_incident_face(Volume w, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Face>::value, "Wrong function cell parameter type");
		DartMarkerStore marker(*this);
		foreach_dart_of_orbit(w, [&] (Dart d)
		{
			if (!marker.is_marked(d) && !this->is_boundary(d))
			{
				marker.mark_orbit(Face(d));
				func(Face(d));
			}
		});
	}

	/*******************************************************************************
	 * Adjacence traversal
	 *******************************************************************************/

	template <typename FUNC>
	inline void foreach_adjacent_vertex_through_edge(Vertex v, const FUNC& f) const
	{
		static_assert(is_func_parameter_same<FUNC, Vertex>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(v, [this, &f] (Dart d)
		{
				f(Vertex(phi2(d)));
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_vertex_through_face(Vertex v, const FUNC& f) const
	{
		static_assert(is_func_parameter_same<FUNC, Vertex>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(v, [this, &f] (Dart vd)
		{
			if (!this->is_boundary(vd))
			{
				Dart vd1 = phi1(vd);
				foreach_dart_of_orbit(Face(vd), [&f, vd, vd1] (Dart fd)
				{
					// skip Vertex v itself and its first successor around current face
					if (fd != vd && fd != vd1)
						f(Vertex(fd));
				});
			}
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_edge_through_vertex(Edge e, const FUNC& f) const
	{
		static_assert(is_func_parameter_same<FUNC, Edge>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(e, [&f, this] (Dart ed)
		{
			foreach_dart_of_orbit(Vertex(ed), [&, ed] (Dart vd)
			{
				if (!this->is_boundary(vd) && !this->is_boundary(phi2(vd)))
					// skip Edge e itself
					if (vd != ed)
						f(Edge(vd));
			});
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_edge_through_face(Edge e, const FUNC& f) const
	{
		static_assert(is_func_parameter_same<FUNC, Edge>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(e, [&f, this] (Dart ed)
		{
			if (!this->is_boundary(ed))
			{
				foreach_dart_of_orbit(Face(ed), [&f, ed] (Dart fd)
				{
					// skip Edge e itself
					if (fd != ed)
						f(Edge(fd));
				});
			}
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_face_through_vertex(Face f, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Face>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(f, [this, &func] (Dart fd)
		{
			Dart fd1 = phi2(phi_1(fd));
			foreach_dart_of_orbit(Vertex(fd), [this, &func, fd, fd1] (Dart vd)
			{
				// skip Face f itself and its first successor around current vertex
				if (vd != fd && vd != fd1 && !this->is_boundary(vd))
					func(Face(vd));
			});
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_face_through_edge(Face f, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Face>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(f, [this, &func] (Dart d)
		{
			const Dart d2 = phi2(d);
			if (!this->is_boundary(d2))
				func(Face(d2));
		});
	}

	inline std::pair<Vertex,Vertex> vertices(Edge e) const
	{
		return std::pair<Vertex, Vertex>(Vertex(e.dart), Vertex(phi1(e.dart)));
	}

protected:

	/**
	 * @brief check if embedding of map is also embedded in this (create if not). Used by merge method
	 * @param map
	 */
	void merge_check_embedding(const Self& map)
	{
		const static auto create_embedding = [=](Self* map_ptr, Orbit orb)
		{
			switch (orb) {
				case Orbit::DART: map_ptr->template create_embedding<Orbit::DART>(); break;
				case Orbit::PHI1: map_ptr->template create_embedding<Orbit::PHI1>(); break;
				case Orbit::PHI2: map_ptr->template create_embedding<Orbit::PHI2>(); break;
				case Orbit::PHI21: map_ptr->template create_embedding<Orbit::PHI21>(); break;
				case Orbit::PHI1_PHI2: map_ptr->template create_embedding<Orbit::PHI1_PHI2>(); break;
				default: break;
			}
		};

		for (Orbit orb : { DART, PHI1, PHI2, PHI21, PHI1_PHI2 })
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
				default: break;
			}
		};

		for (uint32 j = first, end = this->topology_.end(); j != end; this->topology_.next(j))
		{
			for (Orbit orb : { DART, PHI1, PHI2, PHI21, PHI1_PHI2 })
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

struct CMap2QuadType
{
	using TYPE = CMap2Quad_T<CMap2QuadType>;
};

using CMap2Quad = CMap2Quad_T<CMap2QuadType>;

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_CMAP_CMAP2_QUAD_CPP_))
extern template class CGOGN_CORE_API CMap2Builder_T<CMap2Quad>;
extern template class CGOGN_CORE_API DartMarker<CMap2Quad>;
extern template class CGOGN_CORE_API DartMarkerStore<CMap2Quad>;
extern template class CGOGN_CORE_API DartMarkerNoUnmark<CMap2Quad>;
extern template class CGOGN_CORE_API CellMarker<CMap2Quad, CMap2Quad::Vertex::ORBIT>;
extern template class CGOGN_CORE_API CellMarker<CMap2Quad, CMap2Quad::Edge::ORBIT>;
extern template class CGOGN_CORE_API CellMarker<CMap2Quad, CMap2Quad::Face::ORBIT>;
extern template class CGOGN_CORE_API CellMarker<CMap2Quad, CMap2Quad::Volume::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerNoUnmark<CMap2Quad, CMap2Quad::Vertex::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerNoUnmark<CMap2Quad, CMap2Quad::Edge::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerNoUnmark<CMap2Quad, CMap2Quad::Face::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerNoUnmark<CMap2Quad, CMap2Quad::Volume::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap2Quad, CMap2Quad::Vertex::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap2Quad, CMap2Quad::Edge::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap2Quad, CMap2Quad::Face::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap2Quad, CMap2Quad::Volume::ORBIT>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_MAP_MAP2_CPP_))

} // namespace cgogn

#endif // CGOGN_CORE_CMAP_CMAP2_QUAD_H_
