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

#ifndef CGOGN_CORE_CMAP_CMAP2_TRI_H_
#define CGOGN_CORE_CMAP_CMAP2_TRI_H_

#include <cgogn/core/cmap/map_base.h>
#include <cgogn/core/cmap/cmap2_builder.h>

namespace cgogn
{

template <typename MAP_TYPE>
class CMap2Tri_T : public MapBase<MAP_TYPE>
{
public:

	static const uint8 DIMENSION = 2;
	static const uint8 PRIM_SIZE = 3;

	using MapType = MAP_TYPE;
	using Inherit = MapBase<MAP_TYPE>;
	using Self = CMap2Tri_T<MAP_TYPE>;

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

	CMap2Tri_T() : Inherit()
	{
		init();
	}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CMap2Tri_T);

	~CMap2Tri_T() override
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
	 * \brief Init newly added dart.
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
	 * PHI2 should be an involution without fixed point
	 */
	inline bool check_integrity(Dart d) const
	{
		return (phi1(phi_1(d)) == d &&
				 phi_1(phi1(d)) == d &&
				phi2(phi2(d)) == d &&
				phi2(d) != d);
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
		switch (d.index%3)
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
		switch (d.index%3)
		{
			case 2: return Dart(d.index-1);break;
			case 1: return Dart(d.index-1);break;
		}
		return Dart(d.index+2);
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
		switch (N % 10)
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
	 * @brief Add a triangle with fixed point phi2
	 * @return
	 */
	inline Dart add_tri_topo_fp()
	{
		Dart d = this->add_topology_element(); // in fact insert PRIM_SIZE darts
		// no need to set phi1
		return d;
	}

	inline Dart add_face_topo_fp(std::size_t size)
	{
		cgogn_message_assert(size == 3u, "Can create only triangles");
		if (size != 3)
		{
			cgogn_log_warning("add_face_topo_fp") << "Attempt to create a face which is not a triangle in CMap2Tri";
			return Dart();
		}
		return add_tri_topo_fp();
	}

	/**
	 * @brief remove a triangle (3 darts)
	 * @param d
	 */
	inline void remove_tri_topo_fp(Dart d)
	{
		this->remove_topology_element(d); // in fact remove PRIM_SIZE darts
	}

	/**
	 * \brief Add a triangle in the map.
	 * \return A dart of the built face.
	 * Two 1-triangle (f.p.) are built. The first one is the returned face,
	 * the second is a boundary face that closes the map.
	 */
	Dart add_tri_topo()
	{
		Dart d = add_tri_topo_fp();
		Dart e = add_tri_topo_fp();

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
	 * \brief Add a face (triangle) in the map. Necessary function for import
	 * \param size : 3 or assert failed
	 * \return The built face
	 * If the map has Dart, Vertex, Edge, Face or Volume attributes,
	 * the inserted cells are automatically embedded on new attribute elements.
	 * More precisely :
	 *  - a Face attribute is created, if needed, for the new face.
	 */
	Face add_face(uint32 size)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		cgogn_message_assert(size == 3u, "Can create only triangles");
		if (size != 3)
			cgogn_log_warning("add_face") << "Attempt to create a face which is not a triangle";

		const Face f(add_tri_topo());

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

	inline Dart add_tetra_topo()
	{
		Dart f1 = add_tri_topo_fp();
		Dart f2 = add_tri_topo_fp();
		Dart f3 = add_tri_topo_fp();
		Dart f4 = add_tri_topo_fp();

		phi2_sew(phi_1(f2), phi1(f1));
		phi2_sew(phi_1(f3), phi1(f2));
		phi2_sew(phi_1(f1), phi1(f3));

		phi2_sew(f1,f4);
		f4 = phi1(f4);
		phi2_sew(f3,f4);
		f4 = phi1(f4);
		phi2_sew(f2,f4);

		return f1;
	}

public:

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
	 * @brief Flip an edge
	 * @param d : a dart of the edge to flip
	 * @return true if the edge has been flipped, false otherwise
	 * Each end of the edge is detached from its initial vertex
	 * and inserted in the next vertex within its incident face.
	 * An end of the edge that is a vertex of degree 1 is not moved.
	 * If one of the faces have co-degree 1 then nothing is done.
	 */
	inline bool flip_edge_topo(Dart d)
	{
		Dart e = phi2(d);
		if (!this->is_incident_to_boundary(Edge(d)))
		{
			Dart d1  = phi1(d);
			Dart d11 = phi1(d1);
			Dart e1  = phi1(e);
			Dart e11 = phi1(e1);

			Dart xd1  = phi2(d1);
			Dart xd11 = phi2(d11);
			Dart xe1  = phi2(e1);
			Dart xe11 = phi2(e11);

#ifndef	NDEBUG
			phi2_unsew(d1);
			phi2_unsew(d11);
			phi2_unsew(e1);
			phi2_unsew(e11);
#endif

			phi2_sew(d1,xd11);
			phi2_sew(d11,xe1);
			phi2_sew(e1,xe11);
			phi2_sew(e11,xd1);

			return true;
		}
		return false;
	}

public:

	/**
	 * @brief Flip an edge
	 * @param ed : the edge to flip
	 * The two endpoints of the given edge are moved to the next vertices
	 * of their two adjacent faces
	 */
	inline void flip_edge(Edge e)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		if (flip_edge_topo(e.dart))
		{
			Dart d = e.dart;
			Dart dd = phi2(d);

			Dart d1  = phi1(d);
			Dart d11 = phi1(d1);
			Dart dd1  = phi1(dd);
			Dart dd11 = phi1(dd1);

			if (this->template is_embedded<Vertex>())
			{
				// warning not all darts of boundary are embedded
				// only those which are phi2-sewed with non-boundary
				this->template copy_embedding<Vertex>(d1, phi2(dd11));
				this->template copy_embedding<Vertex>(dd1, phi2(d11));
				this->template copy_embedding<Vertex>(d11, phi2(d1));
				this->template copy_embedding<Vertex>(dd11, phi2(dd1));
				this->template copy_embedding<Vertex>(d, dd1);
				this->template copy_embedding<Vertex>(dd, d1);

			}

			if (this->template is_embedded<Edge>())
			{
				this->template copy_embedding<Edge>(d1, phi2(d1));
				this->template copy_embedding<Edge>(d11, phi2(d11));
				this->template copy_embedding<Edge>(dd1, phi2(dd1));
				this->template copy_embedding<Edge>(dd11, phi2(dd11));
			}
		}
	}

protected:

	/**
	 * @brief Collapse an edge (only topo)
	 * @param d : a dart of the edge to collapse
	 * @return a dart of the resulting vertex
	 */
	inline Dart collapse_edge_topo(Dart d)
	{
		Dart res = phi2(phi_1(d));
		Dart e = phi2(d);

#ifndef	NDEBUG
		phi2_unsew(phi<12>(d));
		phi2_unsew(res);
		phi2_unsew(phi<12>(e));
		phi2_unsew(phi2(phi_1(e)));
#endif

		phi2_sew(phi<12>(d),res);
		phi2_sew(phi<12>(e),phi2(phi_1(e)));

		remove_tri_topo_fp(d);
		remove_tri_topo_fp(e);

		return res;
	}

public:

	/**
	 * @brief Collapse an edge
	 * @param e : the edge to collapse
	 * @return the resulting vertex
	 */
	inline Vertex collapse_edge(Edge e)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		// dart for edge of one side
		Dart d2 = phi2(phi_1(phi2(e.dart)));
		// dart for edge of other side, and vertex
		Dart d1 = collapse_edge_topo(e.dart);

		Vertex v(d1);

		if (this->template is_embedded<Vertex>())
			this->template set_orbit_embedding<Vertex>(v, this->embedding(v));

		if (this->template is_embedded<Edge>())
		{
			this->template copy_embedding<Edge>(d1, phi2(d1));
			this->template copy_embedding<Edge>(d2, phi2(d2));
		}

		return v;
	}

protected:

	/**
	 * @brief split a vertex into an edge (2 triangles are inserted) TOPO ONLY
	 * @param d1 Dart of vertex to split (dart give edge to be replaced by a triangle)
	 * @param d2 Dart of vertex to split (dart give edge to be replaced by a triangle)
	 * @return a dart of the inserted edge
	 */
	Dart split_vertex_topo(Dart d1, Dart d2)
	{
		cgogn_message_assert(this->same_orbit(Vertex(d1), Vertex(d2)), "CMap2Tri::split_vertex_topo: d1 & d2 must be the same vertex");
		cgogn_message_assert(d1 != d2, "CMap2Tri::split_vertex_topo: d1 & d2 must be different darts ");

		Dart e1  = d1;
		Dart ee1 = phi2(e1);
		Dart e2  = d2;
		Dart ee2 = phi2(e2);

#ifndef	NDEBUG
		phi2_unsew(e1);
		phi2_unsew(ee1);
		phi2_unsew(e2);
		phi2_unsew(ee2);
#endif

		Dart f1 = add_tri_topo_fp();
		Dart f2 = add_tri_topo_fp();

		phi2_sew(f1,f2);

		phi2_sew(phi1(f1),ee1);
		phi2_sew(phi_1(f1),e1);

		phi2_sew(phi1(f2),ee2);
		phi2_sew(phi_1(f2),e2);

		return f1;
	}

public:

	/**
	 * @brief split a vertex into an edge (2 triangles are inserted)
	 * @param d1 Dart of vertex to split (dart give edge to be replaced by a triangle)
	 * @param d2 Dart of vertex to split (dart give edge to be replaced by a triangle)
	 * @return the inserted edge
	 */
	Edge split_vertex(Dart d1, Dart d2)
	{
		Edge res_edge(split_vertex_topo(d1,d2));

		if (this->template is_embedded<CDart>())
		{
			foreach_incident_face(res_edge, [this] (Face nf)
			{
				foreach_dart_of_orbit(nf, [this] (Dart d)
				{
					this->new_orbit_embedding(CDart(d));
				});
			});
		}

		if (this->template is_embedded<Vertex>())
		{
			this->new_orbit_embedding(Vertex(res_edge.dart));
			Dart vd1 = phi2(res_edge.dart);
			Dart d0 = phi2(phi_1(vd1));
			this->template copy_embedding<Vertex>(vd1, d0);
			Dart vd2 = phi1(res_edge.dart);
			this->template copy_embedding<Vertex>(vd2, d0);

			foreach_incident_face(res_edge, [this] (Face nf)
			{
				Dart dv1 = phi_1(nf.dart); // new dart of exiting vertex
				Dart dv0 = phi2(phi1(nf.dart)); // old dart of exiting vertex
				this->template copy_embedding<Vertex>(dv1, dv0);
			});
		}

		if (this->template is_embedded<Edge>())
		{
			this->new_orbit_embedding(res_edge); // new edge
			foreach_incident_face(res_edge, [this] (Face nf)
			{
				Dart d = phi1(nf.dart);
				this->new_orbit_embedding(Edge(d));
				d = phi1(d);
				this->template copy_embedding<Edge>(d, phi2(d)); // more efficient to use the old emb
			});
		}

		if (this->template is_embedded<Face>())
		{
			foreach_incident_face(res_edge, [this] (Face nf)
			{
				this->new_orbit_embedding(nf);
			});
		}

		if (this->template is_embedded<Volume>())
		{
			uint32 emb = this->embedding(Volume(this->template phi<12>(res_edge.dart)));
			foreach_incident_face(res_edge, [this, emb] (Face nf)
			{
				this->template set_orbit_embedding<Volume>(nf, emb);
			});
		}

		return res_edge;
	}

protected:

	/**
	 * @brief cut an edge and the two incident triangles
	 * @param e the edge to cut
	 * @return a dart of created vertex
	 */
	Dart cut_edge_topo(Edge e)
	{
		Dart d = phi1(e.dart);
		Dart e1 = phi2(d);
		d = phi1(d);
		Dart e2 = phi2(d);
		d = phi1(phi2(e.dart));
		Dart e3 = phi2(d);
		d = phi1(d);
		Dart e4 = phi2(d);

#ifndef	NDEBUG
		phi2_unsew(e1);
		phi2_unsew(e2);
		phi2_unsew(e3);
		phi2_unsew(e4);
#endif

		remove_tri_topo_fp(e.dart);
		remove_tri_topo_fp(phi2(e.dart));

		Dart f1 = add_tri_topo_fp();
		Dart f2 = add_tri_topo_fp();
		Dart f3 = add_tri_topo_fp();
		Dart f4 = add_tri_topo_fp();

		phi2_sew(f1,f4);
		phi2_sew(f2,f3);
		phi2_sew(phi1(f1),phi_1(f2));
		phi2_sew(phi1(f3),phi_1(f4));

		phi2_sew(phi_1(f1),e2);
		phi2_sew(phi1(f2),e1);
		phi2_sew(phi_1(f3),e4);
		phi2_sew(phi1(f4),e3);

		return f2;
	}

public:

	/**
	 * @brief cut an edge and the two incident triangles
	 * @param e the edge to cut
	 * @return the created vertex
	 */
	Vertex cut_edge(Edge e)
	{
		Vertex nv(cut_edge_topo(e));

		if (this->template is_embedded<CDart>())
		{
			foreach_incident_face(nv, [this] (Face nf)
			{
				foreach_dart_of_orbit(nf, [this] (Dart d)
				{
					this->new_orbit_embedding(CDart(d));
				});
			});
		}

		if (this->template is_embedded<Vertex>())
		{
			this->new_orbit_embedding(Vertex(nv));

			foreach_incident_edge(nv, [this] (Edge ne)
			{
				Dart v1 = phi2(ne.dart); // new dart of existing vertex
				Dart v2 = phi1(ne.dart); // new dart of existing vertex
				Dart v0 = phi2(phi_1(v1)); // old dart of existing vertex
				this->template copy_embedding<Vertex>(v1,v0);
				this->template copy_embedding<Vertex>(v2,v0);
			});
		}

		if (this->template is_embedded<Edge>())
		{
			foreach_incident_edge(nv, [this] (Edge ne)
			{
				this->new_orbit_embedding(ne); // new edge
				Dart ne2 = phi1(ne.dart); // new dart of existing edge
				this->template copy_embedding<Edge>(ne2, phi2(ne2));
			});
		}

		if (this->template is_embedded<Face>())
		{
			foreach_incident_face(nv, [this] (Face nf)
			{
				this->new_orbit_embedding(nf);
			});
		}

		if (this->template is_embedded<Volume>())
		{
			uint32 emb = this->embedding(Volume(phi<12>(nv.dart)));
			foreach_incident_face(nv, [this, emb] (Face nf)
			{
				this->template set_orbit_embedding<Volume>(nf, emb);
			});
		}

		return nv;
	}

protected:
	/**
	 * @brief split a triangle in 3 triangles (TOPO ONLY)
	 * @param f the triangle to split
	 * @return a dart of central vertex
	 */
	Vertex split_triangle_topo(Face f)
	{
		Dart e1 = phi2(f.dart);
		Dart e2 = phi2(phi1(f.dart));
		Dart e3 = phi2(phi_1(f.dart));

#ifndef	NDEBUG
		phi2_unsew(e1);
		phi2_unsew(e2);
		phi2_unsew(e3);

#endif

		Dart f1 = add_tri_topo_fp();
		Dart f2 = add_tri_topo_fp();
		Dart f3 = add_tri_topo_fp();

		phi2_sew(phi1(f1),phi_1(f2));
		phi2_sew(phi1(f2),phi_1(f3));
		phi2_sew(phi1(f3),phi_1(f1));

		phi2_sew(e1,f1);
		phi2_sew(e2,f2);
		phi2_sew(e3,f3);
		remove_tri_topo_fp(f.dart);

		return Vertex(phi_1(f1));
	}

public:

	/**
	 * @brief split a triangle in 3 triangles
	 * @param f
	 * @return centroid vertex inserted
	 */
	Vertex split_triangle(Face f)
	{
		Vertex vc = split_triangle_topo(f);

		if (this->template is_embedded<CDart>())
		{
			foreach_incident_face(vc, [this] (Face nf)
			{
				foreach_dart_of_orbit(nf, [this] (Dart d)
				{
					this->new_orbit_embedding(CDart(d));
				});
			});
		}

		if (this->template is_embedded<Vertex>())
		{
			this->new_orbit_embedding(Vertex(vc));

			foreach_incident_edge(vc, [this] (Edge ne)
			{
				Dart v1 = phi2(ne.dart); // new dart of exiting vertex
				Dart v2 = phi1(ne.dart); // new dart of exiting vertex
				Dart v0 = phi2(phi_1(v1)); // old dart of exiting vertex
				this->template copy_embedding<Vertex>(v1,v0);
				this->template copy_embedding<Vertex>(v2,v0);
			});
		}

		if (this->template is_embedded<Edge>())
		{
			foreach_incident_edge(vc, [this] (Edge ne)
			{
				this->new_orbit_embedding(ne); // new edge
				Dart ne2 = phi1(ne.dart); // new dart of existing dart
				this->template copy_embedding<Edge>(ne2, phi2(ne2));
			});
		}

		if (this->template is_embedded<Face>())
		{
			foreach_incident_face(vc, [this] (Face nf)
			{
				this->new_orbit_embedding(nf);
			});
		}

		if (this->template is_embedded<Volume>())
		{
			uint32 emb = this->embedding(Volume(this->template phi<12>(vc.dart)));
			foreach_incident_face(vc, [this, emb] (Face nf)
			{
				this->template set_orbit_embedding<Volume>(nf, emb);
			});
		}

		return vc;
	}

protected:

	/**
	 * @brief Close the topological hole that contains Dart d (a fixed point for PHI2) with a fan
	 * @param d : a dart of the hole
	 * @return a dart of one of the faces that closes the hole
	 */
	inline Dart close_hole_topo(Dart d)
	{
		cgogn_message_assert(phi2(d) == d, "CMap2Tri: close hole called on a dart that is not a phi2 fix point");

		Dart first = add_tri_topo_fp();	// First edge of the face that will fill the hole
		phi2_sew(d, first);				// 2-sew the new edge to the hole

		Dart prec_tri = first;
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
				Dart tri = add_tri_topo_fp();
				phi2_sew(d_next, tri);
				phi2_sew(phi_1(prec_tri), phi1(tri));
				prec_tri = tri;
			}
		} while (d_phi1 != d);

		phi2_sew(phi_1(prec_tri), phi1(first));

		return first;
	}

	/**
	 * @brief Close a hole with a triangle fan
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
	 * \brief close_map
	 * \return the number of holes (filled)
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
				Vertex fan_center(phi_1(f.dart));
				foreach_incident_face(fan_center, [&] (Face ff)
				{
					this->boundary_mark(ff);
				});
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
		return 3;
	}

	inline uint32 degree(Face) const
	{
		return 1;
	}

	inline bool has_codegree(Face, uint32 codegree) const
	{
		return codegree == 3;
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
		uint32 first = (d.index/3)*3;
		f(Dart(first));
		f(Dart(first+1));
		f(Dart(first+2));
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

struct CMap2TriType
{
	using TYPE = CMap2Tri_T<CMap2TriType>;
};

using CMap2Tri = CMap2Tri_T<CMap2TriType>;

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_CMAP_CMAP2_TRI_CPP_))
extern template class CGOGN_CORE_API CMap2Builder_T<CMap2Tri>;
extern template class CGOGN_CORE_API DartMarker<CMap2Tri>;
extern template class CGOGN_CORE_API DartMarkerStore<CMap2Tri>;
extern template class CGOGN_CORE_API DartMarkerNoUnmark<CMap2Tri>;
extern template class CGOGN_CORE_API CellMarker<CMap2Tri, CMap2Tri::Vertex::ORBIT>;
extern template class CGOGN_CORE_API CellMarker<CMap2Tri, CMap2Tri::Edge::ORBIT>;
extern template class CGOGN_CORE_API CellMarker<CMap2Tri, CMap2Tri::Face::ORBIT>;
extern template class CGOGN_CORE_API CellMarker<CMap2Tri, CMap2Tri::Volume::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerNoUnmark<CMap2Tri, CMap2Tri::Vertex::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerNoUnmark<CMap2Tri, CMap2Tri::Edge::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerNoUnmark<CMap2Tri, CMap2Tri::Face::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerNoUnmark<CMap2Tri, CMap2Tri::Volume::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap2Tri, CMap2Tri::Vertex::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap2Tri, CMap2Tri::Edge::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap2Tri, CMap2Tri::Face::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap2Tri, CMap2Tri::Volume::ORBIT>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_MAP_MAP2_CPP_))

} // namespace cgogn

#endif // CGOGN_CORE_CMAP_CMAP2_TRI_H_
