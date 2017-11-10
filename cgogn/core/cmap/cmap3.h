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

#ifndef CGOGN_CORE_CMAP_CMAP3_H_
#define CGOGN_CORE_CMAP_CMAP3_H_

#include <cgogn/core/cmap/cmap2.h>
#include <cgogn/core/cmap/cmap3_builder.h>

namespace cgogn
{

template <typename MAP_TYPE>
class CMap3_T : public CMap2_T<MAP_TYPE>
{
public:

	static const uint8 DIMENSION = 3;
	static const uint8 PRIM_SIZE = 1;

	using MapType = MAP_TYPE;
	using Inherit = CMap2_T<MAP_TYPE>;
	using Self = CMap3_T<MAP_TYPE>;

	using Builder = CMap3Builder_T<Self>;

	friend class MapBase<MAP_TYPE>;
	friend class CMap3Builder_T<Self>;
	friend class DartMarker_T<Self>;
	friend class cgogn::DartMarkerStore<Self>;

	using CDart   = typename Inherit::CDart;
	using Vertex2 = typename Inherit::Vertex;
	using Vertex  = Cell<Orbit::PHI21_PHI31>;
	using Edge2   = typename Inherit::Edge;
	using Edge    = Cell<Orbit::PHI2_PHI3>;
	using Face2   = typename Inherit::Face;
	using Face    = Cell<Orbit::PHI1_PHI3>;
	using Volume  = typename Inherit::Volume;

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
	template <typename T>
	using CCAttribute = Attribute<T, ConnectedComponent::ORBIT>;

	using DartMarker = typename cgogn::DartMarker<Self>;
	using DartMarkerStore = typename cgogn::DartMarkerStore<Self>;

	template <Orbit ORBIT>
	using CellMarker = typename cgogn::CellMarker<Self, ORBIT>;
	template <Orbit ORBIT>
	using CellMarkerNoUnmark = typename cgogn::CellMarkerNoUnmark<Self, ORBIT>;
	template <Orbit ORBIT>
	using CellMarkerStore = typename cgogn::CellMarkerStore<Self, ORBIT>;

	using FilteredQuickTraversor = typename cgogn::FilteredQuickTraversor<Self>;
	using QuickTraversor = typename cgogn::QuickTraversor<Self>;
	using CellCache = typename cgogn::CellCache<Self>;
	using BoundaryCache = typename cgogn::BoundaryCache<Self>;

protected:

	ChunkArray<Dart>* phi3_;

	inline void init()
	{
		phi3_ = this->topology_.template add_chunk_array<Dart>("phi3");
	}

public:

	CMap3_T() : Inherit()
	{
		init();
	}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CMap3_T);

	~CMap3_T() override
	{}

	/*!
	 * \brief Check the integrity of embedding information
	 */
	inline bool check_embedding_integrity()
	{
		CGOGN_CHECK_CONCRETE_TYPE;

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
	* \brief Init an newly added dart.
	* The dart is defined as a fixed point for PHI3.
	*/
	inline void init_dart(Dart d)
	{
		Inherit::init_dart(d);
		(*phi3_)[d.index] = d;
	}

	/**
	 * @brief Check the integrity of a dart
	 * @param d the dart to check
	 * @return true if the integrity constraints are locally statisfied
	 * PHI3_PHI1 should be an involution without fixed point and
	 */
	inline bool check_integrity(Dart d) const
	{
		return (Inherit::check_integrity(d) &&
				phi3(phi3(d)) == d &&
				phi3(d) != d &&
				phi3(this->phi1(phi3(this->phi1(d)))) == d &&
				( this->is_boundary(d) == this->is_boundary(this->phi2(d)) ));
	}

	/**
	 * @brief Check the integrity of a boundary dart
	 * @param d the dart to check
	 * @return true if the bondary constraints are locally statisfied
	 * The boundary is a 2-manyfold: the boundary marker is the same
	 * for all darts of a face and for two adjacent faces.
	 */
	inline bool check_boundary_integrity(Dart d) const
	{
		return (( this->is_boundary(d) == this->is_boundary(this->phi1(d))  ) &&
				( this->is_boundary(d) == this->is_boundary(this->phi2(d)) ));
	}

	/**
	 * \brief Link dart d with dart e by an involution
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
	 * \brief phi composition
	 * @param d
	 * @return applied composition of phi in order of declaration : phi<123>(d) = phi3(phi2(phi1(d)))
	 */
	template <uint64 N>
	inline Dart phi(Dart d) const
	{
		static_assert((N % 10) <= 3, "Composition of PHI: invalid index (phi1/phi2/phi3 only)");
		switch (N%10)
		{
			case 1 : return this->phi1(phi<N/10>(d));
			case 2 : return this->phi2(phi<N/10>(d));
			case 3 : return this->phi3(phi<N/10>(d));
			default : return d;
		}
	}

	/*******************************************************************************
	 * High-level embedded and topological operations
	 *******************************************************************************/

protected:

	/**
	 * @brief add_pyramid_topo_fp adds a new pyramid whose base is composed of size sides
	 * Darts are fixed points of phi3 relation
	 * @param size number of sides of the base of the pyramid
	 * @return a dart of the base face
	 */
	inline Dart add_pyramid_topo_fp(std::size_t size)
	{
		return Inherit::add_pyramid_topo(size);
	}

	/**
	 * @brief add_prism_topo_fp adds a new prism whose base is composed of size sides
	 * Darts are fixed points of phi3 relation
	 * @param size number of sides of the base of the prism
	 * @return a dart of the base face
	 */
	inline Dart add_prism_topo_fp(std::size_t size)
	{
		return Inherit::add_prism_topo(size);
	}

	/**
	 * @brief add_stamp_volume_topo_fp : a flat volume with one face composed of two triangles and another composed of one quad
	 * Darts are fixed points of phi3 relation
	 * @return a dart of the quad
	 */
	inline Dart add_stamp_volume_topo_fp()
	{
		const Dart d_quad = Inherit::Inherit::add_face_topo(4u);
		const Dart d_tri1 = Inherit::Inherit::add_face_topo(3u);
		const Dart d_tri2 = Inherit::Inherit::add_face_topo(3u);

		this->phi2_sew(d_tri1, d_tri2);
		this->phi2_sew(d_quad, this->phi1(d_tri1));
		this->phi2_sew(this->phi1(d_quad), this->phi_1(d_tri2));
		this->phi2_sew(this->phi1(this->phi1(d_quad)), this->phi1(d_tri2));
		this->phi2_sew(this->phi_1(d_quad), this->phi_1(d_tri1));

		return d_quad;
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
	 * \brief Cut an edge.
	 * \param d : A dart that represents the edge to cut
	 * \return A dart of the inserted vertex
	 * The edge of d is cut by inserting a new vertex.
	 * The returned dart is the dart of the inserted vertex that belongs to the face of d.
	 */
	inline Dart cut_edge_topo(Dart d)
	{
		Dart prev = d;
		Dart d23 = phi3(this->phi2(d));

		const Dart nd = Inherit::cut_edge_topo(d);

		while (d23 != d)
		{
			prev = d23;
			d23 = phi3(this->phi2(d23));

			Inherit::cut_edge_topo(prev);

			const Dart d3 = phi3(prev);
			phi3_unsew(prev);
			phi3_sew(prev, this->phi1(d3));
			phi3_sew(d3, this->phi1(prev));
		}

		const Dart d3 = phi3(d);
		phi3_unsew(d);
		phi3_sew(d, this->phi1(d3));
		phi3_sew(d3, this->phi1(d));

		return nd;
	}

public:

	/**
	 * \brief Cut an edge.
	 * \param e : the edge to cut
	 * \return The inserted vertex
	 * The edge e is cut by inserting a new vertex.
	 * The returned vertex is represented by the dart of the inserted vertex that belongs to the face of e.
	 * If the map has Dart, Vertex2, Vertex, Edge2, Edge, Face2, Face or Volume attributes,
	 * the inserted cells are automatically embedded on new attribute elements.
	 * More precisely :
	 *  - Vertex2 attributes are created, if needed, for each inserted Vertex2.
	 *  - a Vertex attribute is created, if needed, for the inserted vertex.
	 *  - Edge2 attributes are created, if needed, for each inserted Edge2.
	 *  - an Edge attribute is created, if needed, for the edge inserted after e.
	 *  - the Edge attribute of e is kept unchanged.
	 */
	inline Vertex cut_edge(Edge e)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		const Dart v = cut_edge_topo(e.dart);

		if (this->template is_embedded<CDart>())
		{
			foreach_dart_of_PHI23(e.dart, [this] (Dart d)
			{
				Dart nv1 = this->phi1(d);
				Dart nv2 = this->phi2(d);
				if (!is_boundary_cell(CDart(nv1))) this->new_orbit_embedding(CDart(nv1));
				if (!is_boundary_cell(CDart(nv2))) this->new_orbit_embedding(CDart(nv2));
			});
		}

		if (this->template is_embedded<Vertex2>())
		{
			foreach_dart_of_PHI23(e.dart, [this] (Dart d)
			{
				if (!is_boundary_cell(Vertex2(this->phi1(d))))
					this->new_orbit_embedding(Vertex2(this->phi1(d)));
			});
		}

		if (this->template is_embedded<Vertex>())
			this->new_orbit_embedding(Vertex(v));

		if (this->template is_embedded<Edge2>())
		{
			foreach_dart_of_PHI23(e.dart, [this] (Dart d)
			{
				if (!is_boundary_cell(Edge2(d)))
				{
					this->template copy_embedding<Edge2>(this->phi2(d), d);
					this->new_orbit_embedding(Edge2(this->phi1(d)));
				}
			});
		}

		if (this->template is_embedded<Edge>())
		{
			foreach_dart_of_PHI23(e.dart, [this] (Dart d)
			{
				this->template copy_embedding<Edge>(this->phi2(d), d);
			});
			this->new_orbit_embedding(Edge(v));
		}

		if (this->template is_embedded<Face2>())
		{
			foreach_dart_of_PHI23(e.dart, [this] (Dart d)
			{
				if (!is_boundary_cell(Face2(d)))
				{
					this->template copy_embedding<Face2>(this->phi1(d), d);
					this->template copy_embedding<Face2>(this->phi2(d), this->phi2(this->phi1(d)));
				}
			});
		}

		if (this->template is_embedded<Face>())
		{
			foreach_dart_of_PHI23(e.dart, [this] (Dart d)
			{
				this->template copy_embedding<Face>(this->phi1(d), d);
				this->template copy_embedding<Face>(phi3(d), d);
			});
		}

		if (this->template is_embedded<Volume>())
		{
			foreach_dart_of_PHI23(e.dart, [this] (Dart d)
			{
				if (!is_boundary_cell(Volume(d)))
				{
					this->template copy_embedding<Volume>(this->phi1(d), d);
					this->template copy_embedding<Volume>(this->phi2(d), d);
				}
			});
		}

		return Vertex(v);
	}

protected:

	inline bool flip_edge_topo(Dart e)
	{
		if (this->degree(Edge(e)) == 2u)
			return Inherit::flip_edge_topo(e) && Inherit::flip_back_edge_topo(phi3(e));
		else
			return false;
	}

public:

	/**
	 * @brief Flip an Edge (rotation in phi1 order)
	 * @param e : the edge to flip
	 * The edge has to be incident to exactly 2 faces.
	 */
	inline bool flip_edge(Edge e)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		if (!flip_edge_topo(e.dart))
			return false;

		const Dart e2 = this->phi2(e.dart);
		const Dart e3 = phi3(e.dart);
		const Dart e32 = this->phi2(e3);

		if (this->is_embedded(Vertex2::ORBIT))
		{
			if (!is_boundary_cell(Vertex2(e.dart)))
			{
				this->template copy_embedding<Vertex2>(e.dart, this->phi1(e2));
				this->template copy_embedding<Vertex2>(e2, this->phi1(e.dart));
			}
			if (!is_boundary_cell(Vertex2(e3)))
			{
				this->template copy_embedding<Vertex2>(e3, this->phi1(e32));
				this->template copy_embedding<Vertex2>(e32, this->phi1(e3));
			}
		}

		if (this->is_embedded(Vertex::ORBIT))
		{
			this->template copy_embedding<Vertex>(e.dart, this->phi1(e2));
			this->template copy_embedding<Vertex>(e2, this->phi1(e.dart));
			this->template copy_embedding<Vertex>(e3, this->phi1(e32));
			this->template copy_embedding<Vertex>(e32, this->phi1(e3));
		}

		if (this->is_embedded(Face2::ORBIT))
		{
			if (!is_boundary_cell(Face2(e.dart)))
			{
				this->template copy_embedding<Face2>(this->phi_1(e.dart), e.dart);
				this->template copy_embedding<Face2>(this->phi_1(e2), e2);
			}
			if (!is_boundary_cell(Face2(e3)))
			{
				this->template copy_embedding<Face2>(this->phi1(e3), e3);
				this->template copy_embedding<Face2>(this->phi1(e32), e32);
			}
		}

		if (this->is_embedded(Face::ORBIT))
		{
			this->template copy_embedding<Face>(this->phi_1(e.dart), e.dart);
			this->template copy_embedding<Face>(this->phi_1(e2), e2);
			this->template copy_embedding<Face>(this->phi1(e3), e3);
			this->template copy_embedding<Face>(this->phi1(e32), e32);
		}

		return true;
	}

protected:

	inline bool flip_back_edge_topo(Dart e)
	{
		if (this->degree(Edge(e)) == 2u)
			return Inherit::flip_back_edge_topo(e) && Inherit::flip_edge_topo(phi3(e));
		else
			return false;
	}

public:

	/**
	 * @brief Flip an Edge (rotation in phi_1 order)
	 * @param e : the edge to flip
	 * The edge has to be incident to exactly 2 faces.
	 */
	inline bool flip_back_edge(Edge e)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		if (!flip_back_edge_topo(e.dart))
			return false;

		const Dart e2 = this->phi2(e.dart);
		const Dart e3 = phi3(e.dart);
		const Dart e32 = this->phi2(e3);

		if (this->is_embedded(Vertex2::ORBIT))
		{
			if (!is_boundary_cell(Vertex2(e.dart)))
			{
				this->template copy_embedding<Vertex2>(e.dart, this->phi1(e2));
				this->template copy_embedding<Vertex2>(e2, this->phi1(e.dart));
			}
			if (!is_boundary_cell(Vertex2(e3)))
			{
				this->template copy_embedding<Vertex2>(e3, this->phi1(e32));
				this->template copy_embedding<Vertex2>(e32, this->phi1(e3));
			}
		}

		if (this->is_embedded(Vertex::ORBIT))
		{
			this->template copy_embedding<Vertex>(e.dart, this->phi1(e2));
			this->template copy_embedding<Vertex>(e2, this->phi1(e.dart));
			this->template copy_embedding<Vertex>(e3, this->phi1(e32));
			this->template copy_embedding<Vertex>(e32, this->phi1(e3));
		}

		if (this->is_embedded(Face2::ORBIT))
		{
			if (!is_boundary_cell(Face2(e.dart)))
			{
				this->template copy_embedding<Face2>(this->phi1(e.dart), e.dart);
				this->template copy_embedding<Face2>(this->phi1(e2), e2);
			}
			if (!is_boundary_cell(Face2(e3)))
			{
				this->template copy_embedding<Face2>(this->phi_1(e3), e3);
				this->template copy_embedding<Face2>(this->phi_1(e32), e32);
			}
		}

		if (this->is_embedded(Face::ORBIT))
		{
			this->template copy_embedding<Face>(this->phi1(e.dart), e.dart);
			this->template copy_embedding<Face>(this->phi1(e2), e2);
			this->template copy_embedding<Face>(this->phi_1(e3), e3);
			this->template copy_embedding<Face>(this->phi_1(e32), e32);
		}

		return true;
	}

protected:

	// TODO: write test in cmap3_topo_test
	Dart split_vertex_topo(std::vector<Dart>& vd)
	{
		Dart prev = vd.front();	//elt 0

		Dart db1;
		if (this->is_incident_to_boundary(Face(prev)))
			db1 = this->phi2(phi3(this->phi1(this->phi2(prev))));

		this->Inherit::split_vertex_topo(prev, this->phi2(this->phi_1(this->phi2(this->phi_1(prev)))));

		for (uint32 i = 1; i < vd.size(); ++i)
		{
			prev = vd[i];
			const Dart fs = this->phi_1(this->phi2(this->phi_1(prev)));	//first side
			this->Inherit::split_vertex_topo(prev, this->phi2(fs));
			const Dart d1 = this->phi_1(this->phi2(this->phi_1(vd[i-1])));
			const Dart d2 = this->phi1(this->phi2(vd[i]));

			phi3_sew(d1, d2);
		}

		Dart db2;
		if (this->is_incident_to_boundary(Face(this->phi2(this->phi_1(prev)))))
			db2 = this->phi2(phi3(this->phi2(this->phi_1(prev))));

		if (!db1.is_nil() && !db2.is_nil())
		{
			this->Inherit::split_vertex_topo(db1, db2);
			phi3_sew(this->phi1(this->phi2(db2)), this->phi_1(phi3(this->phi2(db2))));
			phi3_sew(this->phi1(this->phi2(db1)), this->phi_1(phi3(this->phi2(db1))));
		}
		else
		{
			Dart dbegin = this->phi1(this->phi2(vd.front()));
			Dart dend = this->phi_1(this->phi2(this->phi_1(vd.back())));
			phi3_sew(dbegin, dend);
		}

		return this->phi_1(this->phi2(this->phi_1(prev)));
	}

public:

	inline Dart split_vertex(std::vector<Dart>& vd)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		const Dart d1 = vd.front();
		const Dart d2 = this->phi1(this->phi2(d1));
		const Dart res = split_vertex_topo(vd);

		if (this->template is_embedded<CDart>())
		{
			// TODO ...
			cgogn_log_debug("CMap3::split_vertex") << "the CDart embeddings are not updated.";
		}

		if (this->template is_embedded<Vertex2>())
		{
			// TODO ...
			cgogn_log_debug("CMap3::split_vertex") << "the Vertex2 embeddings are not updated.";
		}

		if (this->template is_embedded<Vertex>())
		{
			this->new_orbit_embedding(Vertex(d2));
			this->template set_orbit_embedding<Vertex>(Vertex(d1), this->embedding(Vertex(d1)));
		}

		if (this->template is_embedded<Edge>())
			this->new_orbit_embedding(Edge(res));

		if (this->template is_embedded<Face2>())
		{
			// TODO ...
			cgogn_log_debug("CMap3::split_vertex") << "the Face2 embeddings are not updated.";
		}

		if (this->template is_embedded<Face>())
		{
			// TODO ...
			cgogn_log_debug("CMap3::split_vertex") << "the Face embeddings are not updated.";
		}

		if (this->template is_embedded<Volume>())
		{
			for (auto dit1 : vd)
				this->template set_orbit_embedding<Volume>(Volume(dit1), this->embedding(Volume(dit1)));
		}

		return res;
	}

protected:

	/**
	 * \brief Cut the face of d and e by inserting an edge between the vertices of d and e
	 * \param d : first vertex
	 * \param e : second vertex
	 * \return A dart of the inserted edge
	 * Darts d and e should belong to the same Face2 and be distinct from each other.
	 * An edge made of four new darts is inserted between the two given vertices.
	 * The returned dart is the dart of the inserted edge that belongs to the Face2 of d.
	 */
	inline Dart cut_face_topo(Dart d, Dart e)
	{
		cgogn_message_assert(d != e, "cut_face_topo: d and e should be distinct");
		cgogn_message_assert(this->same_cell(Face2(d), Face2(e)), "cut_face_topo: d and e should belong to the same Face2");

		Dart dd = this->phi1(phi3(d));
		Dart ee = this->phi1(phi3(e));

		Dart nd = Inherit::cut_face_topo(d, e);
		Dart ndd = Inherit::cut_face_topo(dd, ee);

		phi3_sew(nd, this->phi_1(ee));
		phi3_sew(ndd, this->phi_1(e));

		return nd;
	}

public:

	/**
	 * \brief Cut a face by inserting an edge between the vertices of d and e
	 * \param d : a dart of the first vertex
	 * \param e : a dart of the second vertex
	 * \return The inserted edge
	 * The darts d and e should belong to the same Face2 and be distinct from each other.
	 * An edge is inserted between the two given vertices.
	 * The returned edge is represented by the dart of the inserted edge that belongs to the Face2 of d.
	 * If the map has Dart, Vertex2, Vertex, Edge2, Edge, Face2, Face or Volume attributes,
	 * the inserted cells are automatically embedded on new attribute elements.
	 * More precisely :
	 *  - two Edge2 attribute are created, if needed, for the inserted Edge2.
	 *  - an Edge attribute is created, if needed, for the inserted edge.
	 *  - two Face2 attributes are created, if needed, for the subdivided Face2 of e and phi3(e).
	 *  - a Face attribute is created, if needed, for the subdivided face that e belongs to.
	 *  - the Face attribute of the subdivided face that d belongs to is kept unchanged.
	 */
	inline Edge cut_face(Dart d, Dart e)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		Dart nd = cut_face_topo(d, e);
		Dart ne = this->phi_1(e);
		Dart nd3 = phi3(nd);
		Dart ne3 = phi3(ne);

		if (this->template is_embedded<CDart>())
		{
			if (!is_boundary_cell(CDart(nd))) this->new_orbit_embedding(CDart(nd));
			if (!is_boundary_cell(CDart(ne))) this->new_orbit_embedding(CDart(ne));
			if (!is_boundary_cell(CDart(nd3))) this->new_orbit_embedding(CDart(nd3));
			if (!is_boundary_cell(CDart(ne3))) this->new_orbit_embedding(CDart(ne3));
		}

		if (this->template is_embedded<Vertex2>())
		{
			if (!is_boundary_cell(Vertex2(nd)))
			{
				this->template copy_embedding<Vertex2>(nd, e);
				this->template copy_embedding<Vertex2>(ne, d);
			}
			if (!is_boundary_cell(Vertex2(nd3)))
			{
				this->template copy_embedding<Vertex2>(nd3, this->phi1(ne3));
				this->template copy_embedding<Vertex2>(ne3, this->phi1(nd3));
			}
		}

		if (this->template is_embedded<Vertex>())
		{
			this->template copy_embedding<Vertex>(nd, e);
			this->template copy_embedding<Vertex>(ne3, e);
			this->template copy_embedding<Vertex>(ne, d);
			this->template copy_embedding<Vertex>(nd3, d);
		}

		if (this->template is_embedded<Edge2>())
		{
			if (!is_boundary_cell(Edge2(nd)))
				this->new_orbit_embedding(Edge2(nd));
			if (!is_boundary_cell(Edge2(nd3)))
				this->new_orbit_embedding(Edge2(nd3));
		}

		if (this->template is_embedded<Edge>())
			this->new_orbit_embedding(Edge(nd));

		if (this->template is_embedded<Face2>())
		{
			if (!is_boundary_cell(Face2(nd)))
			{
				this->template copy_embedding<Face2>(nd, d);
				this->new_orbit_embedding(Face2(ne));
			}
			if (!is_boundary_cell(Face2(nd3)))
			{
				this->template copy_embedding<Face2>(nd3, phi3(d));
				this->new_orbit_embedding(Face2(ne3));
			}
		}

		if (this->template is_embedded<Face>())
		{
			this->template copy_embedding<Face>(nd, d);
			this->template copy_embedding<Face>(nd3, d);
			this->new_orbit_embedding(Face(ne));
		}

		if (this->template is_embedded<Volume>())
		{
			if (!is_boundary_cell(Volume(d)))
			{
				this->template copy_embedding<Volume>(nd, d);
				this->template copy_embedding<Volume>(ne, d);
			}
			Dart d3 = phi3(d);
			if (!is_boundary_cell(Volume(d3)))
			{
				this->template copy_embedding<Volume>(nd3, d3);
				this->template copy_embedding<Volume>(ne3, d3);
			}
		}

		return Edge(nd);
	}

protected:

	bool merge_incident_faces_topo(Dart d)
	{
		if (this->degree(Edge(d)) != 2u)
			return false;

		const Dart d3 = phi3(d);

		phi3_unsew(d);

		Dart d2 = this->phi2(d);
		this->phi2_unsew(d);

		this->phi1_sew(this->phi_1(d), d2);
		this->phi1_sew(this->phi_1(d2), d);
		this->Inherit::Inherit::remove_face_topo(d);

		d2 = this->phi2(d3);
		this->phi2_unsew(d3);

		this->phi1_sew(this->phi_1(d3), d2);
		this->phi1_sew(this->phi_1(d2), d3);
		this->Inherit::Inherit::remove_face_topo(d3);

		return true;
	}

public:

	/**
	 * \brief Merge the two faces incident to the given edge
	 * \param e : the edge
	 * The edge has to be incident to exactly 2 faces.
	 */
	bool merge_incident_faces(Edge e)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		const Dart f = this->phi1(e.dart);

		if (merge_incident_faces_topo(e.dart))
		{
			if (this->template is_embedded<Face2>())
			{
				if (!is_boundary_cell(Face2(f)))
					this->template set_orbit_embedding<Face2>(Face2(f), this->embedding(Face2(f)));

				const Dart f3 = phi3(f);
				if (!is_boundary_cell(Face2(f3)))
					this->template set_orbit_embedding<Face2>(Face2(f3), this->embedding(Face2(f3)));
			}

			if (this->template is_embedded<Face>())
				this->template set_orbit_embedding<Face>(Face(f), this->embedding(Face(f)));

			return true;
		}

		return false;
	}

protected:

	Dart merge_incident_volumes_of_edge_topo(Dart d)
	{
		Dart res;
		if (degree(Vertex(d)) == 2u || degree(Vertex(this->phi1(d))) == 2u || this->is_incident_to_boundary(Edge(d)))
			return res;

		Dart dit = d;
		do
		{
			Dart fit = dit;
			Dart end = fit;
			fit = this->phi1(fit);
			while (fit != end)
			{
				Dart d2 = this->phi2(fit);
				Dart d3 = phi3(fit);
				Dart d32 = this->phi2(d3);

				if (res.is_nil())
					res = d2;

				this->phi2_unsew(d2);
				this->phi2_unsew(d32);
				this->phi2_sew(d2, d32);
				this->phi2_sew(fit, d3);

				fit = this->phi1(fit);
			}
			dit = phi3(this->phi2(dit));
		} while(dit != d);

		{ // removing the darts
			std::vector<Dart>* darts_to_be_deleted = cgogn::dart_buffers()->buffer();
			this->foreach_dart_of_orbit(typename Inherit::ConnectedComponent(d), [=] (Dart it)
			{
				darts_to_be_deleted->push_back(it);
			});
			for (Dart it : *darts_to_be_deleted)
				this->remove_topology_element(it);
			cgogn::dart_buffers()->release_buffer(darts_to_be_deleted);
		}

		return res;
	}

public:

	/**
	 * \brief Merge the volumes incident to the edge e by removing the edge
	 * \param e : the edge
	 * \return true if the volumes have been merged, false otherwise
	 * If the edge is incident to the boundary, nothing is done.
	 */
	Dart merge_incident_volumes(Edge e)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		const Dart res_topo_del = merge_incident_volumes_of_edge_topo(e.dart);
		if (res_topo_del.is_nil())
			return res_topo_del;

		if (this->template is_embedded<Vertex2>())
		{
			// TODO ...
			cgogn_log_debug("CMap3::merge_incident_volumes(Edge)") << "the Vertex2 embeddings are not updated.";
		}

		if (this->template is_embedded<Edge2>())
		{
			// TODO ...
			cgogn_log_debug("CMap3::merge_incident_volumes(Edge)") << "the Edge2 embeddings are not updated.";
		}

		if (this->template is_embedded<Volume>())
			this->template set_orbit_embedding<Volume>(Volume(res_topo_del), this->embedding(Volume(res_topo_del)));

		return res_topo_del;
	}

protected:

	bool merge_incident_volumes_of_face_topo(Dart d)
	{
		if (this->is_incident_to_boundary(Face(d)))
			return false;

		Dart f = d;
		do
		{
			Dart ff = phi3(f);
			Dart f2 = this->phi2(f);
			Dart ff2 = this->phi2(ff);
#ifndef	NDEBUG
			this->phi2_unsew(f);
			this->phi2_unsew(ff);
#endif
			this->phi2_sew(f2, ff2);
			f = this->phi1(f);
		} while (f != d);

		Dart d3 = phi3(d);
		this->Inherit::Inherit::remove_face_topo(d);
		this->Inherit::Inherit::remove_face_topo(d3);

		return true;
	}

public:

	/**
	 * \brief Merge the volumes incident to the face f by removing the face
	 * \param f : the face
	 * \return true if the volumes have been merged, false otherwise
	 * If the face is incident to the boundary, nothing is done.
	 */
	bool merge_incident_volumes(Face f)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		std::vector<Dart>* face_path = cgogn::dart_buffers()->buffer();
		foreach_dart_of_orbit(f, [face_path, this] (Dart d)
		{
			face_path->push_back(this->phi2(d));
		});

		if (merge_incident_volumes_of_face_topo(f.dart))
		{
			if (this->template is_embedded<Vertex2>())
			{
				for (Dart d : *face_path)
					this->template set_orbit_embedding<Vertex2>(Vertex2(d), this->embedding(Vertex2(d)));
			}

			if (this->template is_embedded<Edge2>())
			{
				for (Dart d : *face_path)
					this->template copy_embedding<Edge2>(this->phi2(d), d);
			}

			if (this->template is_embedded<Volume>())
			{
				Dart d = face_path->front();
				this->template set_orbit_embedding<Volume>(Volume(d), this->embedding(Volume(d)));
			}

			return true;
		}

		return false;
	}

protected:

	/**
	 * @brief Cut a single volume following a simple closed oriented path
	 * @param path a vector of darts representing the path
	 * @return a dart of the inserted face
	 */
	Dart cut_volume_topo(const std::vector<Dart>& path)
	{
		cgogn_message_assert(this->simple_closed_oriented_path(path), "cut_volume_topo: the given path should be a simple closed oriented path");

		Dart face1 = Inherit::Inherit::add_face_topo(path.size());
		Dart face2 = Inherit::Inherit::add_face_topo(path.size());

		for (Dart d : path)
		{
			Dart d2 = this->phi2(d);
			this->phi2_unsew(d);

			this->phi2_sew(d, face1);
			this->phi2_sew(d2, face2);

			phi3_sew(face1, face2);

			face1 = this->phi_1(face1);
			face2 = this->phi1(face2);
		}

		return this->phi_1(face1);
	}

public:

	/**
	 * @brief Cut a single volume following a simple closed oriented path
	 * @param path a vector of darts representing the path
	 * @return the inserted face
	 */
	Face cut_volume(const std::vector<Dart>& path)
	{
		CGOGN_CHECK_CONCRETE_TYPE;
		cgogn_message_assert(!this->is_boundary_cell(Volume(path[0])), "cut_volume: should not cut a boundary volume");

		Dart nf = cut_volume_topo(path);

		if (this->template is_embedded<CDart>())
		{
			foreach_dart_of_orbit(Face2(nf), [this] (Dart d)
			{
				this->new_orbit_embedding(CDart(d));
				this->new_orbit_embedding(CDart(phi3(d)));
			});
		}

		if (this->template is_embedded<Vertex2>())
		{
			foreach_dart_of_orbit(Face2(nf), [this] (Dart d)
			{
				this->template copy_embedding<Vertex2>(d, this->phi1(this->phi2(d)));
				this->new_orbit_embedding(Vertex2(phi3(d)));
			});
		}

		if (this->template is_embedded<Vertex>())
		{
			foreach_dart_of_orbit(Face2(nf), [this] (Dart d)
			{
				this->template copy_embedding<Vertex>(d, this->phi1(this->phi2(d)));
				Dart d3 = phi3(d);
				this->template copy_embedding<Vertex>(d3, this->phi1(this->phi2(d3)));
			});
		}

		if (this->template is_embedded<Edge2>())
		{
			foreach_dart_of_orbit(Face2(nf), [this] (Dart d)
			{
				this->template copy_embedding<Edge2>(d, this->phi2(d));
				this->new_orbit_embedding(Edge2(phi3(d)));
			});
		}

		if (this->template is_embedded<Edge>())
		{
			foreach_dart_of_orbit(Face2(nf), [this] (Dart d)
			{
				this->template copy_embedding<Edge>(d, this->phi2(d));
				this->template copy_embedding<Edge>(phi3(d), this->phi2(d));
			});
		}

		if (this->template is_embedded<Face2>())
		{
			this->new_orbit_embedding(Face2(nf));
			this->new_orbit_embedding(Face2(phi3(nf)));
		}

		if (this->template is_embedded<Face>())
			this->new_orbit_embedding(Face(nf));

		if (this->template is_embedded<Volume>())
		{
			foreach_dart_of_orbit(Face2(nf), [this] (Dart d)
			{
				this->template copy_embedding<Volume>(d, this->phi2(d));
			});
			this->new_orbit_embedding(Volume(phi3(nf)));
		}

		return Face(nf);
	}

protected:

	bool sew_volumes_topo(Dart fa, Dart fb)
	{
		if (
			!this->is_incident_to_boundary(Face(fa)) ||
			!this->is_incident_to_boundary(Face(fb)) ||
			this->codegree(Face(fa)) != this->codegree(Face(fb))
		)
			return false;

		const Dart fa3 = phi3(fa);
		const Dart fb3 = phi3(fb);

		Dart fa_it = fa3;
		Dart fb_it = fb3;
		do
		{
			const Dart fa_it2 = this->phi2(fa_it);
			const Dart fb_it2 = this->phi2(fb_it);
			if (fa_it2 != fb_it)
			{
				this->phi2_unsew(fa_it);
				this->phi2_unsew(fb_it);
				this->phi2_sew(fa_it2, fb_it2);
				this->phi2_sew(fa_it, fb_it);
			}
			phi3_unsew(fa_it);
			phi3_unsew(fb_it);
			fa_it = this->phi1(fa_it);
			fb_it = this->phi_1(fb_it);
		} while(fa_it != fa3);

		{ // removing the darts
			std::vector<Dart>* darts_to_be_deleted = cgogn::dart_buffers()->buffer();
			this->foreach_dart_of_orbit(Volume(fa3), [=] (Dart it)
			{
				darts_to_be_deleted->push_back(it);
			});
			for (Dart it : *darts_to_be_deleted)
				this->remove_topology_element(it);
			cgogn::dart_buffers()->release_buffer(darts_to_be_deleted);
		}

		fa_it = fa;
		fb_it = fb;
		do
		{
			phi3_sew(fa_it, fb_it);
			fa_it = this->phi1(fa_it);
			fb_it = this->phi_1(fb_it);
		} while(fa_it != fa);

		return true;
	}

public:

	/**
	 * @brief Sew the volumes incident to the faces fa and fb
	 * @param fa, fb : the faces
	 * The given faces must have the same codegree and be incident to the boundary
	 */
	void sew_volumes(Face fa, Face fb)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		if (!sew_volumes_topo(fa.dart, fb.dart))
			return;

		if (this->template is_embedded<Vertex>())
		{
			Dart dit = fa.dart;
			do
			{
				this->template set_orbit_embedding<Vertex>(Vertex(dit), this->embedding(Vertex(dit)));
				dit = this->phi1(dit);
			} while (dit != fa.dart);
		}

		if (this->template is_embedded<Edge>())
		{
			Dart dit = fa.dart;
			do
			{
				this->template set_orbit_embedding<Edge>(Edge(dit), this->embedding(Edge(dit)));
				dit = this->phi1(dit);
			} while (dit != fa.dart);
		}

		if (this->template is_embedded<Face>())
			this->template set_orbit_embedding<Face>(fb, this->embedding(fa));
	}

protected:

	// TODO: write test in cmap3_topo_test
	bool unsew_volumes_topo(Dart f)
	{
		if (this->is_incident_to_boundary(Face(f)))
			return false;

		const uint32 nb_edges = this->codegree(Face(f));
		const Dart d3 = phi3(f);

		const Dart b1 = Inherit::Inherit::add_face_topo(nb_edges);
		const Dart b2 = Inherit::Inherit::add_face_topo(nb_edges);

		this->foreach_dart_of_orbit(Face2(b1), [this] (Dart d) { this->set_boundary(d,true); });
		this->foreach_dart_of_orbit(Face2(b2), [this] (Dart d) { this->set_boundary(d,true); });

		Dart fit1 = f;
		Dart fit2 = d3;
		Dart fitB1 = b1;
		Dart fitB2 = b2;
		do
		{
			const Face boundary_face = boundary_face_of_edge(Edge(fit1));
			if (boundary_face.is_valid())
			{
				const Dart f2 = this->phi2(boundary_face.dart);
				this->phi2_unsew(boundary_face.dart);
				this->phi2_sew(fitB1, boundary_face.dart);
				this->phi2_sew(fitB2, f2);
			}
			else
				this->phi2_sew(fitB1, fitB2);

			phi3_unsew(fit1);
			phi3_sew(fit1, fitB1);
			phi3_sew(fit2, fitB2);

			fit1 = this->phi1(fit1);
			fit2 = this->phi_1(fit2);
			fitB1 = this->phi_1(fitB1);
			fitB2 = this->phi1(fitB2);
		} while (fitB1 != b1);

		return true;
	}

public:

	/**
	 * @brief Unsew the volumes incident to the face f
	 * @param f : the face
	 * @return true if the volumes have been unsewn, false otherwise
	 * The two volumes are detached, a 2-faced boundary volume is inserted.
	 * For each of the edges of the face, if it is already incident to a boundary
	 * volume, the new boundary volume is merged with the existing boundary, resulting in a split
	 * of the edge into two edges (vertices can be split in the same process).
	 */
	void unsew_volumes(Face f)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		Dart dd = phi3(this->phi_1(f.dart));
		Dart dit = f.dart;

		if (!this->unsew_volumes_topo(f.dart))
			return;

		do
		{
			if (this->template is_embedded<Vertex>() && !this->same_orbit(Vertex(dit), Vertex(dd)))
				this->new_orbit_embedding(Vertex(dd));

			dd = this->phi_1(dd);

			if (this->template is_embedded<Edge>() && !this->same_orbit(Edge(dit), Edge(dd)))
				this->new_orbit_embedding(Edge(dd));

			dit = this->phi1(dit);
		} while (dit != f.dart);

		if (this->template is_embedded<Face>())
			this->new_orbit_embedding(Face(dd));
	}

protected:

	// TODO: write test in cmap3_topo_test
	void delete_volume_topo(Volume w)
	{
		this->Inherit::foreach_incident_face(w, [&](Face2 f)
		{
			if (!this->is_incident_to_boundary(Face(f.dart)))
				this->unsew_volumes_topo(f.dart);
		});

		{ // removing the darts
			const Volume w3(phi3(w.dart));
			std::vector<Dart>* darts_to_be_deleted = cgogn::dart_buffers()->buffer();
			this->foreach_dart_of_orbit(w, [=] (Dart it) { darts_to_be_deleted->push_back(it); });
			this->foreach_dart_of_orbit(w3, [=] (Dart it) { darts_to_be_deleted->push_back(it); });
			for (Dart it : *darts_to_be_deleted)
				this->remove_topology_element(it);
			cgogn::dart_buffers()->release_buffer(darts_to_be_deleted);
		}
	}

public:

	void delete_volume(Volume w)
	{
		CGOGN_CHECK_CONCRETE_TYPE;
		this->delete_volume_topo(w);
	}

protected:

	/**
	 * @brief close_hole_topo closes the topological hole that contains Dart d (a fixed point of phi3 relation)
	 * @param d a dart incident to the hole
	 * @return a dart of the volume that closes the hole
	 */
	inline Dart close_hole_topo(Dart d)
	{
		cgogn_message_assert(phi3(d) == d, "CMap3: close hole called on a dart that is not a phi3 fix point");

		DartMarkerStore dmarker(*this);
		DartMarkerStore boundary_marker(*this);

		std::vector<Dart> visitedFaces;	// Faces that are traversed
		visitedFaces.reserve(1024u);

		visitedFaces.push_back(d);		// Start with the face of d
		dmarker.mark_orbit(Face2(d));

		uint32 count = 0u;

		// For every face added to the list
		for (uint32 i = 0u; i < visitedFaces.size(); ++i)
		{
			Dart it = visitedFaces[i];
			Dart f = it;

			const Dart b = this->add_face_topo_fp(codegree(Face(f)));
			boundary_marker.mark_orbit(Face2(b));
			++count;

			Dart bit = b;
			do
			{
				Dart e = phi3(this->phi2(f));
				bool found = false;
				do
				{
					if (phi3(e) == e)
					{
						found = true;
						if (!dmarker.is_marked(e))
						{
							visitedFaces.push_back(e);
							dmarker.mark_orbit(Face2(e));
						}
					}
					else
					{
						if (boundary_marker.is_marked(e))
						{
							found = true;
							this->phi2_sew(e, bit);
						}
						else
							e = phi3(this->phi2(e));
					}
				} while(!found);

				phi3_sew(f, bit);
				bit = this->phi_1(bit);
				f = this->phi1(f);
			} while(f != it);
		}

		return phi3(d);
	}

	inline Volume close_hole(Dart d)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		const Volume v(close_hole_topo(d));

		if (this->template is_embedded<Vertex>())
		{
			this->foreach_dart_of_orbit(v, [this] (Dart it)
			{
				this->template copy_embedding<Vertex>(it, this->phi1(phi3(it)));
			});
		}

		if (this->template is_embedded<Edge>())
		{
			this->foreach_dart_of_orbit(v, [this] (Dart it)
			{
				this->template copy_embedding<Edge>(it, phi3(it));
			});
		}

		if (this->template is_embedded<Face>())
		{
			this->foreach_dart_of_orbit(v, [this] (Dart it)
			{
				this->template copy_embedding<Face>(it, phi3(it));
			});
		}

		return v;
	}

	/**
	 * @brief close_map closes the map removing topological holes (only for import/creation)
	 * Add volumes to the map that close every existing hole.
	 * @return the number of closed holes
	 */
	inline uint32 close_map()
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		uint32 nb_holes = 0;

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
				Volume v = close_hole(d);
				this->boundary_mark(v);
				++nb_holes;
			}
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
		return Inherit::degree(v);
	}

	inline uint32 degree(Vertex v) const
	{
		uint32 result = 0;
		foreach_incident_edge(v, [&result] (Edge) { ++result; });
		return result;
	}

	inline uint32 codegree(Edge2 e) const
	{
		return Inherit::codegree(e);
	}

	inline uint32 degree(Edge2 e) const
	{
		return Inherit::degree(e);
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

	inline uint32 codegree(Face2 f) const
	{
		return Inherit::codegree(f);
	}

	inline uint32 degree(Face2 f) const
	{
		return Inherit::degree(f);
	}

	inline uint32 codegree(Face f) const
	{
		return codegree(Face2(f.dart));
	}

	inline uint32 degree(Face f) const
	{
		if (this->is_boundary(f.dart) || this->is_boundary(phi3(f.dart)))
			return 1;
		else
			return 2;
	}

	inline uint32 codegree(Volume v) const
	{
		uint32 result = 0;
		foreach_incident_face(v, [&result] (Face) { ++result; });
		return result;
	}

	inline bool has_codegree(Face2 f, uint32 codegree) const
	{
		return Inherit::has_codegree(f, codegree);
	}

	inline bool has_codegree(Face f, uint32 codegree) const
	{
		return Inherit::has_codegree(Face2(f.dart), codegree);
	}

	/*******************************************************************************
	 * Boundary information
	 *******************************************************************************/

	bool is_adjacent_to_boundary(Boundary c)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		bool result = false;
		foreach_dart_of_orbit(c, [this, &result] (Dart d)
		{
			if (this->is_boundary(phi3(d))) { result = true; return false; }
			return true;
		});
		return result;
	}

#pragma warning(push)
#pragma warning(disable:4702)
	template <Orbit ORBIT>
	inline bool is_boundary_cell(Cell<ORBIT> c) const
	{
		switch (ORBIT)
		{
			case Orbit::DART: return this->is_boundary(c.dart); break;
			case Orbit::PHI1: return this->is_boundary(c.dart); break;
			case Orbit::PHI2: return this->is_boundary(c.dart); break;
			case Orbit::PHI21: return this->is_boundary(c.dart); break;
			case Orbit::PHI1_PHI2: return this->is_boundary(c.dart); break;
			case Orbit::PHI1_PHI3: return false; break;
			case Orbit::PHI2_PHI3: return false; break;
			case Orbit::PHI21_PHI31: return false; break;
			case Orbit::PHI1_PHI2_PHI3: return false; break;
			default: cgogn_assert_not_reached_false("Orbit not supported in a CMap3"); break;
		}
	}
#pragma warning(push)

	inline Face boundary_face_of_edge(Edge e) const
	{
		Face res;
		this->foreach_dart_of_PHI23(e.dart, [this,&res] (Dart it) -> bool
		{
			if (this->is_boundary(it))
			{
				res.dart = it;
				return false;
			}
			else
				return true;
		});
		return res;
	}

	inline Face boundary_face_of_Vertex(Vertex v) const
	{
		Face res;
		this->foreach_dart_of_PHI21_PHI31(v.dart, [this,&res] (Dart it) -> bool
		{
			if (this->is_boundary(it))
			{
				res.dart = it;
				return false;
			}
			else
				return true;
		});
		return res;
	}

public:

	/********************************************************************************
	* Orbits traversal                                                              *
	********************************************************************************/

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_dart_of_orbit(Cell<ORBIT> c, const FUNC& f) const
	{
		static_assert(is_func_parameter_same<FUNC, Dart>::value, "Wrong function parameter type");
		static_assert(ORBIT == Orbit::DART || ORBIT == Orbit::PHI1 || ORBIT == Orbit::PHI2 ||
					  ORBIT == Orbit::PHI21 || ORBIT == Orbit::PHI1_PHI2 ||
					  ORBIT == Orbit::PHI1_PHI3 || ORBIT == Orbit::PHI2_PHI3 ||
					  ORBIT == Orbit::PHI21_PHI31 || ORBIT == Orbit::PHI1_PHI2_PHI3,
					  "Orbit not supported in a CMap3");

		switch (ORBIT)
		{
			case Orbit::DART: f(c.dart); break;
			case Orbit::PHI1: this->foreach_dart_of_PHI1(c.dart, f); break;
			case Orbit::PHI2: this->foreach_dart_of_PHI2(c.dart, f); break;
			case Orbit::PHI21: this->foreach_dart_of_PHI21(c.dart, f); break;
			case Orbit::PHI1_PHI2: this->foreach_dart_of_PHI1_PHI2(c.dart, f); break;
			case Orbit::PHI1_PHI3: foreach_dart_of_PHI1_PHI3(c.dart, f); break;
			case Orbit::PHI2_PHI3: foreach_dart_of_PHI2_PHI3(c.dart, f); break;
			case Orbit::PHI21_PHI31: foreach_dart_of_PHI21_PHI31(c.dart, f); break;
			case Orbit::PHI1_PHI2_PHI3: foreach_dart_of_PHI1_PHI2_PHI3(c.dart, f); break;
			default: cgogn_assert_not_reached("This orbit is not handled"); break;
		}
	}

protected:

	template <typename FUNC>
	inline void foreach_dart_of_PHI21_PHI31(Dart d, const FUNC& f) const
	{
		DartMarkerStore marker(*this);
		const std::vector<Dart>& marked_darts = marker.marked_darts();

		marker.mark(d);
		for (uint32 i = 0; i < marked_darts.size(); ++i)
		{
			const Dart curr_dart = marked_darts[i];
			if ( !(this->is_boundary(curr_dart) && this->is_boundary(phi3(curr_dart))) )
				if (!internal::void_to_true_binder(f, curr_dart))
					break;

			const Dart d_1 = this->phi_1(curr_dart);
			const Dart d2_1 = this->phi2(d_1); // turn in volume
			const Dart d3_1 = phi3(d_1); // change volume

			if (!marker.is_marked(d2_1))
				marker.mark(d2_1);
			if (!marker.is_marked(d3_1))
				marker.mark(d3_1);
		}
	}

	template <typename FUNC>
	inline void foreach_dart_of_PHI2_PHI3(Dart d, const FUNC& f) const
	{
		Dart it = d;
		do
		{
			if (!internal::void_to_true_binder(f, it))
				break;
			it = this->phi2(it);
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

		std::vector<Dart>* visited_face2 = cgogn::dart_buffers()->buffer();
		visited_face2->push_back(d); // Start with the face of d

		// For every face added to the list
		for (uint32 i = 0; i < visited_face2->size(); ++i)
		{
			const Dart e = (*visited_face2)[i];
			if (!marker.is_marked(e))	// Face2 has not been visited yet
			{
				// mark visited darts (current face2)
				// and add non visited phi2-adjacent face2 to the list of face2
				Dart it = e;
				do
				{
					if (!internal::void_to_true_binder(f, it)) // apply the function to the darts of the face2
					{
						cgogn::dart_buffers()->release_buffer(visited_face2);
						return;
					}
					marker.mark(it);				// Mark
					const Dart adj2 = this->phi2(it);		// Get phi2-adjacent face2
					if (!marker.is_marked(adj2))
						visited_face2->push_back(adj2);	// Add it
					it = this->phi1(it);
				} while (it != e);
				// add phi3-adjacent face2 to the list
				visited_face2->push_back(phi3(it));
			}
		}
		cgogn::dart_buffers()->release_buffer(visited_face2);
	}

public:

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
		foreach_dart_of_PHI23(e.dart, [&func] (Dart d) -> bool { return internal::void_to_true_binder(func, Face(d)); });
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
		foreach_dart_of_orbit(Face2(f.dart), [&func] (Dart e) -> bool { return internal::void_to_true_binder(func,Edge(e)); });
	}

	template <typename FUNC>
	inline void foreach_incident_volume(Face f, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Volume>::value, "Wrong function cell parameter type");
		bool res1 = true;
		if (!this->is_boundary(f.dart))
			res1 = internal::void_to_true_binder(func, Volume(f.dart));
		const Dart d3 = phi3(f.dart);
		if (!this->is_boundary(d3) && res1)
			func(Volume(d3));
	}

	template <typename FUNC>
	inline void foreach_incident_vertex(Volume v, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Vertex>::value, "Wrong function cell parameter type");
		Inherit::foreach_incident_vertex(v, [&func] (Vertex2 ve) -> bool
		{
			return internal::void_to_true_binder(func, Vertex(ve.dart));
		});
	}

	template <typename FUNC>
	inline void foreach_incident_edge(Volume v, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Edge>::value, "Wrong function cell parameter type");
		Inherit::foreach_incident_edge(v, [&func] (Edge2 e) -> bool
		{
			return internal::void_to_true_binder(func, Edge(e.dart));
		});
	}

	template <typename FUNC>
	inline void foreach_incident_face(Volume v, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Face>::value, "Wrong function cell parameter type");
		DartMarkerStore marker(*this);
		foreach_dart_of_orbit(v, [&] (Dart d) -> bool
		{
			if (!marker.is_marked(d))
			{
				marker.mark_orbit(Face2(d));
				return internal::void_to_true_binder(func, Face(d));
			}
			return true;
		});
	}

	template <typename FUNC>
	inline void foreach_incident_vertex(ConnectedComponent cc, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Vertex>::value, "Wrong function cell parameter type");
		DartMarkerStore marker(*this);
		foreach_dart_of_orbit(cc, [&] (Dart d) -> bool
		{
			if (!marker.is_marked(d))
			{
				marker.mark_orbit(Vertex(d));
				return internal::void_to_true_binder(func, Vertex(d));
			}
			return true;
		});
	}

	template <typename FUNC>
	inline void foreach_incident_edge(ConnectedComponent cc, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Edge>::value, "Wrong function cell parameter type");
		DartMarkerStore marker(*this);
		foreach_dart_of_orbit(cc, [&] (Dart d) -> bool
		{
			if (!marker.is_marked(d))
			{
				marker.mark_orbit(Edge(d));
				return internal::void_to_true_binder(func, Edge(d));
			}
			return true;
		});
	}

	template <typename FUNC>
	inline void foreach_incident_face(ConnectedComponent cc, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Face>::value, "Wrong function cell parameter type");
		DartMarkerStore marker(*this);
		foreach_dart_of_orbit(cc, [&] (Dart d) -> bool
		{
			if (!marker.is_marked(d))
			{
				marker.mark_orbit(Face(d));
				return internal::void_to_true_binder(func, Face(d));
			}
			return true;
		});
	}

	template <typename FUNC>
	inline void foreach_incident_volume(ConnectedComponent cc, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Volume>::value, "Wrong function cell parameter type");
		DartMarkerStore marker(*this);
		foreach_dart_of_orbit(cc, [&] (Dart d) -> bool
		{
			if (!marker.is_marked(d))
			{
				marker.mark_orbit(Volume(d));
				return internal::void_to_true_binder(func, Volume(d));
			}
			return true;
		});
	}

	// redeclare CMap2 hidden functions

	template <typename FUNC>
	inline void foreach_incident_edge(Vertex2 v, const FUNC& func) const
	{
		Inherit::foreach_incident_edge(v, func);
	}

	template <typename FUNC>
	inline void foreach_incident_face(Vertex2 v, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Face2>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(v, [this, &func] (Dart d)
		{
			return internal::void_to_true_binder(func, Face2(d));
		});
	}

	template <typename FUNC>
	inline void foreach_incident_volume(Vertex2 v, const FUNC& func) const
	{
		Inherit::foreach_incident_volume(v, func);
	}

	template <typename FUNC>
	inline void foreach_incident_vertex(Edge2 e, const FUNC& func) const
	{
		Inherit::foreach_incident_vertex(e, func);
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
		Inherit::foreach_incident_volume(e, func);
	}

	template <typename FUNC>
	inline void foreach_incident_vertex(Face2 f, const FUNC& func) const
	{
		Inherit::foreach_incident_vertex(f, func);
	}

	template <typename FUNC>
	inline void foreach_incident_edge(Face2 f, const FUNC& func) const
	{
		Inherit::foreach_incident_edge(f, func);
	}

	template <typename FUNC>
	inline void foreach_incident_volume(Face2 f, const FUNC& func) const
	{
		Inherit::foreach_incident_volume(f, func);
	}

	/*******************************************************************************
	 * Adjacence traversal
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
		DartMarkerStore marker_vertex(*this);
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
		DartMarkerStore marker_vertex(*this);
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
		DartMarkerStore marker_edge(*this);
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
		DartMarkerStore marker_edge(*this);
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
		DartMarkerStore marker_face(*this);
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
		DartMarkerStore marker_face(*this);
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
		DartMarkerStore marker_volume(*this);
		marker_volume.mark_orbit(v);
		foreach_incident_vertex(v, [&] (Vertex inc_vert)
		{
			bool res_nested_lambda = true;
			foreach_incident_volume(inc_vert, [&](Volume inc_vol)
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
		DartMarkerStore marker_volume(*this);
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
		DartMarkerStore marker_volume(*this);
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

	// redeclare CMap2 hidden functions

	template <typename FUNC>
	inline void foreach_adjacent_vertex_through_edge(Vertex2 v, const FUNC& func) const
	{
		Inherit::foreach_adjacent_vertex_through_edge(v, func);
	}

	template <typename FUNC>
	inline void foreach_adjacent_vertex_through_face(Vertex2 v, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Vertex2>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(v, [this, &func] (Dart vd) -> bool
		{
			bool res_nested_lambda = true;
			const Dart vd1 = this->phi1(vd);
			this->foreach_dart_of_orbit(Face2(vd), [&func, &res_nested_lambda, vd, vd1] (Dart fd) -> bool
			{
				// skip Vertex2 v itself and its first successor around current face
				if (fd != vd && fd != vd1)
					res_nested_lambda = internal::void_to_true_binder(func, Vertex2(fd));
				return res_nested_lambda;
			});
			return res_nested_lambda;
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_edge_through_vertex(Edge2 e, const FUNC& func) const
	{
		Inherit::foreach_adjacent_edge_through_vertex(e, func);
	}

	template <typename FUNC>
	inline void foreach_adjacent_edge_through_face(Edge2 e, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Edge2>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(e, [this, &func] (Dart ed) -> bool
		{
			bool res_nested_lambda = true;
			this->foreach_dart_of_orbit(Face2(ed), [&func, &res_nested_lambda, ed] (Dart fd) -> bool
			{
				// skip Edge2 e itself
				if (fd != ed)
					res_nested_lambda = internal::void_to_true_binder(func, Edge2(fd));
				return res_nested_lambda;
			});
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
			const Dart fd1 = this->phi2(this->phi_1(fd));
			this->foreach_dart_of_orbit(Vertex2(fd), [this, &func, &res_nested_lambda, fd, fd1] (Dart vd) -> bool
			{
				// skip Face2 f itself and its first successor around current vertex
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
			return internal::void_to_true_binder(func, Face2(this->phi2(d)));
		});
	}

	inline std::pair<Vertex, Vertex> vertices(Edge e) const
	{
		return std::pair<Vertex, Vertex>(Vertex(e.dart), Vertex(this->phi1(e.dart)));
	}

	inline std::pair<Vertex2, Vertex2> vertices(Edge2 e) const
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

public:

	using Inherit::Inherit::Inherit::Inherit::merge;

	/**
	 * @brief merge the given CMap2 in the current CMap3
	 * @param map2
	 * @param newdarts a DartMarker in which the new imported darts are marked
	 * @return false if the merge can not be done (incompatible attributes), true otherwise
	 */
	bool merge(const CMap2& map2, DartMarker& newdarts)
	{
		// check attributes compatibility
		for (uint32 i = 0; i < NB_ORBITS; ++i)
		{
			if (this->embeddings_[i] != nullptr)
			{
				if (!this->attributes_[i].check_before_merge(map2.attribute_container(Orbit(i))))
					return false;
			}
		}

		// compact topology container
		this->compact_topo();
		uint32 first = this->topology_.size();

		// ensure that orbits that are embedded in given map2 are also embedded in this map
		const static auto create_embedding = [=] (Self* map, Orbit orb)
		{
			switch (orb)
			{
				case Orbit::DART: map->template create_embedding<Orbit::DART>(); break;
				case Orbit::PHI1: map->template create_embedding<Orbit::PHI1>(); break;
				case Orbit::PHI2: map->template create_embedding<Orbit::PHI2>(); break;
				case Orbit::PHI21: map->template create_embedding<Orbit::PHI21>(); break;
				case Orbit::PHI1_PHI2: map->template create_embedding<Orbit::PHI1_PHI2>(); break;
				default: break;
			}
		};
		for (Orbit orb : { DART, PHI1, PHI2, PHI21, PHI1_PHI2 })
			if (!this->is_embedded(orb) && map2.is_embedded(orb))
				create_embedding(this, orb);

		// store index of copied darts
		std::vector<uint32> old_new_topo = this->topology_.template merge<PRIM_SIZE>(map2.topology_container());

		// mark new darts with the given dartmarker
		newdarts.unmark_all();
		map2.foreach_dart([&] (Dart d) { newdarts.mark(Dart(old_new_topo[d.index])); });

		// change topo relations of copied darts
		for (uint32 i = first; i != this->topology_.end(); this->topology_.next(i))
		{
			Dart& d1 = (*this->phi1_)[i];
			uint32 idx = d1.index;
			if (old_new_topo[idx] != INVALID_INDEX)
				d1 = Dart(old_new_topo[idx]);

			Dart& d_1 = (*this->phi_1_)[i];
			idx = d_1.index;
			if (old_new_topo[idx] != INVALID_INDEX)
				d_1 = Dart(old_new_topo[idx]);

			Dart& d2 = (*this->phi2_)[i];
			idx = d2.index;
			if (old_new_topo[idx] != INVALID_INDEX)
				d2 = Dart(old_new_topo[idx]);

			// set copied Darts in fix point for phi3
			(*phi3_)[i] = Dart(i);
		}

		// the boundary marker of the merged Map2 is ignored

		// close the map
		// and mark new darts with the dartmarker
		Builder mb(*this);
		for (uint32 i = first; i != this->topology_.end(); this->topology_.next(i))
		{
			Dart d(i);
			if (phi3(d) == d)
			{
				close_hole_topo(d);
				Dart d3 = phi3(d);
				this->foreach_dart_of_orbit(Volume(d3), [this, &newdarts] (Dart v)
				{
					this->set_boundary(v, true);
					newdarts.mark(v);
				});
			}
		}

		// change embedding indices of moved lines
		for (uint32 i = 0; i < NB_ORBITS; ++i)
		{
			ChunkArray<uint32>* emb = this->embeddings_[i];
			if (emb != nullptr)
			{
				if (!map2.is_embedded(Orbit(i))) // set embedding to INVALID for further easy detection
				{
					for (uint32 j = first; j != this->topology_.end(); this->topology_.next(j))
						(*emb)[j] = INVALID_INDEX;
				}
				else
				{
					std::vector<uint32> old_new = this->attributes_[i].template merge<1>(map2.attribute_container(Orbit(i)));
					for (uint32 j = first; j != this->topology_.end(); this->topology_.next(j))
					{
						uint32& e = (*emb)[j];
						if (e != INVALID_INDEX)
						{
							if (old_new[e] != INVALID_INDEX)
								e = old_new[e];
						}
					}
				}
			}
		}

		// embed remaining cells
		merge_finish_embedding(first);

		// ok
		return true;
	}
};

struct CMap3Type
{
	using TYPE = CMap3_T<CMap3Type>;
};

using CMap3 = CMap3_T<CMap3Type>;

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_CMAP_CMAP3_CPP_))
extern template class CGOGN_CORE_API CMap3_T<CMap3Type>;
extern template class CGOGN_CORE_API CMap3Builder_T<CMap3>;
extern template class CGOGN_CORE_API DartMarker<CMap3>;
extern template class CGOGN_CORE_API DartMarkerStore<CMap3>;
extern template class CGOGN_CORE_API DartMarkerNoUnmark<CMap3>;
extern template class CGOGN_CORE_API CellMarker<CMap3, CMap3::Vertex::ORBIT>;
extern template class CGOGN_CORE_API CellMarker<CMap3, CMap3::Edge::ORBIT>;
extern template class CGOGN_CORE_API CellMarker<CMap3, CMap3::Face::ORBIT>;
extern template class CGOGN_CORE_API CellMarker<CMap3, CMap3::Volume::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerNoUnmark<CMap3, CMap3::Vertex::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerNoUnmark<CMap3, CMap3::Edge::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerNoUnmark<CMap3, CMap3::Face::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerNoUnmark<CMap3, CMap3::Volume::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap3, CMap3::Vertex::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap3, CMap3::Edge::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap3, CMap3::Face::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap3, CMap3::Volume::ORBIT>;
extern template class CGOGN_CORE_API CellCache<CMap3>;
extern template class CGOGN_CORE_API BoundaryCache<CMap3>;
extern template class CGOGN_CORE_API QuickTraversor<CMap3>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_MAP_MAP3_CPP_))

} // namespace cgogn

#endif // CGOGN_CORE_CMAP_CMAP3_H_
