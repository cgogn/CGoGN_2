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

namespace cgogn
{

// forward declaration of CMap3Builder_T
template <typename MAP_TRAITS> class CMap3Builder_T;

template <typename MAP_TRAITS, typename MAP_TYPE>
class CMap3_T : public CMap2_T<MAP_TRAITS, MAP_TYPE>
{
public:

	static const uint8 DIMENSION = 3;

	static const uint8 PRIM_SIZE = 1;

	using MapTraits = MAP_TRAITS;
	using MapType = MAP_TYPE;
	using Inherit = CMap2_T<MAP_TRAITS, MAP_TYPE>;
	using Self = CMap3_T<MAP_TRAITS, MAP_TYPE>;

	using Builder = CMap3Builder_T<MapTraits>;

	friend class MapBase<MAP_TRAITS, MAP_TYPE>;
	friend class CMap3Builder_T<MapTraits>;
	friend class DartMarker_T<Self>;
	friend class cgogn::DartMarkerStore<Self>;

	using CDart		= typename Inherit::CDart;
	using Vertex2	= typename Inherit::Vertex;
	using Vertex	= Cell<Orbit::PHI21_PHI31>;
	using Edge2		= typename Inherit::Edge;
	using Edge		= Cell<Orbit::PHI2_PHI3>;
	using Face2		= typename Inherit::Face;
	using Face		= Cell<Orbit::PHI1_PHI3>;
	using Volume	= typename Inherit::Volume;

	using Boundary  = Volume;
	using ConnectedComponent = Cell<Orbit::PHI1_PHI2_PHI3>;

	template <typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;
	template <typename T>
	using ChunkArrayContainer = typename Inherit::template ChunkArrayContainer<T>;

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
	template <Orbit ORBIT>
	using CellMarkerStore = typename cgogn::CellMarkerStore<Self, ORBIT>;

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
		static_assert((N%10)<=3,"composition on phi1/phi2/only");
		switch(N%10)
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
	 * @brief add_stamp_volume_topo : a flat volume with one face composed of two triangles and another compose of one quad
	 * @return a dart of the quad
	 */
	Dart add_stamp_volume_topo()
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

	/**
	 * @brief Flip an Edge (rotation in phi1 order)
	 * @param e : the edge to flip
	 * @return  true iff the flip operation has been successfull
	 * The edge has to :
	 * 1) Be incident to the boundary.
	 * 2) Be incident to exactly 2 faces.
	 */
	inline bool flip_edge_topo(Dart e)
	{
		if (this->is_incident_to_boundary(Edge(e)) && this->degree(Edge(e)) == 2u)
			return Inherit::flip_edge_topo(e) && Inherit::flip_back_edge_topo(phi3(e));
		else
			return false;
	}

	/**
	 * @brief Flip an Edge (rotation in phi_1 order)
	 * @param e : the edge to flip
	 * @return  true iff the flip operation has been successfull
	 * The edge has to :
	 * 1) Be incident to the boundary.
	 * 2) Be incident to exactly 2 faces.
	 */
	inline bool flip_back_edge_topo(Dart e)
	{
		if (this->is_incident_to_boundary(Edge(e))&& this->degree(Edge(e)) == 2u)
			return Inherit::flip_back_edge_topo(e) && Inherit::flip_edge_topo(phi3(e));
		else
			return false;
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
				if (!this->is_boundary(nv1)) this->new_orbit_embedding(CDart(nv1));
				if (!this->is_boundary(nv2)) this->new_orbit_embedding(CDart(nv2));
			});
		}

		if (this->template is_embedded<Vertex2>())
		{
			foreach_dart_of_PHI23(e.dart, [this] (Dart d)
			{
				this->new_orbit_embedding(Vertex2(this->phi1(d)));
			});
		}

		if (this->template is_embedded<Vertex>())
			this->new_orbit_embedding(Vertex(v));

		if (this->template is_embedded<Edge2>())
		{
			foreach_dart_of_PHI23(e.dart, [this] (Dart d)
			{
				this->template copy_embedding<Edge2>(this->phi2(d), d);
				this->new_orbit_embedding(Edge2(this->phi1(d)));
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
				this->template copy_embedding<Face2>(this->phi1(d), d);
				this->template copy_embedding<Face2>(this->phi2(d), this->phi2(this->phi1(d)));
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
				if (!this->is_boundary(d))
				{
					this->template copy_embedding<Volume>(this->phi1(d), d);
					this->template copy_embedding<Volume>(this->phi2(d), d);
				}
			});
		}

		return Vertex(v);
	}

	inline void flip_edge(Edge e)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		if (!flip_edge_topo(e.dart))
			return;

		const Dart e2 = this->phi2(e.dart);
		const Dart e3 = phi3(e.dart);
		const Dart e32 = this->phi2(e3);

		if (this->is_embedded(Vertex2::ORBIT))
		{
			if (!this->is_boundary(e.dart))
			{
				this->template copy_embedding<Vertex2>(e.dart, this->phi1(e2));
				this->template copy_embedding<Vertex2>(e2, this->phi1(e.dart));
			} else {
				this->template copy_embedding<Vertex2>(e3, this->phi1(e32));
				this->template copy_embedding<Vertex2>(e32, this->phi1(e3));
			}
		}

		if (this->is_embedded(Vertex::ORBIT))
		{
			if (!this->is_boundary(e.dart))
			{
				this->template copy_embedding<Vertex>(e.dart, this->phi1(e2));
				this->template copy_embedding<Vertex>(e2, this->phi1(e.dart));
			} else {
				this->template copy_embedding<Vertex>(e3, this->phi1(e32));
				this->template copy_embedding<Vertex>(e32, this->phi1(e3));
			}
		}

		if (this->is_embedded(Face2::ORBIT))
		{
			if (!this->is_boundary(e.dart))
			{
				this->template copy_embedding<Face2>(this->phi_1(e.dart), e.dart);
				this->template copy_embedding<Face2>(this->phi_1(e2), e2);
			} else {
				this->template copy_embedding<Face2>(this->phi1(e3), e3);
				this->template copy_embedding<Face2>(this->phi1(e32), e32);
			}
		}

		if (this->is_embedded(Face::ORBIT))
		{
			if (!this->is_boundary(e.dart))
			{
				this->template copy_embedding<Face>(this->phi_1(e.dart), e.dart);
				this->template copy_embedding<Face>(this->phi_1(e2), e2);
			} else {
				this->template copy_embedding<Face>(this->phi1(e3), e3);
				this->template copy_embedding<Face>(this->phi1(e32), e32);
			}
		}
	}

	inline void flip_back_edge(Edge e)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		if (!flip_back_edge_topo(e.dart))
			return;

		const Dart e2 = this->phi2(e.dart);
		const Dart e3 = phi3(e.dart);
		const Dart e32 = this->phi2(e3);

		if (this->is_embedded(Vertex2::ORBIT))
		{
			if (!this->is_boundary(e.dart))
			{
				this->template copy_embedding<Vertex2>(e.dart, this->phi1(e2));
				this->template copy_embedding<Vertex2>(e2, this->phi1(e.dart));
			} else {
				this->template copy_embedding<Vertex2>(e3, this->phi1(e32));
				this->template copy_embedding<Vertex2>(e32, this->phi1(e3));
			}
		}

		if (this->is_embedded(Vertex::ORBIT))
		{
			if (!this->is_boundary(e.dart))
			{
				this->template copy_embedding<Vertex>(e.dart, this->phi1(e2));
				this->template copy_embedding<Vertex>(e2, this->phi1(e.dart));
			} else {
				this->template copy_embedding<Vertex>(e3, this->phi1(e32));
				this->template copy_embedding<Vertex>(e32, this->phi1(e3));
			}
		}

		if (this->is_embedded(Face2::ORBIT))
		{
			if (!this->is_boundary(e.dart))
			{
				this->template copy_embedding<Face2>(this->phi1(e.dart), e.dart);
				this->template copy_embedding<Face2>(this->phi1(e2), e2);
			} else {
				this->template copy_embedding<Face2>(this->phi_1(e3), e3);
				this->template copy_embedding<Face2>(this->phi_1(e32), e32);
			}
		}

		if (this->is_embedded(Face::ORBIT))
		{
			if (!this->is_boundary(e.dart))
			{
				this->template copy_embedding<Face>(this->phi1(e.dart), e.dart);
				this->template copy_embedding<Face>(this->phi1(e2), e2);
			} else {
				this->template copy_embedding<Face>(this->phi_1(e3), e3);
				this->template copy_embedding<Face>(this->phi_1(e32), e32);
			}
		}
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
			if (!this->is_boundary(nd)) this->new_orbit_embedding(CDart(nd));
			if (!this->is_boundary(ne)) this->new_orbit_embedding(CDart(ne));
			if (!this->is_boundary(nd3)) this->new_orbit_embedding(CDart(nd3));
			if (!this->is_boundary(ne3)) this->new_orbit_embedding(CDart(ne3));
		}

		if (this->template is_embedded<Vertex2>())
		{
			this->template copy_embedding<Vertex2>(nd, e);
			this->template copy_embedding<Vertex2>(ne, d);
			this->template copy_embedding<Vertex2>(nd3, this->phi1(ne3));
			this->template copy_embedding<Vertex2>(ne3, this->phi1(nd3));
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
			this->new_orbit_embedding(Edge2(nd));
			this->new_orbit_embedding(Edge2(nd3));
		}

		if (this->template is_embedded<Edge>())
			this->new_orbit_embedding(Edge(nd));

		if (this->template is_embedded<Face2>())
		{
			this->template copy_embedding<Face2>(nd, d);
			this->new_orbit_embedding(Face2(ne));
			this->template copy_embedding<Face2>(nd3, phi3(d));
			this->new_orbit_embedding(Face2(ne3));
		}

		if (this->template is_embedded<Face>())
		{
			this->template copy_embedding<Face>(nd, d);
			this->template copy_embedding<Face>(nd3, d);
			this->new_orbit_embedding(Face(ne));
		}

		if (this->template is_embedded<Volume>())
		{
			if (!this->is_boundary(d))
			{
				this->template copy_embedding<Volume>(nd, d);
				this->template copy_embedding<Volume>(ne, d);
			}
			Dart d3 = phi3(d);
			if (!this->is_boundary(d3))
			{
				this->template copy_embedding<Volume>(nd3, d3);
				this->template copy_embedding<Volume>(ne3, d3);
			}
		}

		return Edge(nd);
	}

	Dart delete_edge(Edge e)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		const Dart res_topo_del = delete_edge_topo(e.dart);
		if (res_topo_del.is_nil())
			return res_topo_del;

		if (this->template is_embedded<Volume>())
		{
			const uint32 emb = this->embedding(Volume(res_topo_del));
			foreach_dart_of_orbit(Volume(res_topo_del), [this,emb] (Dart dit)
			{
				this->template set_embedding<Volume>(dit, emb);
			});
		}
		return res_topo_del;
	}

protected:
	Dart delete_edge_topo(Dart d)
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

				if(res.is_nil())
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
			this->foreach_dart_of_orbit(typename Inherit::ConnectedComponent(d), [=](Dart it) {darts_to_be_deleted->push_back(it);});
			for (Dart it : *darts_to_be_deleted)
				this->remove_topology_element(it);
			cgogn::dart_buffers()->release_buffer(darts_to_be_deleted);
		}

		return res;
	}

	Dart split_vertex_topo(std::vector<Dart>& vd)
	{
		Dart prev = vd.front();	//elt 0

		Dart db1;
		if (this->is_incident_to_boundary(Face(prev)))
			db1 = this->phi2(phi3(this->phi1(this->phi2(prev))));

		this->Inherit::split_vertex_topo(prev, this->phi2(this->phi_1(this->phi2(this->phi_1(prev)))));

		for(unsigned int i = 1; i < vd.size(); ++i)
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

		if(!db1.is_nil() && !db2.is_nil())
		{
			this->Inherit::split_vertex_topo(db1, db2);
			phi3_sew(this->phi1(this->phi2(db2)), this->phi_1(phi3(this->phi2(db2))));
			phi3_sew(this->phi1(this->phi2(db1)), this->phi_1(phi3(this->phi2(db1))));
		} else {
			Dart dbegin = this->phi1(this->phi2(vd.front()));
			Dart dend = this->phi_1(this->phi2(this->phi_1(vd.back())));
			phi3_sew(dbegin, dend);
		}

		return this->phi_1(this->phi2(this->phi_1(prev)));
	}

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

		if (this->template is_embedded<Vertex>())
		{
			this->new_orbit_embedding(Vertex(d2));
			const uint32 emb = this->embedding(Vertex(d1));
			foreach_dart_of_orbit(Vertex(d1), [this,emb] (Dart dit)
			{
				this->template set_embedding<Vertex>(dit, emb);
			});
		}

		if (this->template is_embedded<Edge>())
		{
			this->new_orbit_embedding(Edge(res));
		}

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
			for(auto dit1 : vd)
			{
				const uint32 emb = this->embedding(Volume(dit1));
				foreach_dart_of_orbit(Volume(dit1), [this,emb] (Dart dit2)
				{
					this->template set_embedding<Volume>(dit2, emb);
				});
			}
		}

		return res;
	}

	/**
	 * @brief Cut a single volume following a simple closed oriented path
	 * @param path a vector of darts representing the path
	 * @return the inserted face
	 */
	Face cut_volume(const std::vector<Dart>& path)
	{
		CGOGN_CHECK_CONCRETE_TYPE;
		cgogn_message_assert(!this->is_boundary(path[0]), "cut_volume: should not cut a boundary volume");

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

	bool merge_incident_volumes_topo(Dart d)
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
		this->remove_face_topo(d);
		this->remove_face_topo(d3);

		return true;
	}

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

	bool sew_volumes_topo(Dart fa, Dart fb)
	{
		if (this->codegree(Face(fa)) != this->codegree(Face(fb)))
			return false;

		const Dart fa3 = phi3(fa);
		const Dart fb3 = phi3(fb);

		Dart fa_it = fa3;
		Dart fb_it = fb3;
		do
		{
			const Dart fa_it2 = this->phi2(fa_it);
			const Dart fb_it2 = this->phi2(fb_it);
			if(fa_it2 != fb_it)
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
			this->foreach_dart_of_orbit(Volume(fa3), [=](Dart it) {darts_to_be_deleted->push_back(it);});
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

	bool unsew_volumes_topo(Face f)
	{
		if (this->is_incident_to_boundary(f))
			return false;

		const uint32 nb_edges = this->codegree(f);
		const Dart d3 = phi3(f.dart);

		const Dart b1 = Inherit::Inherit::add_face_topo(nb_edges);
		const Dart b2 = Inherit::Inherit::add_face_topo(nb_edges);

		this->foreach_dart_of_orbit(Face2(b1), [this] (Dart d) {this->set_boundary(d,true);});
		this->foreach_dart_of_orbit(Face2(b2), [this] (Dart d) {this->set_boundary(d,true);});

		Dart fit1 = f.dart;
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
			} else
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

	void delete_volume_topo(Volume w)
	{
		this->Inherit::foreach_incident_face(w, [&](Face2 f)
		{
			if (!this->is_incident_to_boundary(Face(f.dart)))
				this->unsew_volumes_topo(Face(f.dart));
		});

		{ // removing the darts
			const Volume w3(phi3(w.dart));
			std::vector<Dart>* darts_to_be_deleted = cgogn::dart_buffers()->buffer();
			this->foreach_dart_of_orbit(w, [=](Dart it) {darts_to_be_deleted->push_back(it);});
			this->foreach_dart_of_orbit(w3, [=](Dart it) {darts_to_be_deleted->push_back(it);});
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

	void sew_volumes(Face fa, Face fb)
	{
		CGOGN_CHECK_CONCRETE_TYPE;
		if (!sew_volumes_topo(fa.dart, fb.dart))
			return;

		if (this->template is_embedded<Vertex>())
		{
			Dart dit = fa.dart;
			do {
				const uint32 emb = this->embedding(Vertex(dit));
				foreach_dart_of_orbit(Vertex(dit), [this, emb] (Dart d)
				{
					this->template set_embedding<Vertex>(d, emb);
				});
				dit = this->phi1(dit);
			} while (dit != fa.dart);
		}

		if (this->template is_embedded<Edge>())
		{
			Dart dit = fa.dart;
			do {
				const uint32 emb = this->embedding(Edge(dit));
				foreach_dart_of_orbit(Edge(dit), [this, emb] (Dart d)
				{
					this->template set_embedding<Edge>(d, emb);
				});
				dit = this->phi1(dit);
			} while (dit != fa.dart);
		}

		if (this->template is_embedded<Face>())
		{
			const uint32 emb = this->embedding(fa);
			foreach_dart_of_orbit(fb, [this, emb] (Dart d)
			{
				this->template set_embedding<Face>(d, emb);
			});
		}
	}

	void unsew_volumes(Face f)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		Dart dd = phi3(this->phi_1(f.dart));
		Dart dit = f.dart;

		if (!this->unsew_volumes_topo(f))
			return;

		do {
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

	inline Face boundary_face_of_edge(Edge e) const
	{
		Face res;
		this->foreach_dart_of_PHI23_until(e.dart, [this,&res](Dart it) -> bool
		{
			if (this->is_boundary(it))
			{
				res.dart = it;
				return false;
			} else
				return true;
		});
		return res;
	}

	void merge_incident_faces(Dart e)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		const Dart f = this->phi1(e);

		if (merge_incident_faces_topo(e))
		{
			if (this->template is_embedded<Face2>())
			{
				const uint32 emb = this->embedding(Face2(f));
				foreach_dart_of_orbit(Face2(f), [this, emb] (Dart d)
				{
					this->template set_embedding<Face2>(d, emb);
				});
			}

			if (this->template is_embedded<Face>())
			{
				const uint32 emb = this->embedding(Face(f));
				foreach_dart_of_orbit(Face(f), [this, emb] (Dart d)
				{
					this->template set_embedding<Face>(d, emb);
				});
			}
		}
	}

	void merge_incident_volumes(Face f)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		Dart d2 = this->phi2(f.dart);
		if (merge_incident_volumes_topo(f.dart))
		{
			if (this->template is_embedded<Vertex2>())
			{
				Dart it = d2;
				do
				{
					uint32 emb = this->embedding(Vertex2(it));
					foreach_dart_of_orbit(Vertex2(it), [this, emb] (Dart d) { this->template set_embedding<Vertex2>(d, emb); });
					it = this->phi<121>(it);
				} while (it != d2);
			}

			if (this->template is_embedded<Edge2>())
			{
				Dart it = d2;
				do
				{
					this->template copy_embedding<Edge2>(this->phi2(it), it);
					it = this->phi<121>(it);
				} while (it != d2);
			}

			if (this->template is_embedded<Volume>())
			{
				uint32 emb = this->embedding(Volume(d2));
				foreach_dart_of_orbit(Volume(d2), [this, emb] (Dart d)
				{
					this->template set_embedding<Volume>(d, emb);
				});
			}
		}
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
	inline void foreach_dart_of_PHI21_PHI31(Dart d, const FUNC& f) const
	{
		DartMarkerStore marker(*this);
		const std::vector<Dart>* marked_darts = marker.marked_darts();

		marker.mark(d);
		for(uint32 i = 0; i < marked_darts->size(); ++i)
		{
			const Dart curr_dart = marked_darts->operator[](i);
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
			f(it);
			it = this->phi2(it);
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
			it = phi3(this->phi2(it));
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

		std::vector<Dart>* visited_face2 = cgogn::dart_buffers()->buffer();
		visited_face2->push_back(d); // Start with the face of d

		// For every face added to the list
		for(uint32 i = 0; i < visited_face2->size(); ++i)
		{
			Dart e = (*visited_face2)[i];
			if (!marker.is_marked(e))	// Face2 has not been visited yet
			{
				// mark visited darts (current face2)
				// and add non visited phi2-adjacent face2 to the list of face2
				Dart it = e;
				do
				{
					f(it); // apply the function to the darts of the face2
					marker.mark(it);				// Mark
					Dart adj2 = this->phi2(it);		// Get phi2-adjacent face2
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

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_dart_of_orbit(Cell<ORBIT> c, const FUNC& f) const
	{
		static_assert(is_func_parameter_same<FUNC, Dart>::value, "Wrong function parameter type");
		static_assert(ORBIT == Orbit::DART || ORBIT == Orbit::PHI1 || ORBIT == Orbit::PHI2 ||
					  ORBIT == Orbit::PHI1_PHI2 || ORBIT == Orbit::PHI21 ||
					  ORBIT == Orbit::PHI1_PHI3 || ORBIT == Orbit::PHI2_PHI3 ||
					  ORBIT == Orbit::PHI21_PHI31 || ORBIT == Orbit::PHI1_PHI2_PHI3,
					  "Orbit not supported in a CMap3");

		switch (ORBIT)
		{
			case Orbit::DART: f(c.dart); break;
			case Orbit::PHI1: this->foreach_dart_of_PHI1(c.dart, f); break;
			case Orbit::PHI2: this->foreach_dart_of_PHI2(c.dart, f); break;
			case Orbit::PHI1_PHI2: this->foreach_dart_of_PHI1_PHI2(c.dart, f); break;
			case Orbit::PHI1_PHI3: foreach_dart_of_PHI1_PHI3(c.dart, f); break;
			case Orbit::PHI2_PHI3: foreach_dart_of_PHI2_PHI3(c.dart, f); break;
			case Orbit::PHI21: this->foreach_dart_of_PHI21(c.dart, f); break;
			case Orbit::PHI21_PHI31: foreach_dart_of_PHI21_PHI31(c.dart, f); break;
			case Orbit::PHI1_PHI2_PHI3: foreach_dart_of_PHI1_PHI2_PHI3(c.dart, f); break;
			default: cgogn_assert_not_reached("This orbit is not handled"); break;
		}
	}

protected:

	template <typename FUNC>
	inline void foreach_dart_of_PHI21_PHI31_until(Dart d, const FUNC& f) const
	{
		DartMarkerStore marker(*this);
		const std::vector<Dart>* marked_darts = marker.marked_darts();

		marker.mark(d);
		for(uint32 i = 0; i < marked_darts->size(); ++i)
		{
			const Dart curr_dart = marked_darts->operator[](i);
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
			if (!f(it))
				break;
			it = this->phi2(it);
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

		std::vector<Dart>* visited_face2 = cgogn::dart_buffers()->buffer();
		visited_face2->push_back(d); // Start with the face of d

		// For every face added to the list
		for(uint32 i = 0; i < visited_face2->size(); ++i)
		{
			Dart e = (*visited_face2)[i];
			if (!marker.is_marked(e))	// Face2 has not been visited yet
			{
				// mark visited darts (current face2)
				// and add non visited phi2-adjacent face2 to the list of face2
				Dart it = e;
				do
				{
					if (!f(it)) // apply the function to the darts of the face2
					{
						cgogn::dart_buffers()->release_buffer(visited_face2);
						return;
					}
					marker.mark(it);				// Mark
					Dart adj2 = this->phi2(it);		// Get phi2-adjacent face2
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

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_dart_of_orbit_until(Cell<ORBIT> c, const FUNC& f) const
	{
		static_assert(is_func_parameter_same<FUNC, Dart>::value, "Wrong function parameter type");
		static_assert(is_func_return_same<FUNC, bool>::value, "Wrong function return type");
		static_assert(ORBIT == Orbit::DART || ORBIT == Orbit::PHI1 || ORBIT == Orbit::PHI2 ||
					  ORBIT == Orbit::PHI1_PHI2 || ORBIT == Orbit::PHI21 ||
					  ORBIT == Orbit::PHI1_PHI3 || ORBIT == Orbit::PHI2_PHI3 ||
					  ORBIT == Orbit::PHI21_PHI31 || ORBIT == Orbit::PHI1_PHI2_PHI3,
					  "Orbit not supported in a CMap3");

		switch (ORBIT)
		{
			case Orbit::DART: f(c.dart); break;
			case Orbit::PHI1: this->foreach_dart_of_PHI1_until(c.dart, f); break;
			case Orbit::PHI2: this->foreach_dart_of_PHI2_until(c.dart, f); break;
			case Orbit::PHI1_PHI2: this->foreach_dart_of_PHI1_PHI2_until(c.dart, f); break;
			case Orbit::PHI1_PHI3: foreach_dart_of_PHI1_PHI3_until(c.dart, f); break;
			case Orbit::PHI2_PHI3: foreach_dart_of_PHI2_PHI3_until(c.dart, f); break;
			case Orbit::PHI21: this->foreach_dart_of_PHI21_until(c.dart, f); break;
			case Orbit::PHI21_PHI31: foreach_dart_of_PHI21_PHI31_until(c.dart, f); break;
			case Orbit::PHI1_PHI2_PHI3: foreach_dart_of_PHI1_PHI2_PHI3_until(c.dart, f); break;
			default: cgogn_assert_not_reached("This orbit is not handled"); break;
		}
	}

	/*******************************************************************************
	 * Incidence traversal
	 *******************************************************************************/

	template <typename FUNC>
	inline void foreach_incident_edge(Vertex v, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Edge>::value, "Wrong function cell parameter type");
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
		static_assert(is_func_parameter_same<FUNC, Face>::value, "Wrong function cell parameter type");
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
		static_assert(is_func_parameter_same<FUNC, Volume>::value, "Wrong function cell parameter type");
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
		static_assert(is_func_parameter_same<FUNC, Vertex>::value, "Wrong function cell parameter type");
		f(Vertex(e.dart));
		f(Vertex(this->phi2(e.dart)));
	}

	template <typename FUNC>
	inline void foreach_incident_face(Edge e, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Face>::value, "Wrong function cell parameter type");
		foreach_dart_of_PHI23(e.dart, [&func] (Dart d) { func(Face(d)); });
	}

	template <typename FUNC>
	inline void foreach_incident_volume(Edge e, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Volume>::value, "Wrong function cell parameter type");
		foreach_dart_of_PHI23(e.dart, [this, &func] (Dart d)
		{
			if (!this->is_boundary(d))
				func(Volume(d));
		});
	}

	template <typename FUNC>
	inline void foreach_incident_vertex(Face f, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Vertex>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(Face2(f.dart), [&func] (Dart v) { func(Vertex(v)); });
	}


	template <typename FUNC>
	inline void foreach_incident_edge(Face f, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Edge>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(Face2(f.dart), [&func] (Dart e) { func(Edge(e)); });
	}

	template <typename FUNC>
	inline void foreach_incident_volume(Face f, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Volume>::value, "Wrong function cell parameter type");
		if (!this->is_boundary(f.dart))
			func(Volume(f.dart));
		const Dart d3 = phi3(f.dart);
		if (!this->is_boundary(d3))
			func(Volume(d3));
	}

	template <typename FUNC>
	inline void foreach_incident_vertex(Volume v, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Vertex>::value, "Wrong function cell parameter type");
		Inherit::foreach_incident_vertex(v, [&func] (Vertex2 ve)
		{
			func(Vertex(ve.dart));
		});
	}

	template <typename FUNC>
	inline void foreach_incident_edge(Volume v, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Edge>::value, "Wrong function cell parameter type");
		Inherit::foreach_incident_edge(v, [&func] (Edge2 e)
		{
			func(Edge(e.dart));
		});
	}

	template <typename FUNC>
	inline void foreach_incident_face(Volume v, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Face>::value, "Wrong function cell parameter type");
		DartMarkerStore marker(*this);
		foreach_dart_of_orbit(v, [&] (Dart d)
		{
			if (!marker.is_marked(d))
			{
				marker.mark_orbit(Face2(d));
				func(Face(d));
			}
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
			func(Face2(d));
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
		foreach_dart_of_orbit(e, [this, &func] (Dart d)
		{
			func(Face2(d));
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
		foreach_incident_edge(v, [&] (Edge e)
		{
			func(Vertex(this->phi2(e.dart)));
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_vertex_through_face(Vertex v, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Vertex>::value, "Wrong function cell parameter type");
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
		static_assert(is_func_parameter_same<FUNC, Vertex>::value, "Wrong function cell parameter type");
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
		static_assert(is_func_parameter_same<FUNC, Edge>::value, "Wrong function cell parameter type");
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
		static_assert(is_func_parameter_same<FUNC, Edge>::value, "Wrong function cell parameter type");
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
		static_assert(is_func_parameter_same<FUNC, Edge>::value, "Wrong function cell parameter type");
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
		static_assert(is_func_parameter_same<FUNC, Face>::value, "Wrong function cell parameter type");
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
		static_assert(is_func_parameter_same<FUNC, Face>::value, "Wrong function cell parameter type");
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
		static_assert(is_func_parameter_same<FUNC, Face>::value, "Wrong function cell parameter type");
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
		static_assert(is_func_parameter_same<FUNC, Volume>::value, "Wrong function cell parameter type");
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
		static_assert(is_func_parameter_same<FUNC, Volume>::value, "Wrong function cell parameter type");
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
		static_assert(is_func_parameter_same<FUNC, Volume>::value, "Wrong function cell parameter type");
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
		foreach_dart_of_orbit(v, [this, &func] (Dart vd)
		{
			Dart vd1 = this->phi1(vd);
			this->foreach_dart_of_orbit(Face2(vd), [&func, vd, vd1] (Dart fd)
			{
				// skip Vertex2 v itself and its first successor around current face
				if (fd != vd && fd != vd1)
					func(Vertex2(fd));
			});
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
		foreach_dart_of_orbit(e, [this, &func] (Dart ed)
		{
			this->foreach_dart_of_orbit(Face2(ed), [&func, ed] (Dart fd)
			{
				// skip Edge2 e itself
				if (fd != ed)
					func(Edge2(fd));
			});
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_face_through_vertex(Face2 f, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Face2>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(f, [this, &func] (Dart fd)
		{
			Dart fd1 = this->phi2(this->phi_1(fd));
			this->foreach_dart_of_orbit(Vertex2(fd), [this, &func, fd, fd1] (Dart vd)
			{
				// skip Face2 f itself and its first successor around current vertex
				if (vd != fd && vd != fd1)
					func(Face2(vd));
			});
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_face_through_edge(Face2 f, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Face2>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(f, [this, &func] (Dart d)
		{
			const Dart d2 = this->phi2(d);
				func(Face2(d2));
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
		const static auto create_embedding = [=] (Self* map, Orbit orb)
		{
			switch (orb) {
				case Orbit::DART: map->template create_embedding<Orbit::DART>(); break;
				case Orbit::PHI1: map->template create_embedding<Orbit::PHI1>(); break;
				case Orbit::PHI2:map->template create_embedding<Orbit::PHI2>(); break;
				case Orbit::PHI1_PHI2: map->template create_embedding<Orbit::PHI1_PHI2>(); break;
				case Orbit::PHI1_PHI3: map->template create_embedding<Orbit::PHI1_PHI3>(); break;
				case Orbit::PHI2_PHI3: map->template create_embedding<Orbit::PHI2_PHI3>(); break;
				case Orbit::PHI21: map->template create_embedding<Orbit::PHI21>(); break;
				case Orbit::PHI21_PHI31: map->template create_embedding<Orbit::PHI21_PHI31>(); break;
				case Orbit::PHI1_PHI2_PHI3: map->template create_embedding<Orbit::PHI1_PHI2_PHI3>(); break;
				default: break;
			}
		};

		for (Orbit orb : {DART, PHI1, PHI2, PHI1_PHI2, PHI1_PHI3, PHI2_PHI3, PHI21, PHI21_PHI31, PHI1_PHI2_PHI3})
			if (!this->is_embedded(orb) && this->is_embedded(orb))
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
			switch (orb) {
				case Orbit::DART: map->new_orbit_embedding(Cell<Orbit::DART>(d)); break;
				case Orbit::PHI1: map->new_orbit_embedding(Cell<Orbit::PHI1>(d)); break;
				case Orbit::PHI2:map->new_orbit_embedding(Cell<Orbit::PHI2>(d)); break;
				case Orbit::PHI1_PHI2: map->new_orbit_embedding(Cell<Orbit::PHI1_PHI2>(d)); break;
				case Orbit::PHI1_PHI3: map->new_orbit_embedding(Cell<Orbit::PHI1_PHI3>(d)); break;
				case Orbit::PHI2_PHI3: map->new_orbit_embedding(Cell<Orbit::PHI2_PHI3>(d)); break;
				case Orbit::PHI21: map->new_orbit_embedding(Cell<Orbit::PHI21>(d)); break;
				case Orbit::PHI21_PHI31: map->new_orbit_embedding(Cell<Orbit::PHI21_PHI31>(d)); break;
				case Orbit::PHI1_PHI2_PHI3: map->new_orbit_embedding(Cell<Orbit::PHI1_PHI2_PHI3>(d)); break;
				default: break;
			}
		};

		for (Orbit orb : {DART, PHI1, PHI2, PHI1_PHI2, PHI1_PHI3, PHI2_PHI3, PHI21, PHI21_PHI31, PHI1_PHI2_PHI3})
		{
			if (this->is_embedded(orb))
			{
				for (uint32 j=first, end = this->topology_.end(); j!= end; this->topology_.next(j))
				{
					if (((orb != Boundary::ORBIT) && (orb != Orbit::DART)) || (!this->is_boundary(Dart(j))))
						if ((*this->embeddings_[orb])[j] == INVALID_INDEX)
							new_orbit_embedding(this, Dart(j), orb);
				}
			}
		}
	}
};

template <typename MAP_TRAITS>
struct CMap3Type
{
	using TYPE = CMap3_T<MAP_TRAITS, CMap3Type<MAP_TRAITS>>;
};

template <typename MAP_TRAITS>
using CMap3 = CMap3_T<MAP_TRAITS, CMap3Type<MAP_TRAITS>>;

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_CMAP_CMAP3_CPP_))
extern template class CGOGN_CORE_API CMap3_T<DefaultMapTraits, CMap3Type<DefaultMapTraits>>;
extern template class CGOGN_CORE_API DartMarker<CMap3<DefaultMapTraits>>;
extern template class CGOGN_CORE_API DartMarkerStore<CMap3<DefaultMapTraits>>;
extern template class CGOGN_CORE_API DartMarkerNoUnmark<CMap3<DefaultMapTraits>>;
extern template class CGOGN_CORE_API CellMarker<CMap3<DefaultMapTraits>, CMap3<DefaultMapTraits>::Vertex::ORBIT>;
extern template class CGOGN_CORE_API CellMarker<CMap3<DefaultMapTraits>, CMap3<DefaultMapTraits>::Edge::ORBIT>;
extern template class CGOGN_CORE_API CellMarker<CMap3<DefaultMapTraits>, CMap3<DefaultMapTraits>::Face::ORBIT>;
extern template class CGOGN_CORE_API CellMarker<CMap3<DefaultMapTraits>, CMap3<DefaultMapTraits>::Volume::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap3<DefaultMapTraits>, CMap3<DefaultMapTraits>::Vertex::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap3<DefaultMapTraits>, CMap3<DefaultMapTraits>::Edge::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap3<DefaultMapTraits>, CMap3<DefaultMapTraits>::Face::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap3<DefaultMapTraits>, CMap3<DefaultMapTraits>::Volume::ORBIT>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_MAP_MAP3_CPP_))

} // namespace cgogn

#include <cgogn/core/cmap/cmap3_builder.h>

#endif // CGOGN_CORE_CMAP_CMAP3_H_
