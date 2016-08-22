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

#ifndef CGOGN_CORE_CMAP_CMAP2_H_
#define CGOGN_CORE_CMAP_CMAP2_H_

#include <cgogn/core/cmap/cmap1.h>

namespace cgogn
{

// forward declaration of CMap2Builder_T
template <typename MAP_TRAITS> class CMap2Builder_T;

template <typename MAP_TRAITS, typename MAP_TYPE>
class CMap2_T : public CMap1_T<MAP_TRAITS, MAP_TYPE>
{
public:
	static const uint8 DIMENSION = 2;

	static const uint8 PRIM_SIZE = 1;

	using MapTraits = MAP_TRAITS;
	using MapType = MAP_TYPE;
	using Inherit = CMap1_T<MAP_TRAITS, MAP_TYPE>;
	using Self = CMap2_T<MAP_TRAITS, MAP_TYPE>;

	using Builder = CMap2Builder_T<MapTraits>;

	friend class MapBase<MAP_TRAITS, MAP_TYPE>;
	friend class CMap2Builder_T<MapTraits>;
	friend class DartMarker_T<Self>;
	friend class cgogn::DartMarkerStore<Self>;

	using CDart		= typename Inherit::Vertex;
	using Vertex	= Cell<Orbit::PHI21>;
	using Edge		= Cell<Orbit::PHI2>;
	using Face		= typename Inherit::Face;
	using Volume	= Cell<Orbit::PHI1_PHI2>;

	using Boundary  = Face;
	using ConnectedComponent = Volume;

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
	using DartMarkerNoUnmark = typename cgogn::DartMarkerNoUnmark<Self>;

	template <Orbit ORBIT>
	using CellMarker = typename cgogn::CellMarker<Self, ORBIT>;
	template <Orbit ORBIT>
	using CellMarkerStore = typename cgogn::CellMarkerStore<Self, ORBIT>;

protected:

	ChunkArray<Dart>* phi2_;

	inline void init()
	{
		phi2_ = this->topology_.template add_chunk_array<Dart>("phi2");
	}

public:

	CMap2_T() : Inherit()
	{
		init();
	}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CMap2_T);

	~CMap2_T() override
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
	 * \brief Init an newly added dart.
	 * The dart is defined as a fixed point for PHI2.
	 */
	inline void init_dart(Dart d)
	{
		Inherit::init_dart(d);
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
		return (Inherit::check_integrity(d) &&
				phi2(phi2(d)) == d &&
				phi2(d) != d);
	}

	/**
	 * @brief Check the integrity of a boundary dart
	 * @param d the dart to check
	 * @return true if the bondary constraints are locally statisfied
	 * The boundary is a 1-manifold: the boundary marker is the same
	 * for all darts of a face and two boundary faces cannot be adjacent.
	 */
	inline bool check_boundary_integrity(Dart d) const
	{
		return ((  this->is_boundary(d) ==  this->is_boundary(this->phi1(d)) ) &&
				( !this->is_boundary(d) || !this->is_boundary(this->phi2(d)) ));
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
	 * The composition of phi is applied in order of declaration : phi<12>(d) = phi2(phi1(d))
	 */
	template <uint64 N>
	inline Dart phi(Dart d) const
	{
		static_assert((N % 10) <= 2, "Composition of PHI: invalid index");
		switch(N % 10)
		{
			case 1 : return this->phi1(phi<N / 10>(d));
			case 2 : return this->phi2(phi<N / 10>(d));
			default : return d;
		}
	}

	/*******************************************************************************
	 * High-level embedded and topological operations
	 *******************************************************************************/

protected:

	/**
	 * \brief Add a face in the map.
	 * \param size : the number of darts in the built face
	 * \return A dart of the built face.
	 * Two 1-face are built. The first one is the returned face,
	 * the second is a boundary face that closes the map.
	 */
	Dart add_face_topo(std::size_t size)
	{
		Dart d = Inherit::add_face_topo(size);
		Dart e = Inherit::add_face_topo(size);

		this->foreach_dart_of_PHI1(d, [&] (Dart it)
		{
			this->set_boundary(e, true);
			phi2_sew(it, e);
			e = this->phi_1(e);
		});

		return d;
	}

public:

	/**
	 * \brief Add a face in the map.
	 * \param size : the number of edges in the built face
	 * \return The built face
	 * If the map has Dart, Vertex, Edge, Face or Volume attributes,
	 * the inserted cells are automatically embedded on new attribute elements.
	 * More precisely :
	 *  - a Face attribute is created, if needed, for the new face.
	 */
	Face add_face(uint32 size)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		const Face f(add_face_topo(size));

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

	/**
	 * \brief Add a pyramid whose base has n sides.
	 * \param size : the number of darts in the base face
	 * \return A dart of the base face
	 * The base is a face with n vertices and edges.
	 * Each edge is adjacent to a triangular face.
	 * These triangles are pairwise sewn to build the top of the pyramid.
	 */
	inline Dart add_pyramid_topo(uint32 size)
	{
		cgogn_message_assert(size > 0u, "The pyramid cannot be empty");

		Dart first = this->Inherit::add_face_topo(3u);	// First triangle
		Dart current = first;

		for (uint32 i = 1u; i < size; ++i)				// Next triangles
		{
			Dart next = this->Inherit::add_face_topo(3u);
			this->phi2_sew(this->phi_1(current), this->phi1(next));
			current = next;
		}
														// End the umbrella
		this->phi2_sew(this->phi_1(current), this->phi1(first));

		return this->close_hole_topo(first);			// Add the base face
	}

	/**
	 * \brief Add a prism with n sides.
	 * \param size : the number of sides of the prism
	 * \return A dart of the base face
	 * The base and the top are faces with n vertices and edges.
	 * A set of n pairwise linked quads are built.
	 * These quads are sewn to the base and top faces.
	 */
	Dart add_prism_topo(uint32 size)
	{
		cgogn_message_assert(size > 0u, "The prism cannot be empty");

		Dart first = this->Inherit::add_face_topo(4u);			// First quad
		Dart current = first;

		for (uint32 i = 1u; i < size; ++i)						// Next quads
		{
			Dart next = this->Inherit::add_face_topo(4u);
			this->phi2_sew(this->phi_1(current), this->phi1(next));
			current = next;
		}

		this->phi2_sew(this->phi_1(current), this->phi1(first));// Close the quad strip

		this->close_hole_topo(this->phi1(this->phi1(first)));	// Add the top face

		return this->close_hole_topo(first);					// Add the base face
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
		Dart e = phi2(d);						// Get the adjacent 1D-edge

#ifndef	NDEBUG
		phi2_unsew(d);							// Separate the two 1D-edges of the edge
#endif
		Dart nd = this->Inherit::split_vertex_topo(d);
		Dart ne = this->Inherit::split_vertex_topo(e);	// Cut the two adjacent 1D-edges

		phi2_sew(d, ne);						// Sew the new 1D-edges
		phi2_sew(e, nd);						// To build the new 2D-edges

		this->set_boundary(nd, this->is_boundary(d));
		this->set_boundary(ne, this->is_boundary(e));

		return nd;
	}

public:

	/**
	 * \brief Cut an edge.
	 * \param e : the edge to cut
	 * \return The inserted vertex
	 * The edge e is cut by inserting a new vertex.
	 * The returned vertex is represented by the dart of the inserted vertex that belongs to the face of e.dart
	 * If the map has Dart, Vertex, Edge, Face or Volume attributes,
	 * the inserted cells are automatically embedded on new attribute elements.
	 * More precisely :
	 *  - a Vertex attribute is created, if needed, for the inserted vertex.
	 *  - an Edge attribute is created, if needed, for the edge inserted after e.
	 *  - the Edge attribute of e is kept unchanged.
	 */
	inline Vertex cut_edge(Edge e)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		const Dart v = cut_edge_topo(e.dart);
		const Dart nf = phi2(e.dart);
		const Dart f = phi2(v);

		if (this->template is_embedded<CDart>())
		{
			if (!this->is_boundary(v)) this->new_orbit_embedding(CDart(v));
			if (!this->is_boundary(nf)) this->new_orbit_embedding(CDart(nf));
		}

		if (this->template is_embedded<Vertex>())
			this->new_orbit_embedding(Vertex(v));

		if (this->template is_embedded<Edge>())
		{
			this->template copy_embedding<Edge>(nf, e.dart);
			this->new_orbit_embedding(Edge(v));
		}

		if (this->template is_embedded<Face>())
		{
			if (!this->is_boundary(e.dart))
				this->template copy_embedding<Face>(v, e.dart);
			if (!this->is_boundary(f))
				this->template copy_embedding<Face>(nf, f);
		}

		if (this->template is_embedded<Volume>())
		{
			this->template copy_embedding<Volume>(v, e.dart);
			this->template copy_embedding<Volume>(nf, e.dart);
		}

		return Vertex(v);
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
		if (!this->is_boundary(d) && !this->is_boundary(e))
		{
			Dart d1 = this->phi1(d);
			Dart d_1 = this->phi_1(d);
			Dart e1 = this->phi1(e);
			Dart e_1 = this->phi_1(e);

			// Cannot flip edge whose incident faces have co-degree 1
			if (d == d1  || e == e1) return false;

			// Both vertices have degree 1 and thus nothing is done // TODO may return true ?
			if (d == e_1 && e == d_1) return false;

			if (d != e_1) this->phi1_sew(d, e_1);	// Detach the edge from its
			if (e != d_1) this->phi1_sew(e, d_1);	// two incident vertices

			if (d != e_1) this->phi1_sew(d, d1);	// Insert the first end in its new vertices
			if (e != d_1) this->phi1_sew(e, e1);	// Insert the second end in its new vertices

			return true;
		}
		return false;
	}

	inline bool flip_back_edge_topo(Dart d)
	{
		const Dart e = phi2(d);
		if (!this->is_boundary(d) && !this->is_boundary(e))
		{
			const Dart d_1 = this->phi_1(d);
			const Dart e_1= this->phi_1(e);
			this->phi1_sew(d, e_1);				// Detach the two
			this->phi1_sew(e, d_1);				// vertices of the edge
			this->phi1_sew(e, this->phi_1(d_1));	// Insert the edge in its
			this->phi1_sew(d, this->phi_1(e_1));	// new vertices after flip
			return true;
		}
		return false;
	}

public:

	/**
	 * @brief Flip an edge
	 * @param e : the edge to flip
	 * The two endpoints of the given edge are moved to the next vertices
	 * of their two incident faces
	 */
	inline void flip_edge(Edge e)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		if (flip_edge_topo(e.dart))
		{
			Dart d = e.dart;
			Dart d2 = phi2(d);

			if (this->template is_embedded<Vertex>())
			{
				this->template copy_embedding<Vertex>(d, this->phi1(d2));
				this->template copy_embedding<Vertex>(d2, this->phi1(d));
			}

			if (this->template is_embedded<Face>())
			{
				this->template copy_embedding<Face>(this->phi_1(d), d);
				this->template copy_embedding<Face>(this->phi_1(d2), d2);
			}
		}
	}

	inline void flip_back_edge(Edge e)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		if (flip_back_edge_topo(e.dart))
		{
			const Dart d = e.dart;
			const Dart d2 = phi2(d);

			if (this->template is_embedded<Vertex>())
			{
				this->template copy_embedding<Vertex>(d, this->phi1(d2));
				this->template copy_embedding<Vertex>(d2, this->phi1(d));
			}

			if (this->template is_embedded<Face>())
			{
				this->template copy_embedding<Face>(this->phi1(d), d);
				this->template copy_embedding<Face>(this->phi1(d2), d2);
			}
		}
	}

protected:

	/**
	 * @brief Collapse an edge
	 * @param d : a dart of the edge to collapse
	 * @return a dart of the resulting vertex
	 */
	inline Dart collapse_edge_topo(Dart d)
	{
		Dart res = phi2(this->phi_1(d));

		Dart e = phi2(d);
		this->remove_vertex_topo(d);
		this->remove_vertex_topo(e);

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

		Vertex v(collapse_edge_topo(e.dart));

		if (this->template is_embedded<Vertex>())
		{
			uint32 emb = this->embedding(v);
			foreach_dart_of_orbit(v, [this, emb] (Dart d) { this->template set_embedding<Vertex>(d, emb); });
		}

		return v;
	}

	inline void split_vertex(Dart d, Dart e)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		const Dart dd = phi2(d) ;
		const Dart ee = phi2(e) ;

		split_vertex_topo(d, e) ;

		if (this->template is_embedded<CDart>())
		{
			this->new_orbit_embedding(CDart(this->phi1(dd)));
			this->new_orbit_embedding(CDart(this->phi1(ee)));
		}

		if (this->template is_embedded<Vertex>())
		{
			this->new_orbit_embedding(Vertex(e));
			this->template set_embedding<Vertex>(this->phi1(dd), this->embedding(Vertex(d)));
		}

		if (this->template is_embedded<Edge>())
		{
			this->new_orbit_embedding(Edge(this->phi1(dd)));
		}

		if (this->template is_embedded<Face>())
		{
			this->template set_embedding<Vertex>(this->phi1(dd), this->embedding(Face(dd)));
			this->template set_embedding<Vertex>(this->phi1(ee), this->embedding(Face(ee)));
		}
	}

protected:

	inline void split_vertex_topo(Dart d, Dart e)
	{
		cgogn_assert(this->same_orbit(Vertex(d), Vertex(e)));

		const Dart d2 = phi2(d);
		const Dart e2 = phi2(e);
		cgogn_assert(d != d2);
		cgogn_assert(e != e2);
		const Dart nd = Inherit::split_vertex_topo(d2);
		const Dart ne = Inherit::split_vertex_topo(e2);
		phi2_sew(nd,ne);
	}

	void merge_adjacent_edges_topo(Dart d)
	{
		Dart e = this->phi_1(this->phi2(d));
		cgogn_message_assert(d == this->phi_1(this->phi2(e)),
							 "merge_adjacent_edge: the degree of the vertex of d should be 2");
		// TODO
	}

	/**
	 * \brief Merge the two faces incident to the edge of d by removing the edge of d
	 * \param d : dart of the edge
	 * \return true if the faces have been merged, false otherwise
	 * The endpoints of the edge of d are detached from their vertex and then the edge is removed.
	 * If the edge of d is incident to the boundary, nothing is done.
	 */
	bool merge_incident_faces_topo(Dart d)
	{
		if (this->is_incident_to_boundary(Edge(d)))
			return false;

		Dart d2 = phi2(d);
		this->phi1_sew(this->phi_1(d), d2);
		this->phi1_sew(this->phi_1(d2), d);
		this->Inherit::remove_face_topo(d);

		return true;
	}

public:

	/**
	 * \brief Merge the two faces incident to the given edge
	 * \param e : the edge
	 * The endpoints of the edge of d are detached from their vertex and then the edge is removed.
	 * If the map has Face attributes, the attributes of the Face of e.dart are kept on the resulting face.
	 * If the edge of d is incident to the boundary, nothing is done.
	 */
	void merge_incident_faces(Edge e)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		Dart d1 = this->phi1(e.dart);
		if (merge_incident_faces_topo(e.dart))
		{
			if (this->template is_embedded<Face>())
			{
				uint32 emb = this->embedding(Face(d1));
				foreach_dart_of_orbit(Face(d1), [this, emb] (Dart d)
				{
					this->template set_embedding<Face>(d, emb);
				});
			}
		}
	}

protected:

	/**
	 * \brief Cut the face of d and e by inserting an edge between the vertices of d and e
	 * \param d : first vertex
	 * \param e : second vertex
	 * \return A dart of the inserted edge
	 * Darts d and e should belong to the same face and be distinct from each other.
	 * An edge made of two new darts is inserted between the two given vertices.
	 * The returned dart is the dart of the inserted edge that belongs to the face of d.
	 */
	inline Dart cut_face_topo(Dart d, Dart e)
	{
		cgogn_message_assert(d != e, "cut_face_topo: d and e should be distinct");
		cgogn_message_assert(this->same_cell(Face(d), Face(e)), "cut_face_topo: d and e should belong to the same face");

		Dart dd = this->phi_1(d);
		Dart ee = this->phi_1(e);
		Dart nd = Inherit::split_vertex_topo(dd);	// cut the edge before d (insert a new dart before d)
		Dart ne = Inherit::split_vertex_topo(ee);	// cut the edge before e (insert a new dart before e)
		this->phi1_sew(dd, ee);						// subdivide phi1 cycle at the inserted darts
		phi2_sew(nd, ne);							// build the new 2D-edge from the inserted darts

		this->set_boundary(nd, this->is_boundary(dd));
		this->set_boundary(ne, this->is_boundary(ee));

		return nd;
	}

public:

	/**
	 * \brief Cut a face by inserting an edge between the vertices of d and e
	 * \param d : a dart of the first vertex
	 * \param e : a dart of the second vertex
	 * \return The inserted edge
	 * The darts d and e should belong to the same Face and be distinct from each other.
	 * An edge is inserted between the two given vertices.
	 * The returned edge is represented by the dart of the inserted edge that belongs to the face of d.dart
	 * If the map has Dart, Vertex, Edge, Face or Volume attributes,
	 * the inserted cells are automatically embedded on new attribute elements.
	 * More precisely :
	 *  - an Edge attribute is created, if needed, for the inserted edge.
	 *  - a Face attribute is created, if needed, for the subdivided face that e belongs to.
	 *  - the Face attribute of the subdivided face that d belongs to is kept unchanged.
	 */
	inline Edge cut_face(Dart d, Dart e)
	{
		CGOGN_CHECK_CONCRETE_TYPE;
		cgogn_message_assert(!this->is_boundary(d), "cut_face: should not cut a boundary face");

		Dart nd = cut_face_topo(d, e);
		Dart ne = this->phi_1(e);

		if (this->template is_embedded<CDart>())
		{
			this->new_orbit_embedding(CDart(nd));
			this->new_orbit_embedding(CDart(ne));
		}

		if (this->template is_embedded<Vertex>())
		{
			this->template copy_embedding<Vertex>(nd, e);
			this->template copy_embedding<Vertex>(ne, d);
		}

		if (this->template is_embedded<Edge>())
			this->new_orbit_embedding(Edge(nd));

		if (this->template is_embedded<Face>())
		{
			this->template copy_embedding<Face>(nd, d);
			this->new_orbit_embedding(Face(ne));
		}

		if (this->template is_embedded<Volume>())
		{
			this->template copy_embedding<Volume>(nd, d);
			this->template copy_embedding<Volume>(ne, d);
		}

		return Edge(nd);
	}

protected:

	/**
	 * @brief Unsew the faces incident to the edge of d
	 * @param d : dart of the edge
	 * @return true if the faces have been unsewn, false otherwise
	 * The two faces are detached, a 2-sided boundary face is inserted.
	 * For each of the two end vertices of the edge, if it is already incident to a boundary
	 * face, the new boundary face is merged with the existing boundary, resulting in a separation
	 * of the vertex into two vertices.
	 */
	inline bool unsew_faces_topo(Dart d)
	{
		if (this->is_incident_to_boundary(Edge(d)))
			return false;

		Dart dd = phi2(d);

		Dart e = Inherit::add_face_topo(2);
		Dart ee = this->phi1(e);
		this->set_boundary(e, true);
		this->set_boundary(ee, true);

		Dart f = this->boundary_dart(Vertex(d));
		Dart ff = this->boundary_dart(Vertex(dd));

		if(!f.is_nil())
			this->phi1_sew(e, this->phi_1(f));

		if(!ff.is_nil())
			this->phi1_sew(ee, this->phi_1(ff));

		phi2_unsew(d);
		phi2_sew(d, e);
		phi2_sew(dd, ee);

		return true;
	}

public:

	/**
	 * @brief Unsew the faces incident to the edge of d
	 * @param d : dart of the edge
	 * @return true if the faces have been unsewn, false otherwise
	 * The two faces are detached, a 2-sided boundary face is inserted.
	 * For each of the two end vertices of the edge, if it is already incident to a boundary
	 * face, the new boundary face is merged with the existing boundary, resulting in a split
	 * of the vertex into two vertices.
	 * If the map has Vertex attributes, in case of vertex split, the attributes of the vertices of e.dart
	 * and phi2(e.dart) are kept on their side and new attributes are created for the new vertices
	 * If the map has Edge attributes, a new attribute is created for the edge of phi2(e.dart)
	 * If the map has Volume attribute, in the case of volume split, a new attribute is created
	 * for the volume
	 */
	inline void unsew_faces(Edge e)
	{
		Dart d = e.dart;
		Dart d2 = phi2(d);
		if (unsew_faces_topo(d))
		{
			if (this->template is_embedded<Vertex>())
			{
				Dart d21 = this->phi1(d2);
				if (this->same_orbit(Vertex(d), Vertex(d21)))
					this->template copy_embedding<Vertex>(phi2(d2), d21);
				else
					this->new_orbit_embedding(Vertex(d21));

				Dart d1 = this->phi1(d);
				if (this->same_orbit(Vertex(d2), Vertex(d1)))
					this->template copy_embedding<Vertex>(phi2(d), d1);
				else
					this->new_orbit_embedding(Vertex(d1));
			}

			if (this->template is_embedded<Edge>())
				this->new_orbit_embedding(Edge(d2));

			if (this->template is_embedded<Volume>())
			{
				if (this->same_orbit(Volume(d), Volume(d2)))
				{
					this->template copy_embedding<Volume>(phi2(d2), d);
					this->template copy_embedding<Volume>(phi2(d), d);
				}
				else
					this->new_orbit_embedding(Volume(d2));
			}
		}
	}

protected:

	/*!
	 * \brief Close the topological hole that contains Dart d (a fixed point for PHI2).
	 * \param d : a vertex of the hole
	 * \return a vertex of the face that closes the hole
	 * This method is used to close a CMap2 that has been build through the 2-sewing of 1-faces.
	 * A face is inserted on the boundary that begin at dart d.
	 */
	inline Dart close_hole_topo(Dart d)
	{
		cgogn_message_assert(phi2(d) == d, "CMap2: close hole called on a dart that is not a phi2 fix point");

		Dart first = this->add_topology_element();	// First edge of the face that will fill the hole
		phi2_sew(d, first);							// 2-sew the new edge to the hole

		Dart d_next = d;							// Turn around the hole
		Dart d_phi1;								// to complete the face
		do
		{
			do
			{
				d_phi1 = this->phi1(d_next); // Search and put in d_next
				d_next = phi2(d_phi1); // the next dart of the hole
			} while (d_next != d_phi1 && d_phi1 != d);

			if (d_phi1 != d)
			{
				Dart next = this->Inherit::split_vertex_topo(first);	// Add a vertex into the built face
				phi2_sew(d_next, next);						// and 2-sew the face to the hole
			}
		} while (d_phi1 != d);

		return first;
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
		if (this->phi1(e.dart) == e.dart)
			return 1;
		else
			return 2;
	}

	inline uint32 degree(Edge e) const
	{
		if (this->is_boundary(e.dart) || this->is_boundary(phi2(e.dart)))
			return 1;
		else
			return 2;
	}

	inline uint32 codegree(Face f) const
	{
		return Inherit::codegree(f);
	}

	inline uint32 degree(Face) const
	{
		return 1;
	}

	inline uint32 codegree(Volume v) const
	{
		uint32 result = 0;
		foreach_incident_face(v, [&result] (Face) { ++result; });
		return result;
	}

	/**
	 * @brief Check if the given darts form a simple closed oriented path within a single Volume
	 * @param path : a vector of darts
	 * @return
	 */
	bool simple_closed_oriented_path(const std::vector<Dart>& path)
	{
		DartMarkerStore dm(*this);
		Dart prev = path.back();
		for (Dart d : path)
		{
			if(dm.is_marked(d))
				return false;
			dm.mark_orbit(Vertex(d));

			if(!this->same_cell(Vertex(d), Vertex(this->phi1(prev))))
				return false;

			prev = d;
		}
		return true;
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
			f(it);
			it = phi2(this->phi_1(it));
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
					it = this->phi1(it);
				} while (it != e);
			}
		}
		cgogn::dart_buffers()->release_buffer(visited_faces);
	}

public:

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
			case Orbit::PHI1: this->foreach_dart_of_PHI1(c.dart, f); break;
			case Orbit::PHI2: foreach_dart_of_PHI2(c.dart, f); break;
			case Orbit::PHI1_PHI2: foreach_dart_of_PHI1_PHI2(c.dart, f); break;
			case Orbit::PHI21: foreach_dart_of_PHI21(c.dart, f); break;
			case Orbit::PHI2_PHI3:
			case Orbit::PHI1_PHI3:
			case Orbit::PHI21_PHI31:
			case Orbit::PHI1_PHI2_PHI3:
			default: cgogn_assert_not_reached("Orbit not supported in a CMap2"); break;
		}
	}

protected:

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
			if (!f(it))
				break;
			it = phi2(this->phi_1(it));
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
					it = this->phi1(it);
				} while (it != e);
			}
		}
		cgogn::dart_buffers()->release_buffer(visited_faces);
	}

public:

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
			case Orbit::PHI1: this->foreach_dart_of_PHI1_until(c.dart, f); break;
			case Orbit::PHI2: foreach_dart_of_PHI2_until(c.dart, f); break;
			case Orbit::PHI1_PHI2: foreach_dart_of_PHI1_PHI2_until(c.dart, f); break;
			case Orbit::PHI21: foreach_dart_of_PHI21_until(c.dart, f); break;
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

	template <typename FUNC>
	inline void foreach_incident_edge(Vertex v, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Edge>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(v, [&func] (Dart d) { func(Edge(d)); });
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
		foreach_dart_of_orbit(e, [&func] (Dart d) { func(Vertex(d)); });
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
		foreach_dart_of_orbit(v, [this, &f] (Dart d) { f(Vertex(this->phi2(d))); });
	}

	template <typename FUNC>
	inline void foreach_adjacent_vertex_through_face(Vertex v, const FUNC& f) const
	{
		static_assert(is_func_parameter_same<FUNC, Vertex>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(v, [this, &f] (Dart vd)
		{
			if (!this->is_boundary(vd))
			{
				Dart vd1 = this->phi1(vd);
				this->foreach_dart_of_orbit(Face(vd), [&f, vd, vd1] (Dart fd)
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
			this->foreach_dart_of_orbit(Vertex(ed), [&f, ed] (Dart vd)
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
				this->foreach_dart_of_orbit(Face(ed), [&f, ed] (Dart fd)
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
			Dart fd1 = this->phi2(this->phi_1(fd));
			this->foreach_dart_of_orbit(Vertex(fd), [this, &func, fd, fd1] (Dart vd)
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
			const Dart d2 = this->phi2(d);
			if (!this->is_boundary(d2))
				func(Face(d2));
		});
	}

	inline std::pair<Vertex,Vertex> vertices(Edge e) const
	{
		return std::pair<Vertex, Vertex>(Vertex(e.dart), Vertex(this->phi1(e.dart)));
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
				case Orbit::PHI21: map->template create_embedding<Orbit::PHI21>(); break;
				default: break;
			}
		};

		for (Orbit orb : {DART, PHI1, PHI2, PHI1_PHI2, PHI21})
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
			switch (orb) {
				case Orbit::DART: map->new_orbit_embedding(Cell<Orbit::DART>(d)); break;
				case Orbit::PHI1: map->new_orbit_embedding(Cell<Orbit::PHI1>(d)); break;
				case Orbit::PHI2:map->new_orbit_embedding(Cell<Orbit::PHI2>(d)); break;
				case Orbit::PHI1_PHI2: map->new_orbit_embedding(Cell<Orbit::PHI1_PHI2>(d)); break;
				case Orbit::PHI21: map->new_orbit_embedding(Cell<Orbit::PHI21>(d)); break;
				default: break;
			}
		};

		for (Orbit orb : {DART, PHI1, PHI2, PHI1_PHI2, PHI21})
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
struct CMap2Type
{
	using TYPE = CMap2_T<MAP_TRAITS, CMap2Type<MAP_TRAITS>>;
};

template <typename MAP_TRAITS>
using CMap2 = CMap2_T<MAP_TRAITS, CMap2Type<MAP_TRAITS>>;

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_MAP_MAP2_CPP_))
extern template class CGOGN_CORE_API CMap2_T<DefaultMapTraits, CMap2Type<DefaultMapTraits>>;
extern template class CGOGN_CORE_API DartMarker<CMap2<DefaultMapTraits>>;
extern template class CGOGN_CORE_API DartMarkerStore<CMap2<DefaultMapTraits>>;
extern template class CGOGN_CORE_API DartMarkerNoUnmark<CMap2<DefaultMapTraits>>;
extern template class CGOGN_CORE_API CellMarker<CMap2<DefaultMapTraits>, CMap2<DefaultMapTraits>::Vertex::ORBIT>;
extern template class CGOGN_CORE_API CellMarker<CMap2<DefaultMapTraits>, CMap2<DefaultMapTraits>::Edge::ORBIT>;
extern template class CGOGN_CORE_API CellMarker<CMap2<DefaultMapTraits>, CMap2<DefaultMapTraits>::Face::ORBIT>;
extern template class CGOGN_CORE_API CellMarker<CMap2<DefaultMapTraits>, CMap2<DefaultMapTraits>::Volume::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap2<DefaultMapTraits>, CMap2<DefaultMapTraits>::Vertex::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap2<DefaultMapTraits>, CMap2<DefaultMapTraits>::Edge::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap2<DefaultMapTraits>, CMap2<DefaultMapTraits>::Face::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap2<DefaultMapTraits>, CMap2<DefaultMapTraits>::Volume::ORBIT>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_MAP_MAP2_CPP_))

} // namespace cgogn


#include <cgogn/core/cmap/cmap2_builder.h>

#endif // CGOGN_CORE_CMAP_CMAP2_H_
