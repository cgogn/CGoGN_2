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

#ifndef CORE_CMAP_CMAP2_H_
#define CORE_CMAP_CMAP2_H_

#include <core/cmap/cmap1.h>
#include <core/basic/dart_marker.h>

namespace cgogn
{

// forward declaration of CMap2Builder_T
template <typename MAP_TRAITS>
class CMap2Builder_T;

template <typename MAP_TRAITS, typename MAP_TYPE>
class CMap2_T : public CMap1_T<MAP_TRAITS, MAP_TYPE>
{
public:

	static const int PRIM_SIZE = 1;

	using MapTraits = MAP_TRAITS;
	using MapType = MAP_TYPE;
	using Inherit = CMap1_T<MAP_TRAITS, MAP_TYPE>;
	using Self = CMap2_T<MAP_TRAITS, MAP_TYPE>;

	friend class MapBase<MAP_TRAITS, MAP_TYPE>;
	friend class CMap2Builder_T<MapTraits>;
	template<typename T> friend class DartMarker_T;
	template<typename T> friend class DartMarkerStore;

	using CDart		= Cell<Orbit::DART>;
	using Vertex	= Cell<Orbit::PHI21>;
	using Edge		= Cell<Orbit::PHI2>;
	using Face		= Cell<Orbit::PHI1>;
	using Volume	= Cell<Orbit::PHI1_PHI2>;

	template <typename T>
	using ChunkArray =  typename Inherit::template ChunkArray<T>;
	template <typename T>
	using ChunkArrayContainer =  typename Inherit::template ChunkArrayContainer<T>;

	template <typename T, Orbit ORBIT>
	using AttributeHandler = typename Inherit::template AttributeHandler<T, ORBIT>;
	template <typename T>
	using VertexAttributeHandler = AttributeHandler<T, Vertex::ORBIT>;
	template <typename T>
	using EdgeAttributeHandler = AttributeHandler<T, Edge::ORBIT>;
	template <typename T>
	using FaceAttributeHandler = AttributeHandler<T, Face::ORBIT>;
	template <typename T>
	using VolumeAttributeHandler = AttributeHandler<T, Volume::ORBIT>;

	using DartMarker = typename cgogn::DartMarker<Self>;
	using DartMarkerStore = typename cgogn::DartMarkerStore<Self>;

	template <Orbit ORBIT>
	using CellMarker = typename cgogn::CellMarker<Self, ORBIT>;

protected:

	ChunkArray<Dart>* phi2_;

	inline void init()
	{
		phi2_ = this->topology_.template add_attribute<Dart>("phi2");
	}

public:

	CMap2_T() : Inherit()
	{
		init();
	}

	~CMap2_T() override
	{}

	CMap2_T(Self const&) = delete;
	CMap2_T(Self &&) = delete;
	Self& operator=(Self const&) = delete;
	Self& operator=(Self &&) = delete;

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

	/*******************************************************************************
	 * High-level embedded and topological operations
	 *******************************************************************************/

protected:

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

		phi2_unsew(d);							// Separate the two 1D-edges of the edge
		Dart nd = this->split_vertex_topo(d);
		Dart ne = this->split_vertex_topo(e);	// Cut the two adjacent 1D-edges

		phi2_sew(d, ne);						// Sew the new 1D-edges
		phi2_sew(e, nd);						// To build the new 2D-edges

		return nd;
	}

public:

	/**
	 * \brief Cut an embedded edge.
	 * \param d : A dart that represents the edge to cut
	 * \return A dart of the inserted vertex
	 * The edge of d is cut by inserting a new vertex.
	 * The returned dart is the dart of the inserted vertex that belongs to the face of d.
	 * If the map has Dart, Vertex, Edge, Face or Volume attributes,
	 * the inserted darts are automatically embedded on new attribute elements.
	 *  - Actually a Vertex attribute is created, if needed, for the inserted vertex.
	 *  - If needed, an Edge attribute is created for the edge inserted after e.
	 *    The Edge attribute of e is kept unchanged.
	 */
	inline Vertex cut_edge(Edge e)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		const Vertex v = cut_edge_topo(e);
		const Dart nf = phi2(e);
		const Dart  f = phi2(v);

		if (this->template is_embedded<CDart>()) {
			this->new_embedding(CDart(v));
			this->new_embedding(CDart(nf));
		}

		if (this->template is_embedded<Vertex>())
		{
			this->new_embedding(Vertex(v));
		}

		if (this->template is_embedded<Edge>())
		{
			this->copy_embedding(Edge(nf), e);
			this->new_orbit_embedding(Edge(v));
		}

		if (this->template is_embedded<Face>())
		{
			this->copy_embedding(Face(v), Face(e));
			this->copy_embedding(Face(nf), Face(f));
		}

		if (this->template is_embedded<Volume>())
		{
			this->copy_embedding(Volume(v),Volume(e));
			this->copy_embedding(Volume(nf), Volume(e));
		}

		return v;
	}

protected:
	void merge_adjacent_edges_topo(Dart d)	{
		Dart e = this->phi_1(this->phi2(d));
		cgogn_message_assert(d == this->phi_1(this->phi2(e)),
							 "merge_adjacent_edge: the degree of the vertex of d should be 2");
// TODO
	}

	void merge_adjacent_faces_topo(Dart d)	{
		Dart e = this->phi2(d);
// TODO
	}

protected:

	/**
	 * \brief Cut the face of d and e by inserting an edge between the vertex of d and e
	 * \param d : first vertex
	 * \param e : second vertex
	 * Darts d and e should belong to the same face and be distinct from each other.
	 * An edge made of two new darts is inserted between the two given vertices.
	 */
	inline void cut_face_topo(Dart d, Dart e)
	{
		cgogn_message_assert(d != e, "cut_face: d and e should be distinct");
		cgogn_message_assert(this->same_cell(Face(d), Face(e)), "cut_face: d and e should belong to the same face");

		Dart dd = this->phi_1(d);
		Dart ee = this->phi_1(e);
		Dart nd = Inherit::split_vertex_topo(dd);	// cut the edge before d (insert a new dart before d)
		Dart ne = Inherit::split_vertex_topo(ee);	// cut the edge before e (insert a new dart before e)
		this->phi1_sew(dd, ee);						// subdivide phi1 cycle at the inserted darts
		phi2_sew(nd, ne);							// build the new 2D-edge from the inserted darts
	}

public:

	/**
	 * \brief Cut an enbedded face by inserting an edge between the vertices d and e
	 * \param d : first vertex
	 * \param e : second vertex
	 * The vertices d and e should belong to the same face and be distinct from each other.
	 * An edge made of two new darts is inserted between the two given vertices.
	 * If the map has Dart, Vertex, Edge, Face or Volume attributes,
	 * the inserted darts are automatically embedded on new attribute elements.
	 * Actually an Edge attribute is created, if needed, for the inserted edge
	 * and a new Face attribute is created for the subdived face that e belongs to.
	 * The Face attribute of the subdived face that d belongs to is kept unchanged.
	 */
	inline void cut_face(Vertex d, Vertex e)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		cut_face_topo(d,e);
		Dart nd = this->phi_1(d);
		Dart ne = this->phi_1(e);

		if (this->template is_embedded<CDart>()) {
			this->new_embedding(CDart(nd));
			this->new_embedding(CDart(ne));
		}

		if (this->template is_embedded<Vertex>())
		{
			this->copy_embedding(Vertex(nd), e);
			this->copy_embedding(Vertex(ne), d);
		}

		if (this->template is_embedded<Edge>())
		{
			this->new_orbit_embedding(Edge(nd));
		}

		if (this->template is_embedded<Face>())
		{
			this->copy_embedding(Face(nd), Face(d.dart));
			this->new_orbit_embedding(Face(ne));
		}

		if (this->template is_embedded<Volume>())
		{
			this->copy_embedding(Volume(nd), Volume(d.dart));
			this->copy_embedding(Volume(ne), Volume(d.dart));
		}
	}

protected:

	/*!
	 * \brief Add an embedded face in the map.
	 * \param size : the number of darts in the built face
	 * \return A dart of the built face.
	 */
	Dart add_face_topo(unsigned int size)
	{
		Dart d = Inherit::add_face_topo(size);
		Dart e = Inherit::add_face_topo(size);

		Dart it = d;
		do
		{
			phi2_sew(it, e);
			it = this->phi1(it);
			e = this->phi_1(e);
		} while (it != d);

		return d;
	}

public:

	/*!
	 * \brief Add a face in the map.
	 * \param size : the number of edges in the built face
	 * \return A dart of the built face
	 * If the map has Dart, Vertex, Edge, Face or Volume attributes,
	 * the inserted darts are automatically embedded on new attribute elements.
	 * Actually a Face attribute is created, if needed, for the new face.
	 */
	Face add_face(unsigned int size)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		const Face f = add_face_topo(size);

		if (this->template is_embedded<CDart>())
			foreach_dart_of_orbit(f, [this] (CDart d)
			{
				this->new_embedding(d);
				this->new_embedding(CDart(phi2(d)));
			});

		if (this->template is_embedded<Vertex>())
			foreach_dart_of_orbit(f, [this] (Vertex v)
			{
				this->new_orbit_embedding(v);
			});

		if (this->template is_embedded<Edge>())
			foreach_dart_of_orbit(f, [this] (Edge e)
			{
				this->new_orbit_embedding(e);
			});

		if (this->template is_embedded<Face>())
			this->new_orbit_embedding(f);

		if (this->template is_embedded<Volume>())
			this->new_orbit_embedding(Volume(f.dart));

		return f;
	}

protected:

	inline void close_hole_topo(Dart d)
	{
		cgogn_message_assert(phi2(d) == d, "CMap2: close hole called on a dart that is not a phi2 fix point");

		Dart first = this->add_dart();	// First edge of the face that will fill the hole
		phi2_sew(d, first);				// phi2-link the new edge to the hole

		Dart d_next = d;				// Turn around the hole
		Dart d_phi1;					// to complete the face
		do
		{
			do
			{
				d_phi1 = this->phi1(d_next);	// Search and put in d_next
				d_next = phi2(d_phi1);			// the next dart of the hole
			} while (d_next != d_phi1 && d_phi1 != d);

			if (d_phi1 != d)
			{
				Dart next = this->add_dart();	// Add a new edge there and link it to the face
				this->phi1_sew(first, next);	// the edge is linked to the face
				phi2_sew(d_next, next);			// the face is linked to the hole
			}
		} while (d_phi1 != d);
	}

protected:

	/**
	 * @brief close_map closes the map so that there are no phi2 fix points
	 */
	void close_map()
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		for (Dart d : *this)
		{
			if (phi2(d) == d)
			{
				close_hole_topo(d);
				const Face new_face = phi2(d);

				if (this->template is_embedded<CDart>())
					foreach_dart_of_orbit(new_face, [this] (CDart d)
					{
						this->new_embedding(d);
					});

				if (this->template is_embedded<Vertex>())
					foreach_dart_of_orbit(new_face, [this] (Vertex v)
					{
						this->copy_embedding(v, Vertex(this->phi1(phi2(v))));
					});

				if (this->template is_embedded<Edge>())
					foreach_dart_of_orbit(new_face, [this] (Edge e)
					{
						this->copy_embedding(e, Edge(phi2(e)));
					});

				if (this->template is_embedded<Face>())
				{
					this->new_orbit_embedding(new_face);
				}

				if (this->template is_embedded<Volume>())
				{
					const unsigned int idx = this->template get_embedding(Volume(d));
					foreach_dart_of_orbit(new_face, [this, idx] (Volume v)
					{
						this->set_embedding(v, idx);
					});
				}
			}
		}
	}

public:

	inline unsigned int degree(Face f) const
	{
		return Inherit::degree(f);
	}

	inline unsigned int degree(Vertex v) const
	{
		return this->nb_darts(v);
	}

	/*******************************************************************************
	 * Orbits traversal
	 *******************************************************************************/

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

		std::vector<Dart>* visited_faces = cgogn::get_dart_buffers()->get_buffer();
		visited_faces->push_back(d); // Start with the face of d

		// For every face added to the list
		for(unsigned int i = 0; i < visited_faces->size(); ++i)
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
		cgogn::get_dart_buffers()->release_buffer(visited_faces);
	}

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_dart_of_orbit(Cell<ORBIT> c, const FUNC& f) const
	{
		static_assert(ORBIT == Orbit::DART || ORBIT == Orbit::PHI1 || ORBIT == Orbit::PHI2 ||
					  ORBIT == Orbit::PHI1_PHI2 || ORBIT == Orbit::PHI21,
					  "Orbit not supported in a CMap2");

		switch (ORBIT)
		{
			case Orbit::DART: f(c.dart); break;
			case Orbit::PHI1: this->foreach_dart_of_PHI1(c, f); break;
			case Orbit::PHI2: foreach_dart_of_PHI2(c, f); break;
			case Orbit::PHI1_PHI2: foreach_dart_of_PHI1_PHI2(c, f); break;
			case Orbit::PHI21: foreach_dart_of_PHI21(c, f); break;
			case Orbit::PHI2_PHI3:
			case Orbit::PHI1_PHI3:
			case Orbit::PHI21_PHI31:
			default: cgogn_assert_not_reached("Orbit not supported in a CMap2"); break;
		}
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
			if (!f(it))
				break;
			it = phi2(this->phi_1(it));
		} while (it != d);
	}

	template <typename FUNC>
	void foreach_dart_of_PHI1_PHI2_until(Dart d, const FUNC& f) const
	{
		DartMarkerStore marker(*this);

		std::vector<Dart>* visited_faces = cgogn::get_dart_buffers()->get_buffer();
		visited_faces->push_back(d); // Start with the face of d

		// For every face added to the list
		for(unsigned int i = 0; i < visited_faces->size(); ++i)
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
						cgogn::get_dart_buffers()->release_buffer(visited_faces);
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
		cgogn::get_dart_buffers()->release_buffer(visited_faces);
	}

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_dart_of_orbit_until(Cell<ORBIT> c, const FUNC& f) const
	{
		static_assert(check_func_return_type(FUNC, bool),
					  "Wrong function return type");

		static_assert(ORBIT == Orbit::DART || ORBIT == Orbit::PHI1 || ORBIT == Orbit::PHI2 ||
					  ORBIT == Orbit::PHI1_PHI2 || ORBIT == Orbit::PHI21,
					  "Orbit not supported in a CMap2");

		switch (ORBIT)
		{
			case Orbit::DART: f(c.dart); break;
			case Orbit::PHI1: this->foreach_dart_of_PHI1_until(c, f); break;
			case Orbit::PHI2: foreach_dart_of_PHI2_until(c, f); break;
			case Orbit::PHI1_PHI2: foreach_dart_of_PHI1_PHI2_until(c, f); break;
			case Orbit::PHI21: foreach_dart_of_PHI21_until(c, f); break;
			case Orbit::PHI2_PHI3:
			case Orbit::PHI1_PHI3:
			case Orbit::PHI21_PHI31:
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
		static_assert(check_func_parameter_type(FUNC, Edge), "Wrong function cell parameter type");
		foreach_dart_of_orbit(v, func);
	}

	template <typename FUNC>
	inline void foreach_incident_face(Vertex v, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Face), "Wrong function cell parameter type");
		foreach_dart_of_orbit(v, func);
	}

	template <typename FUNC>
	inline void foreach_incident_vertex(Edge e, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Vertex), "Wrong function cell parameter type");
		foreach_dart_of_orbit(e, func);
	}

	template <typename FUNC>
	inline void foreach_incident_face(Edge e, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Face), "Wrong function cell parameter type");
		foreach_dart_of_orbit(e, func);
	}

	template <typename FUNC>
	inline void foreach_incident_vertex(Face f, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Vertex), "Wrong function cell parameter type");
		foreach_dart_of_orbit(f, func);
	}

	template <typename FUNC>
	inline void foreach_incident_edge(Face f, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Edge), "Wrong function cell parameter type");
		foreach_dart_of_orbit(f, func);
	}

	template <typename FUNC>
	inline void foreach_incident_vertex(Volume w, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Vertex), "Wrong function cell parameter type");
		DartMarkerStore marker(*this);
		foreach_dart_of_orbit(w, [&] (Vertex v)
		{
			if (!marker.is_marked(v))
			{
				marker.template mark_orbit(v);
				f(v);
			}
		});
	}

	template <typename FUNC>
	inline void foreach_incident_edge(Volume w, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Edge), "Wrong function cell parameter type");
		DartMarkerStore marker(*this);
		foreach_dart_of_orbit(w, [&] (Edge e)
		{
			if (!marker.is_marked(e))
			{
				marker.template mark_orbit(e);
				f(e);
			}
		});
	}

	template <typename FUNC>
	inline void foreach_incident_face(Volume w, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Face), "Wrong function cell parameter type");
		DartMarkerStore marker(*this);
		foreach_dart_of_orbit(w, [&] (Dart d)
		{
			if (!marker.is_marked(d))
			{
				marker.template mark_orbit<Face::ORBIT>(d);
				f(d);
			}
		});
	}

	/*******************************************************************************
	 * Adjacence traversal
	 *******************************************************************************/

	template <typename FUNC>
	inline void foreach_adjacent_vertex_through_edge(Vertex v, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Vertex), "Wrong function cell parameter type");
		foreach_dart_of_orbit(v, [this, &f] (Dart d) { f(Vertex(this->phi2(d))); });
	}

	template <typename FUNC>
	inline void foreach_adjacent_vertex_through_face(Vertex v, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Vertex), "Wrong function cell parameter type");
		foreach_dart_of_orbit(v, [this, &f] (Dart vd)
		{
			Dart vd1 = this->phi1(vd);
			this->foreach_dart_of_orbit(Face(vd), [&f, vd, vd1] (Dart fd)
			{
				// skip Vertex v itself and its first successor around current face
				if (fd != vd && fd != vd1)
					f(Vertex(fd));
			});
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_edge_through_vertex(Edge e, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Edge), "Wrong function cell parameter type");
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
		static_assert(check_func_parameter_type(FUNC, Edge), "Wrong function cell parameter type");
		foreach_dart_of_orbit(e, [&f, this] (Dart ed)
		{
			this->foreach_dart_of_orbit(Face(ed), [&f, ed] (Dart fd)
			{
				// skip Edge e itself
				if (fd != ed)
					f(Edge(fd));
			});
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_face_through_vertex(Face f, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Face), "Wrong function cell parameter type");
		foreach_dart_of_orbit(f, [this, &func] (Dart fd)
		{
			Dart fd1 = this->phi2(this->phi_1(fd));
			this->foreach_dart_of_orbit(Vertex(fd), [&func, fd, fd1] (Dart vd)
			{
				// skip Face f itself and its first successor around current vertex
				if (vd != fd && vd != fd1)
					func(Face(vd));
			});
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_face_through_edge(Face f, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Face), "Wrong function cell parameter type");
		foreach_dart_of_orbit(f, [this, &func] (Dart d) { func(Face(this->phi2(d))); });
	}
};

template <typename MAP_TRAITS>
struct CMap2Type
{
	using TYPE = CMap2_T<MAP_TRAITS, CMap2Type<MAP_TRAITS>>;
};

template <typename MAP_TRAITS>
using CMap2 = CMap2_T<MAP_TRAITS, CMap2Type<MAP_TRAITS>>;

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CORE_MAP_MAP2_CPP_))
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
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CORE_MAP_MAP2_CPP_))

} // namespace cgogn

#endif // CORE_CMAP_CMAP2_H_
