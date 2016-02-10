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

	typedef MAP_TRAITS MapTraits;
	typedef MAP_TYPE MapType;
	typedef CMap1_T<MAP_TRAITS, MAP_TYPE> Inherit;
	typedef CMap2_T<MAP_TRAITS, MAP_TYPE> Self;

	friend typename Self::Inherit;
	friend typename Inherit::Inherit;
	friend class CMap2Builder_T<MapTraits>;
	friend class DartMarker_T<Self>;
	friend class cgogn::DartMarkerStore<Self>;
	friend class DartValidator;

	static const Orbit DART	  = Orbit::DART;
	static const Orbit VERTEX = Orbit::PHI21;
	static const Orbit EDGE   = Orbit::PHI2;
	static const Orbit FACE   = Orbit::PHI1;
	static const Orbit VOLUME = Orbit::PHI1_PHI2;

	typedef Cell<Self::VERTEX> Vertex;
	typedef Cell<Self::EDGE> Edge;
	typedef Cell<Self::FACE> Face;
	typedef Cell<Self::VOLUME> Volume;

	static const Orbit BOUNDARY = FACE;

	template <typename T>
	using ChunkArray =  typename Inherit::template ChunkArray<T>;
	template <typename T>
	using ChunkArrayContainer =  typename Inherit::template ChunkArrayContainer<T>;

	template <typename T, Orbit ORBIT>
	using AttributeHandler = typename Inherit::template AttributeHandler<T, ORBIT>;
	template <typename T>
	using VertexAttributeHandler = AttributeHandler<T, Self::VERTEX>;
	template <typename T>
	using EdgeAttributeHandler = AttributeHandler<T, Self::EDGE>;
	template <typename T>
	using FaceAttributeHandler = AttributeHandler<T, Self::FACE>;
	template <typename T>
	using VolumeAttributeHandler = AttributeHandler<T, Self::VOLUME>;

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

	/*******************************************************************************
	 * Low-level topological operations
	 *******************************************************************************/

	/**
	 * \brief Link dart d with dart e by an involution
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
	 * \brief Unlink the current dart by an involution
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
	 * Basic topological operations
	 *******************************************************************************/

	/**
	 * \brief phi2
	 * @param d
	 * @return phi2(d)
	 */
	inline Dart phi2(Dart d) const
	{
		return (*phi2_)[d.index];
	}

protected:

	/**
	* \brief add a Dart in the map
	* @return the new Dart
	*/
	inline Dart add_dart()
	{
		CGOGN_CHECK_CONCRETE_TYPE;
		Dart d = Inherit::add_dart();
		(*phi2_)[d.index] = d;
		return d;
	}

	/**
	 * @brief close_map closes the map so that there are no phi2 fix points
	 */
	void close_map()
	{
		std::vector<Dart> fix_point_darts;
		this->foreach_dart([&] (Dart d)
		{
			if (phi2(d) == d)
				fix_point_darts.push_back(d);
		});
		for (Dart d : fix_point_darts)
		{
			if (phi2(d) == d)
			{
				close_hole_topo(d);
				const Dart new_face = phi2(d);

				if (this->template is_orbit_embedded<DART>())
				{
					foreach_dart_of_orbit<FACE>(new_face, [this] (Dart fd)
					{
						this->template set_orbit_embedding<DART>(fd, this->template add_attribute_element<DART>());
					});
				}
				if (this->template is_orbit_embedded<VERTEX>())
				{
					foreach_dart_of_orbit<FACE>(new_face, [this] (Dart fd)
					{
						this->template set_embedding<VERTEX>(fd, this->template get_embedding<VERTEX>(this->phi1(phi2(fd))));
					});
				}
				if (this->template is_orbit_embedded<EDGE>())
				{
					foreach_dart_of_orbit<FACE>(new_face, [this] (Dart fd)
					{
						this->template set_embedding<EDGE>(fd, this->template get_embedding<EDGE>(phi2(fd)));
					});
				}
				if (this->template is_orbit_embedded<FACE>())
				{
					this->template set_orbit_embedding<FACE>(new_face, this->template add_attribute_element<FACE>());
				}
				if (this->template is_orbit_embedded<VOLUME>())
				{
					const unsigned int idx = this->template get_embedding<VOLUME>(d);
					foreach_dart_of_orbit<FACE>(new_face, [this, idx] (Dart fd)
					{
						this->template set_embedding<VOLUME>(fd, idx);
					});
				}
			}
		}
	}

public:

	/*******************************************************************************
	 * High-level topological operations
	 *******************************************************************************/

	Face add_face(unsigned int nb_edges)
	{
		cgogn_message_assert(nb_edges > 0, "Cannot create a face with no edge");

		Dart d = Inherit::add_face_topo(nb_edges);
		Dart b = Inherit::add_face_topo(nb_edges);
		Dart it = d;
		do
		{
			phi2_sew(it, b);
			this->set_boundary(b, true);
			it = this->phi1(it);
			b = this->phi_1(b);
		} while (it != d);

		Face f(d);

		if (this->template is_orbit_embedded<DART>())
		{
			this->foreach_dart_of_orbit(f, [this] (Dart df)
			{
				this->template set_orbit_embedding<DART>(df, this->template add_attribute_element<DART>());
			});
		}

		if (this->template is_orbit_embedded<VERTEX>())
		{
			foreach_incident_vertex(f, [this] (Vertex v)
			{
				this->set_orbit_embedding(v, this->template add_attribute_element<VERTEX>());
			});
		}

		if (this->template is_orbit_embedded<EDGE>())
		{
			foreach_incident_edge(f, [this] (Edge e)
			{
				this->set_orbit_embedding(e, this->template add_attribute_element<EDGE>());
			});
		}

		if (this->template is_orbit_embedded<FACE>())
			this->set_orbit_embedding(f, this->template add_attribute_element<FACE>());

		if (this->template is_orbit_embedded<VOLUME>())
			this->set_orbit_embedding(Volume(d), this->template add_attribute_element<VOLUME>());

		return f;
	}

	inline Vertex cut_edge(Edge d)
	{
		const Dart e = phi2(d);
		const Dart nd = cut_edge_topo(d);
		const Dart ne = phi2(d);
		const Vertex v(nd);

		if(this->template is_orbit_embedded<DART>())
		{
			this->template set_embedding<DART>(nd, this->template add_attribute_element<DART>());
			this->template set_embedding<DART>(ne, this->template add_attribute_element<DART>());
		}

		if (this->template is_orbit_embedded<VERTEX>())
		{
			this->set_orbit_embedding(v, this->template add_attribute_element<VERTEX>());
		}

		if (this->template is_orbit_embedded<EDGE>())
		{
			this->template set_embedding<EDGE>(ne, this->template get_embedding<EDGE>(d.dart));
			this->set_orbit_embedding(Edge(nd), this->template add_attribute_element<EDGE>());
		}

		if (this->template is_orbit_embedded<FACE>())
		{
			this->template set_embedding<FACE>(nd, this->template get_embedding<FACE>(d.dart));
			this->template set_embedding<FACE>(ne, this->template get_embedding<FACE>(e));
		}

		if (this->template is_orbit_embedded<VOLUME>())
		{
			const unsigned int idx = this->template get_embedding<VOLUME>(d.dart);
			this->template set_embedding<VOLUME>(nd, idx);
			this->template set_embedding<VOLUME>(ne, idx);
		}

		return v;
	}

	inline void split_face(Dart d, Dart e)
	{
		split_face_topo(d,e);
		const Dart nd = this->phi_1(e);
		const Dart ne = this->phi_1(d);

		if(this->template is_orbit_embedded<DART>())
		{
			this->template set_embedding<DART>(nd, this->template add_attribute_element<DART>());
			this->template set_embedding<DART>(ne, this->template add_attribute_element<DART>());
		}

		if (this->template is_orbit_embedded<VERTEX>())
		{
			this->template set_embedding<VERTEX>(nd, this->template get_embedding<VERTEX>(d));
			this->template set_embedding<VERTEX>(ne, this->template get_embedding<VERTEX>(e));
		}

		if (this->template is_orbit_embedded<EDGE>())
		{
			this->template set_orbit_embedding<EDGE>(nd, this->template add_attribute_element<EDGE>());
		}

		if (this->template is_orbit_embedded<FACE>())
		{
			this->template set_embedding<FACE>(ne, this->template get_embedding<FACE>(d));
			this->template set_orbit_embedding<FACE>(e, this->template add_attribute_element<FACE>());
		}

		if (this->template is_orbit_embedded<VOLUME>())
		{
			const unsigned int idx = this->template get_embedding<VOLUME>(d);
			this->template set_orbit_embedding<VOLUME>(nd, idx);
			this->template set_orbit_embedding<VOLUME>(ne, idx);
		}
	}

	inline unsigned int degree(Face f) const
	{
		return Inherit::degree(f);
	}

protected:

	inline Dart cut_edge_topo(Dart d)
	{
		Dart e = phi2(d);						// Get the adjacent 1D-edge

		phi2_unsew(d);							// Unsew the initial 2D-edge,
		// separating its two 1D-edges
		Dart nd = Inherit::cut_edge_topo(d);
		Dart ne = Inherit::cut_edge_topo(e);	// Cut the two adjacent 1D-edges

		phi2_sew(d, ne);						// Sew the new 1D-edges
		phi2_sew(e, nd);						// To build the new 2D-edges

		return nd;
	}

	inline void split_face_topo(Dart d, Dart e)
	{
		cgogn_message_assert(d != e, "split_face: d and e should be distinct");
		cgogn_message_assert(this->same_cell(Face(d), Face(e)), "split_face: d and e should belong to the same face");

		Dart nd = Inherit::cut_edge_topo(this->phi_1(d));	// cut the edge before d (insert a new dart before d)
		Dart ne = Inherit::cut_edge_topo(this->phi_1(e));	// cut the edge before e (insert a new dart before e)

		Inherit::split_face_topo(nd, ne);					// subdivide phi1 cycle at the inserted darts
		phi2_sew(nd, ne);									// build the new 2D-edge from the inserted darts
	}

	inline void close_hole_topo(Dart d)
	{
		cgogn_message_assert(phi2(d) == d, "CMap2: close hole called on a dart that is not a phi2 fix point");

		Dart first = add_dart(); // First edge of the face that will fill the hole
		this->set_boundary(first, true);
		phi2_sew(d, first);      // phi2-link the new edge to the hole

		Dart d_next = d; // Turn around the hole
		Dart d_phi1;     // to complete the face
		do
		{
			do
			{
				d_phi1 = this->phi1(d_next); // Search and put in d_next
				d_next = phi2(d_phi1);       // the next dart of the hole
			} while (d_next != d_phi1 && d_phi1 != d);

			if (d_phi1 != d)
			{
				Dart next = add_dart(); // Add a new edge there and link it to the face
				this->set_boundary(next, true);
				this->phi1_sew(first, next); // the edge is linked to the face
				phi2_sew(d_next, next);      // the face is linked to the hole
			}
		} while (d_phi1 != d);
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
			if (!marker.is_marked((*visited_faces)[i]))	// Face has not been visited yet
			{
				// mark visited darts (current face)
				// and add non visited adjacent faces to the list of face
				Dart e = (*visited_faces)[i];
				do
				{
					f(e); // apply the function to the darts of the face
					marker.mark(e);				// Mark
					Dart adj = phi2(e);			// Get adjacent face
					if (!marker.is_marked(adj))
						visited_faces->push_back(adj);	// Add it
					e = this->phi1(e);
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
			case Orbit::DART: this->foreach_dart_of_DART(c, f); break;
			case Orbit::PHI1: this->foreach_dart_of_PHI1(c, f); break;
			case Orbit::PHI2: foreach_dart_of_PHI2(c, f); break;
			case Orbit::PHI1_PHI2: foreach_dart_of_PHI1_PHI2(c, f); break;
			case Orbit::PHI21: foreach_dart_of_PHI21(c, f); break;
			case Orbit::PHI2_PHI3:
			case Orbit::PHI1_PHI3:
			case Orbit::PHI21_PHI31:
			default: cgogn_assert_not_reached("Cells of this dimension are not handled"); break;
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
			if (!marker.is_marked((*visited_faces)[i]))	// Face has not been visited yet
			{
				// mark visited darts (current face)
				// and add non visited adjacent faces to the list of face
				Dart e = (*visited_faces)[i];
				do
				{
					if (!f(e)) // apply the function to the darts of the face
					{
						cgogn::get_dart_buffers()->release_buffer(visited_faces);
						return;
					}
					marker.mark(e);				// Mark
					Dart adj = phi2(e);			// Get adjacent face
					if (!marker.is_marked(adj))
						visited_faces->push_back(adj);	// Add it
					e = this->phi1(e);
				} while (e != (*visited_faces)[i]);
			}
		}

		cgogn::get_dart_buffers()->release_buffer(visited_faces);
	}

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_dart_of_orbit_until(Cell<ORBIT> c, const FUNC& f) const
	{
		static_assert(ORBIT == Orbit::DART || ORBIT == Orbit::PHI1 ||
					  ORBIT == Orbit::PHI2 || ORBIT == Orbit::PHI1_PHI2 || ORBIT == Orbit::PHI21,
					  "Orbit not supported in a CMap2");
		static_assert(check_func_return_type(FUNC, bool), "Wrong function return type");

		switch (ORBIT)
		{
			case Orbit::DART: this->foreach_dart_of_DART(c, f); break;
			case Orbit::PHI1: this->foreach_dart_of_PHI1_until(c, f); break;
			case Orbit::PHI2: foreach_dart_of_PHI2_until(c, f); break;
			case Orbit::PHI1_PHI2: foreach_dart_of_PHI1_PHI2_until(c, f); break;
			case Orbit::PHI21: foreach_dart_of_PHI21_until(c, f); break;
			case Orbit::PHI2_PHI3:
			case Orbit::PHI1_PHI3:
			case Orbit::PHI21_PHI31:
			default: cgogn_assert_not_reached("Cells of this dimension are not handled"); break;
		}
	}

public:

	/*******************************************************************************
	 * Incidence traversal
	 *******************************************************************************/

	template <typename FUNC>
	inline void foreach_incident_edge(Vertex v, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Edge), "Wrong function cell parameter type");
		foreach_dart_of_orbit(v, f);
	}

	template <typename FUNC>
	inline void foreach_incident_face(Vertex v, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Face), "Wrong function cell parameter type");
		foreach_dart_of_orbit(v, f);
	}

	template <typename FUNC>
	inline void foreach_incident_vertex(Edge e, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Vertex), "Wrong function cell parameter type");
		f(e.dart);
		f(phi2(e.dart));
	}

	template <typename FUNC>
	inline void foreach_incident_face(Edge e, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Face), "Wrong function cell parameter type");
		f(e.dart);
		f(phi2(e.dart));
	}

	template <typename FUNC>
	inline void foreach_incident_vertex(Face f, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Vertex), "Wrong function cell parameter type");
		foreach_dart_of_orbit<FACE>(f, func);
	}

	template <typename FUNC>
	inline void foreach_incident_edge(Face f, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Edge), "Wrong function cell parameter type");
		foreach_dart_of_orbit<FACE>(f, func);
	}

	template <typename FUNC>
	inline void foreach_incident_vertex(Volume v, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Vertex), "Wrong function cell parameter type");
		DartMarkerStore marker(*this);
		foreach_dart_of_orbit<VOLUME>(v, [&] (Dart d)
		{
			if (!marker.is_marked(d))
			{
				marker.mark_orbit<VERTEX>(d);
				f(d);
			}
		});
	}

	template <typename FUNC>
	inline void foreach_incident_edge(Volume v, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Edge), "Wrong function cell parameter type");
		DartMarkerStore marker(*this);
		foreach_dart_of_orbit<VOLUME>(v, [&] (Dart d)
		{
			if (!marker.is_marked(d))
			{
				marker.mark_orbit<EDGE>(d);
				f(d);
			}
		});
	}

	template <typename FUNC>
	inline void foreach_incident_face(Volume v, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Face), "Wrong function cell parameter type");
		DartMarkerStore marker(*this);
		foreach_dart_of_orbit<VOLUME>(v, [&] (Dart d)
		{
			if (!marker.is_marked(d))
			{
				marker.mark_orbit<FACE>(d);
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
			this->foreach_dart_of_orbit<FACE>(vd, [&f, vd, vd1] (Dart fd)
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
			this->foreach_dart_of_orbit<VERTEX>(ed, [&f, ed] (Dart vd)
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
			this->foreach_dart_of_orbit<FACE>(ed, [&f, ed] (Dart fd)
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
			this->foreach_dart_of_orbit<VERTEX>(fd, [&func, fd, fd1] (Dart vd)
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
	typedef CMap2_T<MAP_TRAITS, CMap2Type<MAP_TRAITS>> TYPE;
};

template <typename MAP_TRAITS>
using CMap2 = CMap2_T<MAP_TRAITS, CMap2Type<MAP_TRAITS>>;

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CORE_MAP_MAP2_CPP_))
extern template class CGOGN_CORE_API CMap2_T<DefaultMapTraits, CMap2Type<DefaultMapTraits>>;
extern template class CGOGN_CORE_API DartMarker<CMap2<DefaultMapTraits>>;
extern template class CGOGN_CORE_API DartMarkerStore<CMap2<DefaultMapTraits>>;
extern template class CGOGN_CORE_API DartMarkerNoUnmark<CMap2<DefaultMapTraits>>;
extern template class CGOGN_CORE_API CellMarker<CMap2<DefaultMapTraits>, Orbit::PHI21>;
extern template class CGOGN_CORE_API CellMarker<CMap2<DefaultMapTraits>, Orbit::PHI2>;
extern template class CGOGN_CORE_API CellMarker<CMap2<DefaultMapTraits>, Orbit::PHI1>;
extern template class CGOGN_CORE_API CellMarker<CMap2<DefaultMapTraits>, Orbit::PHI1_PHI2>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap2<DefaultMapTraits>, Orbit::PHI21>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap2<DefaultMapTraits>, Orbit::PHI2>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap2<DefaultMapTraits>, Orbit::PHI1>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap2<DefaultMapTraits>, Orbit::PHI1_PHI2>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CORE_MAP_MAP2_CPP_))



} // namespace cgogn

#endif // CORE_CMAP_CMAP2_H_
