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

#ifndef CORE_MAP_MAP3_H_
#define CORE_MAP_MAP3_H_

#include <core/map/cmap2.h>
#include <core/basic/dart_marker.h>

namespace cgogn
{

template <typename DATA_TRAITS, typename TOPO_TRAITS>
class CMap3_T : public CMap2_T<DATA_TRAITS, TOPO_TRAITS>
{
public:

	typedef CMap2_T<DATA_TRAITS, TOPO_TRAITS> Inherit;
	typedef CMap3_T<DATA_TRAITS, TOPO_TRAITS> Self;

	friend typename Self::Inherit;
	friend typename Inherit::Inherit;
	friend typename Inherit::Inherit::Inherit;
	friend class DartMarker_T<Self>;

	static const Orbit VERTEX = Orbit::PHI21_PHI31;
	static const Orbit EDGE   = Orbit::PHI2_PHI3;
	static const Orbit FACE   = Orbit::PHI1_PHI3;
	static const Orbit VOLUME = Orbit::PHI1_PHI2;

	typedef Cell<Self::VERTEX> Vertex;
	typedef Cell<Self::EDGE> Edge;
	typedef Cell<Self::FACE> Face;
	typedef Cell<Self::VOLUME> Volume;

	template<typename T>
	using ChunkArray =  typename Inherit::template ChunkArray<T>;
	template<typename T>
	using ChunkArrayContainer =  typename Inherit::template ChunkArrayContainer<T>;

	template<typename T, Orbit ORBIT>
	using AttributeHandler = typename Inherit::template AttributeHandler<T, ORBIT>;
	template<typename T>
	using VertexAttributeHandler = AttributeHandler<T, Self::VERTEX>;
	template<typename T>
	using EdgeAttributeHandler = AttributeHandler<T, Self::EDGE>;
	template<typename T>
	using FaceAttributeHandler = AttributeHandler<T, Self::FACE>;
	template<typename T>
	using VolumeAttributeHandler = AttributeHandler<T, Self::VOLUME>;

	using DartMarker = typename Inherit::DartMarker;
	using DartMarkerStore = typename Inherit::DartMarkerStore;

protected:

	ChunkArray<Dart>* phi3_;

	inline void init()
	{
		phi3_ = this->topology_.template add_attribute<Dart>("phi3");
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
	inline void phi3_sew(Dart d, Dart e)
	{
		cgogn_assert(phi3(d) == d);
		cgogn_assert(phi3(e) == e);
		(*phi3_)[d.index] = e;
		(*phi3_)[e.index] = d;
	}

	/**
	 * \brief Unlink the current dart by an involution
	 * @param d the dart to unlink
	 * - Before: d->e and e->d
	 * - After:  d->d and e->e
	 */
	inline void phi3_unsew(Dart d)
	{
		Dart e = phi3(d) ;
		(*phi3_)[d.index] = d;
		(*phi3_)[e.index] = e;
	}

public:

	CMap3_T() : Inherit()
	{
		init();
	}

	~CMap3_T() override
	{}

	CMap3_T(Self const&) = delete;
	CMap3_T(Self &&) = delete;
	Self& operator=(Self const&) = delete;
	Self& operator=(Self &&) = delete;

	/*******************************************************************************
	 * Basic topological operations
	 *******************************************************************************/

	/**
	 * \brief phi3
	 * @param d
	 * @return phi3(d)
	 */
	inline Dart phi3(Dart d) const
	{
		return (*phi3_)[d.index];
	}

protected:

	/**
	* \brief add a Dart in the map
	* @return the new Dart
	*/
	inline Dart add_dart()
	{
		Dart d = Inherit::add_dart();
		(*phi3_)[d.index] = d;
		return d;
	}

public:

	/*******************************************************************************
	 * High-level topological operations
	 *******************************************************************************/



protected:

	/*******************************************************************************
	 * Orbits traversal
	 *******************************************************************************/

	template <typename FUNC>
	inline void foreach_dart_of_vertex(Dart d, const FUNC& f) const
	{
		DartMarkerStore marker(*this);
		const std::vector<Dart>* marked_darts = marker->get_marked_darts();

		marker.mark(d);
		for(unsigned int i = 0; i < marked_darts->size(); ++i)
		{
			f((*marked_darts)[i]);

			Dart d2 = this->phi2((*marked_darts)[i]);
			Dart d21 = this->phi1(d2); // turn in volume
			Dart d23 = phi3(d2); // change volume
			if(!marker.isMarked(d21))
				marker.mark(d21);
			if(!marker.isMarked(d23))
				marker.mark(d23);
		}
	}

	template <typename FUNC>
	inline void foreach_dart_of_edge(Dart d, const FUNC& f) const
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
	inline void foreach_dart_of_oriented_edge(Dart d, const FUNC& f) const
	{
		Dart it = d;
		do
		{
			f(it);
			it = phi3(this->phi2(it));
		} while (it != d);
	}

	template <typename FUNC>
	inline void foreach_dart_of_face(Dart d, const FUNC& f) const
	{
		foreach_dart_of_orbit<Inherit::FACE>(d, [&] (Dart fd)
		{
			f(fd);
			f(phi3(fd));
		});
	}

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_dart_of_orbit(Cell<ORBIT> c, const FUNC& f) const
	{
		switch (ORBIT)
		{
			case Orbit::DART: Inherit::foreach_dart_of_orbit(c, f); break;
			case Orbit::PHI1: Inherit::foreach_dart_of_orbit(c, f); break;
			case Orbit::PHI2: Inherit::foreach_dart_of_orbit(c, f); break;
			case Orbit::PHI1_PHI2: Inherit::foreach_dart_of_orbit(c, f); break;
			case Orbit::PHI1_PHI3: foreach_dart_of_face(c, f); break;
			case Orbit::PHI2_PHI3: foreach_dart_of_edge(c, f); break;
			case Orbit::PHI21: Inherit::foreach_dart_of_orbit(c, f); break;
			case Orbit::PHI21_PHI31: foreach_dart_of_vertex(c, f); break;
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
		DartMarkerStore marker(*this);
		foreach_dart_of_orbit(v, [&] (Dart d)
		{
			if (!marker.is_marked(d))
			{
				foreach_dart_of_oriented_edge(d, [&marker] (Dart dd) { marker.mark(dd); });
				f(d);
			}
		});
	}

	template <typename FUNC>
	inline void foreach_incident_face(Vertex v, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Face), "Wrong function cell parameter type");
		DartMarkerStore marker(*this);
		foreach_dart_of_orbit(v, [&] (Dart d)
		{
			if (!marker.is_marked(d))
			{
				marker.mark(d);
				marker.mark(this->phi1(phi3(d)));
				f(d);
			}
		});
	}

	template <typename FUNC>
	inline void foreach_incident_volume(Vertex v, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Volume), "Wrong function cell parameter type");
		DartMarkerStore marker(*this);
		foreach_dart_of_orbit(v, [&] (Dart d)
		{
			if (!marker.is_marked(d))
			{
				marker.mark_orbit<Inherit::VERTEX>(d);
				f(d);
			}
		});
	}

	template <typename FUNC>
	inline void foreach_incident_vertex(Edge e, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Vertex), "Wrong function cell parameter type");
		f(e.dart);
		f(this->phi2(e.dart));
	}

	template <typename FUNC>
	inline void foreach_incident_face(Edge e, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Face), "Wrong function cell parameter type");
		foreach_dart_of_oriented_edge(e, f);
	}

	template <typename FUNC>
	inline void foreach_incident_volume(Edge e, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Volume), "Wrong function cell parameter type");
		foreach_dart_of_oriented_edge(e, f);
	}

	template <typename FUNC>
	inline void foreach_incident_vertex(Face f, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Vertex), "Wrong function cell parameter type");
		foreach_dart_of_orbit<Inherit::FACE>(f.dart, func);
	}

	template <typename FUNC>
	inline void foreach_incident_edge(Face f, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Edge), "Wrong function cell parameter type");
		foreach_dart_of_orbit<Inherit::FACE>(f.dart, func);
	}

	template <typename FUNC>
	inline void foreach_incident_volume(Face f, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Volume), "Wrong function cell parameter type");
		func(f);
		func(phi3(f.dart));
	}

	template <typename FUNC>
	inline void foreach_incident_vertex(Volume v, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Vertex), "Wrong function cell parameter type");
		Inherit::foreach_incident_vertex(v, f);
	}

	template <typename FUNC>
	inline void foreach_incident_edge(Volume v, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Edge), "Wrong function cell parameter type");
		Inherit::foreach_incident_edge(v, f);
	}

	template <typename FUNC>
	inline void foreach_incident_face(Volume v, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Face), "Wrong function cell parameter type");
		Inherit::foreach_incident_face(v, f);
	}

	/*******************************************************************************
	 * Adjacence traversal
	 *******************************************************************************/

	template <typename FUNC>
	inline void foreach_adjacent_vertex_through_edge(Vertex v, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Vertex), "Wrong function cell parameter type");
		foreach_incident_edge(v, [&] (Edge e)
		{
			f(Vertex(this->phi2(e.dart)));
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_vertex_through_face(Vertex v, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Vertex), "Wrong function cell parameter type");
		// TODO
	}

	template <typename FUNC>
	inline void foreach_adjacent_vertex_through_volume(Vertex v, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Vertex), "Wrong function cell parameter type");
		// TODO
	}

	template <typename FUNC>
	inline void foreach_adjacent_edge_through_vertex(Edge e, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Edge), "Wrong function cell parameter type");
		foreach_incident_vertex(e, [&] (Vertex iv)
		{
			foreach_incident_edge(iv, [&] (Edge ie)
			{
				if (ie.dart != iv.dart)
					f(ie);
			});
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_edge_through_face(Edge e, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Edge), "Wrong function cell parameter type");
		// TODO
	}

	template <typename FUNC>
	inline void foreach_adjacent_edge_through_volume(Edge e, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Edge), "Wrong function cell parameter type");
		// TODO
	}

	template <typename FUNC>
	inline void foreach_adjacent_face_through_vertex(Face f, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Face), "Wrong function cell parameter type");
		// TODO
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
		// TODO
	}

	template <typename FUNC>
	inline void foreach_adjacent_volume_through_vertex(Volume v, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Volume), "Wrong function cell parameter type");
		// TODO
	}

	template <typename FUNC>
	inline void foreach_adjacent_volume_through_edge(Volume v, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Volume), "Wrong function cell parameter type");
		// TODO
	}

	template <typename FUNC>
	inline void foreach_adjacent_volume_through_face(Volume v, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Volume), "Wrong function cell parameter type");
		// TODO
	}

protected:

	/*******************************************************************************
	 * Embedding management
	 *******************************************************************************/

	template <Orbit ORBIT>
	inline void init_orbit_embedding(Cell<ORBIT> c, unsigned int emb)
	{
		foreach_dart_of_orbit(c, [this, emb] (Dart d) { this->template init_embedding<ORBIT>(d, emb); });
	}

	template <Orbit ORBIT>
	inline void set_orbit_embedding(Cell<ORBIT> c, unsigned int emb)
	{
		foreach_dart_of_orbit(c, [this, emb] (Dart d) {	this->template set_embedding<ORBIT>(d, emb); });
	}
};

template <typename DataTraits>
struct CMap3TopoTraits
{
	static const int PRIM_SIZE = 1;
	typedef CMap3_T<DataTraits, CMap3TopoTraits<DataTraits>> CONCRETE;
};

struct CMap3DataTraits
{
	static const unsigned int CHUNK_SIZE = 4096;
};

using CMap3 = CMap3_T<CMap3DataTraits, CMap3TopoTraits<CMap3DataTraits>>;

} // namespace cgogn

#endif // CORE_MAP_MAP3_H_
