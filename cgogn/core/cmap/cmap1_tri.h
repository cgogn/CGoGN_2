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

#ifndef CGOGN_CORE_CMAP_CMAP1_TRI_H_
#define CGOGN_CORE_CMAP_CMAP1_TRI_H_

#include <cgogn/core/cmap/map_base.h>

namespace cgogn
{

template <typename MAP_TRAITS, typename MAP_TYPE>
class CMap1Tri_T : public MapBase<MAP_TRAITS, MAP_TYPE>
{
public:

	static const uint8 DIMENSION = 1;
	static const uint8 PRIM_SIZE = 3;

	using MapTraits = MAP_TRAITS;
	using MapType = MAP_TYPE ;
	using Inherit = MapBase<MAP_TRAITS, MAP_TYPE>;
	using Self = CMap1Tri_T<MAP_TRAITS, MAP_TYPE>;

	friend class MapBase<MAP_TRAITS, MAP_TYPE>;
	friend class DartMarker_T<Self>;
	friend class cgogn::DartMarkerStore<Self>;

	using Vertex	= Cell<Orbit::DART>;
	using Face		= Cell<Orbit::PHI1>;

	using Boundary = Vertex;
	using ConnectedComponent = Face;

	template <typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;
	template <typename T>
	using ChunkArrayContainer = typename Inherit::template ChunkArrayContainer<T>;

	template <typename T, Orbit ORBIT>
	using Attribute = typename Inherit::template Attribute<T, ORBIT>;
	template <typename T>
	using VertexAttribute = Attribute<T, Vertex::ORBIT>;
	template <typename T>
	using FaceAttribute = Attribute<T, Face::ORBIT>;

	using DartMarker = typename cgogn::DartMarker<Self>;
	using DartMarkerStore = typename cgogn::DartMarkerStore<Self>;

	template <Orbit ORBIT>
	using CellMarker = typename cgogn::CellMarker<Self, ORBIT>;

protected:

	void init()
	{
	}

public:

	CMap1Tri_T() : Inherit()
	{
		init();
	}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CMap1Tri_T);

	~CMap1Tri_T() override
	{}

	/*!
	 * \brief Check the integrity of embedding information
	 */
	inline bool check_embedding_integrity()
	{
		bool result = true;

		if (this->template is_embedded<Vertex>())
			result = result && this->template is_well_embedded<Vertex>();

		if (this->template is_embedded<Face>())
			result = result && this->template is_well_embedded<Face>();

		return result;
	}

	/*******************************************************************************
	 * Low-level topological operations
	 *******************************************************************************/

protected:

	/*!
	* \brief Init an newly added dart.
	* The dart is defined as a fixed point for PHI1.
	*/
	inline void init_dart(Dart d)
	{
	}

	/**
	 * @brief Check the integrity of a dart
	 * @param d the dart to check
	 * @return true if the integrity constraints are locally statisfied
	 * PHI1 and PHI_1 are inverse relations.
	 */
	inline bool check_integrity(Dart d) const
	{
		return (phi1(phi_1(d)) == d &&
				phi_1(phi1(d)) == d);
	}

	/**
	 * @brief Check the integrity of a boundary dart
	 * @param d the dart to check
	 * @return true if the bondary constraints are locally statisfied
	 * No boundary dart is accepted.
	 */
	inline bool check_boundary_integrity(Dart d) const
	{
		return !this->is_boundary(d);
	}


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
		case 0: return Dart(d.index+2);break;
		case 1: return Dart(d.index-1);break;
		}
		return Dart(d.index-1);
	}

	/**
	 * \brief Composition of PHI calls
	 * @param d
	 * @return The result of successive applications of PHI1 on d.
	 * The template parameter contains a sequence (Base10 encoded) of PHI indices.
	 * If N=0 the identity is used.
	 */
	template <uint64 N>
	inline Dart phi(Dart d) const
	{
		static_assert((N%10)<=1,"Composition of PHI: invalid index");
		if (N >=10)
			return this->phi1(phi<N/10>(d));

		if (N == 1)
			return this->phi1(d);

		return d;
	}

	/*******************************************************************************
	 * High-level embedded and topological operations
	 *******************************************************************************/

protected:


	inline Dart add_tri_topo()
	{
		Dart d = this->add_dart(); // this insert 3 darts
		// no need to set phi1
		return d;
	}

//	/*!
//	 * \brief Add a face in the map.
//	 * \param size : the number of darts in the built face (only 3 allowed here)
//	 * \return A dart of the built face
//	 */
//	inline Dart add_face_topo(uint32 size)
//	{
//		cgogn_message_assert(size == 3u, "Can create only triangle");

//		if (size != 3)
//			cgogn_log_warning("add_face_topo") << "Attempt to create a face which is not a triangle";

//		return add_tri_topo();
//	}

public:

	/*!
	 * \brief Add a face in the map.
	 * \param size : the number of vertices in the built face (3)
	 * \return The built face. If the map has Vertex or Face attributes,
	 * the new inserted cells are automatically embedded on new attribute elements.
	 */
	Face add_face(uint32 size)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		cgogn_message_assert(size == 3u, "Can create only triangle");
		if (size != 3)
			cgogn_log_warning("add_face_topo") << "Attempt to create a face which is not a triangle";

		const Face f(add_tri_topo());

		if (this->template is_embedded<Vertex>())
		{
			foreach_dart_of_orbit(f, [this] (Dart d)
			{
				this->new_orbit_embedding(Vertex(d));
			});
		}

		if (this->template is_embedded<Face>())
			this->new_orbit_embedding(f);

		return f;
	}

protected:

	inline void remove_face_topo(Dart d)
	{
		this->remove_dart(d); // in fact remove_dart remove PRIM_SIZE darts
	}

public:

	/*!
	 * \brief Remove a face from the map.
	 * \param d : a dart of the face to remove
	 */
	inline void remove_face(Face f)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		remove_face_topo(f.dart);
	}

	/*******************************************************************************
	 * Connectivity information
	 *******************************************************************************/

public:

	inline uint32 degree(Vertex) const
	{
		return 1;
	}

	inline uint32 codegree(Face) const
	{
		return 3;
	}


	inline bool has_codegree(Face, uint32 codegree) const
	{
		return codegree == 3;
	}

	/*******************************************************************************
	 * Orbits traversal
	 *******************************************************************************/

protected:

	template <typename FUNC>
	inline void foreach_dart_of_PHI1(Dart d, const FUNC& f) const
	{
		Dart it = d;
		do
		{
			f(it);
			it = phi1(it);
		} while (it != d);
	}

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_dart_of_orbit(Cell<ORBIT> c, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Dart), "Wrong function parameter type");
		static_assert(ORBIT == Orbit::DART || ORBIT == Orbit::PHI1, "Orbit not supported in a CMap1Tri");

		switch (ORBIT)
		{
			case Orbit::DART: f(c.dart); break;
			case Orbit::PHI1: foreach_dart_of_PHI1(c.dart, f); break;
			case Orbit::PHI2:
			case Orbit::PHI1_PHI2:
			case Orbit::PHI1_PHI3:
			case Orbit::PHI2_PHI3:
			case Orbit::PHI21:
			case Orbit::PHI21_PHI31:
			case Orbit::PHI1_PHI2_PHI3:
			default: cgogn_assert_not_reached("Orbit not supported in a CMap1Tri"); break;
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

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_dart_of_orbit_until(Cell<ORBIT> c, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Dart), "Wrong function parameter type");
		static_assert(check_func_return_type(FUNC, bool), "Wrong function return type");
		static_assert(ORBIT == Orbit::DART || ORBIT == Orbit::PHI1, "Orbit not supported in a CMap1Tri");

		switch (ORBIT)
		{
			case Orbit::DART: f(c.dart); break;
			case Orbit::PHI1: foreach_dart_of_PHI1_until(c, f); break;
			case Orbit::PHI2:
			case Orbit::PHI1_PHI2:
			case Orbit::PHI1_PHI3:
			case Orbit::PHI2_PHI3:
			case Orbit::PHI21:
			case Orbit::PHI21_PHI31:
			case Orbit::PHI1_PHI2_PHI3:
			default: cgogn_assert_not_reached("Orbit not supported in a CMap1Tri"); break;
		}
	}

public:

	/*******************************************************************************
	 * Incidence traversal
	 *******************************************************************************/

	template <typename FUNC>
	inline void foreach_incident_vertex(Face f, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Vertex), "Wrong function cell parameter type");
		foreach_dart_of_orbit(f, [&func](Dart v) {func(Vertex(v));});
	}

protected:

	/**
	 * @brief check if embedding of map is also embedded in this (create if not). Used by merge method
	 * @param map
	 */
	void merge_check_embedding(const Self& map)
	{
		if (!this->template is_embedded<Orbit::DART>() && map.template is_embedded<Orbit::DART>())
			this->template create_embedding<Orbit::DART>();
		if (!this->template is_embedded<Orbit::PHI1>() && map.template is_embedded<Orbit::PHI1>())
			this->template create_embedding<Orbit::PHI1>();

	}

	/**
	 * @brief ensure all cells (introduced while merging) are embedded.
	 * @param first index of first dart to scan
	 */
	void merge_finish_embedding(uint32 first)
	{
		if (this->template is_embedded<Orbit::DART>())
			for (uint32 j=first; j!= this->topology_.end(); this->topology_.next(j))
			{
				if ((*this->embeddings_[Orbit::DART])[j] == std::numeric_limits<uint32>::max())
					this->new_orbit_embedding(Cell<Orbit::DART>(Dart(j)));
			}

		if (this->template is_embedded<Orbit::PHI1>())
			for (uint32 j=first; j!= this->topology_.end(); this->topology_.next(j))
			{
				if ((*this->embeddings_[Orbit::PHI1])[j] == std::numeric_limits<uint32>::max())
					this->new_orbit_embedding(Cell<Orbit::PHI1>(Dart(j)));
			}
	}

};

template <typename MAP_TRAITS>
struct CMap1TriType
{
	using TYPE = CMap1Tri_T<MAP_TRAITS, CMap1TriType<MAP_TRAITS>>;
};

template <typename MAP_TRAITS>
using CMap1Tri = CMap1Tri_T<MAP_TRAITS, CMap1TriType<MAP_TRAITS>>;

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_MAP_MAP1TRI_CPP_))
extern template class CGOGN_CORE_API CMap1Tri_T<DefaultMapTraits, CMap1TriType<DefaultMapTraits>>;
extern template class CGOGN_CORE_API DartMarker<CMap1Tri<DefaultMapTraits>>;
extern template class CGOGN_CORE_API DartMarkerStore<CMap1Tri<DefaultMapTraits>>;
extern template class CGOGN_CORE_API DartMarkerNoUnmark<CMap1Tri<DefaultMapTraits>>;
extern template class CGOGN_CORE_API CellMarker<CMap1Tri<DefaultMapTraits>, CMap1Tri<DefaultMapTraits>::Vertex::ORBIT>;
extern template class CGOGN_CORE_API CellMarker<CMap1Tri<DefaultMapTraits>, CMap1Tri<DefaultMapTraits>::Face::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap1Tri<DefaultMapTraits>, CMap1Tri<DefaultMapTraits>::Vertex::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap1Tri<DefaultMapTraits>, CMap1Tri<DefaultMapTraits>::Face::ORBIT>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_MAP_MAP1_CPP_))

} // namespace cgogn

#endif // CGOGN_CORE_CMAP_CMAP1_H_
