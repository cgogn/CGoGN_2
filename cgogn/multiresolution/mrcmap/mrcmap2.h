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

#ifndef CGOGN_MULTIRESOLUTION_MRCMAP_MRCMAP2_H_
#define CGOGN_MULTIRESOLUTION_MRCMAP_MRCMAP2_H_

#include <cgogn/core/cmap/cmap2.h>
#include <cgogn/multiresolution/mrcmap/mr_base.h>

namespace cgogn
{

template <typename MAP_TRAITS, typename MAP_TYPE>
class MRCMap2_T : public MRBase<CMap2_T<MAP_TRAITS, MAP_TYPE>>
{
public:

	using Self = MRCMap2_T<MAP_TRAITS, MAP_TYPE>;
	using CMap2 = CMap2_T<MAP_TRAITS, MAP_TYPE>;

	static const Orbit VERTEX = CMap2::VERTEX;
	static const Orbit EDGE   = CMap2::EDGE;
	static const Orbit FACE   = CMap2::FACE;
	static const Orbit VOLUME = CMap2::VOLUME;

	using Vertex = CMap2::Vertex;
	using Edge = CMap2::Edge;
	using Face = CMap2::Face;
	using Volume = CMap2::Volume;

public:

	inline MRCMap2_T() {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(MRCMap2_T);
	inline ~MRCMap2_T() {}



protected:
	/*******************************************************************************
	 * Orbits traversal
	 *******************************************************************************/

	inline Dart phi1(Dart d)
	{
		return this->current()->phi1(d);
	}

	inline Dart phi_1(Dart d)
	{
		return this->current()->phi_1(d);
	}

	inline Dart phi2(Dart d)
	{
		return this->current()->phi2(d);
	}

	template <typename FUNC>
	inline void foreach_dart_of_vertex(Dart d, const FUNC& f) const
	{
		this->current()->foreach_dart_of_vertex(d, f);
	}

	template <typename FUNC>
	inline void foreach_dart_of_face(Dart d, const FUNC& f) const
	{
		this->current()->foreach_dart_of_face(d, f);
	}

	template <typename FUNC>
	void foreach_dart_of_volume(Dart d, const FUNC& f) const
	{
		this->current()->foreach_dart_of_volume(d, f);
	}

public:
	template <Orbit ORBIT, typename FUNC>
	inline void foreach_dart_of_orbit(Cell<ORBIT> c, const FUNC& f) const
	{
		switch (ORBIT)
		{
			case Orbit::DART: foreach_dart_of_orbit(c, f); break;
			case Orbit::PHI1: foreach_dart_of_orbit(c, f); break;
			case Orbit::PHI2: //TODO add a foreach_dart_of_edge to cmap2 f(c.dart); f(phi2(c.dart)); break;
			case Orbit::PHI1_PHI2: foreach_dart_of_volume(c, f); break;
			case Orbit::PHI21: foreach_dart_of_vertex(c, f); break;
			case Orbit::PHI2_PHI3:
			case Orbit::PHI1_PHI3:
			case Orbit::PHI21_PHI31:
			default: cgogn_assert_not_reached("Cells of this dimension are not handled"); break;
		}
	}
};

template <typename MAP_TRAITS>
struct MRCMap2Type
{
	using TYPE = MRCMap2_T<MAP_TRAITS, MRCMap2Type<MAP_TRAITS>>;
};

template <typename MAP_TRAITS>
using MRCMap2 = MRCMap2_T<MAP_TRAITS, MRCMap2Type<MAP_TRAITS>>;

} // namespace cgogn

#endif // CGOGN_MULTIRESOLUTION_MRCMAP_MRCMAP2_H_
