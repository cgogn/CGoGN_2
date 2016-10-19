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

#ifndef CGOGN_GEOMETRY_ALGOS_AREA_H_
#define CGOGN_GEOMETRY_ALGOS_AREA_H_

#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/geometry/functions/area.h>
#include <cgogn/geometry/algos/centroid.h>

#include <cgogn/core/cmap/attribute.h>

#include <cgogn/core/utils/masks.h>

namespace cgogn
{

namespace geometry
{

template <typename VEC3, typename MAP>
inline typename vector_traits<VEC3>::Scalar convex_area(
	const MAP& map,
	const typename MAP::Face f,
	const typename MAP::template VertexAttribute<VEC3>& position
)
{
	using Scalar = typename vector_traits<VEC3>::Scalar;
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;

	if (map.codegree(f) == 3)
		return area<VEC3>(position[Vertex(f.dart)], position[Vertex(map.phi1(f.dart))], position[Vertex(map.phi_1(f.dart))]);
	else
	{
		Scalar face_area{0};
		VEC3 center = centroid<VEC3>(map, f, position);
		map.foreach_incident_edge(f, [&] (Edge e)
		{
			face_area += area<VEC3>(center, position[Vertex(e.dart)], position[Vertex(map.phi1(e.dart))]);
		});
		return face_area;
	}
}

template <typename VEC3, typename MAP>
inline typename vector_traits<VEC3>::Scalar area(
	const MAP& map,
	const typename MAP::Face f,
	const typename MAP::template VertexAttribute<VEC3>& position
)
{
	return convex_area<VEC3>(map, f, position);
}

template <typename VEC3, typename CellType, typename MAP>
inline auto area(
	const MAP& map,
	const CellType c,
	const typename MAP::template VertexAttribute<VEC3>& position
) -> typename std::enable_if<!std::is_same<CellType, typename MAP::Face>::value, typename vector_traits<VEC3>::Scalar>::type
{
	using Scalar = typename vector_traits<VEC3>::Scalar;
	using Face = typename MAP::Face;

	Scalar cell_area(0);
	map.foreach_incident_face(c, [&] (Face f)
	{
		cell_area += area<VEC3>(map, f, position) / map.codegree(f);
	});
	return cell_area;
}

template <typename VEC3, typename CellType, typename MAP, typename MASK>
inline void compute_area(
	const MAP& map,
	const MASK& mask,
	const typename MAP::template VertexAttribute<VEC3>& position,
	Attribute<typename vector_traits<VEC3>::Scalar, CellType::ORBIT>& cell_area
)
{
	map.parallel_foreach_cell([&] (CellType c, uint32)
	{
		cell_area[c] = area<VEC3>(map, c, position);
	},
	mask);
}

template <typename VEC3, typename CellType, typename MAP>
inline void compute_area(
	const MAP& map,
	const typename MAP::template VertexAttribute<VEC3>& position,
	Attribute<typename vector_traits<VEC3>::Scalar, CellType::ORBIT>& cell_area
)
{
	compute_area<VEC3, CellType>(map, CellFilters(), position, cell_area);
}

template <typename VEC3, typename CellType, typename MAP>
inline typename vector_traits<VEC3>::Scalar incident_faces_area(
	const MAP& map,
	const CellType c,
	const typename MAP::template VertexAttribute<VEC3>& position
)
{
	using Scalar = typename vector_traits<VEC3>::Scalar;
	using Face = typename MAP::Face;

	Scalar incident_area(0);
	map.foreach_incident_face(c, [&] (Face f)
	{
		incident_area += area<VEC3>(map, f, position);
	});
	return incident_area;
}

template <typename VEC3, typename CellType, typename MAP, typename MASK>
inline void compute_incident_faces_area(
	const MAP& map,
	const MASK& mask,
	const typename MAP::template VertexAttribute<VEC3>& position,
	Attribute<typename vector_traits<VEC3>::Scalar, CellType::ORBIT>& area
)
{
	map.parallel_foreach_cell([&] (CellType c, uint32)
	{
		area[c] = incident_faces_area<VEC3>(map, c, position);
	},
	mask);
}

template <typename VEC3, typename CellType, typename MAP>
inline void compute_incident_faces_area(
	const MAP& map,
	const typename MAP::template VertexAttribute<VEC3>& position,
	Attribute<typename vector_traits<VEC3>::Scalar, CellType::ORBIT>& area
)
{
	compute_incident_faces_area<VEC3, CellType>(map, CellFilters(), position, area);
}

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_ALGOS_AREA_H_
