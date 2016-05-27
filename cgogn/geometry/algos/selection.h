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

#ifndef CGOGN_GEOMETRY_ALGOS_SELECTION_H_
#define CGOGN_GEOMETRY_ALGOS_SELECTION_H_

#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/core/utils/masks.h>

namespace cgogn
{

namespace geometry
{

template <typename VEC3, typename MAP>
class Collector : public CellTraversor
{
public:

	using Scalar = typename vector_traits<VEC3>::Scalar;
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Face = typename MAP::Face;

	using iterator = std::vector<Dart>::iterator;
	using const_iterator = std::vector<Dart>::const_iterator;

	inline Collector(const MAP& m) : map_(m)
	{}

	template <typename CellType>
	inline const_iterator begin() const
	{
		return cells_[CellType::ORBIT].begin();
	}

	template <typename CellType>
	inline iterator begin()
	{
		return cells_[CellType::ORBIT].begin();
	}

	template <typename CellType>
	inline const_iterator end() const
	{
		return cells_[CellType::ORBIT].end();
	}

	template <typename CellType>
	inline iterator end()
	{
		return cells_[CellType::ORBIT].end();
	}

	template <typename CellType>
	inline std::size_t size() const
	{
		return cells_[CellType::ORBIT].size();
	}

	template <typename CellType>
	void collect_within_sphere(
		const Vertex center,
		Scalar radius,
		const typename MAP::template VertexAttribute<VEC3>& position
	)
	{
		static const Orbit ORBIT = CellType::ORBIT;

		cells_[ORBIT].clear();
		cells_[ORBIT].reserve(1024u);

		switch (ORBIT)
		{
			case Vertex::ORBIT: {
				const VEC3& center_position = position[center];
				cells_[ORBIT].push_back(center.dart);
				typename MAP::template CellMarkerStore<Vertex::ORBIT> cmv(map_);
				uint32 i = 0;
				while (i < cells_[ORBIT].size())
				{
					map_.foreach_adjacent_vertex_through_edge(Vertex(cells_[ORBIT][i]), [&] (Vertex nv)
					{
						if (!cmv.is_marked(nv) && (position[nv] - center_position).norm() < radius)
						{
							cells_[ORBIT].push_back(nv.dart);
							cmv.mark(nv);
						}
					});
				}
				break;
			}
			case Edge::ORBIT: {

				break;
			}
			case Face::ORBIT: {

				break;
			}
		}
	}

protected:

	const MAP& map_;
	std::array<std::vector<Dart>, NB_ORBITS> cells_;
};

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_ALGOS_SELECTION_H_
