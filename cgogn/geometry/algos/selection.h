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
#include <cgogn/geometry/algos/area.h>
#include <cgogn/geometry/functions/inclusion.h>
#include <cgogn/geometry/functions/intersection.h>

namespace cgogn
{

namespace geometry
{

template <typename VEC3, typename MAP>
class Collector
{
public:

	using Scalar = typename vector_traits<VEC3>::Scalar;
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Face = typename MAP::Face;

	inline Collector(const MAP& m) : map_(m)
	{}

	template <typename CellType>
	inline std::size_t size() const
	{
		return cells_[CellType::ORBIT].size();
	}

	template <typename CellType>
	inline const std::vector<CellType>& cells() const
	{
		return reinterpret_cast<const std::vector<CellType>&>(cells_[CellType::ORBIT]);
	}

	template <typename FUNC>
	void foreach_cell(const FUNC& f)
	{
		using CellType = func_parameter_type(FUNC);
		static const Orbit ORBIT = CellType::ORBIT;
		for (Dart d : cells_[ORBIT])
			f(CellType(d));
	}

	template <typename FUNC>
	void foreach_border(const FUNC& f)
	{
		for (Dart d : border_)
			f(d);
	}

	virtual void collect(const Vertex center) = 0;

	virtual Scalar area(const typename MAP::template VertexAttribute<VEC3>& position) const = 0;

protected:

	void clear()
	{
		for (auto& cells_vector : cells_)
		{
			cells_vector.clear();
			cells_vector.reserve(256u);
		}
		border_.clear();
		border_.reserve(256u);
	}

	const MAP& map_;
	Vertex center_;
	std::array<std::vector<Dart>, NB_ORBITS> cells_;
	std::vector<Dart> border_;
};

template <typename VEC3, typename MAP>
class Collector_OneRing : public Collector<VEC3, MAP>
{
public:

	using Self = Collector_OneRing<VEC3, MAP>;
	using Inherit = Collector<VEC3, MAP>;

	using Scalar = typename vector_traits<VEC3>::Scalar;
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Face = typename MAP::Face;

	Collector_OneRing(const MAP& map) : Inherit(map)
	{}

	void collect(const Vertex center) override
	{
		this->clear();
		this->center_ = center;

		this->cells_[Vertex::ORBIT].push_back(center.dart);
		this->map_.foreach_adjacent_vertex_through_edge(center, [&] (Vertex nv)
		{
			this->cells_[Edge::ORBIT].push_back(nv.dart);
			this->cells_[Face::ORBIT].push_back(nv.dart);
			this->border_.push_back(this->map_.phi1(nv.dart));
		});
	}

	Scalar area(const typename MAP::template VertexAttribute<VEC3>& position) const override
	{
		Scalar result = 0;
		for (Dart d : this->cells_[Face::ORBIT])
		{
			result += geometry::area<VEC3>(this->map_, Face(d), position);
		}
		return result;
	}
};

template <typename VEC3, typename MAP>
class Collector_WithinSphere : public Collector<VEC3, MAP>
{
public:

	using Self = Collector_WithinSphere<VEC3, MAP>;
	using Inherit = Collector<VEC3, MAP>;

	using Scalar = typename vector_traits<VEC3>::Scalar;
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Face = typename MAP::Face;

	Collector_WithinSphere(
		const MAP& map,
		const Scalar radius,
		const typename MAP::template VertexAttribute<VEC3>& position
	) : Inherit(map),
		radius_(radius),
		position_(position)
	{}

	void collect(const Vertex center) override
	{
		this->clear();
		this->center_ = center;

		const VEC3& center_position = position_[center];

		typename MAP::DartMarkerStore dm(this->map_);

		auto mark_vertex = [&] (Vertex v)
		{
			this->map_.foreach_dart_of_orbit(v, [&] (Dart d)
			{
				// mark a dart of the vertex
				dm.mark(d);

				// check if the edge of d is now completely marked
				// (which means all the vertices of the edge are in the sphere)
				Edge e(d);
				bool all_in = true;
				this->map_.foreach_dart_of_orbit_until(e, [&] (Dart dd) -> bool
				{
					if (!dm.is_marked(dd))
					{
						all_in = false;
						return false;
					}
					return true;
				});
				if (all_in)
					this->cells_[Edge::ORBIT].push_back(d);

				// check if the face of d is now completely marked
				// (which means all the vertices of the face are in the sphere)
				Face f(d);
				all_in = true;
				this->map_.foreach_dart_of_orbit_until(f, [&] (Dart dd) -> bool
				{
					if (!dm.is_marked(dd))
					{
						all_in = false;
						return false;
					}
					return true;
				});
				if (all_in)
					this->cells_[Face::ORBIT].push_back(d);
			});
		};

		this->cells_[Vertex::ORBIT].push_back(center.dart);
		mark_vertex(center);

		uint32 i = 0;
		while (i < this->cells_[Vertex::ORBIT].size())
		{
			Dart vd = this->cells_[Vertex::ORBIT][i];
			this->map_.foreach_dart_of_orbit(Vertex(vd), [&] (Dart d)
			{
				// check if the neighbor vertex through the edge is in the sphere
				// if it is in the sphere and has not been marked yet, put it in the queue
				Dart d2 = this->map_.phi2(d);
				if (in_sphere(position_[Vertex(d2)], center_position, radius_))
				{
					if (!dm.is_marked(d2))
					{
						this->cells_[Vertex::ORBIT].push_back(d2);
						mark_vertex(Vertex(d2));
					}
				}
				// if it is not in the sphere, put the dart in the border list
				else
					this->border_.push_back(d);
			});

			++i;
		}
	}

	Scalar area(const typename MAP::template VertexAttribute<VEC3>& position) const override
	{
		Scalar result = 0;
		const VEC3& center_position = position[this->center_];
		for (Dart d : this->cells_[Face::ORBIT])
		{
			result += geometry::area<VEC3>(this->map_, Face(d), position);
		}
		// TODO: the following works only for triangle meshes
		for (Dart d : this->border_)
		{
			// Vertex(d) is inside
			const Dart f = this->map_.phi1(d); // Vertex(f) is outside
			const Dart g = this->map_.phi1(f);
			if (geometry::in_sphere(position[Vertex(g)], center_position, radius_)) // Vertex(g) is inside
			{
				Scalar alpha, beta;
				geometry::intersection_sphere_segment<VEC3>(center_position, radius_, position[Vertex(d)], position[Vertex(f)], alpha);
				geometry::intersection_sphere_segment<VEC3>(center_position, radius_, position[Vertex(g)], position[Vertex(f)], beta);
				result += (alpha+beta - alpha*beta) * geometry::area<VEC3>(this->map_, Face(d), position);
			}
			else // Vertex(g) is outside
			{
				Scalar alpha, beta;
				geometry::intersection_sphere_segment<VEC3>(center_position, radius_, position[Vertex(d)], position[Vertex(f)], alpha);
				geometry::intersection_sphere_segment<VEC3>(center_position, radius_, position[Vertex(d)], position[Vertex(g)], beta);
				result += alpha * beta * geometry::area<VEC3>(this->map_, Face(d), position);
			}
		}
		return result;
	}

protected:

	Scalar radius_;
	const typename MAP::template VertexAttribute<VEC3>& position_;
};

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_ALGOS_SELECTION_H_
