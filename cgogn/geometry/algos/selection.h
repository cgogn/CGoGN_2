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

#include <cgogn/core/utils/masks.h>
#include <cgogn/core/cmap/cmap3.h>
#include <cgogn/core/cmap/attribute.h>
#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/geometry/algos/area.h>
#include <cgogn/geometry/functions/inclusion.h>
#include <cgogn/geometry/functions/intersection.h>

namespace cgogn
{

namespace geometry
{

template <typename VEC3>
class CollectorGen : public CellTraversor
{
public:

	using Inherit = CellTraversor;
	using Self = CollectorGen<VEC3>;

	using Scalar = typename vector_traits<VEC3>::Scalar;
	using const_iterator = std::vector<Dart>::const_iterator;

	inline CollectorGen() : Inherit()
	{}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CollectorGen);

	virtual ~CollectorGen() {}

	template <typename CellType>
	inline const_iterator begin() const
	{
		return cells_[CellType::ORBIT].begin();
	}

	template <typename CellType>
	inline const_iterator end() const
	{
		return cells_[CellType::ORBIT].end();
	}

	template <typename CellType>
	inline std::size_t size() const
	{
		return cells_[CellType::ORBIT].size();
	}

	virtual void collect(const Dart v_center) = 0;

	inline const std::vector<Dart>& cells(cgogn::Orbit orbit) const
	{
		return cells_[orbit];
	}

	template <typename CellType>
	inline const std::vector<CellType>& cells() const
	{
		return reinterpret_cast<const std::vector<CellType>&>(cells_[CellType::ORBIT]);
	}

	template <typename FUNC>
	inline void foreach_border(const FUNC& f) const
	{
		for (Dart d : border_)
			f(d);
	}

	inline void clear()
	{
		for (auto& cells_vector : cells_)
		{
			cells_vector.clear();
			cells_vector.reserve(256u);
		}
		this->border_.clear();
		this->border_.reserve(256u);
	}

	virtual Scalar area(const Attribute_T<VEC3>& position) const = 0;

protected:

	Dart center_;
	std::array<std::vector<Dart>, NB_ORBITS> cells_;
	std::vector<Dart> border_;
};

template <typename VEC3, typename MAP>
class Collector : public CollectorGen<VEC3>
{
public:

	using Inherit = CollectorGen<VEC3>;
	using Self = Collector<VEC3, MAP>;

	using Scalar = typename Inherit::Scalar;
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Face = typename MAP::Face;

	inline Collector(const MAP& m) : map_(m)
	{}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(Collector);

	virtual void collect(const Vertex center) = 0;
	virtual void collect(const Dart v_center) override
	{
		this->collect(Vertex(v_center));
	}

	virtual Scalar area(const typename MAP::template VertexAttribute<VEC3>& position) const = 0;
	virtual Scalar area(const Attribute_T<VEC3>& position) const override
	{
		const typename MAP::template VertexAttribute<VEC3>* pos_att = dynamic_cast<const typename MAP::template VertexAttribute<VEC3>*>(&position);
		if (pos_att && pos_att->is_valid())
			return this->area(*pos_att);
		return std::numeric_limits<Scalar>::quiet_NaN();
	}

protected:

	const MAP& map_;
};

template <typename VEC3, typename MAP>
class Collector_OneRing : public Collector<VEC3, MAP>
{
public:

	using Inherit = Collector<VEC3, MAP>;
	using Self = Collector_OneRing<VEC3, MAP>;

	using Scalar = typename vector_traits<VEC3>::Scalar;
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Face = typename MAP::Face;

	using Inherit::collect;
	using Inherit::area;

	Collector_OneRing(const MAP& map) : Inherit(map)
	{}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(Collector_OneRing);

	void collect(const Vertex center) override
	{
		this->clear();
		this->center_ = center.dart;

		this->cells_[Vertex::ORBIT].push_back(center.dart);
		this->map_.foreach_adjacent_vertex_through_edge(center, [&] (Vertex nv)
		{
			this->cells_[Edge::ORBIT].push_back(nv.dart);
			this->cells_[Face::ORBIT].push_back(nv.dart);
			this->border_.push_back(this->map_.phi_1(nv.dart));
		});
	}

	Scalar area(const typename MAP::template VertexAttribute<VEC3>& position) const override
	{
		Scalar result = 0;
		for (Dart d : this->cells_[Face::ORBIT])
			result += geometry::area<VEC3>(this->map_, Face(d), position);
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

	using Inherit::collect;
	using Inherit::area;

	Collector_WithinSphere(
		const MAP& map,
		const Scalar radius,
		const typename MAP::template VertexAttribute<VEC3>& position
	) : Inherit(map),
		radius_(radius),
		position_(position)
	{}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(Collector_WithinSphere);

	void collect(const Vertex center) override
	{
		this->clear();
		this->center_ = center.dart;

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
				this->map_.foreach_dart_of_orbit(e, [&] (Dart dd) -> bool
				{
					if (!dm.is_marked(dd))
						all_in = false;
					return all_in;
				});
				if (all_in)
					this->cells_[Edge::ORBIT].push_back(d);

				// check if the face of d is now completely marked
				// (which means all the vertices of the face are in the sphere)
				Face f(d);
				all_in = true;
				this->map_.foreach_dart_of_orbit(f, [&] (Dart dd) -> bool
				{
					if (!dm.is_marked(dd))
						all_in = false;
					return all_in;
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
			Vertex v = Vertex(this->cells_[Vertex::ORBIT][i]);
			this->map_.foreach_adjacent_vertex_through_edge(v, [&] (Vertex av)
			{
				// check if the neighbor vertex through the edge is in the sphere
				// if it is in the sphere and has not been marked yet, put it in the queue
				if (in_sphere(position_[av], center_position, radius_))
				{
					if (!dm.is_marked(av.dart))
					{
						this->cells_[Vertex::ORBIT].push_back(av.dart);
						mark_vertex(av);
					}
				}
				// if it is not in the sphere, put the dart (pointing out of the sphere) in the border list
				else
					this->border_.push_back(this->map_.phi2(av.dart));
			});

			++i;
		}

		this->traversed_cells_ |= orbit_mask<Vertex>() | orbit_mask<Edge>() | orbit_mask<Face>();
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

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_GEOMETRY_ALGOS_SELECTION_CPP_))
extern template CGOGN_GEOMETRY_API class Collector_OneRing<Eigen::Vector3f, CMap2>;
extern template CGOGN_GEOMETRY_API class Collector_OneRing<Eigen::Vector3d, CMap2>;
extern template CGOGN_GEOMETRY_API class Collector_OneRing<Eigen::Vector3f, CMap3>;
extern template CGOGN_GEOMETRY_API class Collector_OneRing<Eigen::Vector3d, CMap3>;
extern template CGOGN_GEOMETRY_API class Collector_WithinSphere<Eigen::Vector3f, CMap2>;
extern template CGOGN_GEOMETRY_API class Collector_WithinSphere<Eigen::Vector3d, CMap2>;
extern template CGOGN_GEOMETRY_API class Collector_WithinSphere<Eigen::Vector3f, CMap3>;
extern template CGOGN_GEOMETRY_API class Collector_WithinSphere<Eigen::Vector3d, CMap3>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_GEOMETRY_ALGOS_SELECTION_CPP_))

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_ALGOS_SELECTION_H_
