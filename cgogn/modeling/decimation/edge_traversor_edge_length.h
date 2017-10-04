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

#ifndef CGOGN_MODELING_DECIMATION_EDGE_TRAVERSOR_EDGE_LENGTH_H_
#define CGOGN_MODELING_DECIMATION_EDGE_TRAVERSOR_EDGE_LENGTH_H_

#include <cgogn/core/utils/masks.h>
#include <cgogn/geometry/algos/length.h>

namespace cgogn
{

namespace modeling
{

template <typename MAP, typename VEC3>
class EdgeTraversor_EdgeLength : public CellTraversor
{
public:

	using Inherit = CellTraversor;
	using Self = EdgeTraversor_EdgeLength<MAP, VEC3>;
	using Scalar = typename geometry::vector_traits<VEC3>::Scalar;
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;

	struct EdgeInfo
	{
		typename std::multimap<Scalar, Edge>::const_iterator it_;
		bool valid_;

		EdgeInfo() : valid_(false) {}

		friend std::ostream& operator<<(std::ostream& out, const EdgeInfo&) { return out; }
		friend std::istream& operator>>(std::istream& in, const EdgeInfo&) { return in; }
	};

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(EdgeTraversor_EdgeLength);

	inline EdgeTraversor_EdgeLength(
		MAP& m,
		const typename MAP::template VertexAttribute<VEC3>& position
	) : Inherit(),
		map_(m),
		position_(position)
	{
		einfo_ = map_.template add_attribute<EdgeInfo, Edge>("EdgeTraversor_EdgeLength_EdgeInfo");

		map_.foreach_cell([&] (Edge e)
		{
			einfo_[e].valid_ = false;
			update_edge_info(e);
		});

		this->traversed_cells_ |= orbit_mask<Edge>();
	}

	~EdgeTraversor_EdgeLength() override
	{
		map_.remove_attribute(einfo_);
	}

	void pre_collapse(Edge e)
	{
		EdgeInfo& ei = einfo_[e];
		if (ei.valid_) { edges_.erase(ei.it_); ei.valid_ = false; }

		e1_ = Edge(map_.phi2(map_.phi_1(e.dart)));
		e2_ = Edge(map_.phi2(map_.phi_1(map_.phi2(e.dart))));

		Dart ed1 = e.dart;
		Dart ed2 = map_.phi2(ed1);

		EdgeInfo& ei1 = einfo_[Edge(map_.phi1(ed1))];
		if (ei1.valid_) { edges_.erase(ei1.it_); ei1.valid_ = false; }

		EdgeInfo& ei2 = einfo_[Edge(map_.phi_1(ed1))];
		if (ei2.valid_) { edges_.erase(ei2.it_); ei2.valid_ = false; }

		EdgeInfo& ei3 = einfo_[Edge(map_.phi1(ed2))];
		if (ei3.valid_) { edges_.erase(ei3.it_); ei3.valid_ = false; }

		EdgeInfo& ei4 = einfo_[Edge(map_.phi_1(ed2))];
		if (ei4.valid_) { edges_.erase(ei4.it_); ei4.valid_ = false; }
	}

	void post_collapse()
	{
		Dart vit = e1_.dart;
		do
		{
			update_edge_info(Edge(map_.phi1(vit)));
			if(vit == e1_.dart || vit == e2_.dart)
			{
				update_edge_info(Edge(vit));

				Dart vit2 = map_.template phi<121>(vit);
				Dart stop = map_.phi2(vit);
				do
				{
					update_edge_info(Edge(vit2));
					update_edge_info(Edge(map_.phi1(vit2)));
					vit2 = map_.phi1(map_.phi2(vit2));
				} while(vit2 != stop);
			}
			else
				update_edge_info(Edge(vit));

			vit = map_.phi2(map_.phi_1(vit));
		} while(vit != e1_.dart);
	}

	void update_edge_info(Edge e)
	{
		EdgeInfo& ei = einfo_[e];
		if (map_.edge_can_collapse(e))
		{
			Scalar cost = geometry::length<VEC3>(map_, e, position_);
			if (ei.valid_)
			{
				edges_.erase(ei.it_);
				ei.it_ = edges_.insert(std::make_pair(cost, e));
			}
			else
			{
				ei.it_ = edges_.insert(std::make_pair(cost, e));
				ei.valid_ = true;
			}
		}
		else
		{
			if (ei.valid_)
			{
				edges_.erase(ei.it_);
				ei.valid_ = false;
			}
		}
	}

	class const_iterator
	{
	public:

		const Self* const trav_ptr_;
		typename std::multimap<Scalar, Edge>::const_iterator edge_it_;

		inline const_iterator(const Self* trav, typename std::multimap<Scalar, Edge>::const_iterator it) :
			trav_ptr_(trav),
			edge_it_(it)
		{}

		inline const_iterator(const const_iterator& it) :
			trav_ptr_(it.trav_ptr_),
			edge_it_(it.edge_it_)
		{}

		inline const_iterator& operator=(const const_iterator& it)
		{
			trav_ptr_ = it.trav_ptr_;
			edge_it_ = it.edge_it_;
			return *this;
		}

		inline const_iterator& operator++()
		{
			edge_it_ = trav_ptr_->edges_.begin();
			return *this;
		}

		inline const Edge& operator*() const
		{
			return (*edge_it_).second;
		}

		inline bool operator!=(const_iterator it) const
		{
			cgogn_assert(trav_ptr_ == it.trav_ptr_);
			return edge_it_ != it.edge_it_;
		}
	};

	template <typename CellType,
			  typename std::enable_if<std::is_same<CellType, typename MAP::Edge>::value>::type* = nullptr>
	inline const_iterator begin() const
	{
		return const_iterator(this, edges_.begin());
	}

	template <typename CellType,
			  typename std::enable_if<std::is_same<CellType, typename MAP::Edge>::value>::type* = nullptr>
	inline const_iterator end() const
	{
		return const_iterator(this, edges_.end());
	}

private:

	MAP& map_;
	const typename MAP::template VertexAttribute<VEC3>& position_;
	typename MAP::template EdgeAttribute<EdgeInfo> einfo_;
	std::multimap<Scalar, Edge> edges_;
	Edge e1_, e2_;
};

} // namespace modeling

} // namespace cgogn

#endif // CGOGN_MODELING_DECIMATION_EDGE_TRAVERSOR_EDGE_LENGTH_H_
