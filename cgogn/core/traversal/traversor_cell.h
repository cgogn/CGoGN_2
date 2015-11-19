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

#ifndef CORE_TRAVERSAL_TRAVERSORCELL_H_
#define CORE_TRAVERSAL_TRAVERSORCELL_H_

#include <core/basic/cell.h>
#include <core/basic/dart_marker.h>

namespace cgogn
{

template <typename MAP, unsigned int ORBIT>
class TraversorCell
{
public:
	typedef TraversorCell<MAP,ORBIT> Self;
	typedef MAP Map;
	typedef std::integral_constant<unsigned int, ORBIT> orbit_type;
	using   DartMarker = cgogn::DartMarker<Map>;
protected:

	MAP& map_;
	DartMarker* dm_;

public:

	TraversorCell(MAP& map) :
		map_(map)
	{
		dm_ = new DartMarker(map_);
	}

	virtual ~TraversorCell()
	{
		delete dm_;
	}

	class iterator
	{
	public:

		TraversorCell& traversor_;
		typename MAP::const_iterator map_it_;

		inline iterator(TraversorCell& t) :
			traversor_(t),
			map_it_(t.map_.begin())
		{
//			unsigned int dim = map_.dimension();
//			while(map_it_ != map_.end() && map_.is_boundary_marked(dim, *map_it_))
//				++map_it_;

			if (map_it_ != traversor_.map_.end())
				traversor_.dm_->template mark_orbit<ORBIT>(*map_it_);
		}

		inline iterator(TraversorCell& t, typename MAP::const_iterator it) :
			traversor_(t),
			map_it_(it)
		{
//			unsigned int dim = map_.dimension();
//			while(map_it_ != map_.end() && map_.is_boundary_marked(dim, *map_it_))
//				++map_it_;

			if (map_it_ != traversor_.map_.end())
				traversor_.dm_->template mark_orbit<ORBIT>(*map_it_);
		}

		inline iterator& operator++()
		{
			cgogn_message_assert(map_it_ != traversor_.map_.end(), "TraversorCell: iterator ++ after end");
			while (map_it_ != traversor_.map_.end() && (traversor_.dm_->is_marked(*map_it_) /*|| traversor_.map_.is_boundary_marked(dim, *it)*/))
				++map_it_;
			if (map_it_ != traversor_.map_.end())
				traversor_.dm_->template mark_orbit<ORBIT>(*map_it_);
			return *this;
		}

		inline Cell<ORBIT> operator*()
		{
			return Cell<ORBIT>(*map_it_);
		}

		inline bool operator!=(iterator it) const
		{
			return map_it_ != it.map_it_;
		}
	};

	inline iterator begin()
	{
		return iterator(*this);
	}

	inline iterator end()
	{
		return iterator(*this, map_.end());
	}
};

} // namespace cgogn

#endif // CORE_TRAVERSAL_TRAVERSORCELL_H_
