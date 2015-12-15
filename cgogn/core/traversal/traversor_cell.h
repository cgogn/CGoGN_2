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
#include <core/basic/cell_marker.h>

namespace cgogn
{

enum TraversorStrategy
{
	AUTO = 0,
	FORCE_DART_MARKING,
	FORCE_CELL_MARKING,
	FORCE_TOPO_CACHE
};

template <typename MAP, unsigned int ORBIT, TraversorStrategy STRATEGY = AUTO>
class TraversorCell
{
public:

	typedef TraversorCell<MAP, ORBIT, STRATEGY> Self;
	typedef MAP Map;

	using   DartMarker = cgogn::DartMarker<Map>;
	using   CellMarker = cgogn::CellMarker<Map, ORBIT>;

protected:

	Map& map_;
	DartMarker* dm_;
	CellMarker* cm_;

public:

	inline TraversorCell(MAP& map) :
		map_(map),
		dm_(nullptr),
		cm_(nullptr)
	{
		switch (STRATEGY)
		{
			case FORCE_DART_MARKING :
				dm_ = new DartMarker(map_);
				break;
			case FORCE_CELL_MARKING :
				cm_ = new CellMarker(map_);
				break;
			case FORCE_TOPO_CACHE :
				cgogn_assert_not_reached("FORCE_TOPO_CACHE not implemented yet");
				break;
			case AUTO :
				if (map_.template is_orbit_embedded<ORBIT>())
					cm_ = new CellMarker(map_);
				else
					dm_ = new DartMarker(map_);
				break;
		}
	}

	/**
	 * \brief Copy Constructor
	 * \param tc
	 */
	inline TraversorCell(const TraversorCell& tc) :
		map_(tc.map_),
		dm_(tc.dm_),
		cm_(tc.cm_)
	{}

	/**
	 * \brief Move constructor
	 * \param tc
	 */
	inline TraversorCell(TraversorCell&& tc) CGOGN_NOEXCEPT :
		map_(tc.map_),
		dm_(tc.dm_),
		cm_(tc.cm_)
	{
		//prevents the deallocation of memory in
		//when the destructor is called on tc
		tc.dm_ = nullptr;
		tc.cm_ = nullptr;
	}

	/**
	 * \brief operator =
	 * \param tc
	 * \return
	 */
	inline TraversorCell& operator=(const TraversorCell& tc)
	{
		this->map_ = tc.map_;
		this->cm_ = tc.cm_;
		this->dm_ = tc.dm_;
		return *this;
	}

	/**
	 * \brief operator =
	 * \param tc
	 * \return
	 */
	inline TraversorCell& operator=(TraversorCell&& tc)
	{
		this->map_ = tc.map_;
		this->cm_ = tc.cm_;
		this->dm_ = tc.dm_;
		return *this;
	}

	virtual ~TraversorCell()
	{
		switch (STRATEGY)
		{
			case FORCE_DART_MARKING :
				delete dm_;
				break;
			case FORCE_CELL_MARKING :
				delete cm_;
				break;
			case FORCE_TOPO_CACHE :
				cgogn_assert_not_reached("FORCE_TOPO_CACHE not implemented yet");
				break;
			case AUTO :
				if(dm_)
					delete dm_;
				else if(cm_)
					delete cm_;
				break;
			default:
				break;
		}
	}

	class iterator
	{
	public:

		Self& traversor_;
		typename MAP::const_iterator map_it_;

		inline iterator(Self& t) :
			traversor_(t),
			map_it_(t.map_.begin())
		{
//			unsigned int dim = map_.dimension();
//			while(map_it_ != map_.end() && map_.is_boundary_marked(dim, *map_it_))
//				++map_it_;

			switch (STRATEGY)
			{
				case FORCE_DART_MARKING :
					if (map_it_ != traversor_.map_.end())
						traversor_.dm_->template mark_orbit<ORBIT>(*map_it_);
					break;
				case FORCE_CELL_MARKING :
					if (map_it_ != traversor_.map_.end())
						traversor_.cm_->mark(*map_it_);
					break;
				case FORCE_TOPO_CACHE :
					cgogn_assert_not_reached("FORCE_TOPO_CACHE not implemented yet");
					break;
				case AUTO :
					if (map_it_ != traversor_.map_.end())
					{
						if (traversor_.dm_)
							traversor_.dm_->template mark_orbit<ORBIT>(*map_it_);
						else
							traversor_.cm_->mark(*map_it_);
					}
					break;
			}

		}

		inline iterator(Self& t, typename MAP::const_iterator it) :
			traversor_(t),
			map_it_(it)
		{
//			unsigned int dim = map_.dimension();
//			while(map_it_ != map_.end() && map_.is_boundary_marked(dim, *map_it_))
//				++map_it_;

			switch (STRATEGY)
			{
				case FORCE_DART_MARKING :
					if (map_it_ != traversor_.map_.end())
						traversor_.dm_->template mark_orbit<ORBIT>(*map_it_);
					break;
				case FORCE_CELL_MARKING :
					if (map_it_ != traversor_.map_.end())
						traversor_.cm_->mark(*map_it_);
					break;
				case FORCE_TOPO_CACHE :
					cgogn_assert_not_reached("FORCE_TOPO_CACHE not implemented yet");
					break;
				case AUTO :
					if (map_it_ != traversor_.map_.end())
					{
						if (traversor_.dm_)
							traversor_.dm_->template mark_orbit<ORBIT>(*map_it_);
						else
							traversor_.cm_->mark(*map_it_);
					}
					break;
			}
		}

		inline iterator& operator++()
		{
			cgogn_message_assert(map_it_ != traversor_.map_.end(), "TraversorCell: iterator ++ after end");

			switch (STRATEGY)
			{
				case FORCE_DART_MARKING :
					while (map_it_ != traversor_.map_.end() && (traversor_.dm_->is_marked(*map_it_) /*|| traversor_.map_.is_boundary_marked(dim, *it)*/))
						++map_it_;
					if (map_it_ != traversor_.map_.end())
						traversor_.dm_->template mark_orbit<ORBIT>(*map_it_);
					break;
				case FORCE_CELL_MARKING :
					while (map_it_ != traversor_.map_.end() && (traversor_.cm_->is_marked(*map_it_) /*|| traversor_.map_.is_boundary_marked(dim, *it)*/))
						++map_it_;
					if (map_it_ != traversor_.map_.end())
						traversor_.cm_->mark(*map_it_);
					break;
				case FORCE_TOPO_CACHE :
					cgogn_assert_not_reached("FORCE_TOPO_CACHE not implemented yet");
					break;
				case AUTO :
					if (traversor_.dm_)
					{
						while (map_it_ != traversor_.map_.end() && (traversor_.dm_->is_marked(*map_it_) /*|| traversor_.map_.is_boundary_marked(dim, *it)*/))
							++map_it_;
						if (map_it_ != traversor_.map_.end())
							traversor_.dm_->template mark_orbit<ORBIT>(*map_it_);
					}
					else
					{
						while (map_it_ != traversor_.map_.end() && (traversor_.cm_->is_marked(*map_it_) /*|| traversor_.map_.is_boundary_marked(dim, *it)*/))
							++map_it_;
						if (map_it_ != traversor_.map_.end())
							traversor_.cm_->mark(*map_it_);
					}
					break;
			}

			return *this;
		}

		inline Cell<ORBIT> operator*()
		{
			return Cell<ORBIT>(*map_it_);
		}

		inline bool operator!=(iterator it) const
		{
			cgogn_assert(&traversor_ == &(it.traversor_));
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
