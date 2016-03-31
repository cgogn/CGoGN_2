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

#ifndef CORE_UTILS_MASKS_H_
#define CORE_UTILS_MASKS_H_

#include <core/utils/definitions.h>
#include <vector>

namespace cgogn
{

template <typename CellType>
class MaskCell
{
public:

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(MaskCell);
	inline MaskCell() {}
	virtual ~MaskCell() {}
	virtual void operator() (CellType) const final {}

	virtual CellType begin() const = 0;
	virtual CellType next() const = 0;
	virtual bool end() const = 0;
};

template <typename CellType, typename MAP>
class CellCache : public MaskCell<CellType>
{
	MAP& map_;
	mutable typename std::vector<CellType>::const_iterator current_;
	std::vector<CellType> cells_;

public:

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CellCache);
	CellCache(MAP& m) : map_(m)
	{
		cells_.reserve(4096u);
		update();
	}

	CellType begin() const
	{
		current_ = cells_.begin();
		if (end()) return CellType();
		return *current_;
	}

	CellType next() const
	{
		++current_;
		return *current_;
	}

	bool end() const { return current_ == cells_.end(); }

	void update()
	{
		cells_.clear();
		map_.foreach_cell([&] (CellType c) { cells_.push_back(c); });
		current_ = cells_.begin();
	}
};

template <typename MAP>
class BoundaryCache : public MaskCell<typename MAP::Boundary>
{
	using CellType = typename MAP::Boundary;

	MAP& map_;
	mutable typename std::vector<CellType>::const_iterator current_;
	std::vector<CellType> cells_;

public:

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(BoundaryCache);
	BoundaryCache(MAP& m) : map_(m)
	{
		cells_.reserve(4096u);
		update();
	}

	CellType begin() const
	{
		current_ = cells_.begin();
		if (end()) return CellType();
		return *current_;
	}

	CellType next() const
	{
		++current_;
		return *current_;
	}

	bool end() const { return current_ == cells_.end(); }

	void update()
	{
		cells_.clear();
		typename MAP::DartMarker dm(map_);
		map_.foreach_dart([&] (Dart d)
		{
			if (!dm.is_marked(d))
			{
				CellType c(d);
				dm.mark_orbit(c);
				if (map_.is_boundary(d))
					cells_.push_back(c);
			}
		});
		current_ = cells_.begin();
	}
};

} // namespace cgogn

#endif // CORE_UTILS_MASKS_H_
