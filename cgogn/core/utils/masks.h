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

#include <vector>

namespace cgogn
{

template <typename CellType>
class MaskCell
{
public:
	virtual ~MaskCell() {}
	virtual void operator() (int) const final {}

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

	CellCache(MAP& m) : map_(m)
	{
		map_.foreach_cell([&] (CellType c) { cells_.push_back(c); });
		current_ = cells_.begin();
	}

	CellType begin() const
	{
		current_ = cells_.begin();
		return *current_;
	}

	CellType next() const
	{
		++current_;
		return *current_;
	}

	bool end() const
	{
		return current_ == cells_.end();
	}

	void update()
	{
		cells_.clear();
		map_.foreach_cell([&] (CellType c) { cells_.push_back(c); });
		current_ = cells_.begin();
	}
};

} // namespace cgogn

#endif // CORE_UTILS_MASKS_H_
