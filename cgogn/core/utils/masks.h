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

class MaskCell
{
public:

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(MaskCell);
	inline MaskCell() {}
	virtual ~MaskCell() {}
	virtual void operator() (uint32) const final {}

//	virtual CellType begin() const = 0;
//	virtual CellType next() const = 0;
//	virtual bool end() const = 0;
};

template <typename MAP>
class CellCache : public MaskCell
{
	const MAP& map_;
	mutable std::array<typename std::vector<Dart>::const_iterator, NB_ORBITS> current_;
	std::array<std::vector<Dart>, NB_ORBITS> cells_;

public:

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CellCache);
	CellCache(const MAP& m) : map_(m)
	{}

	template <typename CellType>
	CellType begin() const
	{
		static const Orbit ORBIT = CellType::ORBIT;
		current_[ORBIT] = cells_[ORBIT].begin();
		if (end<CellType>()) return CellType();
		return CellType(*current_[ORBIT]);
	}

	template <typename CellType>
	CellType next() const
	{
		static const Orbit ORBIT = CellType::ORBIT;
		++current_[ORBIT];
		return CellType(*current_[ORBIT]);
	}

	template <typename CellType>
	bool end() const
	{
		static const Orbit ORBIT = CellType::ORBIT;
		return current_[ORBIT] == cells_[ORBIT].end();
	}

	template <typename CellType>
	uint32 size() const
	{
		static const Orbit ORBIT = CellType::ORBIT;
		return cells_[ORBIT].size();
	}

	template <typename CellType>
//	auto update() -> typename std::enable_if<std::is_same<CellType, Vertex>::value, void>::type
	void update()
	{
		static const Orbit ORBIT = CellType::ORBIT;
		cells_[ORBIT].clear();
		cells_[ORBIT].reserve(4096u);
		map_.foreach_cell([&] (CellType c) { cells_[ORBIT].push_back(c.dart); });
		current_[ORBIT] = cells_[ORBIT].begin();
	}
};

template <typename MAP>
class BoundaryCache : public MaskCell
{
	using BoundaryCellType = typename MAP::Boundary;

	const MAP& map_;
	mutable typename std::vector<BoundaryCellType>::const_iterator current_;
	std::vector<BoundaryCellType> cells_;

public:

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(BoundaryCache);
	BoundaryCache(const MAP& m) : map_(m)
	{
		cells_.reserve(4096u);
		update();
	}

	template <typename CellType = BoundaryCellType>
	CellType begin() const
	{
		static_assert(std::is_same<CellType, BoundaryCellType>::value, "BoundaryCache can only be used with BoundaryCellType");
		current_ = cells_.begin();
		if (end()) return CellType();
		return *current_;
	}

	template <typename CellType = BoundaryCellType>
	CellType next() const
	{
		static_assert(std::is_same<CellType, BoundaryCellType>::value, "BoundaryCache can only be used with BoundaryCellType");
		++current_;
		return *current_;
	}

	template <typename CellType = BoundaryCellType>
	bool end() const
	{
		static_assert(std::is_same<CellType, BoundaryCellType>::value, "BoundaryCache can only be used with BoundaryCellType");
		return current_ == cells_.end();
	}

	template <typename CellType = BoundaryCellType>
	void update()
	{
		static_assert(std::is_same<CellType, BoundaryCellType>::value, "BoundaryCache can only be used with BoundaryCellType");
		cells_.clear();
		typename MAP::DartMarker dm(map_);
		map_.foreach_dart([&] (Dart d)
		{
			if (!dm.is_marked(d))
			{
				BoundaryCellType c(d);
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
