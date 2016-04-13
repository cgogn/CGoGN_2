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

#ifndef CGOGN_CORE_UTILS_MASKS_H_
#define CGOGN_CORE_UTILS_MASKS_H_

#include <vector>

#include <cgogn/core/utils/numerics.h>
#include <cgogn/core/basic/cell.h>

namespace cgogn
{

/**
 * @brief The CellFilters class
 * Classes inheriting from CellFilters can be used as a parameter to map.foreach_cell()
 * They can personalize the filtering function used to filter each Orbit traversal
 */
class CellFilters
{
public:

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CellFilters);
	CellFilters() {}
	virtual ~CellFilters() {}
	virtual void operator() (uint32) const final {}

	template <typename CellType>
	auto filter(CellType c) const -> typename std::enable_if<CellType::ORBIT == Orbit::DART, bool>::type
	{
		return filter_DART(c);
	}
	template <typename CellType>
	auto filter(CellType c) const -> typename std::enable_if<CellType::ORBIT == Orbit::PHI1, bool>::type
	{
		return filter_PHI1(c);
	}
	template <typename CellType>
	auto filter(CellType c) const -> typename std::enable_if<CellType::ORBIT == Orbit::PHI2, bool>::type
	{
		return filter_PHI2(c);
	}
	template <typename CellType>
	auto filter(CellType c) const -> typename std::enable_if<CellType::ORBIT == Orbit::PHI1_PHI2, bool>::type
	{
		return filter_PHI1_PHI2(c);
	}
	template <typename CellType>
	auto filter(CellType c) const -> typename std::enable_if<CellType::ORBIT == Orbit::PHI1_PHI3, bool>::type
	{
		return filter_PHI1_PHI3(c);
	}
	template <typename CellType>
	auto filter(CellType c) const -> typename std::enable_if<CellType::ORBIT == Orbit::PHI2_PHI3, bool>::type
	{
		return filter_PHI2_PHI3(c);
	}
	template <typename CellType>
	auto filter(CellType c) const -> typename std::enable_if<CellType::ORBIT == Orbit::PHI21, bool>::type
	{
		return filter_PHI21(c);
	}
	template <typename CellType>
	auto filter(CellType c) const -> typename std::enable_if<CellType::ORBIT == Orbit::PHI21_PHI31, bool>::type
	{
		return filter_PHI21_PHI31(c);
	}

protected:

	virtual bool filter_DART(Cell<Orbit::DART>) const { return true; }
	virtual bool filter_PHI1(Cell<Orbit::PHI1>) const { return true; }
	virtual bool filter_PHI2(Cell<Orbit::PHI2>) const { return true; }
	virtual bool filter_PHI1_PHI2(Cell<Orbit::PHI1_PHI2>) const { return true; }
	virtual bool filter_PHI1_PHI3(Cell<Orbit::PHI1_PHI3>) const { return true; }
	virtual bool filter_PHI2_PHI3(Cell<Orbit::PHI2_PHI3>) const { return true; }
	virtual bool filter_PHI21(Cell<Orbit::PHI21>) const { return true; }
	virtual bool filter_PHI21_PHI31(Cell<Orbit::PHI21_PHI31>) const { return true; }
};

/**
 * @brief The CellTraversor class
 * Classes inheriting from CellTraversor can be used as a parameter to map.foreach_cell()
 * They should provide the following methods :
 *  - template <typename CellType> CellType begin() const
 *  - template <typename CellType> CellType end() const
 *" - template <typename CellType> CellType next() const
 */
class CellTraversor
{
public:

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CellTraversor);
	inline CellTraversor() {}
	virtual ~CellTraversor() {}
	virtual void operator() (uint32) const final {}
};


template <typename MAP>
class CellCache : public CellTraversor
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
	void build()
	{
		this->build<CellType>([] (CellType) { return true; });
	}

	template <typename CellType, typename FilterFunction>
	void build(const FilterFunction& filter)
	{
		static const Orbit ORBIT = CellType::ORBIT;
		cells_[ORBIT].clear();
		cells_[ORBIT].reserve(4096u);
		map_.foreach_cell([&] (CellType c) { cells_[ORBIT].push_back(c.dart); }, filter);
		current_[ORBIT] = cells_[ORBIT].begin();
	}
};

template <typename MAP>
class BoundaryCache : public CellTraversor
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

#endif // CGOGN_CORE_UTILS_MASKS_H_
