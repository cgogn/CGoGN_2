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
#include <array>

#include <cgogn/core/utils/numerics.h>
#include <cgogn/core/basic/cell.h>

namespace cgogn
{

/**
 * @brief The CellFilters class
 * Classes inheriting from CellFilters can be used as a parameter to map.foreach_cell()
 * They can personalize the filtering function used to filter each Orbit traversal
 */
class CGOGN_CORE_API CellFilters
{
public:

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CellFilters);
	inline CellFilters() {}
	virtual ~CellFilters();
	virtual void operator() (uint32) const final;

	template <typename CellType>
	inline bool filter(CellType) const { return true; }
};

/**
 * @brief The CellTraversor class
 * Classes inheriting from CellTraversor can be used as a parameter to map.foreach_cell()
 * They should have a nested class const_iterator (which has to be copy constructible, copy assignable, equality comparable, dereferenceable and incrementable (i.e. the expression ++it is defined))
 * They should provide the following methods :
 *  - template <typename CellType> const_iterator begin() const
 *" - template <typename CellType> const_iterator end() const
 */
class CGOGN_CORE_API CellTraversor
{
public:

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CellTraversor);
	inline CellTraversor() {}
	virtual ~CellTraversor();
	virtual void operator() (uint32) const final;
};

template<typename MAP, typename CellType>
class QuickTraversor : public CellTraversor
{
public:
	using Inherit = CellTraversor;
	using Self = QuickTraversor<MAP,CellType>;
	using QTAttribute = typename MAP::template Attribute<Dart,CellType::ORBIT>;
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(QuickTraversor);
	using iterator = typename QTAttribute::iterator;
	using const_iterator = typename QTAttribute::const_iterator;

	inline QuickTraversor(MAP& map) : Inherit(),
		map_(map),
		qt_attribute_(map.template add_attribute<Dart, CellType>(std::string("qt_att_orb_") + std::to_string(uint32(CellType::ORBIT))+ std::string("_nb_")+ std::to_string(qt_counter_++)))
	{
		cgogn_message_assert(qt_attribute_.is_valid(), "Error : the QuickTraversor attribute is not valid.");
	}

	template <typename OtherCellType>
	inline const_iterator begin() const
	{
		static_assert(std::is_same<CellType, OtherCellType>::value, "begin() called with wrong cell type.");
		return qt_attribute_.begin();
	}

	template <typename OtherCellType>
	inline iterator begin()
	{
		static_assert(std::is_same<CellType, OtherCellType>::value, "begin() called with wrong cell type.");
		return qt_attribute_.begin();
	}

	template <typename OtherCellType>
	inline const_iterator end() const
	{
		static_assert(std::is_same<CellType, OtherCellType>::value, "end() called with wrong cell type.");
		return qt_attribute_.end();
	}

	template <typename OtherCellType>
	inline iterator end()
	{
		static_assert(std::is_same<CellType, OtherCellType>::value, "end() called with wrong cell type.");
		return qt_attribute_.end();
	}

	inline void build()
	{
		this->build([] (CellType) { return true; });
	}

	template <typename FilterFunction>
	inline void build(const FilterFunction& filter)
	{
		map_.foreach_cell([this] (CellType c) { qt_attribute_[c] = c.dart; }, filter);
	}

	inline void update(CellType c)
	{
		qt_attribute_[c] = c.dart;
	}

	virtual ~QuickTraversor() override
	{
		if (qt_attribute_.is_valid())
			map_.remove_attribute(qt_attribute_);
	}
private:
	MAP& map_;
	QTAttribute qt_attribute_;
	static uint32 qt_counter_;
};

template<typename MAP, typename CellType>
uint32 QuickTraversor<MAP,CellType>::qt_counter_ = 0u;

template <typename MAP>
class CellCache : public CellTraversor
{
public:

	using iterator = std::vector<Dart>::iterator;
	using const_iterator = std::vector<Dart>::const_iterator;

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CellCache);
	inline CellCache(const MAP& m) : map_(m)
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
	inline void build()
	{
		this->build<CellType>([] (CellType) { return true; });
	}

	template <typename CellType, typename FilterFunction>
	inline void build(const FilterFunction& filter)
	{
		static const Orbit ORBIT = CellType::ORBIT;
		cells_[ORBIT].clear();
		cells_[ORBIT].reserve(4096u);
		map_.foreach_cell([&] (CellType c) { cells_[ORBIT].push_back(c.dart); }, filter);
	}

private:

	const MAP& map_;
	std::array<std::vector<Dart>, NB_ORBITS> cells_;
};

template <typename MAP>
class BoundaryCache : public CellTraversor
{
public:

	using BoundaryCellType = typename MAP::Boundary;
	using iterator = typename std::vector<BoundaryCellType>::iterator;
	using const_iterator = typename std::vector<BoundaryCellType>::const_iterator;

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(BoundaryCache);
	inline BoundaryCache(const MAP& m) : map_(m)
	{
		cells_.reserve(4096u);
		build();
	}

	template <typename CellType = BoundaryCellType>
	inline const_iterator begin() const
	{
		static_assert(std::is_same<CellType, BoundaryCellType>::value, "BoundaryCache can only be used with BoundaryCellType");
		return cells_.begin();
	}

	template <typename CellType = BoundaryCellType>
	inline iterator begin()
	{
		static_assert(std::is_same<CellType, BoundaryCellType>::value, "BoundaryCache can only be used with BoundaryCellType");
		return cells_.begin();
	}

	template <typename CellType = BoundaryCellType>
	inline const_iterator end() const
	{
		static_assert(std::is_same<CellType, BoundaryCellType>::value, "BoundaryCache can only be used with BoundaryCellType");
		return cells_.end();
	}

	template <typename CellType = BoundaryCellType>
	inline iterator end()
	{
		static_assert(std::is_same<CellType, BoundaryCellType>::value, "BoundaryCache can only be used with BoundaryCellType");
		return cells_.end();
	}

	template <typename CellType = BoundaryCellType>
	inline void build()
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
	}

private:

	const MAP& map_;
	std::vector<BoundaryCellType> cells_;
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_UTILS_MASKS_CPP_))
// forward declarations
template <typename>
class CMap2_T;
template <typename>
class CMap3_T;
struct CMap2Type;
struct CMap3Type;

extern template class CGOGN_CORE_API CellCache<CMap3<CMap3Type>>;
extern template class CGOGN_CORE_API CellCache<CMap2<CMap2Type>>;
extern template class CGOGN_CORE_API BoundaryCache<CMap3<CMap3Type>>;
extern template class CGOGN_CORE_API BoundaryCache<CMap2<CMap2Type>>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_UTILS_MASKS_CPP_))

} // namespace cgogn

#endif // CGOGN_CORE_UTILS_MASKS_H_
