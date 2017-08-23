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
#include <cgogn/core/cmap/attribute.h>

namespace cgogn
{

/**
 * @brief The CellFilters class
 * Classes inheriting from CellFilters can be used as a parameter to map.foreach_cell()
 * They can personalize the filtering function used to filter each Orbit traversal
 * The filtered_cell method must return a combination of the orbit_masks of the handled orbits
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

	virtual uint32 filtered_cells() const = 0;
};

class CGOGN_CORE_API AllCellsFilter : public CellFilters
{
public:

	inline uint32 filtered_cells() const
	{
		return ALL_CELLS_MASK;
//		uint32 result = 0u;
//		for (Orbit o : { DART, PHI1, PHI2, PHI21, PHI1_PHI2, PHI1_PHI3, PHI2_PHI3, PHI21_PHI31, PHI1_PHI2_PHI3 })
//			result |= orbit_mask(o);
//		return result;
	}
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

	inline CellTraversor() : traversed_cells_(0u) {}
	virtual ~CellTraversor();
	virtual void operator() (uint32) const final;

	template <typename CellType>
	inline bool is_traversed() const
	{
		return traversed_cells_ & orbit_mask<CellType>();
	}

protected:

	uint32 traversed_cells_;
};

template <typename MAP>
class QuickTraversor : public CellTraversor
{
public:

	using Inherit = CellTraversor;
	using Self = QuickTraversor<MAP>;

	using QTAttribute = Attribute_T<Dart>;
	using iterator = typename QTAttribute::iterator;
	using const_iterator = typename QTAttribute::const_iterator;

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(QuickTraversor);

	inline QuickTraversor(MAP& map) : Inherit(),
		map_(map)
	{}

	virtual ~QuickTraversor() override
	{
		for (auto& qta : qt_attributes_)
		{
			if (qta.is_valid())
				map_.remove_attribute(qta);
		}
	}

	template <typename CellType>
	inline const_iterator begin() const
	{
		static const Orbit ORBIT = CellType::ORBIT;
		return qt_attributes_[ORBIT].begin();
	}

	template <typename CellType>
	inline iterator begin()
	{
		static const Orbit ORBIT = CellType::ORBIT;
		return qt_attributes_[ORBIT].begin();
	}

	template <typename CellType>
	inline const_iterator end() const
	{
		static const Orbit ORBIT = CellType::ORBIT;
		return qt_attributes_[ORBIT].end();
	}

	template <typename CellType>
	inline iterator end()
	{
		static const Orbit ORBIT = CellType::ORBIT;
		return qt_attributes_[ORBIT].end();
	}

    template <typename CellType>
    inline CellType cell_from_index(uint32 index)
    {
        static const Orbit ORBIT = CellType::ORBIT;
        cgogn_message_assert(is_traversed<CellType>(), "Try to get a cell on a QuickTraversor that has not been built");
        return CellType(qt_attributes_[ORBIT][index]);
    }

	template <typename CellType, typename DartSelectionFunction>
	inline void build(const DartSelectionFunction& dart_select)
	{
		static_assert(is_func_return_same<DartSelectionFunction, Dart>::value && is_func_parameter_same<DartSelectionFunction, CellType>::value, "Badly formed DartSelectionFunction");
		static const Orbit ORBIT = CellType::ORBIT;
		if (!qt_attributes_[ORBIT].is_valid())
			qt_attributes_[ORBIT] = map_.template add_attribute<Dart, ORBIT>(std::string("qt_att_nb_") + std::to_string(qt_counter_++));
		map_.foreach_cell([&] (CellType c) { qt_attributes_[ORBIT][c.dart] = dart_select(c); });
		traversed_cells_ |= orbit_mask<CellType>();
	}

	template <typename CellType>
	inline void build()
	{
		build<CellType>([] (CellType c) -> Dart { return c.dart; });
	}

	template <typename CellType, typename DartSelectionFunction>
	inline void update(CellType c, const DartSelectionFunction& dart_select)
	{
		static_assert(is_func_return_same<DartSelectionFunction, Dart>::value && is_func_parameter_same<DartSelectionFunction, CellType>::value, "Badly formed DartSelectionFunction");
		static const Orbit ORBIT = CellType::ORBIT;
        cgogn_message_assert(is_traversed<CellType>(), "Try to update a cell on a QuickTraversor that has not been built");
		qt_attributes_[ORBIT][c.dart] = dart_select(c);
	}

	template <typename CellType>
	inline void update(CellType c)
	{
		update(c, [] (CellType c) -> Dart { return c.dart; });
	}

private:

	MAP& map_;
	std::array<QTAttribute, NB_ORBITS> qt_attributes_;
	static uint32 qt_counter_;
};

template <typename MAP>
uint32 QuickTraversor<MAP>::qt_counter_ = 0u;

template <typename MAP>
class CellCache : public CellTraversor
{
public:

	using Inherit = CellTraversor;
	using Self = CellCache<MAP>;

	using iterator = std::vector<Dart>::iterator;
	using const_iterator = std::vector<Dart>::const_iterator;

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CellCache);

	inline CellCache(const MAP& m) : Inherit(),
		map_(m)
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

	template <typename CellType, typename MASK>
	inline void build(const MASK& mask)
	{
		static const Orbit ORBIT = CellType::ORBIT;
		cells_[ORBIT].clear();
		cells_[ORBIT].reserve(4096u);
		map_.foreach_cell([&] (CellType c) { cells_[ORBIT].push_back(c.dart); }, mask);
		traversed_cells_ |= orbit_mask<CellType>();
	}

private:

	const MAP& map_;
	std::array<std::vector<Dart>, NB_ORBITS> cells_;
};

template <typename MAP>
class BoundaryCache : public CellTraversor
{
public:

	using Inherit = CellTraversor;
	using Self = BoundaryCache<MAP>;

	using BoundaryCellType = typename MAP::Boundary;
	using iterator = typename std::vector<BoundaryCellType>::iterator;
	using const_iterator = typename std::vector<BoundaryCellType>::const_iterator;

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(BoundaryCache);

	inline BoundaryCache(const MAP& m) : Inherit(),
		map_(m)
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
		traversed_cells_ |= orbit_mask<CellType>();
	}

private:

	const MAP& map_;
	std::vector<BoundaryCellType> cells_;
};

} // namespace cgogn

#endif // CGOGN_CORE_UTILS_MASKS_H_
