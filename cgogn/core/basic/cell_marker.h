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

#ifndef CGOGN_CORE_BASIC_CELL_MARKER_H_
#define CGOGN_CORE_BASIC_CELL_MARKER_H_

#include <cgogn/core/container/chunk_array.h>
#include <cgogn/core/cmap/map_base_data.h>
#include <type_traits>

namespace cgogn
{

template <typename MAP, Orbit ORBIT>
class CellMarker_T
{
	static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");

public:

	using Self = CellMarker_T<MAP, ORBIT>;
	using Map = MAP;
	using ChunkArrayGen = typename Map::ChunkArrayGen;
	using ChunkArrayBool = typename Map::ChunkArrayBool;

protected:

	MAP& map_;
	ChunkArrayBool* mark_attribute_;

public:

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CellMarker_T);

	CellMarker_T(const MAP& map) :
		map_(const_cast<MAP&>(map))
	{
		mark_attribute_ = map_.template mark_attribute<ORBIT>();
		mark_attribute_->add_external_ref(reinterpret_cast<ChunkArrayGen**>(&mark_attribute_));
	}

	virtual ~CellMarker_T()
	{
		if (is_valid())
		{
			mark_attribute_->remove_external_ref(reinterpret_cast<ChunkArrayGen**>(&mark_attribute_));
			map_.template release_mark_attribute<ORBIT>(mark_attribute_);
		}
	}

	inline void mark(Cell<ORBIT> c)
	{
		cgogn_message_assert(is_valid(), "Invalid CellMarker");
		mark_attribute_->set_true(map_.embedding(c));
	}

	inline void unmark(Cell<ORBIT> c)
	{
		cgogn_message_assert(is_valid(), "Invalid CellMarker");
		mark_attribute_->set_false(map_.embedding(c));
	}

	inline bool is_marked(Cell<ORBIT> c) const
	{
		cgogn_message_assert(is_valid(), "Invalid CellMarker");
		return (*mark_attribute_)[map_.embedding(c)];
	}

	inline bool is_valid() const
	{
		// due to external ref registration and to the fact that mark attributes are only deleted
		// at map destruction, the only way for the marker pointer to be null is that the map
		// has been destroyed. The second (less efficient) test is thus unnecessary, i.e. if
		// the pointer is non-null then the map is still alive, and conversely
		return mark_attribute_ != nullptr; // && MapBaseData::is_alive(&map_);
	}
};

template <typename MAP, Orbit ORBIT>
class CellMarker : public CellMarker_T<MAP, ORBIT>
{
public:

	using Inherit = CellMarker_T<MAP, ORBIT>;
	using Self = CellMarker<MAP, ORBIT>;
	using Map = typename Inherit::Map;

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CellMarker);

	inline CellMarker(const MAP& map) :
		Inherit(map)
	{}

	~CellMarker() override
	{
		if (this->is_valid())
			unmark_all();
	}

	inline void unmark_all()
	{
		cgogn_message_assert(this->is_valid(), "Invalid CellMarker");
		this->mark_attribute_->all_false();
	}
};

template <typename MAP, Orbit ORBIT>
class CellMarkerStore : public CellMarker_T<MAP, ORBIT>
{
public:

	using Inherit = CellMarker_T<MAP, ORBIT>;
	using Self = CellMarkerStore<MAP, ORBIT>;
	using Map = typename Inherit::Map;

protected:

	std::vector<uint32>* marked_cells_;

public:

	inline CellMarkerStore(const MAP& map) :
		Inherit(map)
	{
		marked_cells_ = cgogn::uint_buffers()->buffer();
	}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CellMarkerStore);

	~CellMarkerStore() override
	{
		if (this->is_valid())
			unmark_all();
		auto* ub = cgogn::uint_buffers();
		if (ub)
			ub->release_buffer(marked_cells_);
	}

	inline void mark(Cell<ORBIT> c)
	{
		cgogn_message_assert(this->is_valid(), "Invalid CellMarkerStore");
		if (!this->is_marked(c))
		{
			Inherit::mark(c);
			marked_cells_->push_back(this->map_.embedding(c));
		}
	}

	inline void unmark(Cell<ORBIT> c)
	{
		cgogn_message_assert(this->is_valid(), "Invalid CellMarkerStore");
		auto it = std::find(marked_cells_->begin(), marked_cells_->end(), this->map_.embedding(c));
		if (it !=  marked_cells_->end())
		{
			Inherit::unmark(c);
			std::swap(*it, marked_cells_->back());
			marked_cells_->pop_back();
		}
	}

	inline void unmark_all()
	{
		cgogn_message_assert(this->is_valid(), "Invalid CellMarkerStore");
		for (uint32 i : *(this->marked_cells_))
			this->mark_attribute_->set_false(i);
		marked_cells_->clear();
	}

	inline const std::vector<uint32>& marked_cells() const
	{
		return *marked_cells_;
	}
};

template <typename MAP, Orbit ORBIT>
class CellMarkerNoUnmark : public CellMarker_T<MAP, ORBIT>
{
public:

	using Inherit = CellMarker_T<MAP, ORBIT>;
	using Self = CellMarker< MAP, ORBIT >;
	using Map = typename Inherit::Map;

	inline CellMarkerNoUnmark(const MAP& map) :
		Inherit(map)
	{}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CellMarkerNoUnmark);

	~CellMarkerNoUnmark() override
	{}
};

} // namespace cgogn

#endif // CGOGN_CORE_BASIC_CELL_MARKER_H_
