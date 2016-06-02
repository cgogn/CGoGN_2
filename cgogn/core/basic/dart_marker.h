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

#ifndef CGOGN_CORE_BASIC_DART_MARKER_H_
#define CGOGN_CORE_BASIC_DART_MARKER_H_

#include <cgogn/core/utils/buffers.h>

#include <cgogn/core/cmap/map_base_data.h>
#include <cgogn/core/container/chunk_array.h>

namespace cgogn
{

template <typename MAP>
class DartMarker_T
{
public:

	using Self = DartMarker_T<MAP>;
	using Map = MAP;
	using ChunkArrayBool = typename Map::ChunkArrayBool;

protected:

	Map& map_;
	ChunkArrayBool* mark_attribute_;

public:

	DartMarker_T(const MAP& map) :
		map_(const_cast<MAP&>(map))
	{
		mark_attribute_ = map_.topology_mark_attribute();
	}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(DartMarker_T);

	virtual ~DartMarker_T()
	{
		if (MapGen::is_alive(&map_))
			map_.release_topology_mark_attribute(mark_attribute_);
	}

	inline void mark(Dart d)
	{
		cgogn_message_assert(mark_attribute_ != nullptr, "DartMarker has null mark attribute");
		mark_attribute_->set_true(d.index);
	}

	inline void unmark(Dart d)
	{
		cgogn_message_assert(mark_attribute_ != nullptr, "DartMarker has null mark attribute");
		mark_attribute_->set_false(d.index);
	}

	inline bool is_marked(Dart d) const
	{
		cgogn_message_assert(mark_attribute_ != nullptr, "DartMarker has null mark attribute");
		return (*mark_attribute_)[d.index];
	}

	template <Orbit ORBIT>
	inline void mark_orbit(Cell<ORBIT> c)
	{
		cgogn_message_assert(mark_attribute_ != nullptr, "DartMarker has null mark attribute");
		map_.foreach_dart_of_orbit(c, [&] (Dart d)
		{
			mark_attribute_->set_true(d.index);
		});
	}

	template <Orbit ORBIT>
	inline void unmark_orbit(Cell<ORBIT> c)
	{
		cgogn_message_assert(mark_attribute_ != nullptr, "DartMarker has null mark attribute");
		map_.foreach_dart_of_orbit(c, [&] (Dart d)
		{
			mark_attribute_->set_false(d.index);
		});
	}
};

template <typename MAP>
class DartMarker : public DartMarker_T<MAP>
{
public:

	using Inherit = DartMarker_T<MAP>;
	using Self = DartMarker<MAP>;
	using Map = MAP;

	DartMarker(const MAP& map) :
		Inherit(map)
	{}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(DartMarker);

	~DartMarker() override
	{
		unmark_all();
	}

	inline void unmark_all()
	{
		cgogn_message_assert(this->mark_attribute_ != nullptr, "DartMarker has null mark attribute");
		this->mark_attribute_->all_false();
	}
};

template <typename MAP>
class DartMarkerStore : public DartMarker_T<MAP>
{
public:

	using Self = DartMarkerStore<MAP>;
	using Inherit = DartMarker_T<MAP>;
	using Map = MAP;

protected:

	std::vector<Dart>* marked_darts_;

public:

	DartMarkerStore(const MAP& map) :
		Inherit(map)
	{
		marked_darts_ = cgogn::dart_buffers()->buffer();
	}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(DartMarkerStore);

	~DartMarkerStore() override
	{
		unmark_all();
		cgogn::dart_buffers()->release_buffer(marked_darts_);
	}

	inline void mark(Dart d)
	{
		cgogn_message_assert(this->mark_attribute_ != nullptr, "DartMarkerStore has null mark attribute");
		Inherit::mark(d);
		marked_darts_->push_back(d);
	}

	template <Orbit ORBIT>
	inline void mark_orbit(Cell<ORBIT> c)
	{
		cgogn_message_assert(this->mark_attribute_ != nullptr, "DartMarkerStore has null mark attribute");
		this->map_.foreach_dart_of_orbit(c, [&] (Dart d)
		{
			Inherit::mark(d);
			marked_darts_->push_back(d);
		});
	}

	inline void unmark_all()
	{
		cgogn_message_assert(this->mark_attribute_ != nullptr, "DartMarkerStore has null mark attribute");
		for (Dart d : *marked_darts_)
			this->mark_attribute_->set_false_byte(d.index);
		marked_darts_->clear();
	}

	inline const std::vector<Dart>* marked_darts() const
	{
		return marked_darts_;
	}
};

template <typename MAP>
class DartMarkerNoUnmark : public DartMarker_T<MAP>
{
public:

	using Inherit = DartMarker_T<MAP>;
	using Self = DartMarkerNoUnmark<MAP>;
	using Map = MAP;

	DartMarkerNoUnmark(const MAP& map) :
		Inherit(map)
	{}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(DartMarkerNoUnmark);

	~DartMarkerNoUnmark() override
	{}
};

} // namespace cgogn

#endif // CGOGN_CORE_BASIC_DART_MARKER_H_
