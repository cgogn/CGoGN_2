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

#ifndef CORE_BASIC_DART_MARKER_H_
#define CORE_BASIC_DART_MARKER_H_

#include <core/container/chunk_array.h>
#include <core/map/map_base_data.h>

namespace cgogn
{

class DartMarkerGen
{
public:
	typedef DartMarkerGen Super;
	DartMarkerGen()
	{}

	virtual ~DartMarkerGen();

	DartMarkerGen(const Super& dm) = delete;
	DartMarkerGen(Super&& dm) = delete;
	DartMarkerGen& operator=(Super&& dm) = delete;
	DartMarkerGen& operator=(const Super& dm) = delete;
};

template <typename MAP>
class DartMarkerT : public DartMarkerGen
{
public:

	typedef DartMarkerGen Inherit;
	typedef DartMarkerT<MAP> Super;

	typedef MAP Map;
	typedef typename Map::ChunkSizeType ChunkSizeType;
	using ChunkArrayBool = typename Map::template ChunkArray<bool>;
protected:

	Map& map_;
	ChunkArrayBool* mark_attribute_;

public:

	DartMarkerT(Map& map) :
		Inherit(),
		map_(map)
	{
		mark_attribute_ = map_.template get_topology_mark_attribute();
	}

	~DartMarkerT() override
	{
		if (MapGen::is_alive(&map_))
			map_.template release_topology_mark_attribute(mark_attribute_);
	}

	DartMarkerT(const Super& dm) = delete;
	DartMarkerT(Super&& dm) = delete;
	DartMarkerT<MAP>& operator=(Super& dm) = delete;
	DartMarkerT<MAP>& operator=(const Super& dm) = delete;

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

	inline void is_marked(Dart d) const
	{
		cgogn_message_assert(mark_attribute_ != nullptr, "DartMarker has null mark attribute");
		return (*mark_attribute_)[d.index];
	}

	template <unsigned int ORBIT>
	inline void mark_orbit(Cell<ORBIT> c)
	{
		cgogn_message_assert(mark_attribute_ != nullptr, "DartMarker has null mark attribute");
		map_.foreach_dart_of_orbit(c, [&] (Dart d)
		{
			mark_attribute_->set_true(d.index);
		});
	}

	template <unsigned int ORBIT>
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
class DartMarker : public DartMarkerT<MAP>
{
public:

	typedef DartMarker<MAP> Super;
	typedef DartMarkerT<MAP> Inherit;
	typedef MAP Map;

	DartMarker(MAP& map) :
		Inherit(map)
	{}

	~DartMarker() override
	{
		unmark_all() ;
	}

	DartMarker(const Super& dm) = delete;
	DartMarker(Super&& dm) = delete;
	DartMarker<MAP>& operator=(Super&& dm) = delete;
	DartMarker<MAP>& operator=(const Super& dm) = delete;

	inline void unmark_all()
	{
		cgogn_message_assert(this->mark_attribute_ != nullptr, "DartMarker has null mark attribute");
		this->mark_attribute_->all_false();
	}
};

template <typename MAP>
class DartMarkerStore : public DartMarkerT<MAP>
{
public:
	typedef DartMarkerStore<MAP> Super;
	typedef DartMarkerT<MAP> Inherit;
	typedef MAP Map;

protected:

	std::vector<Dart>* marked_darts_;

public:

	DartMarkerStore(Map& map) :
		Inherit(map)
	{
		marked_darts_ = dart_buffers_thread->get_buffer();
	}

	~DartMarkerStore() override
	{
		unmark_all();
		dart_buffers_thread->release_buffer(marked_darts_);
	}

	DartMarkerStore(const Super& dm) = delete;
	DartMarkerStore(Super&& dm) = delete;
	DartMarkerStore<MAP>& operator=(Super&& dm) = delete;
	DartMarkerStore<MAP>& operator=(const Super& dm) = delete;

	inline void mark(Dart d)
	{
		cgogn_message_assert(this->mark_attribute_ != nullptr, "DartMarker has null mark attribute");
		Inherit::mark(d);
		marked_darts_->push_back(d);
	}

	template <unsigned int ORBIT>
	inline void mark_orbit(Cell<ORBIT> c)
	{
		cgogn_message_assert(this->mark_attribute_ != nullptr, "DartMarker has null mark attribute");
		this->map_.foreach_dart_of_orbit(c, [&] (Dart d)
		{
			Inherit::mark(d);
			marked_darts_->push_back(d);
		});
	}

	inline void unmark_all()
	{
		cgogn_message_assert(this->mark_attribute_ != nullptr, "DartMarker has null mark attribute");
		for (Dart d : marked_darts_)
		{
			Inherit::unmark(d);
		}
	}
};

} // namespace cgogn

#endif // CORE_BASIC_DART_MARKER_H_
