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

#ifndef CORE_BASIC_CELL_MARKER_H_
#define CORE_BASIC_CELL_MARKER_H_

#include <core/container/chunk_array.h>
#include <core/map/map_base_data.h>

namespace cgogn
{

class CellMarkerGen
{
public:

	CellMarkerGen()
	{}

	virtual ~CellMarkerGen();

	CellMarkerGen(const CellMarkerGen& dm) = delete;
	CellMarkerGen(CellMarkerGen&& dm) = delete;
	CellMarkerGen& operator=(CellMarkerGen&& dm) = delete;
	CellMarkerGen& operator=(const CellMarkerGen& dm) = delete;
};

template <typename MAP, unsigned int ORBIT>
class CellMarkerT : public CellMarkerGen
{
protected:

	MAP& map_;
	ChunkArray<MAP::CHUNK_SIZE, bool>* mark_attribute_;

public:

	CellMarkerT(MAP& map) :
		CellMarkerGen(),
		map_(map)
	{
		mark_attribute_ = map_.template get_mark_attribute<ORBIT>();
	}

	CellMarkerT(const MAP& map) :
		CellMarkerGen(),
		map_(const_cast<MAP&>(map))
	{
		mark_attribute_ = map_.template get_mark_attribute<ORBIT>();
	}

	~CellMarkerT() override
	{
		if (MapGen::is_alive(&map_))
			map_.template release_mark_attribute<ORBIT>(mark_attribute_);
	}

	CellMarkerT(const CellMarkerT<MAP, ORBIT>& dm) = delete;
	CellMarkerT(CellMarkerT<MAP, ORBIT>&& dm) = delete;
	CellMarkerT<MAP, ORBIT>& operator=(CellMarkerT<MAP, ORBIT>&& dm) = delete;
	CellMarkerT<MAP, ORBIT>& operator=(const CellMarkerT<MAP, ORBIT>& dm) = delete;

	inline void mark(Cell<ORBIT> c)
	{
		cgogn_message_assert(mark_attribute_ != nullptr, "CellMarker has null mark attribute");
		mark_attribute_->set_true(map_.get_embedding(c));
	}

	inline void unmark(Cell<ORBIT> c)
	{
		cgogn_message_assert(mark_attribute_ != nullptr, "CellMarker has null mark attribute");
		mark_attribute_->set_false(map_.get_embedding(c));
	}

	inline bool is_marked(Cell<ORBIT> c) const
	{
		cgogn_message_assert(mark_attribute_ != nullptr, "CellMarker has null mark attribute");
		return (*mark_attribute_)[map_.get_embedding(c)];
	}
};

template <typename MAP, unsigned int ORBIT>
class CellMarker : public CellMarkerT<MAP, ORBIT>
{
public:

	typedef CellMarkerT<MAP, ORBIT> Inherit;

	CellMarker(MAP& map) :
		Inherit(map)
	{}

	CellMarker(const MAP& map) :
		Inherit(map)
	{}

	~CellMarker() override
	{
		unmark_all() ;
	}

	CellMarker(const CellMarker<MAP, ORBIT>& dm) = delete;
	CellMarker(CellMarker<MAP, ORBIT>&& dm) = delete;
	CellMarker<MAP, ORBIT>& operator=(CellMarker<MAP, ORBIT>&& dm) = delete;
	CellMarker<MAP, ORBIT>& operator=(const CellMarker<MAP, ORBIT>& dm) = delete;

	inline void unmark_all()
	{
		cgogn_message_assert(this->mark_attribute_ != nullptr, "CellMarker has null mark attribute");
		this->mark_attribute_->all_false();
	}
};

template <typename MAP, unsigned int ORBIT>
class CellMarkerStore : public CellMarkerT<MAP, ORBIT>
{
protected:

	std::vector<unsigned int>* marked_cells_;

public:

	typedef CellMarkerT<MAP, ORBIT> Inherit;

	CellMarkerStore(MAP& map) :
		Inherit(map)
	{
		marked_cells_ = uint_buffers_thread->get_buffer();
	}

	CellMarkerStore(const MAP& map) :
		Inherit(map)
	{
		marked_cells_ = uint_buffers_thread->get_buffer();
	}

	~CellMarkerStore() override
	{
		unmark_all();
		uint_buffers_thread->release_buffer(marked_cells_);
	}

	CellMarkerStore(const CellMarkerStore<MAP, ORBIT>& dm) = delete;
	CellMarkerStore(CellMarkerStore<MAP, ORBIT>&& dm) = delete;
	CellMarkerStore<MAP, ORBIT>& operator=(CellMarkerStore<MAP, ORBIT>&& dm) = delete;
	CellMarkerStore<MAP, ORBIT>& operator=(const CellMarkerStore<MAP, ORBIT>& dm) = delete;

	inline void mark(Cell<ORBIT> c)
	{
		cgogn_message_assert(this->mark_attribute_ != nullptr, "CellMarker has null mark attribute");
		Inherit::mark(c);
		marked_cells_->push_back(this->map_.get_embedding(c));
	}

	inline void unmark_all()
	{
		cgogn_message_assert(this->mark_attribute_ != nullptr, "CellMarker has null mark attribute");
		for (unsigned int i : marked_cells_)
		{
			this->mark_attribute_->set_false(i);
		}
		marked_cells_->clear();
	}
};

} // namespace cgogn

#endif // CORE_BASIC_CELL_MARKER_H_
