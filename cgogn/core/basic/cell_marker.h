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

	virtual ~CellMarkerGen()
	{}

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
		mark_attribute_ = map_.template getMarkAttribute<ORBIT>();
	}

	~CellMarkerT() override
	{
		if (MapGen::isAlive(&map_))
			map_.template releaseMarkAttribute<ORBIT>(mark_attribute_);
	}

	CellMarkerT(const CellMarkerT<MAP, ORBIT>& dm) = delete;
	CellMarkerT(CellMarkerT<MAP, ORBIT>&& dm) = delete;
	CellMarkerT<MAP, ORBIT>& operator=(CellMarkerT<MAP, ORBIT>&& dm) = delete;
	CellMarkerT<MAP, ORBIT>& operator=(const CellMarkerT<MAP, ORBIT>& dm) = delete;

	inline void mark(Cell<ORBIT> c)
	{
		cgogn_message_assert(mark_attribute_ != nullptr, "CellMarker has null mark attribute");
		mark_attribute_->setTrue(map_.getEmbedding(c));
	}

	inline void unmark(Cell<ORBIT> c)
	{
		cgogn_message_assert(mark_attribute_ != nullptr, "CellMarker has null mark attribute");
		mark_attribute_->setFalse(map_.getEmbedding(c));
	}

	inline void isMarked(Cell<ORBIT> c) const
	{
		cgogn_message_assert(mark_attribute_ != nullptr, "CellMarker has null mark attribute");
		return (*mark_attribute_)[map_.getEmbedding(c)];
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

	~CellMarker() override
	{
		unmarkAll() ;
	}

	CellMarker(const CellMarker<MAP, ORBIT>& dm) = delete;
	CellMarker(CellMarker<MAP, ORBIT>&& dm) = delete;
	CellMarker<MAP, ORBIT>& operator=(CellMarker<MAP, ORBIT>&& dm) = delete;
	CellMarker<MAP, ORBIT>& operator=(const CellMarker<MAP, ORBIT>& dm) = delete;

	inline void unmarkAll()
	{
		cgogn_message_assert(this->mark_attribute_ != nullptr, "CellMarker has null mark attribute");
		this->mark_attribute_->allFalse();
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
		marked_cells_ = uint_buffers_thread->getBuffer();
	}

	~CellMarkerStore() override
	{
		unmarkAll();
		uint_buffers_thread->releaseBuffer(marked_cells_);
	}

	CellMarkerStore(const CellMarkerStore<MAP, ORBIT>& dm) = delete;
	CellMarkerStore(CellMarkerStore<MAP, ORBIT>&& dm) = delete;
	CellMarkerStore<MAP, ORBIT>& operator=(CellMarkerStore<MAP, ORBIT>&& dm) = delete;
	CellMarkerStore<MAP, ORBIT>& operator=(const CellMarkerStore<MAP, ORBIT>& dm) = delete;

	inline void mark(Cell<ORBIT> c)
	{
		cgogn_message_assert(this->mark_attribute_ != nullptr, "CellMarker has null mark attribute");
		Inherit::mark(c);
		marked_cells_->push_back(this->map_.getEmbedding(c));
	}

	inline void unmarkAll()
	{
		cgogn_message_assert(this->mark_attribute_ != nullptr, "CellMarker has null mark attribute");
		for (unsigned int i : marked_cells_)
		{
			this->mark_attribute_->setFalse(i);
		}
	}
};

} // namespace cgogn

#endif // CORE_BASIC_CELL_MARKER_H_
