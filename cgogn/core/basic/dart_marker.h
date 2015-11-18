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

	DartMarkerGen()
	{}

	virtual ~DartMarkerGen()
	{}

	DartMarkerGen(const DartMarkerGen& dm) = delete;
	DartMarkerGen(DartMarkerGen&& dm) = delete;
	DartMarkerGen& operator=(DartMarkerGen&& dm) = delete;
	DartMarkerGen& operator=(const DartMarkerGen& dm) = delete;
};

template <typename MAP>
class DartMarkerT : public DartMarkerGen
{
protected:

	MAP& map_;
	ChunkArray<MAP::CHUNK_SIZE, bool>* mark_attribute_;

public:

	DartMarkerT(MAP& map) :
		DartMarkerGen(),
		map_(map)
	{
		mark_attribute_ = map_.template getTopologyMarkAttribute();
	}

	~DartMarkerT() override
	{
		if (MapGen::isAlive(&map_))
			map_.template releaseTopologyMarkAttribute(mark_attribute_);
	}

	DartMarkerT(const DartMarkerT<MAP>& dm) = delete;
	DartMarkerT(DartMarkerT<MAP>&& dm) = delete;
	DartMarkerT<MAP>& operator=(DartMarkerT<MAP>&& dm) = delete;
	DartMarkerT<MAP>& operator=(const DartMarkerT<MAP>& dm) = delete;

	inline void mark(Dart d)
	{
		cgogn_message_assert(mark_attribute_ != nullptr, "DartMarker has null mark attribute");
		mark_attribute_->setTrue(d.index);
	}

	inline void unmark(Dart d)
	{
		cgogn_message_assert(mark_attribute_ != nullptr, "DartMarker has null mark attribute");
		mark_attribute_->setFalse(d.index);
	}

	inline void isMarked(Dart d) const
	{
		cgogn_message_assert(mark_attribute_ != nullptr, "DartMarker has null mark attribute");
		return (*mark_attribute_)[d.index];
	}

	template <unsigned int ORBIT>
	inline void markOrbit(Cell<ORBIT> c)
	{
		cgogn_message_assert(mark_attribute_ != nullptr, "DartMarker has null mark attribute");
		map_.foreach_dart_of_orbit(c, [&] (Dart d)
		{
			mark_attribute_->setTrue(d.index);
		});
	}

	template <unsigned int ORBIT>
	inline void unmarkOrbit(Cell<ORBIT> c)
	{
		cgogn_message_assert(mark_attribute_ != nullptr, "DartMarker has null mark attribute");
		map_.foreach_dart_of_orbit(c, [&] (Dart d)
		{
			mark_attribute_->setFalse(d.index);
		});
	}
};

template <typename MAP>
class DartMarker : public DartMarkerT<MAP>
{
public:

	typedef DartMarkerT<MAP> Inherit;

	DartMarker(MAP& map) :
		Inherit(map)
	{}

	~DartMarker() override
	{
		unmarkAll() ;
	}

	DartMarker(const DartMarker<MAP>& dm) = delete;
	DartMarker(DartMarker<MAP>&& dm) = delete;
	DartMarker<MAP>& operator=(DartMarker<MAP>&& dm) = delete;
	DartMarker<MAP>& operator=(const DartMarker<MAP>& dm) = delete;

	inline void unmarkAll()
	{
		cgogn_message_assert(this->mark_attribute_ != nullptr, "DartMarker has null mark attribute");
		this->mark_attribute_->allFalse();
	}
};

template <typename MAP>
class DartMarkerStore : public DartMarkerT<MAP>
{
protected:

	std::vector<Dart>* marked_darts_;

public:

	typedef DartMarkerT<MAP> Inherit;

	DartMarkerStore(MAP& map) :
		Inherit(map)
	{
		marked_darts_ = dart_buffers_thread->getBuffer();
	}

	~DartMarkerStore() override
	{
		unmarkAll();
		dart_buffers_thread->releaseBuffer(marked_darts_);
	}

	DartMarkerStore(const DartMarkerStore<MAP>& dm) = delete;
	DartMarkerStore(DartMarkerStore<MAP>&& dm) = delete;
	DartMarkerStore<MAP>& operator=(DartMarkerStore<MAP>&& dm) = delete;
	DartMarkerStore<MAP>& operator=(const DartMarkerStore<MAP>& dm) = delete;

	inline void mark(Dart d)
	{
		cgogn_message_assert(this->mark_attribute_ != nullptr, "DartMarker has null mark attribute");
		Inherit::mark(d);
		marked_darts_->push_back(d);
	}

	template <unsigned int ORBIT>
	inline void markOrbit(Cell<ORBIT> c)
	{
		cgogn_message_assert(this->mark_attribute_ != nullptr, "DartMarker has null mark attribute");
		this->map_.foreach_dart_of_orbit(c, [&] (Dart d)
		{
			Inherit::mark(d);
			marked_darts_->push_back(d);
		});
	}

	inline void unmarkAll()
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
