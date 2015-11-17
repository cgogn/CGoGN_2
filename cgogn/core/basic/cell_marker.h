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

	inline void mark(Dart d)
	{
		cgogn_message_assert(mark_attribute_ != nullptr, "CellMarker has null mark attribute");
		mark_attribute_->setTrue(d.index);
	}

	inline void unmark(Dart d)
	{
		cgogn_message_assert(mark_attribute_ != nullptr, "CellMarker has null mark attribute");
		mark_attribute_->setFalse(d.index);
	}

	inline void isMarked(Dart d) const
	{
		cgogn_message_assert(mark_attribute_ != nullptr, "CellMarker has null mark attribute");
		return (*mark_attribute_)[d.index];
	}
};

template <typename MAP, unsigned int ORBIT>
class CellMarker : public CellMarkerT<MAP, ORBIT>
{
public:

	CellMarker(MAP& map) :
		CellMarkerT<MAP, ORBIT>(map)
	{}

	~CellMarker() override
	{
		unmarkAll() ;
	}

	CellMarker(const CellMarker<MAP, ORBIT>& dm) = delete;
	CellMarker(CellMarker<MAP, ORBIT>&& dm) = delete;
	CellMarker<MAP, ORBIT>& operator=(CellMarker<MAP, ORBIT>&& dm) = delete;
	CellMarker<MAP, ORBIT>& operator=(const CellMarker<MAP, ORBIT>& dm) = delete;

	void unmarkAll()
	{
		cgogn_message_assert(this->mark_attribute_ != nullptr, "CellMarker has null mark attribute");
		this->mark_attribute_->allFalse();
	}
};

} // namespace cgogn

#endif // CORE_BASIC_CELL_MARKER_H_
