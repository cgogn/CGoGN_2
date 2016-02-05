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

#ifndef CORE_CMAP_CMAP_OBSERVER_H_
#define CORE_CMAP_CMAP_OBSERVER_H_

#include <core/container/chunk_array_container.h>
#include <core/basic/cell.h>

namespace cgogn
{

template <typename T>
class ElementValidator
{
public:

	virtual ~ElementValidator() {}
	virtual bool valid(const T&) const = 0;
};

class CMapObserver
{
protected:

	ElementValidator<Dart>* topo_;
	std::array<ElementValidator<unsigned int>*, NB_ORBITS> attr_;

public:

	inline const ElementValidator<Dart>& topo() const
	{
		return *topo_;
	}

	template <Orbit ORBIT>
	inline const ElementValidator<unsigned int>& attr() const
	{
		return *attr_[ORBIT];
	}
};


// All elements observer

template <typename T>
class DefaultElementValidator : public ElementValidator<T>
{
public:

	inline bool valid(const T&) const override
	{
		return true;
	}
};

class DefaultCMapObserver : public CMapObserver
{
public:

	DefaultCMapObserver()
	{
		this->topo_ = new DefaultElementValidator<Dart>();
		for (unsigned int i = Orbit::DART; i < NB_ORBITS; ++i)
			this->attr_[i] = new DefaultElementValidator<unsigned int>();
	}
};

// No boundary observer

template <typename MAP, typename T>
class NoBoundaryValidator : public ElementValidator<T>
{
protected:

	const MAP& map_;

public:

	NoBoundaryValidator(const MAP& map) : map_(map)
	{}

	inline bool valid(const T& element) const override
	{
		return !map_.is_boundary(element);
	}
};

template <typename MAP>
class NoBoundaryCMapObserver : public CMapObserver
{
public:

	NoBoundaryCMapObserver(const MAP& map)
	{
		this->topo_ = new NoBoundaryValidator<MAP, Dart>(map);
		for (unsigned int i = Orbit::DART; i < NB_ORBITS; ++i)
			this->attr_[i] = new DefaultElementValidator<unsigned int>();
	}
};

// Boundary observer

template <typename MAP, typename T>
class BoundaryDartValidator : public ElementValidator<T>
{
protected:

	const MAP& map_;

public:

	BoundaryDartValidator(const MAP& map) : map_(map)
	{}

	inline bool valid(const T& element) const override
	{
		return map_.is_boundary(element);
	}
};

template <typename MAP>
class BoundaryCMapObserver : public CMapObserver
{
public:

	BoundaryCMapObserver(const MAP& map)
	{
		this->topo_ = new BoundaryDartValidator<MAP, Dart>(map);
		for (unsigned int i = Orbit::DART; i < NB_ORBITS; ++i)
			this->attr_[i] = new DefaultElementValidator<unsigned int>();
	}
};

} // namespace cgogn

#endif // CORE_CMAP_CMAP_OBSERVER_H_
