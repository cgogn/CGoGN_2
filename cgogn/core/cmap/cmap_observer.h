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

class DartValidator
{
public:

	virtual ~DartValidator() {}
	virtual bool valid(Dart) const = 0;

protected:

	template <Orbit ORBIT, typename MAP, typename FUNC>
	inline void foreach_dart_of_orbit_until(const MAP& m, Cell<ORBIT> c, const FUNC& f) const
	{
		m.foreach_dart_of_orbit_until(c, f);
	}
};

class CMapObserver
{
protected:

	DartValidator* topo_;
	std::array<DartValidator*, NB_ORBITS> attr_;

public:

	inline const DartValidator& topo() const { return *topo_; }
	template <Orbit ORBIT> inline const DartValidator& attr() const { return *attr_[ORBIT]; }
};


// All elements observer

class CompleteDartValidator : public DartValidator
{
public:

	inline bool valid(Dart) const override { return true; }
};

class CompleteCMapObserver : public CMapObserver
{
public:

	CompleteCMapObserver()
	{
		this->topo_ = new CompleteDartValidator();
		for (unsigned int i = Orbit::DART; i < NB_ORBITS; ++i)
			this->attr_[i] = new CompleteDartValidator();
	}
};

// No boundary observer

template <typename MAP>
class NoBoundaryDartValidator : public DartValidator
{
protected:

	const MAP& map_;

public:

	NoBoundaryDartValidator(const MAP& map) : map_(map) {}
	inline bool valid(Dart d) const override { return !map_.is_boundary(d); }
};

template <typename MAP>
class NoBoundaryCMapObserver : public CMapObserver
{
public:

	NoBoundaryCMapObserver(const MAP& map)
	{
		this->topo_ = new NoBoundaryDartValidator<MAP>(map);
		for (unsigned int orbit = Orbit::DART; orbit < NB_ORBITS; ++orbit)
		{
			if (orbit == MAP::BOUNDARY)
				this->attr_[orbit] = new NoBoundaryDartValidator<MAP>(map);
			else
				this->attr_[orbit] = new CompleteDartValidator();
		}
	}
};

// Boundary observer

template <typename MAP>
class BoundaryDartValidator : public DartValidator
{
protected:

	const MAP& map_;

public:

	BoundaryDartValidator(const MAP& map) : map_(map) {}
	inline bool valid(Dart d) const override { return map_.is_boundary(d); }
};

//template <typename MAP, Orbit ORBIT>
//class BoundaryCellValidator : public DartValidator
//{
//protected:

//	const MAP& map_;

//public:

//	BoundaryCellValidator(const MAP& map) : map_(map) {}
//	inline bool valid(const Dart& d) const override
//	{
//		bool result = false;
//		this->template foreach_dart_of_orbit_until<ORBIT, MAP>(map_, Cell<ORBIT>(d), [&] (Dart dd)
//		{
//			if (map_.is_boundary(dd))
//			{
//				result = true;
//				return false;
//			}
//			return true;
//		});
//		return result;
//	}
//};

template <typename MAP>
class BoundaryCMapObserver : public CMapObserver
{
public:

	BoundaryCMapObserver(const MAP& map)
	{
		this->topo_ = new BoundaryDartValidator<MAP>(map);
		for (unsigned int orbit = Orbit::DART; orbit < NB_ORBITS; ++orbit)
		{
			if (orbit == MAP::BOUNDARY)
				this->attr_[orbit] = new BoundaryDartValidator<MAP>(map);
			else
				this->attr_[orbit] = new CompleteDartValidator();
		}
	}
};

} // namespace cgogn

#endif // CORE_CMAP_CMAP_OBSERVER_H_
