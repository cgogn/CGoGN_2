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

class StandardElementValidator : public ContainerElementValidator
{
public:

	inline bool valid(unsigned int index) const override
	{
		return true;
	}
};



class CMapObserver
{
protected:

	ContainerElementValidator* topo_;
	std::array<ContainerElementValidator*, NB_ORBITS> attr_;

public:

	inline const ContainerElementValidator& topo() const
	{
		return *topo_;
	}

	template <Orbit ORBIT>
	inline const ContainerElementValidator& attr() const
	{
		return *attr_[ORBIT];
	}
};

class StandardCMapObserver : public CMapObserver
{
public:

	StandardCMapObserver()
	{
		this->topo_ = new StandardElementValidator();
		for (unsigned int i = Orbit::DART; i < NB_ORBITS; ++i)
		{
			this->attr_[i] = new StandardElementValidator();
		}
	}
};

} // namespace cgogn

#endif // CORE_CMAP_CMAP_OBSERVER_H_
