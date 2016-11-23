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

#ifndef CGOGN_TOPOLOGY_TYPES_CRITICAL_POINT_H_
#define CGOGN_TOPOLOGY_TYPES_CRITICAL_POINT_H_

#include <cgogn/core/utils/numerics.h>
#include <iostream>
namespace cgogn
{

namespace topology
{


struct CriticalPoint
{
	enum Type : uint32
	{
		REGULAR = 0,
		MAXIMUM,
		MINIMUM,
		SADDLE,		// SADDLE for 2-manifold
		SADDLE1,	// 1-SADDLE for 3-manifold
		SADDLE2,	// 2-SADDLE for 3-manifold
		UNKNOWN
	};

	CriticalPoint::Type v_;
	uint32 n_;

	inline CriticalPoint(CriticalPoint::Type v): v_(v), n_(0)
	{}

	inline CriticalPoint(CriticalPoint::Type v, uint32 n) : v_(v), n_(n)
	{}
};

inline std::ostream& operator<<(std::ostream& o, CriticalPoint cp)
{
	o << cp.n_ << ' ' << uint32(cp.v_);
	return o;
}

inline std::istream& operator>>(std::istream& is, CriticalPoint cp)
{
	is >> cp.n_;
	uint32 crit_point_type;
	is >> crit_point_type;
	cp.v_ = CriticalPoint::Type(crit_point_type);
	return is;
}

} // namespace topology

} // namespace cgogn

#endif // CGOGN_TOPOLOGY_TYPES_CRITICAL_POINT_H_
