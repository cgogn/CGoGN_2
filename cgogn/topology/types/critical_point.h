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

namespace cgogn
{

namespace topology
{

enum CriticalVertexType: unsigned int
{
	REGULAR = 0,
	MAXIMUM = 1,
	MINIMUM = 2,
	SADDLE  = 4,
	UNKNOWN = 8
};

struct CriticalVertex
{
	unsigned int n_;
	CriticalVertexType v_;

	inline CriticalVertex(CriticalVertexType v): v_(v), n_(0)
	{}

	inline CriticalVertex(CriticalVertexType v, unsigned int n) : v_(v), n_(n)
	{}
};

} // namespace topology

} // namespace cgogn

#endif // CGOGN_TOPOLOGY_TYPES_CRITICAL_POINT_H_
