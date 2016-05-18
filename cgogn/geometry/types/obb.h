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

#ifndef CGOGN_GEOMETRY_TYPES_OBB_H_
#define CGOGN_GEOMETRY_TYPES_OBB_H_

#include <type_traits>
#include <array>

#include <cgogn/core/utils/numerics.h>

#include <cgogn/geometry/dll.h>
#include <cgogn/geometry/types/geometry_traits.h>

namespace cgogn
{

namespace geometry
{

/**
 * Object Bounding Box
 */
template <typename VEC_T>
class OBB
{

private:

	bool initialized_;

public:
	/**********************************************/
	/*                CONSTRUCTORS                */
	/**********************************************/

	OBB() :
		initialized_(false)
	{}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_GEOMETRY_TYPES_OBB_CPP_))
extern template class CGOGN_GEOMETRY_API OBB<Eigen::Vector3d>;
extern template class CGOGN_GEOMETRY_API OBB<Eigen::Vector3f>;
extern template class CGOGN_GEOMETRY_API OBB<Vec_T<std::array<float32, 3>>>;
extern template class CGOGN_GEOMETRY_API OBB<Vec_T<std::array<float64,3>>>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_GEOMETRY_TYPES_OBB_CPP_))

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_TYPES_OBB_H_
