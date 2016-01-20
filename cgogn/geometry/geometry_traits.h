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

#ifndef GEOMETRY_TRAITS_H_
#define GEOMETRY_TRAITS_H_

#include <array>
#include <core/utils/definitions.h>
CGOGN_PRAGMA_EIGEN_REMOVE_WARNINGS_ON
#include <Eigen/Core>
CGOGN_PRAGMA_EIGEN_REMOVE_WARNINGS_OFF

namespace cgogn
{

namespace geometry
{

template<typename Vec_T>
struct vector_traits
{
};


template<>
template<typename Scalar_, std::size_t Size>
struct vector_traits<std::array<Scalar_,Size>>
{
	static const std::size_t SIZE = Size;
	using Scalar = Scalar_;
};



template<>
template<typename Scalar_, int Rows, int Options>
struct vector_traits<Eigen::Matrix<Scalar_,Rows,1,Options,Rows,1>>
{
	static const std::size_t SIZE = Rows;
	using Scalar = Scalar_;
};


} // namespace geometry
} // namespace cgogn

#endif // GEOMETRY_TRAITS_H_
