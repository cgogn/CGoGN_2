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

#ifndef GEOMETRY_EIGEN_H_
#define GEOMETRY_EIGEN_H_

#include <core/utils/name_types.h>

CGOGN_PRAGMA_EIGEN_REMOVE_WARNINGS_ON
#include <Eigen/Dense>
CGOGN_PRAGMA_EIGEN_REMOVE_WARNINGS_OFF

namespace cgogn
{

template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline std::string name_of_type(const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& )
{
	using std::string;
	using std::to_string;
	return string("Eigen::Matrix<") + name_of_type(_Scalar()) + string(",") + to_string(_Rows) + string(",") + to_string(_Cols) + string(",") + to_string(_Options) + string(",") + to_string(_MaxRows) + string(",") + to_string(_MaxCols) + string(">");
}

namespace geometry
{

} // namespace geometry
} // namespace cgogn

#endif // GEOMETRY_EIGEN_H_
