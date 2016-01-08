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

#ifndef IO_MAP_IMPORT_H_
#define IO_MAP_IMPORT_H_

#include <string>

#include <core/map/cmap2.h>
#include <io/surface_import.h>

namespace cgogn
{

namespace io
{

template<class MAP_TRAITS>
inline void import_surface(cgogn::CMap2<MAP_TRAITS>& cmap2, const std::string& filename);

template<class MAP_TRAITS>
inline void import_surface(cgogn::CMap2<MAP_TRAITS>& cmap2, const std::string& filename)
{
	using SurfaceImport = SurfaceImport<MAP_TRAITS>;
	SurfaceImport si;
	si.import_file(filename);
	si.create_map(cmap2);
}

} // namespace io

} // namespace cgogn

#endif // IO_MAP_IMPORT_H_
