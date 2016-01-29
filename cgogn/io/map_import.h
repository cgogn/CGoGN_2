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

#include <core/cmap/cmap2.h>
#include <core/cmap/cmap3.h>
#include <io/surface_import.h>
#include <io/volume_import.h>

namespace cgogn
{

namespace io
{

template <typename VEC3, class MAP_TRAITS>
inline void import_surface(cgogn::CMap2<MAP_TRAITS>& cmap2, const std::string& filename);

template <typename VEC3, class MAP_TRAITS>
inline void import_volume(cgogn::CMap3<MAP_TRAITS>& cmap3, const std::string& filename);




template <typename VEC3, class MAP_TRAITS>
inline void import_surface(cgogn::CMap2<MAP_TRAITS>& cmap2, const std::string& filename)
{
	SurfaceImport<MAP_TRAITS> si;
	si.template import_file<VEC3>(filename);
	si.create_map(cmap2);
}

template <typename VEC3, class MAP_TRAITS>
inline void import_volume(cgogn::CMap3<MAP_TRAITS>& cmap3, const std::string& filename)
{
	VolumeImport<MAP_TRAITS> vi;
	vi.template import_file<VEC3>(filename);
	vi.create_map(cmap3);
}

} // namespace io

} // namespace cgogn

#endif // IO_MAP_IMPORT_H_
