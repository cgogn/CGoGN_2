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

#ifndef CORE_IO_MAP_IMPORT_H_
#define CORE_IO_MAP_IMPORT_H_

#include <string>
#include <core/map/cmap2.h>
#include <io/surface_import.h>

namespace cgogn
{
namespace import
{
template<class DATA_TRAITS, class TOPO_TRAITS>
inline void importSurface(cgogn::CMap2_T<DATA_TRAITS,TOPO_TRAITS>& cmap2, const std::string& filename);

template<class DATA_TRAITS, class TOPO_TRAITS>
inline void importSurface(cgogn::CMap2_T<DATA_TRAITS,TOPO_TRAITS>& cmap2, const std::string& filename)
{
    using CMap2 = cgogn::CMap2_T<DATA_TRAITS,TOPO_TRAITS>;
    using SurfaceImport = cgogn::SurfaceImport<CMap2>;
    SurfaceImport si;
    si.import_file(filename);
    si.createMap(cmap2);
}
}
}
#endif // CORE_IO_MAP_IMPORT_H_
