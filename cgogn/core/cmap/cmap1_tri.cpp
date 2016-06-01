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

#define CGOGN_CORE_DLL_EXPORT
#define CGOGN_CORE_MAP_MAP1_TRI_CPP_

#include <cgogn/core/cmap/cmap1_tri.h>

namespace cgogn
{

	template class CGOGN_CORE_API CMap1Tri_T<DefaultMapTraits, CMap1TriType<DefaultMapTraits>>;
	template class CGOGN_CORE_API DartMarker<CMap1Tri<DefaultMapTraits>>;
	template class CGOGN_CORE_API DartMarkerStore<CMap1Tri<DefaultMapTraits>>;
	template class CGOGN_CORE_API DartMarkerNoUnmark<CMap1Tri<DefaultMapTraits>>;
	template class CGOGN_CORE_API CellMarker<CMap1Tri<DefaultMapTraits>, CMap1Tri<DefaultMapTraits>::Vertex::ORBIT>;
	template class CGOGN_CORE_API CellMarker<CMap1Tri<DefaultMapTraits>, CMap1Tri<DefaultMapTraits>::Face::ORBIT>;
	template class CGOGN_CORE_API CellMarkerStore<CMap1Tri<DefaultMapTraits>, CMap1Tri<DefaultMapTraits>::Vertex::ORBIT>;
	template class CGOGN_CORE_API CellMarkerStore<CMap1Tri<DefaultMapTraits>, CMap1Tri<DefaultMapTraits>::Face::ORBIT>;

} // namespace cgogn
