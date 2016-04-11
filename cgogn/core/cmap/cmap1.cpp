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
#define CGOGN_CORE_MAP_MAP1_CPP_

#include <cgogn/core/cmap/cmap1.h>

namespace cgogn
{

	template class CGOGN_CORE_API CMap1_T<DefaultMapTraits, CMap1Type<DefaultMapTraits>>;
	template class CGOGN_CORE_API DartMarker<CMap1<DefaultMapTraits>>;
	template class CGOGN_CORE_API DartMarkerStore<CMap1<DefaultMapTraits>>;
	template class CGOGN_CORE_API DartMarkerNoUnmark<CMap1<DefaultMapTraits>>;
	template class CGOGN_CORE_API CellMarker<CMap1<DefaultMapTraits>, CMap1<DefaultMapTraits>::Vertex::ORBIT>;
	template class CGOGN_CORE_API CellMarker<CMap1<DefaultMapTraits>, CMap1<DefaultMapTraits>::Face::ORBIT>;
	template class CGOGN_CORE_API CellMarkerStore<CMap1<DefaultMapTraits>, CMap1<DefaultMapTraits>::Vertex::ORBIT>;
	template class CGOGN_CORE_API CellMarkerStore<CMap1<DefaultMapTraits>, CMap1<DefaultMapTraits>::Face::ORBIT>;

} // namespace cgogn
