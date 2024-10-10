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
#define CGOGN_TOPOLOGY_EXTERNAL_TEMPLATES_CPP_

#include <cgogn/topology/algos/distance_field.h>
#include <cgogn/topology/algos/features.h>
#include <cgogn/topology/algos/scalar_field.h>

namespace cgogn
{

namespace topology
{

template class CGOGN_TOPOLOGY_EXPORT DistanceField<float32, CMap2>;
template class CGOGN_TOPOLOGY_EXPORT DistanceField<float64, CMap2>;
template class CGOGN_TOPOLOGY_EXPORT DistanceField<float32, CMap3>;
template class CGOGN_TOPOLOGY_EXPORT DistanceField<float64, CMap3>;

template class CGOGN_TOPOLOGY_EXPORT FeaturesFinder<float32, CMap2>;
template class CGOGN_TOPOLOGY_EXPORT FeaturesFinder<float64, CMap2>;
template class CGOGN_TOPOLOGY_EXPORT FeaturesFinder<float32, CMap3>;
template class CGOGN_TOPOLOGY_EXPORT FeaturesFinder<float64, CMap3>;

template class CGOGN_TOPOLOGY_EXPORT ScalarField<float32, CMap2>;
template class CGOGN_TOPOLOGY_EXPORT ScalarField<float64, CMap2>;
template class CGOGN_TOPOLOGY_EXPORT ScalarField<float32, CMap3>;
template class CGOGN_TOPOLOGY_EXPORT ScalarField<float64, CMap3>;

template class CGOGN_TOPOLOGY_EXPORT AdjacencyCache<CMap2>;
template class CGOGN_TOPOLOGY_EXPORT AdjacencyCache<CMap3>;


} // namespace topology

} // namespace cgogn

