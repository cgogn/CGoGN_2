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

#ifndef CORE_CPH_IHCMAP_MODIFIER_H_
#define CORE_CPH_IHCMAP_MODIFIER_H_

#include <core/cph/ihcmap2.h>

namespace cgogn
{

template <typename MAP_TRAITS>
class IHCMap2Modifier_T
{
public:

	using Self = IHCMap2Modifier_T<MAP_TRAITS>;
	using IHCMap2 = cgogn::IHCMap2<MAP_TRAITS>;

	template<typename T>
	using ChunkArrayContainer = typename IHCMap2::template ChunkArrayContainer<T>;

	inline IHCMap2Modifier_T(IHCMap2& ihmap): ihmap_(ihmap)
	{}
	IHCMap2Modifier_T(const Self&) = delete;
	IHCMap2Modifier_T(Self&&) = delete;
	Self& operator=(const Self&) = delete;
	Self& operator=(Self&&) = delete;
	inline ~IHCMap2Modifier_T() = default;

public:

	inline Vertex cut_edge(Edge d)
	{
		Vertex v = ihmap_.cut_edge(e);
		
		if(ihmap_.is_orbit_embedded<Orbit::DART>())
		{
			ihmap_.init_orbit_embedding<Orbit::DART>(v, ihmap_.add_attribute_element<Orbit::DART>());
			ihmap_.init_orbit_embedding<Orbit::DART>(ihmap_.phi1(ihmap_.phi2(v)), ihmap_.add_attribute_element<Orbit::DART>());
		}	

		if(ihmap_.is_orbit_embedded<Orbit::PHI21>())
		{
			ihmap_.init_orbit_embedding<Orbit::PHI21>(v, ihmap_.add_attribute_element<Orbit::PHI21>());
		}

		if(ihmap_.is_orbit_embedded<Orbit::PHI2>())
		{
			ihmap_.init_orbit_embedding<Orbit::PHI2>(ihmap_.phi2(d), ihmap_.get_embedding<Orbit::PHI2>(d)) ;
			set_orbit_embedding<EDGE>();
			
			set_orbit_embeddingOnNewCell<EDGE>(nd, this->template getEmbedding<Orbit::DART>()) ;
			Algo::Topo::copyCellAttributes<EDGE>(*this, nd, d) ;
		}

		if(ihmap_.is_orbit_embedded<Orbit::PHI1>())
		{
			ihmap_.init_embedding<Orbit::PHI1>(v, ihmap_.get_embedding<Orbit::PHI1>(d)) ;
			Dart e = ihmap_.phi2(v) ;
			ihmap_.init_embedding<Orbit::PHI1>(ihmap_.phi1(e), ihmap_.getEmbedding<Orbit::PHI1>(e)) ;
		}

		if(ihmap_.is_orbit_embedded<Orbit::PHI1_PHI2>())
		{

		}

		return v;
	}

	inline void split_face(Face d, Face e)
	{
		ihmap_.split_face(d, e);

		if(ihmap_.is_orbit_embedded<Orbit::DART>())
		{
			unsigned int cur = m_curLevel ;
			m_curLevel = m_maxLevel ;
			ihmap_.set_orbit_embedding<Orbit::DART>(d, ihmap_.get_embedding<Orbit::DART>(d)) ;
			ihmap_.set_orbit_embedding<Orbit::DART>(e, ihmap_.get_embedding<Orbit::DART>(e)) ;
			m_curLevel = cur ;
		}
	}

private:
	IHCMap2& ihmap_;

};

} // namespace cgogn


#endif // CORE_CPH_IHCMAP_MODIFIER_H_