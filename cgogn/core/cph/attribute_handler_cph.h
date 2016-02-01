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

#ifndef CORE_CPH_ATTRIBUTE_HANDLER_CPH_H_
#define CORE_CPH_ATTRIBUTE_HANDLER_CPH_H_

#include <core/cmap/attribute_handler.h>
#include <core/utils/assert.h>

namespace cgogn
{

/**
 * \brief AttributeHandler class
 * @TPARAM T the data type of the attribute to handlde
 */
template<typename DATA_TRAITS, typename T, Orbit ORBIT>
class AttributeHandlerCPH : public AttributeHandler<DATA_TRAITS, T, ORBIT>
{
public:

	typedef AttributeHandler<DATA_TRAITS, T, ORBIT> Inherit;
	typedef AttributeHandlerCPH<DATA_TRAITS, T, ORBIT> Self;

	typedef T value_type;

	AttributeHandlerCPH() : 
		Inherit()
	{}

	AttributeHandlerCPH(MapData* const m, TChunkArray* const ca) :
		Inherit(m,ca)
	{}

	AttributeHandlerCPH(const Self& att) :
		Inherit(att)
	{}

	AttributeHandlerCPH(Self&& att) CGOGN_NOEXCEPT :
		Inherit(att)
	{}

	TChunkArray const* get_data() const
	{
		return Inherit::get_data();
	}

	bool is_valid() const
	{
		return Inherit::is_valid();
	}

	T& operator[](Dart d)
	{
		switch(ORBIT)
		{
			case Orbit::PHI1: return vertex_embedding(d); break;
			default: edge_face_embedding(d); break;
		}

		ImplicitHierarchicalMap2* m = reinterpret_cast<ImplicitHierarchicalMap2*>(this->m_map) ;
		assert(m->m_dartLevel[d] <= m->m_curLevel || !"Access to a dart introduced after current level") ;
		assert(m->vertexInsertionLevel(d) <= m->m_curLevel || !"Access to the embedding of a vertex inserted after current level") ;

		unsigned int orbit = this->getOrbit() ;
		unsigned int nbSteps = m->m_curLevel - m->vertexInsertionLevel(d) ;
		unsigned int index = m->getEmbedding<ORBIT>(d) ;

		if(index == EMBNULL)
		{
			index = Algo::Topo::setOrbitEmbeddingOnNewCell<ORBIT>(m, d) ;
			m->m_nextLevelCell[orbit]->operator[](index) = EMBNULL ;
		}

		AttributeContainer& cont = m->getAttributeContainer<ORBIT>() ;
		unsigned int step = 0 ;
		while(step < nbSteps)
		{
			step++ ;
			unsigned int nextIdx = m->m_nextLevelCell[orbit]->operator[](index) ;
			if (nextIdx == EMBNULL)
			{
				nextIdx = m->newCell<ORBIT>() ;
				m->copyCell<ORBIT>(nextIdx, index) ;
				m->m_nextLevelCell[orbit]->operator[](index) = nextIdx ;
				m->m_nextLevelCell[orbit]->operator[](nextIdx) = EMBNULL ;
				cont.refLine(index) ;
			}
			index = nextIdx ;
		}
		return this->m_attrib->operator[](index);
		}

	const T& operator[](Dart d) const
	{
		ImplicitHierarchicalMap2* m = reinterpret_cast<ImplicitHierarchicalMap2*>(this->m_map) ;
		assert(m->m_dartLevel[d] <= m->m_curLevel || !"Access to a dart introduced after current level") ;
		assert(m->vertexInsertionLevel(d) <= m->m_curLevel || !"Access to the embedding of a vertex inserted after current level") ;

		unsigned int orbit = this->getOrbit() ;
		unsigned int nbSteps = m->m_curLevel - m->vertexInsertionLevel(d) ;
		unsigned int index = m->getEmbedding<ORBIT>(d) ;

		unsigned int step = 0 ;
		while(step < nbSteps)
		{
			step++ ;
			unsigned int next = m->m_nextLevelCell[orbit]->operator[](index) ;
			if(next != EMBNULL) index = next ;
			else break ;
		}
		return this->m_attrib->operator[](index);	
	}

	T& operator[](unsigned int a)
	{
		return AttributeHandler<T, ORBIT, ImplicitHierarchicalMap2>::operator[](a) ;
	}

	const T& operator[](unsigned int a) const
	{
		return AttributeHandler<T, ORBIT, ImplicitHierarchicalMap2>::operator[](a) ;
	}

} // namespace cgogn

#endif // CORE_CPH_ATTRIBUTE_HANDLER_CPH_H_