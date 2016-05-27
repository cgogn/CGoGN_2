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

#ifndef CGOGN_MULTIRESOLUTION_CPH_ATTRIBUTE_HANDLER_CPH_H_
#define CGOGN_MULTIRESOLUTION_CPH_ATTRIBUTE_HANDLER_CPH_H_

#include <cgogn/core/cmap/attribute.h>
#include <cgogn/core/utils/assert.h>

namespace cgogn
{

/**
 * \brief Attribute class
 * @TPARAM T the data type of the attribute to handlde
 */
template <typename MAP, typename DATA_TRAITS, typename T, Orbit ORBIT>
class AttributeCPH : public Attribute<DATA_TRAITS, T, ORBIT>
{
public:

	using Inherit = Attribute<DATA_TRAITS, T, ORBIT>;
	using Self = AttributeCPH<MAP, DATA_TRAITS, T, ORBIT>;

	using value_type = T;

	using MapData =     typename Inherit::MapData;
	using TChunkArray = typename Inherit::template ChunkArray<T>;


	AttributeCPH() : 
		Inherit()
	{}

	AttributeCPH(MapData* const m, TChunkArray* const ca) :
		Inherit(m,ca)
	{}

	AttributeCPH(const Self& att) :
		Inherit(att)
	{}

	AttributeCPH(Self&& att) CGOGN_NOEXCEPT :
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

	T& operator[](Cell<ORBIT> c)
	{
		cgogn_message_assert(is_valid(), "Invalid Attribute");
		return this->chunk_array_->operator[](this->map_->get_embedding(c));
	}
//		switch(ORBIT)
//		{
//			case Orbit::PHI1: return vertex_embedding(d); break;
//			default: edge_face_embedding(d); break;
//		}

//		cgogn_message_assert(m->get_dart_level(d) <= m->get_current_level(),
//							 "Access to a dart introduced after current level") ;
//		cgogn_message_assert(m->vertexInsertionLevel(d) <= m->m_curLevel,
//							 "Access to the embedding of a vertex inserted after current level") ;

//		uint32 orbit = this->getOrbit() ;
//		uint32 nbSteps = m->m_curLevel - m->vertexInsertionLevel(d) ;
//		uint32 index = m->getEmbedding<ORBIT>(d) ;

//		AttributeContainer& cont = m->getAttributeContainer<ORBIT>() ;
//		uint32 step = 0 ;
//		while(step < nbSteps)
//		{
//			step++ ;
//			uint32 nextIdx = m->m_nextLevelCell[orbit]->operator[](index) ;
//			if (nextIdx == INVALID_INDEX)
//			{
//				nextIdx = m->newCell<ORBIT>() ;
//				m->copyCell<ORBIT>(nextIdx, index) ;
//				m->m_nextLevelCell[orbit]->operator[](index) = nextIdx ;
//				m->m_nextLevelCell[orbit]->operator[](nextIdx) = INVALID_INDEX ;
//				cont.refLine(index) ;
//			}
//			index = nextIdx ;
//		}
//		return this->m_attrib->operator[](index);

	const T& operator[](Dart d) const
	{
//		ImplicitHierarchicalMap2* m = reinterpret_cast<ImplicitHierarchicalMap2*>(this->m_map) ;
//		cgogn_message_assert(m->m_dartLevel[d] <= m->m_curLevel,
//							 "Access to a dart introduced after current level") ;
//		cgogn_message_assert(m->vertexInsertionLevel(d) <= m->m_curLevel,
//							 "Access to the embedding of a vertex inserted after current level") ;

//		uint32 orbit = this->getOrbit() ;
//		uint32 nbSteps = m->m_curLevel - m->vertexInsertionLevel(d) ;
//		uint32 index = m->getEmbedding<ORBIT>(d) ;

//		uint32 step = 0 ;
//		while(step < nbSteps)
//		{
//			step++ ;
//			uint32 next = m->m_nextLevelCell[orbit]->operator[](index) ;
//			if(next != INVALID_INDEX) index = next ;
//			else break ;
//		}
//		return this->m_attrib->operator[](index);
	}

	T& operator[](uint32 a)
	{
		return Attribute<T, ORBIT>::operator[](a) ;
	}

	const T& operator[](uint32 a) const
	{
		return Attribute<T, ORBIT>::operator[](a) ;
	}
};

} // namespace cgogn

#endif // CGOGN_MULTIRESOLUTION_CPH_ATTRIBUTE_HANDLER_CPH_H_
