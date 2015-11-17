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

#ifndef CORE_MAP_MAP_BASE_H_
#define CORE_MAP_MAP_BASE_H_

#include <core/map/map_base_data.h>
#include <core/map/attribute_handler.h>

#include <sstream>

namespace cgogn
{

template<typename DATA_TRAITS, typename TOPO_TRAITS>
class MapBase : public MapBaseData<DATA_TRAITS>
{
protected:

	std::multimap<ChunkArrayGen<DATA_TRAITS::CHUNK_SIZE>*, AttributeHandlerGen<DATA_TRAITS>*> attributeHandlers_;

public:

	//TODO remonter dans map_base
	template<typename T, unsigned int ORBIT>
	using AttributeHandler = cgogn::AttributeHandler<DATA_TRAITS, T, ORBIT>;

	MapBase()
	{}

	~MapBase()
	{}

	template <typename T, unsigned int ORBIT>
	inline AttributeHandler<T, ORBIT> addAttribute(const std::string& attributeName = "")
	{
		if (this->embeddings_[ORBIT] == nullptr)
		{
			std::ostringstream oss;
			oss << "EMB_" << orbitName(ORBIT);
			ChunkArray<DATA_TRAITS::CHUNK_SIZE, unsigned int>* idx = this->topology_.template addAttribute<unsigned int>(oss.str());
			this->embeddings_[ORBIT] = idx;
			for (unsigned int i = this->topology_.begin(); i != this->topology_.end(); this->topology_.next(i))
				(*idx)[i] = EMBNULL;
		}

		ChunkArray<DATA_TRAITS::CHUNK_SIZE, T>* ca = this->attributes_[ORBIT].template addAttribute<T>(attributeName);
		return AttributeHandler<T, ORBIT>(this, ca);
	}

	/**
	 * remove an attribute
	 * @param attr a handler to the attribute to remove
	 * @return true if remove succeed else false
	 */
	template <typename T, unsigned int ORBIT>
	inline bool removeAttribute(AttributeHandler<T,ORBIT>& ah)
	{
		ChunkArray<DATA_TRAITS::CHUNK_SIZE, T>* ca = ah.getData();

		if (this->attributes_[ORBIT].removeAttribute(ca))
		{
			typedef typename std::multimap<ChunkArrayGen<DATA_TRAITS::CHUNK_SIZE>*, AttributeHandlerGen<DATA_TRAITS>*>::iterator IT;
			std::pair<IT, IT> bounds = attributeHandlers_.equal_range(ca);
			for(IT i = bounds.first; i != bounds.second; ++i)
				(*i).second->setInvalid();
			attributeHandlers_.erase(bounds.first, bounds.second);
			return true;
		}
		return false;
	}

	/**
	* search an attribute for a given orbit
	* @param nameAttr attribute name
	* @return an AttributeHandler
	*/
	template <typename T, unsigned int ORBIT>
	inline AttributeHandler< T, ORBIT> getAttribute(const std::string& nameAttr)
	{
		ChunkArray<DATA_TRAITS::CHUNK_SIZE, T>* ca = this->attributes_[ORBIT].template getAttribute<T>(nameAttr);
		return AttributeHandler<T, ORBIT>(this, ca);
	}

	inline Dart add_dart()
	{
		unsigned int di = this->topology_.template insertLines<1>();	// insert a new dart line
		this->topology_.initBooleansOfLine(di);

		for(unsigned int i = 0; i < NB_ORBITS; ++i)
		{
			if (this->embeddings_[i])							// set all its embeddings
				(*(this->embeddings_[i]))[di] = EMBNULL;		// to EMBNULL
		}

		Dart d(di);

		for (auto relPtr: this->topo_relations_)
			(*relPtr)[di] = d;

		return d;
	}
};

} // namespace cgogn

#endif // CORE_MAP_MAP_BASE_H_
