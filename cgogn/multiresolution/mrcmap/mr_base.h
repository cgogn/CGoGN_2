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

#ifndef MULTIRESOLUTION_MRCMAP_MR_BASE_H_
#define MULTIRESOLUTION_MRCMAP_MR_BASE_H_

#include <deque>
#include <stack>
#include <vector>

namespace cgogn
{

template <typename MAP>
class MRBase
{
public:
	using Self = MRBase<MAP>;

	template<typename T>
	using ChunkArray =  typename MAP::template ChunkArray<T>;

protected:
	/**
	 * pointers to maps (one for each level)
	 */
	std::deque<MAP*> maps_;

	/**
	 * pointers to attributs that stores next level
	 * correspondance indices for each dart
	 */
	std::deque<ChunkArray<unsigned int>*> next_level_indices_;

	/**
	 * pointers to attributs that stores previous level
	 * correspondance indices for each dart
	 */
	std::deque<ChunkArray<unsigned int>*> previous_level_indices_;

	/**
	 * stack for current level temporary storage
	 */
	std::stack<unsigned int, std::vector<unsigned int>> levels_stack_ ;

	/**
	 * current level in multiresolution map
	 */
	unsigned int current_level_;

	//TODO le niveau courant doit etre par thread
	//appele sur la carte et non plus un champs de
	//la classe carte

public:

	MRBase()
	{}

	~MRBase()
	{}

	MRBase(Self const&) = delete;
	MRBase(Self &&) = delete;
	Self& operator=(Self const&) = delete;
	Self& operator=(Self &&) = delete;

	inline void add_level_back()
	{
		//ajouter une carte par copie dans maps_
		//ajouter un chunkarray dans next_
		MAP* last = maps_.back();
		maps_.push_back(last);
	}

	inline void remove_level_back()
	{
		maps_.pop_back();
	}

	inline void add_level_front()
	{
		MAP* first = maps_.front();
		maps.push_front(first);
	}

	inline void remove_level_front()
	{
		maps_.pop_front();
	}

	//1 thread par niveau = 1 thread par carte
	//n thread par niveau = n thread par carte

	inline unsigned int get_maximum_level() const
	{
		return static_cast<unsigned int>(maps_.size());
	}

	inline unsigned int get_current_level() const
	{
		return current_level_;
	}

	inline void set_current_level(unsigned int l)
	{
		current_level_ = l;
	}

	inline void inc_current_level()
	{
		cgogn_debug_assert(get_current_level() < maps_.size() - 1, "incCurrentLevel : already at maximum resolution level");
		++current_level_;
	}

	inline void dec_current_level()
	{
		cgogn_debug_assert(get_current_level() > 0, "decCurrentLevel : already at minimum resolution level");
		--current_level_;
	}

	//TODO
	inline unsigned int get_dart_level(Dart d)
	{
		return 0;
	}

	/**
	 * store current resolution level on a stack
	 */
	inline void push_level()
	{
		levels_stack_.push_back(get_current_level()) ;
	}

	/**
	 * set as current the resolution level of the top of the stack
	 */
	inline void pop_level()
	{
		set_current_level(levels_stack_.back()) ;
		levels_stack_.pop_back() ;
	}

	inline const MAP* current() const
	{
		return maps_[get_current_level()];
	}

};


}

#endif // MULTIRESOLUTION_MRCMAP_MR_BASE_H_
