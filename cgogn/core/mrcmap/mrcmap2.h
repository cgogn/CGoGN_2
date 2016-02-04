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

#ifndef CORE_MRCMAP_MRCMAP2_H_
#define CORE_MRCMAP_MRCMAP2_H_

#include <core/map/cmap2.h>
#include <deque>
#include <stack>

namespace cgogn
{

template <typename MAP_TRAITS, typename MAP_TYPE>
class MRCMap2_T
{
public:

	typedef MRCMap2_T<MAP_TRAITS, MAP_TYPE> Self;
	typedef CMap2_T<MAP_TRAITS, MAP_TYPE> CMap2;

	template<typename T>
	using ChunkArray =  typename CMap2::template ChunkArray<T>;


protected:
	/**
	 * pointers to maps (one for each level)
	 */
	std::deque<CMap2*> maps_;

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



	inline void add_level_back()
	{
		//ajouter une carte par copie dans maps_
		//ajouter un chunkarray dans next_
		CMap2* last = maps_.back();
		maps_.emplace_back(last);
	}

	inline void remove_level_back()
	{
		maps_.pop_back();
	}

	inline void add_level_front()
	{
		CMap2* first = maps_.front();
		maps.emplace_front(first);
	}

	inline void remove_level_front()
	{
		maps_.pop_front();
	}

public:

	MRCMap2_T()
	{}

	~MRCMap2_T()
	{}

	MRCMap2_T(Self const&) = delete;
	MRCMap2_T(Self &&) = delete;
	Self& operator=(Self const&) = delete;
	Self& operator=(Self &&) = delete;


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

	inline const CMap2* current() const
	{
		return maps_[get_current_level()];
	}

protected:

	/*******************************************************************************
	 * Orbits traversal
	 *******************************************************************************/

	template <typename FUNC>
	inline void foreach_dart_of_vertex(Dart d, const FUNC& f) const
	{
		current()->foreach_dart_of_vertex(d, f);
	}

	template <typename FUNC>
	inline void foreach_dart_of_face(Dart d, const FUNC& f) const
	{
		current()->foreach_dart_of_face(d, f);
	}

	template <typename FUNC>
	void foreach_dart_of_volume(Dart d, const FUNC& f) const
	{
		current()->foreach_dart_of_volume(d, f);
	}

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_dart_of_orbit(Cell<ORBIT> c, const FUNC& f) const
	{
		switch (ORBIT)
		{
			case Orbit::DART: foreach_dart_of_orbit(c, f); break;
			case Orbit::PHI1: foreach_dart_of_orbit(c, f); break;
			case Orbit::PHI2: //TODO add a foreach_dart_of_edge to cmap2 f(c.dart); f(phi2(c.dart)); break;
			case Orbit::PHI1_PHI2: foreach_dart_of_volume(c, f); break;
			case Orbit::PHI21: foreach_dart_of_vertex(c, f); break;
			case Orbit::PHI2_PHI3:
			case Orbit::PHI1_PHI3:
			case Orbit::PHI21_PHI31:
			default: cgogn_assert_not_reached("Cells of this dimension are not handled"); break;
		}
	}
};

} // namespace cgogn

#endif // CORE_MRCMAP_MRCMAP2_H_
