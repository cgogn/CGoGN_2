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
#include <array>

namespace cgogn
{


template <typename MAP_TRAITS, typename MAP_TYPE>
class MRCMap2_T : public CMap2_T<MAP_TRAITS, MAP_TYPE>
{
public:

	typedef MRCMap2_T<MAP_TRAITS, MAP_TYPE> Self;
	typedef CMap2_T<MAP_TRAITS, MAP_TYPE> Inherit;

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
		CMap2* last = maps_.back();
		maps_.emplace_back(last);
	}

	inline void remove_level_back()
	{
		CMap2* back = maps_.pop_back();
		delete back;
	}

	inline void add_level_front()
	{
		CMap2* first = maps_.front();
		maps_.emplace_front(first);
	}

	inline void remove_level_front()
	{
		CMap2* front = maps_.pop_front();
		delete front;
	}

	inline CMap2* get_current_cmap() const
	{
		return maps_[get_current_level()];
	}

public:

	MRCMap2_T()
	{
		//creation d'un niveau 0 avec une carte vide 
		//a la construction -> should call new CMap2()
		maps_.emplace_back();
	}

	~MRCMap2_T()
	{}

	MRCMap2_T(Self const&) = delete;
	MRCMap2_T(Self &&) = delete;
	Self& operator=(Self const&) = delete;
	Self& operator=(Self &&) = delete;


	//1 thread par niveau = 1 thread par carte
	//n thread par niveau = n thread par carte

	inline std::size_t get_maximum_level() const
	{
		return maps_.size();
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

	inline void push_level()
	{
		levels_stack_.push_back(get_current_level()) ;
	}

	inline void pop_level()
	{
		set_current_level(levels_stack_.back()) ;
		levels_stack_.pop_back() ;
	}

protected:

	/*******************************************************************************
	 * Orbits traversal
	 *******************************************************************************/

	template <typename FUNC>
	inline void foreach_dart_of_vertex(Dart d, const FUNC& f) const
	{
		get_current_cmap()->foreach_dart_of_vertex(d, f);
	}

	template <typename FUNC>
	inline void foreach_dart_of_face(Dart d, const FUNC& f) const
	{
		get_current_cmap()->foreach_dart_of_face(d, f);
	}

	template <typename FUNC>
	void foreach_dart_of_volume(Dart d, const FUNC& f) const
	{
		get_current_cmap()->foreach_dart_of_volume(d, f);
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

/*
template <typename MAP_TRAITS>
class MapBaseMRData: public MapGen
{

public:
	typedef MapGen Inherit;
	typedef MapBaseMRData<MAP_TRAITS> Self;

	static const unsigned int CHUNKSIZE = MAP_TRAITS::CHUNK_SIZE;

	template<typename T_REF>
	using ChunkArrayContainer = cgogn::ChunkArrayContainer<CHUNKSIZE, T_REF>;
	using ChunkArrayGen = cgogn::ChunkArrayGen<CHUNKSIZE>;
	template<typename T>
	using ChunkArray = cgogn::ChunkArray<CHUNKSIZE, T>;


protected:

	/// per level topology & embedding indices
	std::deque<ChunkArrayContainer<unsigned char>> topology_;

	/// per level per orbit attributes
	std::deque<std::array<ChunkArrayContainer<unsigned int>, NB_ORBITS>> attributes_;

	/// per level embedding indices shortcuts
	std::deque<std::array<ChunkArray<unsigned int>*, NB_ORBITS>> embeddings_;

	/// per level boundary markers shortcuts
	std::deque<std::array<ChunkArray<bool>*,2>> boundary_markers_;


	/// vector of available mark attributes per thread on the topology container
	std::deque<std::array<std::vector<ChunkArray<bool>*,NB_THREADS>> mark_attributes_topology_;
	std::mutex mark_attributes_topology_mutex_;

	/// vector of available mark attributes per orbit per thread on attributes containers
	std::vector<ChunkArray<bool>*> mark_attributes_[NB_ORBITS][NB_THREADS];
	std::deque<std::array<std::mutex, NB_ORBITS>> mark_attributes_mutex_;

	/// vector of thread ids known by the map that can pretend to data such as mark vectors
	std::vector<std::thread::id> thread_ids_;

	/// global topo cache shortcuts
	ChunkArray<Dart>* global_topo_cache_[NB_ORBITS];

public:
	MapBaseMRData() : Inherit()
	{
		//TODO
	}

	~MapBaseMRData() override
	{}

	MapBaseMRData(Self const&) = delete;
	MapBaseMRData(Self &&) = delete;
	Self& operator=(Self const&) = delete;
	Self& operator=(Self &&) = delete;

protected:
	inline const ChunkArrayContainer<unsigned int>& get_attribute_container(unsigned int orbit, unsigned int level) const
	{
		cgogn_message_assert(orbit < NB_ORBITS, "Unknown orbit parameter");
		cgogn_message_assert(level < attributes_.size(), "max level reached");

		return attributes_[level][orbit];
	}

	inline ChunkArrayContainer<unsigned int>& get_attribute_container(unsigned int orbit, unsigned int level)
	{
		cgogn_message_assert(orbit < NB_ORBITS, "Unknown orbit parameter");
		cgogn_message_assert(level < attributes_.size(), "max level reached");

		return attributes_[level][orbit];
	}

	inline ChunkArray<bool>* get_topology_mark_attribute()
	{
		unsigned int thread = this->get_current_thread_index();
		if (!this->mark_attributes_topology_[thread].empty())
		{
			ChunkArray<bool>* ca = this->mark_attributes_topology_[thread].back();
			this->mark_attributes_topology_[thread].pop_back();
			return ca;
		}
		else
		{
			std::lock_guard<std::mutex> lock(this->mark_attributes_topology_mutex_);
			ChunkArray<bool>* ca = this->topology_.add_marker_attribute();
			return ca;
		}
	}

public:

	template <Orbit ORBIT>
	inline bool is_orbit_embedded(unsigned int level) const
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		return embeddings_[level][ORBIT] != nullptr;
	}

	template <Orbit ORBIT>
	inline bool is_orbit_embedded() const
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		for(unsigned int i = O ; i < embeddings_.size() ; ++i)
			if(embeddings_[i][ORBIT] != nullptr)
				return true;
		else false;
	}

	template <Orbit ORBIT>
	inline unsigned int get_embedding(Cell<ORBIT> c) const
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		cgogn_message_assert(is_orbit_embedded<ORBIT>(), "Invalid parameter: orbit not embedded");

		return (*embeddings_[ORBIT])[c.dart.index];
	}

	template <typename T>
	inline void add_level_back()
	{
		//ajouter une carte par copie dans maps_
		//ajouter un chunkarray dans next_

		topology_.template add_attribute<T>

		CMap2* last = maps_.back();
		maps_.emplace_back(last);
	}
};
*/

} // namespace cgogn

#endif // CORE_MRCMAP_MRCMAP2_H_
