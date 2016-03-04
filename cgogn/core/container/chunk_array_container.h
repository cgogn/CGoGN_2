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

#ifndef CORE_CONTAINER_CHUNK_ARRAY_CONTAINER_H_
#define CORE_CONTAINER_CHUNK_ARRAY_CONTAINER_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <string>
#include <memory>
#include <climits>

#include <core/dll.h>
#include <core/utils/assert.h>
#include <core/utils/name_types.h>
#include <core/utils/make_unique.h>
#include <core/container/chunk_array.h>
#include <core/container/chunk_stack.h>
#include <core/container/chunk_array_factory.h>

namespace cgogn
{

class CGOGN_CORE_API ContainerBrowser
{
public:

	virtual unsigned int begin() const = 0;
	virtual unsigned int end() const = 0;
	virtual void next(unsigned int &it) const = 0;
	virtual void next_primitive(unsigned int &it, unsigned int primSz) const = 0;
	virtual void enable() = 0;
	virtual void disable() = 0;
	virtual ~ContainerBrowser();
};

template <typename CONTAINER>
class ContainerStandardBrowser : public ContainerBrowser
{
	const CONTAINER* cac_;

public:

	ContainerStandardBrowser(const CONTAINER* cac) : cac_(cac) {}
	virtual unsigned int begin() const { return cac_->real_begin(); }
	virtual unsigned int end() const { return cac_->real_end(); }
	virtual void next(unsigned int &it)  const { cac_->real_next(it); }
	virtual void next_primitive(unsigned int &it, unsigned int primSz) const { cac_->real_next_primitive(it,primSz); }
	virtual void enable() {}
	virtual void disable() {}
	virtual ~ContainerStandardBrowser() {}
};


/**
 * @brief class that manage the storage of several ChunkArray
 * @tparam CHUNKSIZE chunk size for ChunkArray
 */
template <unsigned int CHUNKSIZE, typename T_REF>
class ChunkArrayContainer
{
public:

	using Self = ChunkArrayContainer<CHUNKSIZE, T_REF>;
	using ref_type = T_REF;

	using ChunkArrayGen = cgogn::ChunkArrayGen<CHUNKSIZE>;
	template <class T>
	using ChunkArray = cgogn::ChunkArray<CHUNKSIZE, T>;
	template <class T>
	using ChunkStack = cgogn::ChunkStack<CHUNKSIZE, T>;

	/**
	* constante d'attribut inconnu
	*/
	static const unsigned int UNKNOWN = UINT_MAX;

protected:

	/**
	* vector of pointers to ChunkArray
	*/
	std::vector<ChunkArrayGen*> table_arrays_;

	std::vector<std::string> names_;

	std::vector<std::string> type_names_;

	/**
	* vector of pointers to Marker ChunkArray
	*/
	std::vector<ChunkArray<bool>*> table_marker_arrays_;

	/**
	 * @brief ChunkArray of refs
	 */
	ChunkArray<T_REF> refs_;

	/**
	 * stack of holes
	 */
	ChunkStack<unsigned int> holes_stack_;

	/**
	* size (number of elts) of the container
	*/
	unsigned int nb_used_lines_;

	/**
	* size of the container with holes (also index of next inserted line if no holes)
	*/
	unsigned int nb_max_lines_;

	/**
	 * Browser that allow special traversals
	 */
	ContainerBrowser* current_browser_;

	/**
	 * Browser that allow special traversals
	 */
	ContainerStandardBrowser<Self>* std_browser_;

	/**
	 * @brief get array index from name
	 * @warning do not store index (not stable)
	 * @param attribName name of ChunkArray
	 * @return the index in table
	 */
	unsigned int get_array_index(const std::string& attribute_name) const
	{
		for (unsigned int i = 0u; i != names_.size(); ++i)
		{
			if (names_[i] == attribute_name)
				return i;
		}
		return UNKNOWN;
	}

	/**
	 * @brief get array index from ptr
	 * @warning do not store index (not stable)
	 * @param ptr of ChunkArray
	 * @return the index in table
	 */
	unsigned int get_array_index(const ChunkArrayGen* ptr) const
	{
		for (unsigned int i = 0u; i != table_arrays_.size(); ++i)
		{
			if (table_arrays_[i] == ptr)
				return i;
		}
		return UNKNOWN;
	}

	/**
	 * @brief remove an attribute by its index
	 * @param index index of attribute to remove
	 */
	void remove_attribute(unsigned int index)
	{
		// store ptr for using it before delete
		ChunkArrayGen* ptr_to_del = table_arrays_[index];

		if (index != table_arrays_.size() - std::size_t(1u))
		{
			table_arrays_[index] = table_arrays_.back();
			names_[index]        = names_.back();
			type_names_[index]   = type_names_.back();
		}

		table_arrays_.pop_back();
		names_.pop_back();
		type_names_.pop_back();

		delete ptr_to_del;
	}

public:

	/**
	 * @brief ChunkArrayContainer constructor
	 */
	ChunkArrayContainer():
		nb_used_lines_(0u),
		nb_max_lines_(0u)
	{
		std_browser_ = new ContainerStandardBrowser<Self>(this);
		current_browser_= std_browser_;
	}

	ChunkArrayContainer(Self const& ) = delete;
	ChunkArrayContainer(Self&& ) = delete;
	ChunkArrayContainer& operator=(Self const& ) = delete;
	ChunkArrayContainer& operator=(Self&& ) = delete;

	/**
	 * @brief ChunkArrayContainer destructor
	 */
	~ChunkArrayContainer()
	{
		if (current_browser_ != std_browser_)
			delete current_browser_;
		delete std_browser_;

		for (auto ptr : table_arrays_)
			delete ptr;

		for (auto ptr : table_marker_arrays_)
			delete ptr;
	}

	/**
	 * @brief get an attribute
	 * @param attribute_name name of attribute
	 * @tparam T type of attribute
	 * @return pointer on attribute ChunkArray
	 */
	template <typename T>
	ChunkArray<T>* get_attribute(const std::string& attribute_name) const
	{
		// first check if attribute already exist
		unsigned int index = get_array_index(attribute_name);
		if (index == UNKNOWN)
		{
			std::cerr << "attribute " << attribute_name << " not found." << std::endl;
			return nullptr;
		}

		return static_cast<ChunkArray<T>*>(table_arrays_[index]);
	}

	/**
	 * @brief add an attribute
	 * @param attribute_name name of attribute
	 * @tparam T type of attribute
	 * @return pointer on created attribute ChunkArray
	 */
	template <typename T>
	ChunkArray<T>* add_attribute(const std::string& attribute_name)
	{
		cgogn_assert(attribute_name.size() != 0);

		// first check if attribute already exist
		unsigned int index = get_array_index(attribute_name);
		if (index != UNKNOWN)
		{
			std::cerr << "attribute " << attribute_name << " already exists.." << std::endl;
			return nullptr;
		}

		// create the new attribute
		const std::string& typeName = name_of_type(T());
		ChunkArray<T>* carr = new ChunkArray<T>();
		ChunkArrayFactory<CHUNKSIZE>::template register_CA<T>();

		// reserve memory
		carr->set_nb_chunks(refs_.get_nb_chunks());

		// store pointer, name & typename.
		table_arrays_.push_back(carr);
		names_.push_back(attribute_name);
		type_names_.push_back(typeName);

		return carr;
	}

	/**
	 * @brief remove an attribute by its name
	 * @param attribute_name name of attribute to remove
	 * @return true if attribute exists and has been removed
	 */
	bool remove_attribute(const std::string& attribute_name)
	{
		unsigned int index = get_array_index(attribute_name);

		if (index == UNKNOWN)
		{
			std::cerr << "removeAttribute by name: attribute not found (" << attribute_name << ")" << std::endl;
			return false;
		}

		remove_attribute(index);

		return true;
	}

	/**
	 * @brief remove an attribute by its ChunkArray pointer
	 * @param ptr ChunkArray pointer to the attribute to remove
	 * @return true if attribute exists and has been removed
	 */
	bool remove_attribute(const ChunkArrayGen* ptr)
	{
		unsigned int index = get_array_index(ptr);

		if (index == UNKNOWN)
		{
			std::cerr << "remove_attribute by ptr: attribute not found" << std::endl;
			return false;
		}

		remove_attribute(index);

		return true;
	}

	/**
	 * @brief add a Marker attribute
	 * @return pointer on created ChunkArray
	 */
	ChunkArray<bool>* add_marker_attribute()
	{
		ChunkArray<bool>* mca = new ChunkArray<bool>();
		mca->set_nb_chunks(refs_.get_nb_chunks());
		table_marker_arrays_.push_back(mca);
		return mca;
	}

	/**
	 * @brief remove a marker attribute by its ChunkArray pointer
	 * @param ptr ChunkArray pointer to the attribute to remove
	 * @return true if attribute exists and has been removed
	 */
	void remove_marker_attribute(const ChunkArray<bool>* ptr)
	{
		unsigned int index = 0u;
		while (index < table_marker_arrays_.size() && table_marker_arrays_[index] != ptr)
			++index;

		cgogn_message_assert(index != table_marker_arrays_.size(), "remove_marker_attribute by ptr: attribute not found");

		if (index != table_marker_arrays_.size() - std::size_t(1u))
			table_marker_arrays_[index] = table_marker_arrays_.back();
		table_marker_arrays_.pop_back();

		delete ptr;
	}

	/**
	 * @brief Number of attributes of the container
	 * @return number of attributes
	 */
	std::size_t get_nb_attributes() const
	{
		return table_arrays_.size();
	}

	/**
	* @brief get a chunk_array
	* @param attribute_name name of the array
	* @return pointer on typed chunk_array
	*/
	template <typename T>
	ChunkArray<T>* get_data_array(const std::string& attribute_name)
	{
		unsigned int index = get_array_index(attribute_name);
		if (index == UNKNOWN)
			return nullptr;

		ChunkArray<T>* atm = dynamic_cast<ChunkArray<T>*>(table_arrays_[index]);

		cgogn_message_assert(atm != nullptr, "get_data_array : wrong type");

		return atm;
	}

	/**
	* @brief get a chunk_array
	* @param attribute_name name of the array
	* @return pointer on virtual chunk_array
	*/
	ChunkArrayGen* get_virtual_data_array(const std::string& attribute_name)
	{
		unsigned int index = get_array_index(attribute_name);
		if (index == UNKNOWN)
			return nullptr;

		return table_arrays_[index];
	}

	/**
	 * @brief size (number of used lines)
	 * @return the number of lines
	 */
	unsigned int size() const
	{
		return nb_used_lines_;
	}

	/**
	 * @brief capacity (number of reserved lines)
	 * @return number of reserved lines
	 */
	unsigned int capacity() const
	{
		return refs_.capacity();
	}

	/**
	 * @brief set the current container browser
	 * @param browser, pointer to a heap-allocated ContainerBrowser
	 */
	inline void set_current_browser(ContainerBrowser* browser)
	{
		if (current_browser_ != std_browser_)
			delete current_browser_;
		current_browser_ = browser;
	}

	inline void set_standard_browser()
	{
		if (current_browser_ != std_browser_)
			delete current_browser_;
		current_browser_ = std_browser_;
	}

	/**
	 * @brief begin
	 * @return the index of the first used line of the container
	 */
	inline unsigned int begin() const
	{
		return current_browser_->begin();
	}

	/**
	 * @brief end
	 * @return the index after the last used line of the container
	 */
	inline unsigned int end() const
	{
		return current_browser_->end();
	}

	/**
	 * @brief next it <- next used index in the container
	 * @param it index to "increment"
	 */
	inline void next(unsigned int& it) const
	{
		current_browser_->next(it);
	}

	/**
	 * @brief next primitive: it <- next primitive used index in the container (eq to PRIMSIZE next)
	 * @param it index to "increment"
	 */
	inline void next_primitive(unsigned int& it, unsigned int prim_size) const
	{
		current_browser_->next_primitive(it, prim_size);
	}

	/**
	 * @brief begin of container without browser
	 * @return the real index of the first used line of the container
	 */
	inline unsigned int real_begin() const
	{
		unsigned int it = 0u;
		while ((it < nb_max_lines_) && (!used(it)))
			++it;
		return it;
	}

	/**
	 * @brief end of container without browser
	 * @return the real index after the last used line of the container
	 */
	inline unsigned int real_end() const
	{
		return nb_max_lines_;
	}

	/**
	 * @brief next without browser
	 * @param it
	 */
	inline void real_next(unsigned int& it) const
	{
		do
		{
			++it;
		} while ((it < nb_max_lines_) && (!used(it)));
	}

	/**
	 * @brief next primitive without browser
	 * @param it
	 */
	inline void real_next_primitive(unsigned int &it, unsigned int prim_size) const
	{
		do
		{
			it += prim_size;
		} while ((it < nb_max_lines_) && (!used(it)));
	}

	/**
	 * @brief reverse begin of container without browser
	 * @return the real index of the first used line of the container in reverse order
	 */
	unsigned int real_rbegin() const
	{
		unsigned int it = nb_max_lines_- 1u;
		while ((it != 0xffffffff) && (!used(it)))
			--it;
		return it;
	}

	/**
	 * @brief reverse end of container without browser
	 * @return the real index before the last used line of the container in reverse order
	 */
	unsigned int real_rend() const
	{
		return 0xffffffff; // -1
	}

	/**
	 * @brief reverse next without browser
	 * @param it
	 */
	void real_rnext(unsigned int &it) const
	{
		do
		{
			--it;
		} while ((it != 0xffffffff) && (!used(it)));
	}

	/**
	 * @brief clear the container
	 * @param remove_attributes remove the attributes (not only their data)
	 */
	void clear_attributes()
	{
		nb_used_lines_ = 0u;
		nb_max_lines_ = 0u;

		// clear CA of refs
		refs_.clear();

		// clear holes
		holes_stack_.clear();

		// clear data
		for (auto cagen : table_arrays_)
			 cagen->clear();
		for (auto ca_bool : table_marker_arrays_)
			ca_bool->clear();
	}

	void remove_attributes()
	{
		nb_used_lines_ = 0u;
		nb_max_lines_ = 0u;
		refs_.clear();
		holes_stack_.clear();

		for (auto cagen : table_arrays_)
			delete cagen;
		for (auto ca_bool : table_marker_arrays_)
			delete ca_bool;

		table_arrays_.clear();
		table_marker_arrays_.clear();
		names_.clear();
		type_names_.clear();
	}

	/**
	 * @brief swap
	 * @param container
	 */
	void swap(Self& container)
	{
		table_arrays_.swap(container.table_arrays_);
		names_.swap(container.names_);
		type_names_.swap(container.type_names_);
		table_marker_arrays_.swap(container.table_marker_arrays_);
		refs_.swap(container.refs_);
		holes_stack_.swap(container.holes_stack_);
		std::swap(nb_used_lines_, container.nb_used_lines_);
		std::swap(nb_max_lines_, container.nb_max_lines_);

		ContainerBrowser* browser = current_browser_;

		if (container.current_browser_ != container.std_browser_)
			current_browser_ = container.current_browser_;
		else
			current_browser_ = std_browser_;

		if (browser != std_browser_)
			container.current_browser_ = browser;
		else
			container.current_browser_ = container.std_browser_;
	}

	/**
	 * @brief fragmentation of container (size/index of last lines): 100% = no holes
	 * @return 1 is full filled - 0 is lots of holes
	 */
	float fragmentation() const
	{
		return float(size()) / float(real_end());
	}

	/**
	 * @brief container compacting
	 * @param map_old_new vector that contains a map from old indices to new indices (holes -> 0xffffffff)
	 */
	template <unsigned int PRIMSIZE>
	void compact(std::vector<unsigned int>& map_old_new)
	{
		map_old_new.clear();
		map_old_new.resize(real_end(), 0xffffffff);

		unsigned int up = real_rbegin();
		unsigned int down = 0u;

		while (down < up)
		{
			if (!used(down))
			{
				for(unsigned int i = 0u; i < PRIMSIZE; ++i)
				{
					unsigned rdown = down + PRIMSIZE-1u - i;
					map_old_new[up] = rdown;
					copy_line(rdown, up);
					real_rnext(up);
				}
				down += PRIMSIZE;
			}
			else
				down++;
		}

		nb_max_lines_ = nb_used_lines_;

		// free unused memory blocks
		unsigned int new_nb_blocks = nb_max_lines_/CHUNKSIZE + 1u;

		for (auto arr : table_arrays_)
			arr->set_nb_chunks(new_nb_blocks);

		for (auto arr : table_marker_arrays_)
			arr->set_nb_chunks(new_nb_blocks);

		refs_.set_nb_chunks(new_nb_blocks);

		// clear holes
		holes_stack_.clear();
	}

	/**************************************
	 *          LINES MANAGEMENT          *
	 **************************************/

	/**
	* @brief get if the index is used
	* @param index index to test
	* @return true if the index is used, false otherwise
	*/
	bool used(unsigned int index) const
	{
		return refs_[index] != 0;
	}

	/**
	* @brief insert a group of PRIMSIZE consecutive lines in the container
	* @return index of the first line of group
	*/
	template <unsigned int PRIMSIZE>
	unsigned int insert_lines()
	{
		static_assert(PRIMSIZE < CHUNKSIZE, "Cannot insert lines in a container if PRIMSIZE < CHUNKSIZE");

		unsigned int index;

		if (holes_stack_.empty()) // no holes -> insert at the end
		{
			if (nb_max_lines_ == 0) // add first chunk
			{
				for (auto arr : table_arrays_)
					arr->add_chunk();
				for (auto arr : table_marker_arrays_)
					arr->add_chunk();
				refs_.add_chunk();
			}

			if ((nb_max_lines_ + PRIMSIZE) % CHUNKSIZE < PRIMSIZE) // prim does not fit on current chunk? -> add chunk
			{
				// nb_max_lines_ = refs_.get_nb_chunks() * CHUNKSIZE; // next index will be at start of new chunk

				for (auto arr : table_arrays_)
					arr->add_chunk();
				for (auto arr : table_marker_arrays_)
					arr->add_chunk();
				refs_.add_chunk();
			}

			index = nb_max_lines_;
			nb_max_lines_ += PRIMSIZE;
		}
		else
		{
			index = holes_stack_.head();
			holes_stack_.pop();
		}

		// mark lines as used
		for(unsigned int i = 0u; i < PRIMSIZE; ++i)
			refs_.set_value(index + i, 1u); // do not use [] in case of refs_ is bool

		nb_used_lines_ += PRIMSIZE;

		return index;
	}

	/**
	* @brief remove a group of PRIMSIZE lines in the container
	* @param index index of one line of group to remove
	*/
	template <unsigned int PRIMSIZE>
	void remove_lines(unsigned int index)
	{
		unsigned int begin_prim_idx = (index / PRIMSIZE) * PRIMSIZE;

		cgogn_message_assert(used(begin_prim_idx), "Error removing non existing index");

		holes_stack_.push(begin_prim_idx);

		// mark lines as unused
		for(unsigned int i = 0u; i < PRIMSIZE; ++i)
			refs_.set_value(begin_prim_idx++, 0u); // do not use [] in case of refs_ is bool

		nb_used_lines_ -= PRIMSIZE;
	}

	/**
	 * @brief initialize a line of the container (an element of each attribute)
	 * @param index line index
	 */
	void init_line(unsigned int index)
	{
		cgogn_message_assert(used(index), "init_line only with allocated lines");

		for (auto ptr : table_arrays_)
			ptr->init_element(index);
	}

	/**
	 * @brief initialize the markers of a line of the container
	 * @param index line index
	 */
	void init_markers_of_line(unsigned int index)
	{
		cgogn_message_assert(used(index), "init_markers_of_line only with allocated lines");

		for (auto ptr : table_marker_arrays_)
			ptr->set_false(index);
	}

	/**
	 * @brief copy the content of line src in line dst (with refs & markers)
	 * @param dstIndex destination
	 * @param srcIndex source
	 */
	void copy_line(unsigned int dst, unsigned int src)
	{
		for (auto ptr : table_arrays_)
			ptr->copy_element(dst, src);

		for (auto ptr : table_marker_arrays_)
			ptr->copy_element(dst, src);

		refs_[dst] = refs_[src];
	}

	/**
	* @brief increment the reference counter of the given line (only for PRIMSIZE==1)
	* @param index index of the line
	*/
	void ref_line(unsigned int index)
	{
		// static_assert(PRIMSIZE == 1u, "refLine with container where PRIMSIZE!=1");
		refs_[index]++;
	}

	/**
	* @brief decrement the reference counter of the given line (only for PRIMSIZE==1)
	* @param index index of the line
	* @return true if the line was removed
	*/
	bool unref_line(unsigned int index)
	{
		// static_assert(PRIMSIZE == 1u, "unrefLine with container where PRIMSIZE!=1");
		cgogn_message_assert(refs_[index] > 1u, "Container: unref line with nb_ref == 1");

		refs_[index]--;
		if (refs_[index] == 1u)
		{
			holes_stack_.push(index);
			refs_[index] = 0u;
			--nb_used_lines_;
			return true;
		}
		return false;
	}

	/**
	* @brief get the number of references of the given line
	* @param index index of the line
	* @return number of references of the line
	*/
	T_REF get_nb_refs(unsigned int index) const
	{
		// static_assert(PRIMSIZE == 1u, "getNbRefs with container where PRIMSIZE!=1");
		return refs_[index];
	}

	void save(std::ostream& fs)
	{
		cgogn_assert(fs.good());

		// save info (size+used_lines+max_lines+sizeof names)
		std::vector<unsigned int> buffer;
		buffer.reserve(1024);
		buffer.push_back(static_cast<unsigned int>(table_arrays_.size()));
		buffer.push_back(nb_used_lines_);
		buffer.push_back(nb_max_lines_);

		for(unsigned int i = 0u; i < table_arrays_.size(); ++i)
		{
			buffer.push_back(static_cast<unsigned int>(names_[i].size()+1));
			buffer.push_back(static_cast<unsigned int>(type_names_[i].size()+1));
		}

		fs.write(reinterpret_cast<const char*>(&(buffer[0])), std::streamsize(buffer.size()*sizeof(unsigned int)));

		// save names
		for(unsigned int i = 0; i < table_arrays_.size(); ++i)
		{
			const char* s1 = names_[i].c_str();
			const char* s2 = type_names_[i].c_str();
			fs.write(s1, std::streamsize((names_[i].size()+1u)*sizeof(char)));
			fs.write(s2, std::streamsize((type_names_[i].size()+1u)*sizeof(char)));
		}

		// save chunk arrays
		for(unsigned int i = 0u; i < table_arrays_.size(); ++i)
		{
			table_arrays_[i]->save(fs, nb_max_lines_);
		}

		// save uses/refs
		refs_.save(fs, nb_max_lines_);

		// save stack
		holes_stack_.save(fs, holes_stack_.size());
	}

	bool load(std::istream& fs)
	{
		cgogn_assert(fs.good());

		// check and register all known types if necessaey
		ChunkArrayFactory<CHUNKSIZE>::register_known_types();

		// read info
		unsigned int buff1[4];
		fs.read(reinterpret_cast<char*>(buff1), 3u*sizeof(unsigned int));

		nb_used_lines_ = buff1[1];
		nb_max_lines_ = buff1[2];

		std::vector<unsigned int> buff2(2u*buff1[0]);
		fs.read(reinterpret_cast<char*>(&(buff2[0])), std::streamsize(2u*buff1[0]*sizeof(unsigned int)));

		names_.resize(buff1[0]);
		type_names_.resize(buff1[0]);

		// read name
		char buff3[256];
		for(unsigned int i = 0u; i < buff1[0]; ++i)
		{
			fs.read(buff3, std::streamsize(buff2[2u*i]*sizeof(char)));
			names_[i] = std::string(buff3);

			fs.read(buff3, std::streamsize(buff2[2u*i+1u]*sizeof(char)));
			type_names_[i] = std::string(buff3);
		}
		cgogn_assert(fs.good());
		// read chunk array
		table_arrays_.reserve(buff1[0]);
		bool ok = true;
		for (unsigned int i = 0u; i < names_.size();)
		{
			ChunkArrayGen* cag = ChunkArrayFactory<CHUNKSIZE>::create(type_names_[i]);
			if (cag)
			{
				table_arrays_.push_back(cag);
				ok &= table_arrays_.back()->load(fs);
				++i;
			}
			else
			{
				std::cerr << "ChunkArrayContainer: could not load attribute" << names_[i] << " of type "<< type_names_[i] << std::endl;
				type_names_.erase(type_names_.begin()+i);
				names_.erase(names_.begin()+i);
				ChunkArrayGen::skip(fs);
			}
		}
		ok &= refs_.load(fs);

		return ok;
	}
};



#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CORE_CONTAINER_CHUNK_ARRAY_CONTAINER_CPP_))
extern template class CGOGN_CORE_API ChunkArrayContainer<DEFAULT_CHUNK_SIZE, unsigned int>;
extern template class CGOGN_CORE_API ChunkArrayContainer<DEFAULT_CHUNK_SIZE, unsigned char>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CORE_CONTAINER_CHUNK_ARRAY_CONTAINER_CPP_))

} // namespace cgogn

#endif // CORE_CONTAINER_CHUNK_ARRAY_CONTAINER_H_
