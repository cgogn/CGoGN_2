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

#ifndef __CORE_CONTAINER_CHUNK_ARRAY_CONTAINER_H__
#define __CORE_CONTAINER_CHUNK_ARRAY_CONTAINER_H__

#include <core/basic/nameTypes.h>
#include <core/container/chunk_array.h>
#include <core/container/chunk_stack.h>
#include <core/container/chunk_array_factory.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <string>
#include <memory>
#include <core/basic/assert.h>
#include <utils/make_unique.h>

namespace cgogn
{

class ContainerBrowser
{
public:

	virtual unsigned int begin() const = 0;
	virtual unsigned int end() const = 0;
	virtual void next(unsigned int &it) const = 0;
	virtual void nextPrimitive(unsigned int &it, unsigned int primSz) const = 0;
	virtual void enable() = 0;
	virtual void disable() = 0;
	virtual ~ContainerBrowser() {}
};

template <typename CONTAINER>
class ContainerStandardBrowser:public ContainerBrowser
{
	const CONTAINER* cac_;

public:

	ContainerStandardBrowser(const CONTAINER* cac) : cac_(cac) {}
	virtual unsigned int begin() const { return cac_->realBegin(); }
	virtual unsigned int end() const { return cac_->realEnd(); }
	virtual void next(unsigned int &it)  const { cac_->realNext(it); }
	virtual void nextPrimitive(unsigned int &it, unsigned int primSz) const { cac_->realNextPrimitive(it,primSz); }
	virtual void enable() {}
	virtual void disable() {}
};


/**
 * @brief class that manage the storage of several ChunkArray
 * @tparam CHUNKSIZE chunk size for ChunkArray
 * @detail
 *
 */
template <unsigned int CHUNKSIZE, typename T_REF>
class ChunkArrayContainer
{
public:

	/**
	* constante d'attribut inconnu
	*/
	static const unsigned int UNKNOWN = 0xffffffff;

protected:

	/**
	* vector of pointers to ChunkVector
	*/
	std::vector<ChunkArrayGen<CHUNKSIZE>*> tableArrays_;

	std::vector<std::string> names_;

	std::vector<std::string> typeNames_;

	ChunkArray<CHUNKSIZE, T_REF> refs_;

	/**
	 * stack of holes
	 */
	ChunkStack<CHUNKSIZE, unsigned int> holesStack_;

	/**
	* size (number of elts) of the container
	*/
	unsigned int nbUsedLines_;

	/**
	* size of the container with holes (also index of next inserted line if no holes)
	*/
	unsigned int nbMaxLines_;

	/**
	 * @brief number of bool attribs (which are alway in front of all others)
	 */
	unsigned int nbBoolAttribs_;

	/**
	 * Browser that allow special traversals
	 */
	ContainerBrowser* currentBrowser_;

	/**
	 * Browser that allow special traversals
	 */
	std::unique_ptr< ContainerStandardBrowser< ChunkArrayContainer<CHUNKSIZE, T_REF> > > stdBrowser_;

	/**
	 * @brief get array index from name
	 * @warning do not store index (not stable)
	 * @param attribName name of ChunkArray
	 * @return the index in table
	 */
	unsigned int getArrayIndex(const std::string& attribName) const
	{
		for (unsigned int i=0; i != names_.size(); ++i)
		{
			if (names_[i] == attribName)
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
	unsigned int getArrayIndex(const ChunkArrayGen<CHUNKSIZE>* ptr) const
	{
		for (unsigned int i=0u; i != tableArrays_.size(); ++i)
		{
			if (tableArrays_[i] == ptr)
				return i;
		}
		return UNKNOWN;
	}

	/**
	 * @brief remove an attribute by its name
	 * @param attribName name of attribute to remove
	 * @return true if attribute exist and has been removed
	 */
	bool removeAttribute(unsigned int index)
	{
		// store ptr for using it before delete
		ChunkArrayGen<CHUNKSIZE>* ptrToDel = tableArrays_[index];

		// in case of bool, keep booleans first !
		if (tableArrays_[index]->isBooleanArray())
		{
			nbBoolAttribs_--;

			if (index < nbBoolAttribs_)	// if attribute is not last of boolean
			{
				tableArrays_[index] = tableArrays_[nbBoolAttribs_];	// copy last of boolean on index
				names_[index]       = names_[nbBoolAttribs_];
				typeNames_[index]   = typeNames_[nbBoolAttribs_];
			}
			// now overwrite last of bool with last
			index = nbBoolAttribs_;
		}

		if (index != tableArrays_.size()- std::size_t(1u))
		{
			tableArrays_[index] = tableArrays_.back();
			names_[index]       = names_.back();
			typeNames_[index]   = typeNames_.back();
		}

		tableArrays_.pop_back();
		names_.pop_back();
		typeNames_.pop_back();

		delete ptrToDel ;


		return true ;
	}

public:

	/**
	 * @brief ChunkArrayContainer constructor
	 */
	ChunkArrayContainer():
        nbUsedLines_(0u)
        ,nbMaxLines_(0u)
        ,stdBrowser_(make_unique< ContainerStandardBrowser<ChunkArrayContainer<CHUNKSIZE, T_REF>> >(this))
	{
		currentBrowser_= stdBrowser_.get();
	}

    ChunkArrayContainer(ChunkArrayContainer<CHUNKSIZE, T_REF>const& ) = delete;
    ChunkArrayContainer(ChunkArrayContainer<CHUNKSIZE, T_REF>&& ) = delete;
    ChunkArrayContainer& operator=(ChunkArrayContainer<CHUNKSIZE, T_REF>const& ) = delete;
    ChunkArrayContainer& operator=(ChunkArrayContainer<CHUNKSIZE, T_REF>&& ) = delete;

	/**
	 * @brief ChunkArrayContainer destructor
	 */
	~ChunkArrayContainer()
	{
		if (currentBrowser_ != stdBrowser_.get())
			delete currentBrowser_;
		for (auto ptr: tableArrays_)
			delete ptr;
	}

	template <typename T>
	ChunkArray<CHUNKSIZE,T>* getAttribute(const std::string& attribName)
	{

		// first check if attribute already exist
		unsigned int index = getArrayIndex(attribName) ;
		if (index == UNKNOWN)
		{
			std::cerr << "attribute " << attribName << " not found." << std::endl ;
			return nullptr ;
		}

		return static_cast<ChunkArray<CHUNKSIZE,T>*>(tableArrays_[index]);
	}

	/**
	 * @brief add an attribute
	 * @param attribName name of attribute
	 * @tparam T type of attribute
	 * @return pointer on created ChunkArray
	 */
	template <typename T>
	ChunkArray<CHUNKSIZE,T>* addAttribute(const std::string& attribName)
	{
		// assert(attribName.size() != 0);
		cgogn_assert(attribName.size() != 0);

		// first check if attribute already exist
		unsigned int index = getArrayIndex(attribName) ;
		if (index != UNKNOWN)
		{
			std::cerr << "attribute " << attribName << " already found.." << std::endl ;
			return nullptr ;
		}

		// create the new attribute
		const std::string& typeName = nameOfType(T()) ;
		ChunkArray<CHUNKSIZE,T>* carr = new ChunkArray<CHUNKSIZE,T>() ;
		ChunkArrayFactory<CHUNKSIZE>::template registerCA<T>();

		// reserve memory
		carr->setNbChunks(refs_.getNbChunks()) ;

		// store pointer, name & typename.
		tableArrays_.push_back(carr) ;
		names_.push_back(attribName);
		typeNames_.push_back(typeName);

		// move bool in front of others
		if (std::is_same<bool, T>::value)
		{
			if (tableArrays_.size() > nbBoolAttribs_)
			{
				// swap ptrs
				auto tmp = tableArrays_.back();
				tableArrays_.back() = tableArrays_[nbBoolAttribs_];
				tableArrays_[nbBoolAttribs_] = tmp;
				// swap names & typenames
				names_.back().swap(names_[nbBoolAttribs_]);
				typeNames_.back().swap(typeNames_[nbBoolAttribs_]);
			}
			nbBoolAttribs_++;

		}

		return carr ;
	}

	/**
	 * @brief remove an attribute by its name
	 * @param attribName name of attribute to remove
	 * @return true if attribute exist and has been removed
	 */
	bool removeAttribute(const std::string& attribName)
	{
		unsigned int index = getArrayIndex(attribName) ;

		if (index == UNKNOWN)
		{
			std::cerr << "removeAttribute by name: attribute not found (" << attribName << ")"<< std::endl ;
			return false ;
		}

		removeAttribute(index);

		return true;
	}

	/**
	 * @brief remove an attribute by its name
	 * @param attribName name of attribute to remove
	 * @return true if attribute exist and has been removed
	 */
	bool removeAttribute(const ChunkArrayGen<CHUNKSIZE>* ptr)
	{
		unsigned int index = getArrayIndex(ptr) ;

		if (index == UNKNOWN)
		{
			std::cerr << "removeAttribute by ptr: attribute not found (" << std::endl ;
			return false ;
		}

		removeAttribute(index);

		return true;
	}

	/**
	 * @brief Number of attributes of the container
	 * @return number of attributes
	 */
	unsigned int getNbAttributes() const
	{
		return tableArrays_.size();
	}

	/**
	 * @brief size (number of used lines)
	 * @return the number of lines
	 */
	unsigned int size() const
	{
		return nbUsedLines_;
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
	* @brief is a line used
	* @param index index of line
	* @return true if used
	*/
	bool used(unsigned int index) const
	{
		return refs_[index] != 0;
	}
	/**
	 * @brief setCurrentBrowser
	 * @param browser, pointer to a heap-allocated ContainerBrowser
	 */
	inline void setCurrentBrowser(ContainerBrowser* browser)
	{
		if (currentBrowser_ != stdBrowser_.get())
			delete currentBrowser_;
		currentBrowser_ = browser;
	}

	inline void setStandardBrowser()
	{
		if (currentBrowser_ != stdBrowser_.get())
			delete currentBrowser_;
		currentBrowser_ = stdBrowser_;
	}


	/**
	 * @brief begin
	 * @return the index of the first used line of the container
	 */
	inline unsigned int begin() const
	{
		return currentBrowser_->begin();
	}

	/**
	 * @brief end
	 * @return the index after the last used line of the container
	 */
	inline unsigned int end() const
	{
		return currentBrowser_->end();
	}

	/**
	 * @brief next it <- next used index in the container
	 * @param it index to "increment"
	 */
	inline void next(unsigned int &it) const
	{
		currentBrowser_->next(it);
	}

	/**
	 * @brief next primitive: it <- next primitive used index in the container (eq to PRIMSIZE next)
	 * @param it index to "increment"
	 */
	inline void nextPrimitive(unsigned int &it, unsigned int primSz) const
	{
		currentBrowser_->nextPrimitive(it, primSz);
	}

	/**
	 * @brief begin of container without browser
	 * @return
	 */
	inline unsigned int realBegin() const
	{
		unsigned int it = 0u;
		while ((it < nbMaxLines_) && (!used(it)))
			++it;
		return it;
	}

	/**
	 * @brief end of container without browser
	 * @return
	 */
	inline unsigned int realEnd() const
	{
		return nbMaxLines_;
	}

	/**
	 * @brief next without browser
	 * @param it
	 */
	inline void realNext(unsigned int &it) const
	{
		do
		{
			++it;
		} while ((it < nbMaxLines_) && (!used(it)));
	}

	/**
	 * @brief next primitive without browser
	 * @param it
	 */
	inline void realNextPrimitive(unsigned int &it, unsigned int primSz) const
	{
		do
		{
			it+=primSz;
		} while ((it < nbMaxLines_) && (!used(it)));
	}

	/**
	 * @brief real reverse begin
	 * @return rbegin()
	 */
	unsigned int realRBegin() const
	{
		unsigned int it = nbMaxLines_-1u;
		while ((it != 0xffffffff) && (!used(it)))
			--it;
		return it;
	}

	/**
	 * return the index before the first line of the container
	 */
	/**
	 * @brief real reverse end
	 * @return rend()
	 */
	unsigned int realREnd() const
	{
		return 0xffffffff; // -1
	}

	/**
	 * @brief real next of reverse index
	 * @param it reverse index
	 */
	void realRNext(unsigned int &it) const
	{
		do
		{
			--it;
		} while ((it !=0xffffffff) && (!used(it)));
	}

	/**
	 * @brief clear the container
	 * @param removeAttrib remove the attributes (not only their data)
	 */
	void clear(bool removeAttrib = false)
	{
		nbUsedLines_ = 0u;
		nbMaxLines_ = 0u;

		// clear CA of refs
		refs_.clear();

		// clear holes
		holesStack_.clear();

		//clear data
		for (auto arr: tableArrays_)
			arr->clear();

		// remove CA ?
		if (removeAttrib)
		{
			for (auto arr: tableArrays_)
				delete arr;
			tableArrays_.clear();
		}
	}

	/**
	 * @brief fragmentation of container (size/index of last lines): 100% = no holes
	 * @return 1 if full filled - 0 is lots of holes
	 */
	float fragmentation() const
	{
		return float(size()) / float(realEnd());
	}

	/**
	 * @brief container compacting
	 * @param mapOldNew table that contains a map from old indices to new indices (holes -> 0xffffffff)
	 */
	template <unsigned int PRIMSIZE>
	void compact(std::vector<unsigned int>& mapOldNew)
	{
		mapOldNew.clear();
		mapOldNew.resize(realEnd(),0xffffffff);

		unsigned int up = realRBegin();
		unsigned int down = 0u;

		while (down < up)
		{
			if (!used(down))
			{
				for(unsigned int i=0u; i<PRIMSIZE;++i)
				{
					unsigned rdown = down + PRIMSIZE-1u - i;
					mapOldNew[up] = rdown;
					copyLine(rdown,up);
					realRNext(up);
				}
				down += PRIMSIZE;
			}
			else
				down++;
		}

		nbMaxLines_ = nbUsedLines_;

		// free unused memory blocks
		unsigned int newNbBlocks = nbMaxLines_/CHUNKSIZE + 1u;
		for (auto arr: tableArrays_)
			arr->setNbChunks(newNbBlocks);
		refs_.setNbChunks(newNbBlocks);

		// clear holes
		holesStack_.clear();
	}

	/**************************************
	 *          LINES MANAGEMENT          *
	 **************************************/

	/**
	* @brief insert a group of PRIMSIZE consecutive lines in the container
	* @return index of the first line of group
	*/
	template <unsigned int PRIMSIZE>
	unsigned int insertLines()
	{
		unsigned int index;

		if (holesStack_.empty()) // no holes -> insert at the end
		{
			index = nbMaxLines_;
			nbMaxLines_ += PRIMSIZE;

			if (nbMaxLines_%CHUNKSIZE <= PRIMSIZE) // prim on next block ? -> add block to C.A.
			{
				for (auto arr: tableArrays_)
					arr->addChunk();
				refs_.addChunk();
			}
		}
		else
		{
			index = holesStack_.head();
			holesStack_.pop();
		}

		// mark lines as used
		for(unsigned int i=0u; i<PRIMSIZE; ++i)
			refs_.setVal(index+i,1u); // do not used [] in case of refs_ is bool

		nbUsedLines_ += PRIMSIZE;

		return index;
	}

	/**
	* @brief remove a group of PRIMSIZE lines in the container
	* @param index index of one line of group to remove
	*/
	template <unsigned int PRIMSIZE>
	void removeLines(unsigned int index)
	{
		unsigned int beginPrimIdx = (index/PRIMSIZE) * PRIMSIZE;

		// assert(this->used(beginPrimIdx)|!" Error removing non existing index");
		cgogn_message_assert(this->used(beginPrimIdx), "Error removing non existing index");

		holesStack_.push(beginPrimIdx);

		// mark lines as unused
		for(unsigned int i=0u; i<PRIMSIZE; ++i)
			refs_.setVal(beginPrimIdx++,0u);// do not used [] in case of refs_ is bool

		nbUsedLines_ -= PRIMSIZE;
	}

	/**
	 * @brief initialize a line of the container (an element of each attribute)
	 * @param index line index
	 */
	void initLine(unsigned int index)
	{
		// assert( used(index) && "initLine only with allocated lines");
		cgogn_message_assert(!used(index), "initLine only with allocated lines");

		for (auto ptrAtt: tableArrays_)
//			if (ptrAtt != nullptr) never null !
				ptrAtt->initElt(index);
	}

	void initBooleansOfLine(unsigned int index)
	{
		// assert( used(index) && "initBooleansOfLine only with allocated lines");
		cgogn_message_assert(!used(index), "initBooleansOfLine only with allocated lines");

		for (unsigned int i=0; i<nbBoolAttribs_;++i)
			tableArrays_[i]->initElt(index);
	}

	void initBoolsOfLine(unsigned int index)
	{
//		assert( used(index) && "initLine only with allocated lines");
//		for (auto ptrAtt: tableArrays_)
//			if (ptrAtt != NULL)
//				ptrAtt->initElt(index);
	}

	/**
	 * @brief copy the content of line src in line dst (with refs)
	 * @param dstIndex destination
	 * @param srcIndex source
	 */
	void copyLine(unsigned int dstIndex, unsigned int srcIndex)
	{
		for (auto ptrAtt: tableArrays_)
			if (ptrAtt != nullptr)
				ptrAtt->copyElt(dstIndex, srcIndex);
		refs_[dstIndex] = refs_[srcIndex];
	}

	/**
	* @brief increment the reference counter of the given line (only for PRIMSIZE==1)
	* @param index index of the line
	*/
	void refLine(unsigned int index)
	{
//		static_assert(PRIMSIZE == 1u, "refLine with container where PRIMSIZE!=1");
		refs_[index]++;
	}

	/**
	* @brief decrement the reference counter of the given line (only for PRIMSIZE==1)
	* @param index index of the line
	* @return true if the line was removed
	*/
	bool unrefLine(unsigned int index)
	{
//		static_assert(PRIMSIZE == 1u, "unrefLine with container where PRIMSIZE!=1");
		refs_[index]--;
		if (refs_[index] == 1u)
		{
			holesStack_.push(index);
			refs_[index] = 0u;			// same as removeLine without the "if"
			--nbUsedLines_;
			return true;
		}
		return false;
	}

	/**
	* @brief get the number of references of the given line
	* @param index index of the line
	* @return number of references of the line
	*/
	T_REF getNbRefs(unsigned int index) const
	{
//		static_assert(PRIMSIZE == 1u, "getNbRefs with container where PRIMSIZE!=1");
		return refs_[index];
	}

	/**
	* @brief get a chunk_array
	* @param attribName name of the array
	* @return pointer on typed chunk_array
	*/
	template<typename T>
	ChunkArray<CHUNKSIZE,T>* getDataArray(const std::string& attribName)
	{
		unsigned int index = getArrayIndex(attribName) ;
		if(index == UNKNOWN)
			return nullptr ;

		ChunkArray<CHUNKSIZE,T>* atm = dynamic_cast<ChunkArray<CHUNKSIZE,T>*>(tableArrays_[index]);
		// assert((atm != nullptr) || !"getDataVector: wrong type");

		cgogn_message_assert(atm != nullptr, "getDataVector: wrong type");

		return atm;
	}

	/**
	* @brief get a chunk_array
	* @param attribName name of the array
	* @return pointer on virtual chunk_array
	*/
	ChunkArrayGen<CHUNKSIZE>* getVirtualDataArray(const std::string& attribName)
	{
		unsigned int index = getArrayIndex(attribName) ;
		if(index == UNKNOWN)
			return nullptr ;

		return tableArrays_[index];
	}

	void save(std::ofstream& fs)
	{
		// save info (size+used_lines+max_lines+sizeof names)
		std::vector<unsigned int> buffer;
		buffer.reserve(1024);
		buffer.push_back((unsigned int)(tableArrays_.size()));
		buffer.push_back(nbUsedLines_);
		buffer.push_back(nbMaxLines_);
		buffer.push_back(nbBoolAttribs_);
		for(unsigned int i=0u; i<tableArrays_.size(); ++i)
		{
			buffer.push_back(static_cast<unsigned int>(names_[i].size()+1));
			buffer.push_back(static_cast<unsigned int>(typeNames_[i].size()+1));
		}
		fs.write(reinterpret_cast<const char*>(&(buffer[0])),std::streamsize(buffer.size()*sizeof(unsigned int)));

		// save names
		for(unsigned int i=0; i<tableArrays_.size(); ++i)
		{
			const char* s1 = names_[i].c_str();
			const char* s2 = typeNames_[i].c_str();
			fs.write(s1,std::streamsize((names_[i].size()+1u)*sizeof(char)));
			fs.write(s2,std::streamsize((typeNames_[i].size()+1u)*sizeof(char)));
		}

		// save chunk arrays
		for(unsigned int i=0u; i<tableArrays_.size(); ++i)
		{
			tableArrays_[i]->save(fs,nbMaxLines_);
		}
		// save uses/refs
		refs_.save(fs,nbMaxLines_);

		// save stack
		holesStack_.save(fs,holesStack_.size());
	}

	bool load(std::ifstream& fs)
	{
		// read info
		unsigned int buff1[4];
		fs.read(reinterpret_cast<char*>(buff1),4u*sizeof(unsigned int));

		nbUsedLines_   = buff1[1];
		nbMaxLines_    = buff1[2];
		nbBoolAttribs_ = buff1[3];

		std::vector<unsigned int> buff2(2u*buff1[0]);
		fs.read(reinterpret_cast<char*>(&(buff2[0])),std::streamsize(2u*buff1[0]*sizeof(unsigned int)));

		names_.resize(buff1[0]);
		typeNames_.resize(buff1[0]);

		// read name
		char buff3[256];
		for(unsigned int i=0u; i<buff1[0]; ++i)
		{
			fs.read(buff3, std::streamsize(buff2[2u*i]*sizeof(char)));
			names_[i] = std::string(buff3);

			fs.read(buff3, std::streamsize(buff2[2u*i+1u]*sizeof(char)));
			typeNames_[i] = std::string(buff3);
		}

		// read chunk array
		tableArrays_.resize(buff1[0]);
		bool ok=true;
		for (unsigned int i=0u; i <buff1[0]; ++i)
		{
			tableArrays_[i] = ChunkArrayFactory<CHUNKSIZE>::create(typeNames_[i]);
			ok &= tableArrays_[i]->load(fs);
		}
		ok &= refs_.load(fs);

		return ok;
	}
};

} // namespace cgogn

#endif // __CORE_CONTAINER_CHUNK_ARRAY_CONTAINER_H__
