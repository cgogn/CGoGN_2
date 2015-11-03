/*
 * CGoGN: Combinatorial and Geometric modeling with Generic N-dimensional Maps
 * Copyright (C) 2015, IGG Group, ICube, University of Strasbourg, France
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation; either version 2.1 of the License, or (at your
 * option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.
 *
 * Web site: http://cgogn.unistra.fr/
 * Contact information: cgogn@unistra.fr
 *
 */

#ifndef __CORE_CONTAINER_CHUNK_ARRAY__
#define __CORE_CONTAINER_CHUNK_ARRAY__

#include "core/container/chunk_array_gen.h"
#include <iostream>
#include <string>
#include <cstring>
#include <cassert>


namespace cgogn
{
/**
 *	@brief chunk array class storage
 *	@tparam CHUNKSIZE size of each chunk (in T, not in bytes!), must be a power of 2 >=32
 *	@tparam T type of stored data
 */
template <unsigned int CHUNKSIZE, typename T>
class ChunkArray : public ChunkArrayGen<CHUNKSIZE>
{
protected:

	/// vector of block pointers
	std::vector<T*> tableData_;

public:

	/**
	 * @brief Constructor of ChunkArray
	 */
	ChunkArray()
	{
		tableData_.reserve(1024);
	}

	/**
	 * @brief Destructor of ChunkArray
	 */
	~ChunkArray()
	{
		for(auto chunk: tableData_)
			delete[] chunk;
	}


	bool isBooleanArray() const
	{
		return false;
	}

	/**
	 * @brief create a ChunkArray<CHUNKSIZE,T>
	 * @return generic pointer
	 */
	ChunkArrayGen<CHUNKSIZE>* clone()
	{
		ChunkArrayGen<CHUNKSIZE>* ptr = new ChunkArray<CHUNKSIZE, T>();
		return ptr;
	}

	/**
	 * @brief add a chunk (T[CHUNKSIZE])
	 */
	void addChunk()
	{
		T* ptr = new T[CHUNKSIZE]();
		tableData_.push_back(ptr);
	}


	/**
	 * @brief set number of chunks
	 * @param nbc number of chunks
	 */
	void setNbChunks(unsigned int nbc)
	{
		if (nbc >= tableData_.size())
		{
			for (std::size_t i= tableData_.size(); i <nbc; ++i)
				addChunk();
		}
		else
		{
			for (std::size_t i = nbc; i < tableData_.size(); ++i)
				delete[] tableData_[i];
			tableData_.resize(nbc);
		}
	}


	/**
	 * @brief get the number of chunks of the array
	 * @return the number of chunks
	 */
	unsigned int getNbChunks() const
	{
		return (unsigned int)(tableData_.size());
	}

	/**
	 * @brief number of allocated elements
	 * @return  allocated lines
	 */
	unsigned int capacity() const
	{
		return (unsigned int)(tableData_.size())*CHUNKSIZE;
	}


	/**
	 * @brief clear
	 */
	void clear()
	{
		for(auto chunk: tableData_)
			delete[] chunk;
		tableData_.clear();
	}

	/**
	 * @brief ref operator []
	 * @param i index of element to access
	 * @return ref of element
	 */
	T& operator[](unsigned int i)
	{
		assert(i/CHUNKSIZE < tableData_.size());
		return tableData_[i / CHUNKSIZE][i % CHUNKSIZE];
	}

	/**
	 * @brief const ref operator []
	 * @param i index of element to access
	 * @return const ref of element
	 */
	const T& operator[](unsigned int i) const
	{
		assert(i/CHUNKSIZE < tableData_.size());
		return tableData_[i / CHUNKSIZE][i % CHUNKSIZE];
	}

	/**
	 * @brief set the value of an element (work also with bool
	 * @param i index of element to set
	 * @param v value
	 */
	void setVal(unsigned int i, const T& v)
	{
		assert(i/CHUNKSIZE < tableData_.size());
		tableData_[i / CHUNKSIZE][i % CHUNKSIZE] = v;
	}


	/**
	 * @brief get pointer on all chunks data
	 * @param addr vector to fill
	 * @param byteBlockSize filled with CHUNKSIZE*sizeof(T)
	 * @return addr.size()
	 */
	unsigned int getChunksPointers(std::vector<void*>& addr, unsigned int& byteBlockSize) const
	{
		byteBlockSize = CHUNKSIZE * sizeof(T);

		addr.reserve(tableData_.size());
		addr.clear();

		for (typename std::vector<T*>::const_iterator it = tableData_.begin(); it != tableData_.end(); ++it)
			addr.push_back(*it);

		return (unsigned int)(addr.size());
	}

	/**
	 * @brief init an element (overwrite with T())
	 * @param id index of element
	 */
	void initElt(unsigned int id)
	{
		tableData_[id / CHUNKSIZE][id % CHUNKSIZE] = T();
	}


	/**
	 * @brief copy element
	 * @param dst destination
	 * @param src source
	 */
	void copyElt(unsigned int dst, unsigned int src)
	{
		tableData_[dst / CHUNKSIZE][dst % CHUNKSIZE] = tableData_[src / CHUNKSIZE][src % CHUNKSIZE];
	}

	/**
	 * @brief swap two elements
	 * @param id1 idx first
	 * @param id2 idx second
	 */
	void swapElt(unsigned int id1, unsigned int id2)
	{
		T data = tableData_[id1 / CHUNKSIZE][id1 % CHUNKSIZE] ;
		tableData_[id1 / CHUNKSIZE][id1 % CHUNKSIZE] = tableData_[id2 / CHUNKSIZE][id2 % CHUNKSIZE] ;
		tableData_[id2 / CHUNKSIZE][id2 % CHUNKSIZE] = data ;
	}


//	void save(std::ostream& fs, unsigned int nbLines) const
//	{
//		unsigned int nbs[3];
//		nbs[0] = (unsigned int)(tableData_.size());
//		nbs[1] = nbLines;
//		nbs[2] = CHUNKSIZE*sizeof(T);

//		assert(nbLines/CHUNKSIZE <= tableData_.size());
//		// TODO: if (nbLines==0) nbLines=CHUNKSIZE*tableData_.size(); ??

//		fs.write(reinterpret_cast<const char*>(nbs),3*sizeof(unsigned int));

//		// no data -> finished
//		if (nbs[0] == 0)
//			return;

//		unsigned int nbca = nbs[0]-1;
//		// save data chunks except last
//		for(unsigned int i=0; i<nbca; ++i)
//		{
//			fs.write(reinterpret_cast<const char*>(tableData_[i]),CHUNKSIZE*sizeof(T));
//		}

//		// save last
//		unsigned nbl = nbLines - nbca*CHUNKSIZE;
//		fs.write(reinterpret_cast<const char*>(tableData_[nbca]),std::streamsize(nbl*sizeof(T)));
//	}


//	bool load(std::istream& fs)
//	{
//		unsigned int nbs[3];
//		fs.read(reinterpret_cast<char*>(nbs), 3*sizeof(unsigned int));

//		if (nbs[2] != CHUNKSIZE*sizeof(T))
//		{
//			std::cerr << "Error loading ChunkArray wrong CHUNKSIZE"<< std::endl;
//			return false;
//		}

//		this->setNbChunks(nbs[0]);

//		// no data -> finished
//		if (nbs[0] == 0)
//			return true;

//		// load data chunks except last
//		unsigned int nbca = nbs[0]-1;
//		for(unsigned int i = 0; i < nbca; ++i)
//			fs.read(reinterpret_cast<char*>(tableData_[i]),CHUNKSIZE*sizeof(T));

//		// load last chunk
//		unsigned int nbl = nbs[1] - CHUNKSIZE*nbca;
//		fs.read(reinterpret_cast<char*>(tableData_[nbca]),std::streamsize(nbl*sizeof(T)));

//		return true;
//	}


	void save(std::ostream& fs, unsigned int nbLines) const
	{
		assert(nbLines/CHUNKSIZE <= getNbChunks());

		fs.write(reinterpret_cast<const char*>(&nbLines),sizeof(unsigned int));

		// no data -> finished
		if (nbLines == 0)
			return;

		unsigned int nbc = getNbChunks() - 1;

		// save data chunks except last
		for(unsigned int i=0; i<nbc; ++i)
		{
			fs.write(reinterpret_cast<const char*>(tableData_[i]),CHUNKSIZE*sizeof(T));
		}

		// save last incomplete chunk
		unsigned nb = nbLines - nbc*CHUNKSIZE;
		fs.write(reinterpret_cast<const char*>(tableData_[nbc]),std::streamsize(nb*sizeof(T)));
	}


	bool load(std::istream& fs)
	{
		unsigned int nbLines;
		fs.read(reinterpret_cast<char*>(&nbLines), sizeof(unsigned int));

		// no data -> finished
		if (nbLines == 0)
			return true;

		// compute number of chunks
		unsigned int nbc = nbLines / CHUNKSIZE;
		if (nbLines % CHUNKSIZE != 0)
			nbc++;

		this->setNbChunks(nbc);

		// load data chunks except last
		nbc--;
		for(unsigned int i = 0; i < nbc; ++i)
			fs.read(reinterpret_cast<char*>(tableData_[i]),CHUNKSIZE*sizeof(T));

		// load last incomplete chunk
		unsigned int nb = nbLines - nbc*CHUNKSIZE;
		fs.read(reinterpret_cast<char*>(tableData_[nbc]),std::streamsize(nb*sizeof(T)));

		return true;
	}

};





/**
 * @brief specialized version of ChunkArray for bool data. One bit per bool
 */
template <unsigned int CHUNKSIZE>
class ChunkArray<CHUNKSIZE, bool> : public ChunkArrayGen<CHUNKSIZE>
{
protected:

	/// vector of block pointers
	std::vector<unsigned int*> tableData_;

public:

	ChunkArray()
	{
		tableData_.reserve(1024);
	}


	~ChunkArray()
	{
		for(auto chunk: tableData_)
			delete[] chunk;
	}

	bool isBooleanArray() const
	{
		return true;
	}


	ChunkArrayGen<CHUNKSIZE>* clone()
	{
		ChunkArrayGen<CHUNKSIZE>* ptr = new ChunkArray<CHUNKSIZE, bool>;
		return ptr;
	}


	void addChunk()
	{
		unsigned int* ptr = new unsigned int[CHUNKSIZE/32]();
		tableData_.push_back(ptr);
	}



	void setNbChunks(unsigned int nbc)
	{
		if (nbc >= tableData_.size())
		{
			for (std::size_t i= tableData_.size(); i <nbc; ++i)
				addChunk();
		}
		else
		{
			for (std::size_t i = nbc; i < tableData_.size(); ++i)
				delete[] tableData_[i];
			tableData_.resize(nbc);
		}
	}



	unsigned int getNbChunks() const
	{
		return (unsigned int)(tableData_.size());
	}


	unsigned int capacity() const
	{
		return (unsigned int)(tableData_.size())*CHUNKSIZE/32;
	}


	void clear()
	{
		for(auto chunk: tableData_)
			delete[] chunk;
		tableData_.clear();
	}

	void setFalse(unsigned int i)
	{
		unsigned int jj = i / CHUNKSIZE;
		assert(jj < tableData_.size());
		unsigned int j = i % CHUNKSIZE;
		unsigned int x = j/32;
		unsigned int y = j%32;
		unsigned int mask = (unsigned int)(1) << y;
		tableData_[jj][x] &= ~mask;
	}

	void setTrue(unsigned int i)
	{
		unsigned int jj = i / CHUNKSIZE;
		assert(jj < tableData_.size());
		unsigned int j = i % CHUNKSIZE;
		unsigned int x = j/32;
		unsigned int y = j%32;
		unsigned int mask = (unsigned int)(1) << y;
		tableData_[jj][x] |= mask;
	}

	void setVal(unsigned int i, bool b)
	{
		unsigned int jj = i / CHUNKSIZE;
		assert(jj < tableData_.size());
		unsigned int j = i % CHUNKSIZE;
		unsigned int x = j/32;
		unsigned int y = j%32;
		unsigned int mask = (unsigned int)(1) << y;

		if (b)
			tableData_[jj][x] |= mask;
		else
			tableData_[jj][x] &= ~mask;
	}

	/**
	 * @brief special optimized version of setFalse when goal is to set all to false;
	 * @param i index of element to set to false
	 *
	 * This version overwrite element AND SOME OF THIS NEIGHBOURS with 0
	 * Use only if final goal is to set all array to 0 (MarkerStore)
	 * @todo find another name for the method!
	 */
	void setFalseDirty(unsigned int i)
	{
		unsigned int jj = i / CHUNKSIZE;
		assert(jj < tableData_.size());
		unsigned int j = (i % CHUNKSIZE)/32;
		tableData_[jj][j] = 0;
	}



	bool operator[](unsigned int i) const
	{
		unsigned int jj = i / CHUNKSIZE;
		assert(jj < tableData_.size());
		unsigned int j = i % CHUNKSIZE;
		unsigned int x = j/32;
		unsigned int y = j%32;

		unsigned int mask = (unsigned int)(1) << y;

		return (tableData_[jj][x] & mask) != 0;
	}



	unsigned int getChunksPointers(std::vector<void*>& addr, unsigned int& byteBlockSize) const
	{
		byteBlockSize = CHUNKSIZE / 8;

		addr.reserve(tableData_.size());
		addr.clear();

		for (typename std::vector<unsigned int*>::const_iterator it = tableData_.begin(); it != tableData_.end(); ++it)
			addr.push_back(*it);

		return (unsigned int)(addr.size());
	}


	void initElt(unsigned int id)
	{
		setFalse(id);
	}


	void copyElt(unsigned int dst, unsigned int src)
	{
		setVal(dst,this->operator [](src));
	}


	void swapElt(unsigned int id1, unsigned int id2)
	{
		bool data = this->operator [](id1);
		setVal(id1,this->operator [](id2));
		setVal(id2,data);
	}


	void save(std::ostream& fs, unsigned int nbLines) const
	{
		// round nbLines to 32 multiple
		if (nbLines%32)
			nbLines = ((nbLines/32)+1)*32;

		assert(nbLines/CHUNKSIZE <= tableData_.size());
		// TODO: if (nbLines==0) nbLines=CHUNKSIZE*tableData_.size(); ??

		// save number of lines
		fs.write(reinterpret_cast<const char*>(&nbLines),sizeof(unsigned int));

		// no data -> finished
		if (nbLines == 0)
			return;

		unsigned int nbc = getNbChunks()-1;
		// save data chunks except last
		for(unsigned int i=0; i<nbc; ++i)
		{
			fs.write(reinterpret_cast<const char*>(tableData_[i]),CHUNKSIZE/8);// /8 because bool = 1 bit & octet = 8 bit
		}

		// save last
		unsigned int nb = nbLines - nbc*CHUNKSIZE;
		fs.write(reinterpret_cast<const char*>(tableData_[nbc]),nb/8);
	}


	bool load(std::istream& fs)
	{
		// get number of lines to load
		unsigned int nbLines;
		fs.read(reinterpret_cast<char*>(&nbLines), sizeof(unsigned int));

		// no data -> finished
		if (nbLines == 0)
			return true;

		// compute number of chunks
		unsigned int nbc = nbLines / CHUNKSIZE;
		if (nbLines % CHUNKSIZE != 0)
			nbc++;

		this->setNbChunks(nbc);

		// load data chunks except last
		nbc--;
		for(unsigned int i = 0; i < nbc; ++i)
			fs.read(reinterpret_cast<char*>(tableData_[i]),CHUNKSIZE/8);// /8 because bool = 1 bit & octet = 8 bit

		// load last chunk
		unsigned int nb = nbLines - nbc*CHUNKSIZE;
		fs.read(reinterpret_cast<char*>(tableData_[nbc]),nb/8);

		return true;
	}


};



} // namespace CGoGN

#endif
