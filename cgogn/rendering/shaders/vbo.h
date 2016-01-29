
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

#ifndef RENDERING_VBO_H
#define RENDERING_VBO_H

#include <QOpenGLBuffer>
#include <core/cmap/map_base.h> // impossible to include directly attribute_handler.h !

namespace cgogn
{
namespace rendering
{

class VBO
{
protected:
	unsigned int nb_vecs_;
	unsigned int vec_dim_;

	QOpenGLBuffer buffer_;

public:

	inline VBO():
		nb_vecs_(0),vec_dim_(0)
	{
		buffer_.create();
		buffer_.bind();
		buffer_.setUsagePattern(QOpenGLBuffer::StreamDraw);
	}

	inline ~VBO()
	{
		buffer_.destroy();
	}

	inline void bind()
	{
		buffer_.bind();
	}

	inline void release()
	{
		buffer_.release();
	}

	/**
	 * @brief allocate VBO memory
	 * @param nbv number of vector
	 * @param nbc number of component of vector
	 */
    inline void allocate(int nbv, int nbc)
	{
		buffer_.bind();
		unsigned int total = nbv*nbc;
		if (total != nb_vecs_*vec_dim_) // > ?
			buffer_.allocate(total*4);
		vec_dim_ = nbc;
		nb_vecs_ = nbv;
		buffer_.release();
	}

	/**
	 * @brief allocate
	 * @param nbv number of vector
	 * @param nbc number of component of vector
	 * @param ptr memory pointer for direct copy
	 */
	inline void allocate(int nbv, int nbc, float* ptr)
	{
		buffer_.bind();
		unsigned int total = nbv*nbc;
		if (total != nb_vecs_*vec_dim_) // > ?
			buffer_.allocate(ptr,total*4);
		vec_dim_ = nbc;
		nb_vecs_ = nbv;
		buffer_.release();
	}

	/**
	 * @brief get and lock pointer on buffer memory
	 * @return  the pointer
	 */
	inline float* lock_pointer()
	{
		buffer_.bind();
		 return reinterpret_cast<float*>(buffer_.map(QOpenGLBuffer::ReadWrite));
	}

	/**
	 * @brief release_pointer
	 */
	inline void release_pointer()
	{
		buffer_.unmap();
	}

	/**
	 * @brief copy data
	 * @param offset offset in bytes in the bufffer
	 * @param nb number of bytes to copy
	 * @param src source pointer
	 */
	inline void copy_data(unsigned int offset, unsigned int nb, void* src)
	{
		buffer_.write(offset,src, nb);
	}

	/**
	 * @brief number of components of vectors stored in buffer
	 */
	inline int nb_comp()
	{
		return vec_dim_;
	}
};


template<typename ATTR>
void update_vbo( const ATTR& attr, VBO& vbo)
{
	const typename ATTR::TChunkArray* ca = attr.get_data();
	std::vector<void*> addr;
	unsigned int byte_block_size;
	unsigned int nbp = ca->get_chunks_pointers(addr, byte_block_size);

	if (std::is_same<typename ATTR::value_type::Scalar, float>::value)
	{
		// number of compo
		unsigned int nbc = sizeof(typename ATTR::value_type)/4;
		// number of vec
		unsigned int nbv = nbp*byte_block_size/sizeof(typename ATTR::value_type);
		// realloc
		vbo.allocate(nbv,nbc);
		// copy
		char* dst = reinterpret_cast<char*>(vbo.lock_pointer());
		for (unsigned int i=0; i<nbp; ++i)
		{
			memcpy(dst,addr[i],byte_block_size);
			dst += byte_block_size;
		}
		vbo.release_pointer();
	}
	else if (std::is_same<typename ATTR::value_type::Scalar, double>::value)
	{
		float* buffer_float = new float[byte_block_size/2];
		// number of compo
		unsigned int nbc = sizeof(typename ATTR::value_type)/8;
		// number of vec
		unsigned int nbv = nbp*byte_block_size/sizeof(typename ATTR::value_type);
		// realloc
		vbo.allocate(nbv,nbc);
		// copy (after conversion to float)
		char* dst = reinterpret_cast<char*>(vbo.lock_pointer());
		for (unsigned int i=0; i<nbp; ++i)
		{
			// transform double into float
			float* buf = buffer_float;
			double* src = reinterpret_cast<double*>(addr[i]);
			for (unsigned int j=0; j<byte_block_size/8; ++j)
				*buf++ = *src++;
			// copy to buffer
			memcpy(dst,buffer_float,byte_block_size/2);
			dst += byte_block_size;
		}
		vbo.release_pointer();
		delete[] buffer_float;
	}
	else
	{
		cgogn_message_assert(true,"only float or double allowed for vbo")
	}
}

} // namespace rendering
} // namespace cgogn


#endif // RENDERING_VBO_H
