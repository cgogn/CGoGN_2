
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

#ifndef RENDERING_SHADERS_VBO_H_
#define RENDERING_SHADERS_VBO_H_

#include <QOpenGLBuffer>

#include <core/cmap/attribute_handler.h>
#include <geometry/types/geometry_traits.h>

namespace cgogn
{

namespace rendering
{

class VBO
{
protected:

	unsigned int nb_vectors_;
	unsigned int vector_dimension_;
	QOpenGLBuffer buffer_;

public:

	inline VBO() :
		nb_vectors_(0),
		vector_dimension_(0)
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
	 * @param nb_vectors number of vectors
	 * @param vector_dimension_ number of component of each vector
	 */
	inline void allocate(int nb_vectors, int vector_dimension)
	{
		buffer_.bind();
		unsigned int total = nb_vectors * vector_dimension;
		if (total != nb_vectors_ * vector_dimension_) // only allocate when > ?
			buffer_.allocate(total * sizeof(float));
		nb_vectors_ = nb_vectors;
		vector_dimension_ = vector_dimension;
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
		buffer_.write(offset, src, nb);
	}

	/**
	 * @brief dimension of vectors stored in buffer
	 */
	inline int vector_dimension()
	{
		return vector_dimension_;
	}
};

template <typename ATTR>
void update_vbo(const ATTR& attr, VBO& vbo)
{
	const typename ATTR::TChunkArray* ca = attr.get_data();

	std::vector<void*> chunk_addr;
	unsigned int byte_chunk_size;
	unsigned int nb_chunks = ca->get_chunks_pointers(chunk_addr, byte_chunk_size);
	unsigned int vec_dim = geometry::vector_traits<typename ATTR::value_type>::SIZE;

	vbo.allocate(nb_chunks * ATTR::CHUNKSIZE, vec_dim);

	if (std::is_same<typename geometry::vector_traits<typename ATTR::value_type>::Scalar, float>::value)
	{
		// copy
		char* dst = reinterpret_cast<char*>(vbo.lock_pointer());
		for (unsigned int i = 0; i < nb_chunks; ++i)
		{
			memcpy(dst, chunk_addr[i], byte_chunk_size);
			dst += byte_chunk_size;
		}
		vbo.release_pointer();
	}
	else if (std::is_same<typename geometry::vector_traits<typename ATTR::value_type>::Scalar, double>::value)
	{
		// copy (after conversion to float)
		char* dst = reinterpret_cast<char*>(vbo.lock_pointer());
		float* float_buffer = new float[ATTR::CHUNKSIZE * vec_dim];
		for (unsigned int i = 0; i < nb_chunks; ++i)
		{
			// transform double into float
			float* fit = float_buffer;
			double* src = reinterpret_cast<double*>(chunk_addr[i]);
			for (unsigned int j = 0; j < ATTR::CHUNKSIZE * vec_dim; ++j)
				*fit++ = *src++;
			// copy
			memcpy(dst, float_buffer, ATTR::CHUNKSIZE * vec_dim * sizeof(float));
			dst += ATTR::CHUNKSIZE * vec_dim * sizeof(float);
		}
		vbo.release_pointer();
		delete[] float_buffer;
	}
	else
	{
		cgogn_assert_not_reached("only float or double allowed for vbo");
	}
}

} // namespace rendering

} // namespace cgogn

#endif // RENDERING_SHADERS_VBO_H_
