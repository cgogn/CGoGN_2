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

#ifndef CGOGN_IO_IO_UTILS_H_
#define CGOGN_IO_IO_UTILS_H_

#include <type_traits>
#include <sstream>
#include <streambuf>

#include <cgogn/core/utils/logger.h>
#include <cgogn/core/utils/endian.h>
#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/io/dll.h>

namespace cgogn
{

namespace io
{

enum FileType
{
	FileType_UNKNOWN = 0,
	FileType_OFF,
	FileType_OBJ,
	FileType_PLY,
	FileType_VTK_LEGACY,
	FileType_VTU,
	FileType_VTP,
	FileType_MESHB,
	FileType_MSH,
	FileType_TETGEN,
	FileType_NASTRAN,
	FileType_AIMATSHAPE
};

enum DataType
{
	CHAR = 0,
	INT8,
	UINT8,
	INT16,
	UINT16,
	INT32,
	UINT32,
	INT64,
	UINT64,
	FLOAT,
	DOUBLE,
	UNKNOWN
};
CGOGN_IO_API FileType get_file_type(const std::string& filename);
CGOGN_IO_API DataType get_data_type(const std::string& type_name);
CGOGN_IO_API std::vector<unsigned char> base64_decode(const char* input, std::size_t begin, std::size_t length = std::numeric_limits<std::size_t>::max());

#ifdef CGOGN_WITH_ZLIB
CGOGN_IO_API std::vector<unsigned char> zlib_decompress(const char* input, DataType header_type);
#endif

namespace internal
{

// #1 return default value when U and T don't have the same nb of components.
template<typename U, typename T>
inline auto convert(const T&) -> typename std::enable_if<!std::is_same< std::integral_constant<uint32, geometry::nb_components_traits<T>::value>, std::integral_constant<uint32, geometry::nb_components_traits<U>::value>>::value,U>::type
{
	cgogn_log_warning("convert") << "Cannot convert data of type\"" << name_of_type(T()) << "\" to type \"" << name_of_type(U()) << "\".";
	return U();
}

// #2 cast x if both types have only one component.
template<typename U, typename T>
inline auto convert(const T&x) -> typename std::enable_if<(std::is_arithmetic<T>::value || std::is_floating_point<T>::value) && (std::is_arithmetic<U>::value || std::is_floating_point<U>::value),U>::type
{
	return U(x);
}

// #3 copy component by component if both type have the same number of components (>1)
template<typename U, typename T>
inline auto convert(const T& x) -> typename std::enable_if<!std::is_arithmetic<T>::value && !std::is_floating_point<T>::value && std::is_same< std::integral_constant<uint32, geometry::nb_components_traits<T>::value>, std::integral_constant<uint32, geometry::nb_components_traits<U>::value>>::value, U>::type
{
	U res;
	for(uint32 i = 0u; i < geometry::nb_components_traits<T>::value ; ++i)
		res[i] = typename geometry::vector_traits<U>::Scalar(x[i]);
	return res;
}


template<typename T>
inline typename std::enable_if<std::is_arithmetic<T>::value || std::is_floating_point<T>::value, T>::type swap_endianness(const T& x)
{
	return ::cgogn::swap_endianness(x);
}

template<typename T>
inline typename std::enable_if<(!std::is_arithmetic<T>::value) && !std::is_floating_point<T>::value, T>::type swap_endianness(T& x)
{
	for (std::size_t i = 0u ; i < geometry::vector_traits<T>::SIZE; ++i)
		x[i] = ::cgogn::swap_endianness(x[i]);
	return x;
}

template<typename T>
inline typename std::enable_if<std::is_arithmetic<T>::value || std::is_floating_point<T>::value, std::istream&>::type parse(std::istream& iss, T& x)
{
	iss >> x;
	return iss;
}

template<typename T>
inline typename std::enable_if<!std::is_arithmetic<T>::value && !std::is_floating_point<T>::value, std::istream&>::type parse(std::istream& iss, T& x)
{
	for (std::size_t i = 0u ; i < geometry::vector_traits<T>::SIZE; ++i)
		iss >> x[i];
	return iss;
}

} // namespace internal


/**
 * @brief The CharArrayBuffer class
 * A minimalist buffer that read directly from the string given at the construction instead of copying it.
 * USE WITH CAUTION : the behaviour is undefined if a CharArrayBuffer's string is modified during its lifetime.
 */
class CGOGN_IO_API CharArrayBuffer : public std::streambuf
{
public:
	using Inherit = std::streambuf;
	using Self = CharArrayBuffer;
	using char_type		= Inherit::char_type;
	using traits_type	= Inherit::traits_type; // = char_traits<char_type>

	inline CharArrayBuffer() : Inherit(),
		begin_(nullptr)
	  ,end_(nullptr)
	  ,current_(nullptr)
	{}

	inline explicit CharArrayBuffer(const char* str) : Inherit(),
		begin_(str)
	  ,end_(str + std::strlen(str))
	  ,current_(str)
	{}

	inline CharArrayBuffer(const char* begin, std::size_t size) :Inherit(),
		begin_(begin)
	  ,end_(begin+size)
	  ,current_(begin)
	{}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CharArrayBuffer);

	virtual ~CharArrayBuffer();
private:
	virtual void imbue(const std::locale& __loc) override
	{
		cgogn_log_error("CharArrayBuffer::imbue") << "CharArrayBuffer::imbue method not implemented.";
		return Inherit::imbue(__loc);
	}
	virtual Inherit*setbuf(char_type*, std::streamsize) override
	{
		cgogn_log_error("CharArrayBuffer::setbuf") << "CharArrayBuffer::setbuf does nothing.";
		return this;
	}

	virtual pos_type seekpos(pos_type, std::ios_base::openmode) override
	{
		cgogn_log_error("CharArrayBuffer::seekpos") << "CharArrayBuffer::setbuf does nothing.";
		return pos_type(-1);
	}
	virtual int sync() override
	{
		cgogn_log_error("CharArrayBuffer::sync") << "CharArrayBuffer::sync does nothing.";
		return Inherit::sync();
	}
	virtual std::streamsize showmanyc() override
	{
		return end_ - current_;
	}
	virtual std::streamsize xsgetn(char_type* __s, std::streamsize __n) override
	{
		return Inherit::xsgetn(__s, __n);
	}
	virtual int_type underflow() override
	{
		if (current_ == end_)
			return traits_type::eof();
		return traits_type::to_int_type(*current_);
	}
	virtual int_type uflow() override
	{
		if (current_ == end_)
			return traits_type::eof();
		return traits_type::to_int_type(*current_++);
	}
	virtual int_type pbackfail(int_type c) override
	{
		if (current_ == begin_ || (c != traits_type::eof() && c != current_[-1]))
			return traits_type::eof();

		return traits_type::to_int_type(*--current_);
	}
	virtual std::streamsize xsputn(const char_type* , std::streamsize ) override
	{
		cgogn_log_error("CharArrayBuffer::xsputn") << "CharArrayBuffer::xsputn does nothing.";
		return std::streamsize(-1);
	}
	virtual int_type overflow(int_type c) override
	{
		return Inherit::overflow(c);
	}

private:
	const char* begin_;
	const char* end_;
	const char* current_;
};

/**
 * @brief The IMemoryStream class
 * A custom istream using the CharArrayBuffer as buffer.
 * USE WITH CAUTION : the behaviour is undefined if a IMemoryStream's string is modified during its lifetime.
 */
class CGOGN_IO_API IMemoryStream : public std::istream
{
public:
	using Inherit = std::istream;
	using Self = IMemoryStream;

	inline IMemoryStream() : Inherit(nullptr)
	{
		this->init(&buffer_);
	}

	inline IMemoryStream(const char* str) : Inherit(nullptr),
	buffer_(str)
	{
		this->init(&buffer_);
	}

	inline IMemoryStream(const char* str, std::size_t size) : Inherit(nullptr),
	buffer_(str,size)
	{
		this->init(&buffer_);
	}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(IMemoryStream);

	virtual ~IMemoryStream() override;
private:
	CharArrayBuffer buffer_;
};

} // namespace io
} // namespace cgogn

#endif // CGOGN_IO_IO_UTILS_H_
