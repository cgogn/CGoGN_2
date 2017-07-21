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

#include <cgogn/core/utils/endian.h>
#include <cgogn/core/cmap/attribute.h>
#include <cgogn/core/basic/cell.h>
#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/io/dll.h>

namespace cgogn
{

// stream insertion / extraction for Attribute_T
template <typename T>
inline std::ostream& operator<<(std::ostream& o, const Attribute_T<T>& att)
{
	for(auto it = att.begin(), end = att.end() ; it != end;)
	{
		o << (*it);
		++it;
		if (it != end)
			o << " ";
	}
	return o;
}

template <typename T>
inline std::istream& operator>>(std::istream& in, Attribute_T<T>& att)
{
	for (auto& it : att)
	{
		if (in.good())
			in >> it;
		else
			it = std::move(T());
	}
	return in;
}

namespace io
{


class CGOGN_IO_API ExportOptions final
{
private:

	ExportOptions();

public:

	ExportOptions(const ExportOptions& eo);
	ExportOptions(ExportOptions&& eo);
	inline ~ExportOptions() {}
	inline ExportOptions& filename(const std::string & filename) { filename_ = filename; return *this; }
	inline ExportOptions& position_attribute(Orbit orb, const std::string & name) {
		position_attributes_.insert(std::make_pair(orb, name)); return *this;}
	inline ExportOptions& add_attribute(Orbit orb, const std::string & name) { attributes_to_export_.push_back(std::make_pair(orb,name)); return *this; }
	inline ExportOptions& binary(bool b) { binary_ = b; return *this; }
	inline ExportOptions& compress(bool b) { compress_ = b; return *this; }
	inline ExportOptions& overwrite(bool b) { overwrite_ = b; return *this; }
	inline ExportOptions& cell_filter(const std::function<bool(Dart)>& func) { cell_filter_ = func; return *this; }

#pragma warning(push)
#pragma warning(disable:4251)
	std::string filename_;
	std::map<Orbit, std::string> position_attributes_;
	std::vector<std::pair<Orbit, std::string>> attributes_to_export_;
	std::function<bool(Dart)> cell_filter_;
#pragma warning(pop)
	bool binary_;
	bool compress_;
	bool overwrite_;

	static ExportOptions create();
};

enum FileType
{
	FileType_UNKNOWN = 0,
	FileType_OFF,
	FileType_OBJ,
	FileType_2DM,
	FileType_PLY,
	FileType_STL,
	FileType_VTK_LEGACY,
	FileType_VTU,
	FileType_VTP,
	FileType_MESHB,
	FileType_MSH,
	FileType_TETGEN,
	FileType_NASTRAN,
	FileType_AIMATSHAPE,
	FileType_TETMESH
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

enum VolumeType
{
	Tetra,
	Pyramid,
	TriangularPrism,
	Hexa,
	Connector
};

CGOGN_IO_API bool                           file_exists(const std::string& filename);
CGOGN_IO_API std::unique_ptr<std::ofstream> create_file(const std::string& filename, bool binary, bool overwrite);
CGOGN_IO_API FileType                       file_type(const std::string& filename);
CGOGN_IO_API DataType                       data_type(const std::string& type_name);

/**
 * @brief base64_encode
 * @param input_buffer, the data we want to encode
 * @param buffer_size, the number of bytes we want to encode ( with buffer_size <= strlen(input_buffer) )
 * @return a vector containing the encoded data.
 */
CGOGN_IO_API std::vector<char>          base64_encode(const char* input_buffer, std::size_t buffer_size);

/**
 * @brief base64_decode
 * @param input, the data we want to decode
 * @param length, the number of bytes we want to process
 * @return an empty vector if the process failed. The decoded data if successful.
 */
CGOGN_IO_API std::vector<unsigned char> base64_decode(const char* const input, std::size_t length);

/**
 * @brief zlib_decompress
 * @param input, the data we want to decompress
 * @param header_type, either UINT64 or UINT32
 * @param length, the length of the data we want to decompress.
 * @return  the decompressed data if successful, otherwise an empty vector.
 */
CGOGN_IO_API std::vector<unsigned char>              zlib_decompress(const char* input, DataType header_type, std::size_t length);

/**
 * @brief zlib_compress
 * @param input, the data we want to compress
 * @param size, the number of bytes we want to compress
 * @param chunk_size, the maximum compressed size of a chunk
 * @return a vector of compressed chunk. The size of the chunks is equal to chunk_size except for the last one that can be smaller.
 */
CGOGN_IO_API std::vector<std::vector<unsigned char>> zlib_compress(const unsigned char* input, std::size_t size, std::size_t chunk_size);

namespace internal
{

// #1 return default value when U and T don't have the same nb of components.
template <typename U, typename T>
inline auto convert(const T&) -> typename std::enable_if<!std::is_same< std::integral_constant<uint32, geometry::nb_components_traits<T>::value>, std::integral_constant<uint32, geometry::nb_components_traits<U>::value>>::value,U>::type
{
	cgogn_log_warning("convert") << "Cannot convert data of type\"" << name_of_type(T()) << "\" to type \"" << name_of_type(U()) << "\".";
	return U();
}

// #2 cast x if both types have only one component.
template <typename U, typename T>
inline auto convert(const T&x) -> typename std::enable_if<std::is_arithmetic<T>::value && std::is_arithmetic<U>::value,U>::type
{
	return U(x);
}

// #3 copy component by component if both type have the same number of components (>1)
template <typename U, typename T>
inline auto convert(const T& x) -> typename std::enable_if<!std::is_arithmetic<T>::value && std::is_same< std::integral_constant<uint32, geometry::nb_components_traits<T>::value>, std::integral_constant<uint32, geometry::nb_components_traits<U>::value>>::value, U>::type
{
	U res;
	for(uint32 i = 0u; i < geometry::nb_components_traits<T>::value ; ++i)
		res[i] = typename geometry::vector_traits<U>::Scalar(x[i]);
	return res;
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

	inline CharArrayBuffer() :
		Inherit(),
		begin_(nullptr),
		end_(nullptr),
		current_(nullptr)
	{}

	inline explicit CharArrayBuffer(const char* str) :
		Inherit(),
		begin_(str),
		end_(str + std::strlen(str)),
		current_(str)
	{}

	inline CharArrayBuffer(const char* begin, std::size_t size) :
		Inherit(),
		begin_(begin),
		end_(begin+size),
		current_(begin)
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

CGOGN_IO_API std::istream& getline_safe(std::istream& is, std::string& str);


} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_IO_UTILS_H_
