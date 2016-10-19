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

#ifndef CGOGN_IO_DATA_IO_H_
#define CGOGN_IO_DATA_IO_H_

#include <memory>
#include <string>

#include <cgogn/core/utils/logger.h>
#include <cgogn/core/utils/unique_ptr.h>

#include <cgogn/core/cmap/map_base_data.h>

#include <cgogn/io/io_utils.h>

namespace cgogn
{

namespace io
{

/**
 * @brief The BaseDataIO class : used to read numerical values (scalar & vectors) in mesh files
 */
class DataInputGen
{
public:

	using Self = DataInputGen;

	using ChunkArrayContainer = MapBaseData::ChunkArrayContainer<uint32>;
	using ChunkArrayGen = MapBaseData::ChunkArrayGen;

	inline DataInputGen() {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(DataInputGen);
	virtual ~DataInputGen() {}

	virtual void read_n(std::istream& fp, std::size_t n, bool binary, bool big_endian) = 0;
	virtual void skip_n(std::istream& fp, std::size_t n, bool binary) = 0;
	/**
	 * @brief data
	 * @return a pointer to the first delement stored in the buffer vector
	 */
	virtual void* data() = 0;
	virtual const void* data() const = 0;

	/**
	 * @brief data_size
	 * @return the size of one data element (i.e. sizeof(T))
	 */
	virtual uint8 data_size() const = 0;
	virtual DataType data_type() const = 0;
	/**
	 * @brief buffer_vector
	 * @return return a pointer to the vector used to store the data (WARNING : this is not a pointer to the data contained in the vector)
	 */
	virtual void* buffer_vector() = 0;
	virtual const void* buffer_vector() const = 0;

	virtual void reset() = 0;
	virtual std::size_t size() const = 0;
	/**
	 * @brief simplify, transform a DataInput<PRIM_SIZE, BUFFER_T , T> into a DataInput<PRIM_SIZE, T , T>
	 * @return a DataInput with T = BUFFER_T
	 * WARNING : after a call to simplify, the data is moved to the returned DataInputGen, leaving an empty vector.
	 */
	virtual std::unique_ptr<Self> simplify() = 0;

	virtual void to_chunk_array(ChunkArrayGen* ca_gen) const = 0;
	virtual ChunkArrayGen* add_attribute(ChunkArrayContainer& cac, const std::string& att_name) const = 0;

	virtual uint32 nb_components() const = 0;

	template <uint32 PRIM_SIZE>
	inline static std::unique_ptr<DataInputGen> newDataIO(const std::string type_name);
	template <uint32 PRIM_SIZE>
	inline static std::unique_ptr<DataInputGen> newDataIO(const std::string type_name, uint32 nb_components);

	// This versions converts the data to the type T (if T is different from the type that has been read in the file)
	template <uint32 PRIM_SIZE, typename T>
	inline static std::unique_ptr<DataInputGen> newDataIO(const std::string type_name);
	// This versions converts the data to the type T (if T is different from the type that has been read in the file)
	template <uint32 PRIM_SIZE, typename T>
	inline static std::unique_ptr<DataInputGen> newDataIO(const std::string type_name, uint32 nb_components);
};

template <uint32 PRIM_SIZE, typename BUFFER_T, typename T = BUFFER_T>
class DataInput : public DataInputGen
{
public:

	using Inherit		= DataInputGen;
	using Self			= DataInput<PRIM_SIZE, BUFFER_T , T> ;
	using ChunkArrayContainer = typename Inherit::ChunkArrayContainer;
	using ChunkArrayGen	= typename Inherit::ChunkArrayGen;
	using ChunkArray	= MapBaseData::ChunkArray<T>;

	DataInput()
	{}

	inline DataInput(const Self& other) :
		data_(other.data_)
	{}

	inline DataInput(Self&& other) :
		data_(std::move(other.data_))
	{}

	inline DataInput& operator =(const Self& other)
	{
		if (&other != this)
			data_ = other.data_;
		return *this;
	}

	inline DataInput& operator =(Self&& other)
	{
		if (&other != this)
			data_ = std::move(other.data_);
		return *this;
	}

	virtual void read_n(std::istream& fp, std::size_t n, bool binary, bool big_endian) override
	{
		const std::size_t old_size = data_.size();
		data_.resize(old_size + n);
		if (binary)
		{
			if (std::is_same<T, BUFFER_T>::value)
			{ // if BUFFER_T = T we can directly store the data
				fp.read(reinterpret_cast<char*>(&data_[old_size]), std::streamsize(n * sizeof(T)));
				if ((big_endian && cgogn::internal::cgogn_is_little_endian) || (!big_endian && cgogn::internal::cgogn_is_big_endian))
				{
					for (auto it = data_.begin() + typename std::vector<T>::difference_type(old_size), end = data_.end() ; it != end; ++it)
						*it = cgogn::swap_endianness(*it);
				}

				if (fp.eof() || fp.bad())
					this->reset();
			}
			else
			{ // 2nd case : BUFFER_T and T are different.
				std::vector<BUFFER_T> buffer(old_size+n);
				fp.read(reinterpret_cast<char*>(&buffer[old_size]), std::streamsize(n * sizeof(BUFFER_T)));
				if ((big_endian && cgogn::internal::cgogn_is_little_endian) || (!big_endian && cgogn::internal::cgogn_is_big_endian))
				{
					for (auto it = buffer.begin() + typename std::vector<BUFFER_T>::difference_type(old_size), end = buffer.end() ; it != end; ++it)
						*it = cgogn::swap_endianness(*it);
				}
				if (fp.eof() || fp.bad())
					this->reset();
				// copy
				auto dest_it = data_.begin();
				for (auto & x : buffer)
					*dest_it++ = internal::convert<T>(x);
			}
		}
		else
		{
			std::string line;
			line.reserve(256);
			std::size_t i = 0ul;
//			BUFFER_T buff;
			for (; i < n && (!fp.eof()) && (!fp.bad()); )
			{
				std::getline(fp,line);
				std::istringstream line_stream(line);
				// we need to avoid the specialization of istringstream operator>> for chars
				using type = typename std::conditional<sizeof(BUFFER_T) == sizeof(char), int, BUFFER_T>::type;
				type buff;
				serialization::parse(line_stream, buff);
				bool no_error = static_cast<bool>(line_stream);
				while (i < n && no_error)
				{
					data_[i+old_size] = internal::convert<T>(buff);
					++i;
					serialization::parse(line_stream, buff);
					no_error = static_cast<bool>(line_stream);
				}
				if (!no_error && (!line_stream.eof()))
					break;
			}

			if (i < n)
			{
				cgogn_log_warning("DataInput::read_n") << "An eccor occured while reading the line \n\"" << line << "\".";
				this->reset();
			}
		}
	}

	virtual void skip_n(std::istream& fp, std::size_t n, bool binary) override
	{
		if (binary)
		{
			fp.ignore(n * sizeof(BUFFER_T));
		}
		else
		{
			std::string line;
			line.reserve(256);
			std::size_t i = 0ul;
			for (; i < n && (!fp.eof()) && (!fp.bad()); )
			{
				std::getline(fp,line);
				std::istringstream line_stream(line);
				bool no_error = static_cast<bool>(line_stream.ignore(1, ' '));
				while (i < n && no_error)
				{
					++i;
					no_error = static_cast<bool>(line_stream.ignore(1, ' '));
				}
				if (!no_error && (!line_stream.eof()))
					break;
			}
			if (i < n)
			{
				cgogn_log_warning("DataInput::skip_n") << "An eccor occured while skipping the line \n\"" << line << "\".";
			}
		}
	}

	virtual void reset() override
	{
		data_.clear();
	}

	virtual ChunkArray* add_attribute(ChunkArrayContainer& cac, const std::string& att_name) const override
	{
		for (std::size_t i = cac.capacity(), end = data_.size(); i < end; i += PRIM_SIZE)
			cac.template insert_lines<PRIM_SIZE>();
		return cac.template add_chunk_array<T>(att_name);
	}

	virtual void to_chunk_array(ChunkArrayGen* ca_gen) const override
	{
		ChunkArray* ca = dynamic_cast<ChunkArray*>(ca_gen);
		uint32 i = 0u;
		for (auto& x : data_)
			ca->operator[](i++) = x;
	}

	virtual void* data() override
	{
		return &data_[0];
	}

	virtual const void* data() const override
	{
		return &data_[0];
	}

	virtual uint8 data_size() const override
	{
		return uint8(sizeof(T));
	}

	virtual DataType data_type() const override
	{
		return cgogn::io::data_type(cgogn::name_of_type(T()));
	}

	void* buffer_vector() override
	{
		return &data_;
	}

	virtual const void* buffer_vector() const override
	{
		return &data_;
	}

	inline std::vector<T> const * vec() const
	{
		return &data_;
	}

	virtual uint32 nb_components() const override
	{
		return geometry::nb_components_traits<T>::value;
	}

	virtual std::size_t size() const override
	{
		return data_.size();
	}

	virtual std::unique_ptr<Inherit> simplify() override
	{
		std::unique_ptr<DataInput<PRIM_SIZE, T , T>> res = make_unique<DataInput<PRIM_SIZE, T , T>>();
		std::vector<T>& res_vec = *(static_cast<std::vector<T>*>(res->buffer_vector()));
		res_vec = std::move(this->data_);
		this->data_ = std::vector<T>();
		return std::unique_ptr<Inherit>(res.release());
	}

private:

	std::vector<T> data_;
};

template <uint32 PRIM_SIZE>
std::unique_ptr<DataInputGen> DataInputGen::newDataIO(const std::string type_name)
{
	const DataType type = cgogn::io::data_type(type_name);
	switch (type)
	{
		case DataType::FLOAT:	return make_unique<DataInput<PRIM_SIZE, float32>>();
		case DataType::DOUBLE:	return make_unique<DataInput<PRIM_SIZE, float64>>();
		case DataType::CHAR:	return make_unique<DataInput<PRIM_SIZE, char>>();
		case DataType::INT8:	return make_unique<DataInput<PRIM_SIZE, std::int8_t>>();
		case DataType::UINT8:	return make_unique<DataInput<PRIM_SIZE, std::uint8_t>>();
		case DataType::INT16:	return make_unique<DataInput<PRIM_SIZE, std::int16_t>>();
		case DataType::UINT16:	return make_unique<DataInput<PRIM_SIZE, std::uint16_t>>();
		case DataType::INT32:	return make_unique<DataInput<PRIM_SIZE, std::int32_t>>();
		case DataType::UINT32:	return make_unique<DataInput<PRIM_SIZE, std::uint32_t>>();
		case DataType::INT64:	return make_unique<DataInput<PRIM_SIZE, std::int64_t>>();
		case DataType::UINT64:	return make_unique<DataInput<PRIM_SIZE, std::uint64_t>>();
		case DataType::UNKNOWN:
		default:
			cgogn_log_error("DataInputGen::newDataIO") << "Couldn't create a DataIO of type \"" << type_name << "\".";
			return std::unique_ptr<DataInputGen>();
	}
}

template <uint32 PRIM_SIZE, typename T>
std::unique_ptr<DataInputGen> DataInputGen::newDataIO(const std::string type_name)
{
	const DataType type = cgogn::io::data_type(type_name);
	switch (type)
	{
		case DataType::FLOAT:	return make_unique<DataInput<PRIM_SIZE, float32, T>>();
		case DataType::DOUBLE:	return make_unique<DataInput<PRIM_SIZE, float64, T>>();
		case DataType::CHAR:	return make_unique<DataInput<PRIM_SIZE, char, T>>();
		case DataType::INT8:	return make_unique<DataInput<PRIM_SIZE, std::int8_t, T>>();
		case DataType::UINT8:	return make_unique<DataInput<PRIM_SIZE, std::uint8_t, T>>();
		case DataType::INT16:	return make_unique<DataInput<PRIM_SIZE, std::int16_t, T>>();
		case DataType::UINT16:	return make_unique<DataInput<PRIM_SIZE, std::uint16_t, T>>();
		case DataType::INT32:	return make_unique<DataInput<PRIM_SIZE, std::int32_t, T>>();
		case DataType::UINT32:	return make_unique<DataInput<PRIM_SIZE, std::uint32_t, T>>();
		case DataType::INT64:	return make_unique<DataInput<PRIM_SIZE, std::int64_t, T>>();
		case DataType::UINT64:	return make_unique<DataInput<PRIM_SIZE, std::uint64_t, T>>();
		default:
			cgogn_log_error("DataInputGen::newDataIO") << "Couldn't create a DataIO of type \"" << type_name << "\".";
			return std::unique_ptr<DataInputGen>();
	}
}

template <uint32 PRIM_SIZE>
std::unique_ptr<DataInputGen> DataInputGen::newDataIO(const std::string type_name, uint32 nb_components)
{
	cgogn_assert(nb_components >= 1u && nb_components <= 4u);
	if (nb_components == 1u)
		return DataInputGen::newDataIO<PRIM_SIZE>(type_name);

	if (type_name == name_of_type(std::int32_t()))
	{
		switch (nb_components)
		{
			case 2u: return make_unique<DataInput<PRIM_SIZE, Eigen::Vector2i>>(); break;
			case 3u: return make_unique<DataInput<PRIM_SIZE, Eigen::Vector3i>>(); break;
			case 4u: return make_unique<DataInput<PRIM_SIZE, Eigen::Vector4i>>(); break;
			default: break;
		}
	}
	else
	{
		if (type_name == name_of_type(float32()))
		{
			switch (nb_components)
			{
				case 2u: return make_unique<DataInput<PRIM_SIZE, Eigen::Vector2f>>(); break;
				case 3u: return make_unique<DataInput<PRIM_SIZE, Eigen::Vector3f>>(); break;
				case 4u: return make_unique<DataInput<PRIM_SIZE, Eigen::Vector4f>>(); break;
				default:break;
			}
		}
		else
		{
			if (type_name == name_of_type(float64()))
			{
				switch (nb_components)
				{
					case 2u: return make_unique<DataInput<PRIM_SIZE, Eigen::Vector2d>>(); break;
					case 3u: return make_unique<DataInput<PRIM_SIZE, Eigen::Vector3d>>(); break;
					case 4u: return make_unique<DataInput<PRIM_SIZE, Eigen::Vector4d>>(); break;
					default:break;
				}
			}
		}
	}

	cgogn_log_error("DataInputGen::newDataIO") << "Couldn't create a DataIO of type \"" << type_name << "\" with " << nb_components << " components.";
	return std::unique_ptr<DataInputGen>();
}

template <uint32 PRIM_SIZE, typename T>
std::unique_ptr<DataInputGen> DataInputGen::newDataIO(const std::string type_name, uint32 nb_components)
{
	cgogn_assert(nb_components >= 1u && nb_components <= 4u);
	if (nb_components == 1u)
		return DataInputGen::newDataIO<PRIM_SIZE, T>(type_name);

	if (type_name == name_of_type(std::int32_t()))
	{
		switch (nb_components)
		{
			case 2u: return make_unique<DataInput<PRIM_SIZE, Eigen::Vector2i, T>>(); break;
			case 3u: return make_unique<DataInput<PRIM_SIZE, Eigen::Vector3i, T>>(); break;
			case 4u: return make_unique<DataInput<PRIM_SIZE, Eigen::Vector4i, T>>(); break;
			default:break;
		}
	}
	else
	{
		if (type_name == name_of_type(float32()))
		{
			switch (nb_components)
			{
				case 2u: return make_unique<DataInput<PRIM_SIZE, Eigen::Vector2f, T>>(); break;
				case 3u: return make_unique<DataInput<PRIM_SIZE, Eigen::Vector3f, T>>(); break;
				case 4u: return make_unique<DataInput<PRIM_SIZE, Eigen::Vector4f, T>>(); break;
				default:break;
			}
		}
		else
		{
			if (type_name == name_of_type(float64()))
			{
				switch (nb_components)
				{
					case 2u: return make_unique<DataInput<PRIM_SIZE, Eigen::Vector2d, T>>(); break;
					case 3u: return make_unique<DataInput<PRIM_SIZE, Eigen::Vector3d, T>>(); break;
					case 4u: return make_unique<DataInput<PRIM_SIZE, Eigen::Vector4d, T>>(); break;
					default:break;
				}
			}
		}
	}

	cgogn_log_error("DataInputGen::newDataIO") << "Couldn't create a DataIO of type \"" << type_name << "\" with " << nb_components << " components.";
	return std::unique_ptr<DataInputGen>();
}

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_DATA_IO_H_
