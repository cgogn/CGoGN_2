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

#ifndef IO_DATA_IO_H_
#define IO_DATA_IO_H_

#include <memory>
#include <string>

#include <core/utils/make_unique.h>
#include <core/container/chunk_array.h>
#include <core/container/chunk_array_container.h>

#include <io/io_utils.h>

namespace cgogn
{

namespace io
{

/**
 * @brief The BaseDataIO class : used to read numerical values (scalar & vectors) in mesh files
 */
template<unsigned int CHUNK_SIZE>
class DataIOGen
{
public:
	using ChunkArrayGen = cgogn::ChunkArrayGen<CHUNK_SIZE>;
	using ChunkArrayContainer = cgogn::ChunkArrayContainer<CHUNK_SIZE, unsigned int>;

	virtual void read_n(std::istream& fp, std::size_t n, bool binary, bool big_endian) = 0;
	virtual void skip_n(std::istream& fp, std::size_t n, bool binary) = 0;
	virtual void* get_data() = 0;
	virtual void reset() = 0;

	virtual void to_chunk_array(ChunkArrayGen* ca_gen) const = 0;
	virtual ChunkArrayGen* add_attribute(ChunkArrayContainer& cac, const std::string& att_name) const = 0;

	virtual unsigned int nb_components() const = 0;
	virtual ~DataIOGen() {}

	template<unsigned int PRIM_SIZE>
	inline static std::unique_ptr<DataIOGen> newDataIO(const std::string type_name);
	template<unsigned int PRIM_SIZE>
	inline static std::unique_ptr<DataIOGen> newDataIO(const std::string type_name, unsigned int nb_components);
};


template<typename T, unsigned int CHUNK_SIZE, unsigned int PRIM_SIZE>
class DataIO : public DataIOGen<CHUNK_SIZE>
{
public:
	using Inherit		= DataIOGen<CHUNK_SIZE>;
	using Self			= DataIO<T, CHUNK_SIZE, PRIM_SIZE>;
	using ChunkArrayGen	= typename Inherit::ChunkArrayGen;
	using ChunkArray	= cgogn::ChunkArray<CHUNK_SIZE, T>;
	using ChunkArrayContainer = typename Inherit::ChunkArrayContainer;

	DataIO()
	{
		data_ = make_unique<std::vector<T>>();
	}

	DataIO(const Self&) = delete;
	DataIO& operator =(const Self&) = delete;
	DataIO(Self&&) = default;

	virtual void read_n(std::istream& fp, std::size_t n, bool binary, bool big_endian) override
	{
		const std::size_t old_size = data_->size();
		data_->resize(old_size + n);
		if (binary)
		{
			fp.read(reinterpret_cast<char*>(std::addressof(data_->operator[](old_size))), n * sizeof(T));
			if (big_endian != ::cgogn::internal::cgogn_is_little_endian)
			{
				for (auto it = data_->begin() + old_size, end = data_->end() ; it != end; ++it)
					*it = cgogn::io::internal::swap_endianness(*it);
			}
			if (fp.eof() || fp.bad())
				this->reset();
		} else {
			std::string line;
			line.reserve(256);
			std::size_t i = 0ul;
			for (; i < n && (!fp.eof()) && (!fp.bad()); )
			{
				bool no_error = true;
				std::getline(fp,line);
				std::istringstream line_stream(line);
				while (i < n && (no_error = static_cast<bool>(internal::parse(line_stream, data_->operator[](i+old_size)))))
					++i;
				if (!no_error && (!line_stream.eof()))
					break;
			}
			if (i < n)
			{
				std::cerr << "read_n : An eccor occured while reading the line \n\"" << line << "\"" <<  std::endl;
				this->reset();
			}
		}
	}

	virtual void skip_n(std::istream& fp, std::size_t n, bool binary) override
	{
		if (binary)
		{
			fp.ignore(n * sizeof(T));
		} else {
			std::string line;
			line.reserve(256);
			std::size_t i = 0ul;
			for (; i < n && (!fp.eof()) && (!fp.bad()); )
			{
				bool no_error = true;
				std::getline(fp,line);
				std::istringstream line_stream(line);
				while (i < n && (no_error = static_cast<bool>(line_stream.ignore(1, ' '))))
					++i;
				if (!no_error && (!line_stream.eof()))
					break;
			}
			if (i < n)
			{
				std::cerr << "skip_n : An eccor occured while skipping the line \n\"" << line << "\"" <<  std::endl;
			}
		}
	}
	virtual void reset() override
	{
		data_ = make_unique<std::vector<T>>();
	}

	virtual ChunkArray* add_attribute(ChunkArrayContainer& cac, const std::string& att_name) const override
	{
		for (unsigned i = cac.capacity(), end = data_->size(); i < end; i+=PRIM_SIZE)
			cac.template insert_lines<PRIM_SIZE>();
		return cac.template add_attribute<T>(att_name);
	}

	virtual void to_chunk_array(ChunkArrayGen* ca_gen) const override
	{
		ChunkArray* ca = dynamic_cast<ChunkArray *>(ca_gen);
		unsigned int i = 0u;
		for (auto& x : *data_)
			ca->operator[](i++) = x;
	}

	virtual void* get_data() override
	{
		return data_.get();
	}

	inline std::vector<T> const * get_vec() const
	{
		return data_.get();
	}

	virtual unsigned int nb_components() const override
	{
		return geometry::nb_components_traits<T>::value;
	}

private:
	std::unique_ptr<std::vector<T>>	data_;
};

template<unsigned int CHUNK_SIZE>
template<unsigned int PRIM_SIZE>
std::unique_ptr<DataIOGen<CHUNK_SIZE>> DataIOGen<CHUNK_SIZE>::newDataIO(const std::string type_name)
{
	if (type_name == name_of_type(float()))
		return make_unique<DataIO<float, CHUNK_SIZE, PRIM_SIZE>>();
	else {
		if (type_name == name_of_type(double()))
			return make_unique<DataIO<double, CHUNK_SIZE, PRIM_SIZE>>();
		else {
			if (type_name == name_of_type(char()))
				return make_unique<DataIO<char, CHUNK_SIZE, PRIM_SIZE>>();
			else
			{
				if (type_name == name_of_type(std::int8_t()))
					return make_unique<DataIO<std::int8_t, CHUNK_SIZE, PRIM_SIZE>>();
				else
				{
					if (type_name == name_of_type(std::uint8_t()))
						return make_unique<DataIO<std::uint8_t, CHUNK_SIZE, PRIM_SIZE>>();
					else
					{
						if (type_name == name_of_type(std::int16_t()))
							return make_unique<DataIO<std::int16_t, CHUNK_SIZE, PRIM_SIZE>>();
						else
						{
							if (type_name == name_of_type(std::uint16_t()))
								return make_unique<DataIO<std::uint16_t, CHUNK_SIZE, PRIM_SIZE>>();
							else
							{
								if (type_name == name_of_type(std::uint32_t()))
									return make_unique<DataIO<std::uint32_t, CHUNK_SIZE, PRIM_SIZE>>();
								else
								{
									if (type_name == name_of_type(std::int32_t()))
										return make_unique<DataIO<std::int32_t, CHUNK_SIZE, PRIM_SIZE>>();
									else
									{
										if (type_name == name_of_type(std::uint64_t()))
											return make_unique<DataIO<std::uint64_t, CHUNK_SIZE, PRIM_SIZE>>();
										else
										{
											if (type_name == name_of_type(std::int64_t()))
												return make_unique<DataIO<std::int64_t, CHUNK_SIZE, PRIM_SIZE>>();
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}
	std::cerr << "DataIOGen::newDataIO : couldn't create a DataIO of type \"" << type_name << "\"." << std::endl;
	return std::unique_ptr<DataIOGen<CHUNK_SIZE>>();
}

template<unsigned int CHUNK_SIZE>
template<unsigned int PRIM_SIZE>
std::unique_ptr<DataIOGen<CHUNK_SIZE>> DataIOGen<CHUNK_SIZE>::newDataIO(const std::string type_name, unsigned int nb_components)
{
	cgogn_assert(nb_components >=1u && nb_components <= 4u);
	if (nb_components == 1u)
		return DataIOGen<CHUNK_SIZE>::newDataIO<PRIM_SIZE>(type_name);

	if (type_name == name_of_type(std::int32_t()))
	{
		switch (nb_components)
		{
			case 2u: return make_unique<DataIO<Eigen::Vector2i, CHUNK_SIZE, PRIM_SIZE>>(); break;
			case 3u: return make_unique<DataIO<Eigen::Vector3i, CHUNK_SIZE, PRIM_SIZE>>(); break;
			case 4u: return make_unique<DataIO<Eigen::Vector4i, CHUNK_SIZE, PRIM_SIZE>>(); break;
			default:break;
		}
	} else {
		if (type_name == name_of_type(float()))
		{
			switch (nb_components)
			{
				case 2u: return make_unique<DataIO<Eigen::Vector2f, CHUNK_SIZE, PRIM_SIZE>>(); break;
				case 3u: return make_unique<DataIO<Eigen::Vector3f, CHUNK_SIZE, PRIM_SIZE>>(); break;
				case 4u: return make_unique<DataIO<Eigen::Vector4f, CHUNK_SIZE, PRIM_SIZE>>(); break;
				default:break;
			}
		} else {
			if (type_name == name_of_type(double()))
			{
				switch (nb_components)
				{
					case 2u: return make_unique<DataIO<Eigen::Vector2d, CHUNK_SIZE, PRIM_SIZE>>(); break;
					case 3u: return make_unique<DataIO<Eigen::Vector3d, CHUNK_SIZE, PRIM_SIZE>>(); break;
					case 4u: return make_unique<DataIO<Eigen::Vector4d, CHUNK_SIZE, PRIM_SIZE>>(); break;
					default:break;
				}
			}
		}
	}

	std::cerr << "DataIOGen::newDataIO : couldn't create a DataIO of type \"" << type_name << "\" with " << nb_components << " components." << std::endl;
	return std::unique_ptr<DataIOGen<CHUNK_SIZE>>();
}

} // namespace io
} // namespace cgogn

#endif // IO_DATA_IO_H_
