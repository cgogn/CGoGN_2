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

#include <io/io_utils.h>

namespace cgogn
{

namespace io
{

/**
 * @brief The BaseDataIO class : used to read numerical values (scalar & vectors) in mesh files
 */
template<typename MAP>
class DataIOGen
{
public:
	using ChunkArrayGen = cgogn::ChunkArrayGen<MAP::CHUNKSIZE>;
	using ChunkArrayContainer = typename MAP::template ChunkArrayContainer<unsigned int>;

	virtual void read_n(std::ifstream& fp, std::size_t n, bool binary, bool big_endian) = 0;
	virtual void skip_n(std::ifstream& fp, std::size_t n, bool binary) = 0;
	virtual void* get_data() = 0;
	virtual void reset() = 0;

	virtual void to_chunk_array(ChunkArrayGen* ca_gen) const = 0;
	virtual ChunkArrayGen* add_attribute(ChunkArrayContainer& cac, const std::string& att_name) = 0;

	virtual unsigned int nb_components() const = 0;
	virtual ~DataIOGen() {}

	inline static std::unique_ptr<DataIOGen> newDataIO(const std::string type_name);
	inline static std::unique_ptr<DataIOGen> newDataIO(const std::string type_name, unsigned int nb_components);
};


template<typename MAP, typename T>
class DataIO : public DataIOGen<MAP>
{
public:
	using Inherit		= DataIOGen<MAP>;
	using Self			= DataIO<MAP,T>;
	using ChunkArrayGen	= typename Inherit::ChunkArrayGen;
	using ChunkArray	= cgogn::ChunkArray<MAP::CHUNKSIZE, T>;
	using ChunkArrayContainer = typename Inherit::ChunkArrayContainer;

	DataIO()
	{
		data_ = make_unique<std::vector<T>>();
	}

	DataIO(const Self&) = delete;
	DataIO& operator =(const Self&) = delete;
	DataIO(Self&&) = default;

	virtual void read_n(std::ifstream& fp, std::size_t n, bool binary, bool big_endian) override
	{
		const std::size_t old_size = data_->size();
		data_->resize(old_size + n);
		if (binary)
		{
			fp.read(reinterpret_cast<char*>(std::addressof(data_->operator[](old_size))), n * sizeof(T));
			if (big_endian != ::cgogn::internal::cgogn_is_little_endian)
			{
				for (auto & x : *data_)
					x = cgogn::io::internal::swap_endianness(x);
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

	virtual void skip_n(std::ifstream& fp, std::size_t n, bool binary) override
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

	virtual ChunkArray* add_attribute(ChunkArrayContainer& cac, const std::string& att_name) override
	{
		for (unsigned i = cac.capacity(), end = data_->size(); i < end; ++i)
			cac.template insert_lines<MAP::PRIM_SIZE>();
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

	virtual unsigned int nb_components() const override
	{
		return geometry::nb_components_traits<T>::value;
	}

private:
	std::unique_ptr<std::vector<T>>	data_;
};

template<typename MAP>
std::unique_ptr<DataIOGen<MAP>> DataIOGen<MAP>::newDataIO(const std::string type_name)
{
	if (type_name == name_of_type(float()))
		return make_unique<DataIO<MAP,float>>();
	else {
		if (type_name == name_of_type(double()))
			return make_unique<DataIO<MAP,double>>();
		else {
			if (type_name == name_of_type(char()))
				return make_unique<DataIO<MAP,char>>();
			else
			{
				if (type_name == name_of_type(std::int8_t()))
					return make_unique<DataIO<MAP,std::int8_t>>();
				else
				{
					if (type_name == name_of_type(std::uint8_t()))
						return make_unique<DataIO<MAP,std::uint8_t>>();
					else
					{
						if (type_name == name_of_type(std::int16_t()))
							return make_unique<DataIO<MAP,std::int16_t>>();
						else
						{
							if (type_name == name_of_type(std::uint16_t()))
								return make_unique<DataIO<MAP,std::uint16_t>>();
							else
							{
								if (type_name == name_of_type(std::uint32_t()))
									return make_unique<DataIO<MAP,std::uint32_t>>();
								else
								{
									if (type_name == name_of_type(std::int32_t()))
										return make_unique<DataIO<MAP,std::int32_t>>();
									else
									{
										if (type_name == name_of_type(std::uint64_t()))
											return make_unique<DataIO<MAP,std::uint64_t>>();
										else
										{
											if (type_name == name_of_type(std::int64_t()))
												return make_unique<DataIO<MAP,std::int64_t>>();
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
	return std::unique_ptr<DataIOGen<MAP>>();
}

template<typename MAP>
std::unique_ptr<DataIOGen<MAP>> DataIOGen<MAP>::newDataIO(const std::string type_name, unsigned int nb_components)
{
	cgogn_assert(nb_components >=1u && nb_components <= 4u);
	if (nb_components == 1u)
		return DataIOGen<MAP>::newDataIO(type_name);

	if (type_name == name_of_type(std::int32_t()))
	{
		switch (nb_components)
		{
			case 2u: return make_unique<DataIO<MAP,Eigen::Vector2i>>(); break;
			case 3u: return make_unique<DataIO<MAP,Eigen::Vector3i>>(); break;
			case 4u: return make_unique<DataIO<MAP,Eigen::Vector4i>>(); break;
			default:break;
		}
	} else {
		if (type_name == name_of_type(float()))
		{
			switch (nb_components)
			{
				case 2u: return make_unique<DataIO<MAP,Eigen::Vector2f>>(); break;
				case 3u: return make_unique<DataIO<MAP,Eigen::Vector3f>>(); break;
				case 4u: return make_unique<DataIO<MAP,Eigen::Vector4f>>(); break;
				default:break;
			}
		} else {
			if (type_name == name_of_type(double()))
			{
				switch (nb_components)
				{
					case 2u: return make_unique<DataIO<MAP,Eigen::Vector2d>>(); break;
					case 3u: return make_unique<DataIO<MAP,Eigen::Vector3d>>(); break;
					case 4u: return make_unique<DataIO<MAP,Eigen::Vector4d>>(); break;
					default:break;
				}
			}
		}
	}

	std::cerr << "DataIOGen::newDataIO : couldn't create a DataIO of type \"" << type_name << "\" with " << nb_components << " components." << std::endl;
	return std::unique_ptr<DataIOGen<MAP>>();
}

} // namespace io
} // namespace cgogn

#endif // IO_DATA_IO_H_
