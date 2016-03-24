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

#ifndef IO_MSH_IO_H_
#define IO_MSH_IO_H_

#include <map>
#include <io/dll.h>
#include <io/data_io.h>
#include <io/volume_import.h>

namespace cgogn
{

namespace io
{

template<uint32 CHUNK_SIZE, uint32 PRIM_SIZE, typename VEC3>
class MshIO
{
public :
	enum MSH_CELL_TYPES
	{
		MSH_LINE = 1,
		MSH_TRIANGLE = 2,
		MSH_QUAD = 3,

		MSH_TETRA = 4,
		MSH_HEXA = 5,
		MSH_PRISM = 6,
		MSH_PYRAMID = 7,

		// TODO : second and third order cells
		MSH_VERTEX = 15,

	};

	using Self = MshIO<CHUNK_SIZE, PRIM_SIZE, VEC3>;
	inline MshIO() :
		version_number_()
	  , file_type_(-1)
	  , float_size_(sizeof(float64))
	  , swap_endianness_(false)
	{}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(MshIO);

	inline const std::string& get_version_number() const
	{
		return version_number_;
	}

	inline int32 get_file_type() const
	{
		return file_type_;
	}

	inline bool need_endianness_swap() const
	{
		return swap_endianness_;
	}
protected:
	bool read_header(std::istream& data_stream)
	{
		std::string line;
		std::string word;
		line.reserve(512);
		word.reserve(128);
		std::getline(data_stream, line);

		if (line == "$MeshFormat")
		{
			std::getline(data_stream, line);
			std::istringstream iss(line);
			iss >> version_number_ >> file_type_ >> float_size_;
			if (file_type_ == 1)
			{
				std::array<char,4> buff;
				data_stream.read(&buff[0],4);
				const int one = *reinterpret_cast<int32*>(&buff[0]);
				if (one != 1)
					swap_endianness_ = true;
			}
			std::getline(data_stream, line); // $EndMeshFormat
		} else {
			if (line.compare(0,4, "$NOD") == 0)
			{
				version_number_ = "1.0";
				file_type_ = 0;
				float_size_ = sizeof(float64);
			}
			else
				return false;
		}
		return true;
	}


private:
	std::string	version_number_;
	int32		file_type_;
	int32		float_size_; // sizeof(double) normally since the doc says "currently only data-size = sizeof(double) is supported"
	bool		swap_endianness_;
};

template<typename MAP_TRAITS, typename VEC3>
class MshVolumeImport : public MshIO<MAP_TRAITS::CHUNK_SIZE, CMap3<MAP_TRAITS>::PRIM_SIZE, VEC3>, public VolumeImport<MAP_TRAITS>
{
public:
	using Self = MshVolumeImport<MAP_TRAITS, VEC3>;
	using Inherit_Msh = MshIO<MAP_TRAITS::CHUNK_SIZE, CMap3<MAP_TRAITS>::PRIM_SIZE, VEC3>;
	using Inherit_Import = VolumeImport<MAP_TRAITS>;
//	using DataInputGen = typename Inherit_Msh::DataInputGen;
//	template<typename T>
//	using DataInput = typename Inherit_Msh::template DataInput<T>;
	using MSH_CELL_TYPES = typename Inherit_Msh::MSH_CELL_TYPES;
	template <typename T>
	using ChunkArray = typename Inherit_Import::template ChunkArray<T>;

	inline MshVolumeImport()
	{}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(MshVolumeImport);
	virtual ~MshVolumeImport() override {}

	// MeshImportGen interface
protected:

	inline bool import_legacy_msh_file(std::istream& data_stream)
	{
		ChunkArray<VEC3>* position = this->vertex_attributes_.template add_attribute<VEC3>("position");
		std::map<uint32,uint32> old_new_indices;
		std::string line;
		std::string word;
		line.reserve(512);
		word.reserve(128);
		line = this->skip_empty_lines(data_stream);
		this->nb_vertices_ = uint32(std::stoul(line));

		for (uint32 i = 0u; i < this->nb_vertices_; ++i)
		{
			std::getline(data_stream,line);

			const uint32 new_index = this->vertex_attributes_.template insert_lines<1>();
			auto& v = position->operator [](new_index);
			uint32 old_index;
			std::istringstream iss(line);
			iss >> old_index >> v[0] >> v[1] >> v[2];
			old_new_indices[old_index] = new_index;
		}

		std::getline(data_stream,line);
		if (line.compare(0, 7, "$ENDNOD") != 0)
			return false;
		std::getline(data_stream,line);
		if (line.compare(0, 4, "$ELM") != 0)
			return false;

		std::getline(data_stream,line);
		this->nb_volumes_ = uint32(std::stoul(line));

		for (uint32 i = 0u; i < this->nb_volumes_; ++i)
		{
			std::getline(data_stream,line);
			int32 elem_number;
			int32 elem_type;
			uint32 physical_entity; // MSH legacy DOC : If reg-phys is equal to zero, the element is considered not to belong to any physical entity.
			uint32 elementary_entity_id;
			uint32 number_of_nodes;
			std::vector<uint32> node_ids;

			std::istringstream iss(line);
			iss >> elem_number >> elem_type >> physical_entity >> elementary_entity_id >> number_of_nodes;
			node_ids.resize(number_of_nodes);
			for (uint32 j = 0u; j < number_of_nodes; ++j)
			{
				iss >> node_ids[j];
				node_ids[j] = old_new_indices[node_ids[j]];
			}

			switch (elem_type) {
				case MSH_CELL_TYPES::MSH_TETRA:
					this->add_tetra(*position, node_ids[0], node_ids[1], node_ids[2], node_ids[3],true);
					break;
				case MSH_CELL_TYPES::MSH_HEXA:
					this->add_hexa(*position, node_ids[0], node_ids[1], node_ids[2], node_ids[3], node_ids[4], node_ids[5], node_ids[6], node_ids[7],true);
					break;
				case MSH_CELL_TYPES::MSH_PRISM:
					this->add_triangular_prism(*position, node_ids[0], node_ids[1], node_ids[2], node_ids[3], node_ids[4], node_ids[5], true);
					break;
				case MSH_CELL_TYPES::MSH_PYRAMID:
					this->add_pyramid(*position, node_ids[0], node_ids[1], node_ids[2], node_ids[3], node_ids[4], true);
				default:
					std::cerr << "MSH Element type with index \"" << elem_type << "\" is not supported. Ignoring." << std::endl;
					break;
			}
		}
		return true;
	}

	inline bool import_ascii_msh_file(std::istream& data_stream)
	{
		ChunkArray<VEC3>* position = this->vertex_attributes_.template add_attribute<VEC3>("position");
		std::map<uint32,uint32> old_new_indices;
		std::string line;
		std::string word;
		line.reserve(512);
		word.reserve(128);

		do {
			line = this->skip_empty_lines(data_stream);
		} while (line.compare(0,6,"$Nodes") != 0 && data_stream.good());

		if (data_stream.bad())
			return false;

		std::getline(data_stream, line);
		this->nb_vertices_ = uint32(std::stoul(line));

		for (uint32 i = 0u; i < this->nb_vertices_; ++i)
		{
			std::getline(data_stream,line);

			const uint32 new_index = this->vertex_attributes_.template insert_lines<1>();
			auto& v = position->operator [](new_index);
			uint32 old_index;
			std::istringstream iss(line);
			iss >> old_index >> v[0] >> v[1] >> v[2];
			old_new_indices[old_index] = new_index;
		}

		line = this->skip_empty_lines(data_stream);
		if (line.compare(0,9,"$EndNodes") != 0)
			return false;

		while (data_stream.good() && line.compare(0,9,"$Elements") != 0)
			line = this->skip_empty_lines(data_stream);

		line = this->skip_empty_lines(data_stream);
		this->nb_volumes_ = uint32(std::stoul(line));

		for (uint32 i = 0u; i < this->nb_volumes_; ++i)
		{
			std::getline(data_stream,line);
			int32 elem_number;
			int32 elem_type;
			uint32 nb_tags;
			int32 tags_trash;


			std::istringstream iss(line);
			iss >> elem_number >> elem_type >> nb_tags;
			for (uint32 j = 0u; j < nb_tags; ++j)
				iss >> tags_trash;

			uint32 number_of_nodes;
			std::vector<uint32> node_ids;
			switch(elem_type) {
				case MSH_CELL_TYPES::MSH_TETRA:		number_of_nodes = 4u; break;
				case MSH_CELL_TYPES::MSH_PYRAMID:	number_of_nodes = 5u; break;
				case MSH_CELL_TYPES::MSH_PRISM:		number_of_nodes = 6u; break;
				case MSH_CELL_TYPES::MSH_HEXA:		number_of_nodes = 8u; break;
				default:
					std::cerr << "MSH Element type with index \"" << elem_type << "\" is not supported. Ignoring." << std::endl;
					number_of_nodes = 0u;
					break;
			}

			node_ids.resize(number_of_nodes);
			for (uint32 j = 0u; j < number_of_nodes; ++j)
			{
				iss >> node_ids[j];
				node_ids[j] = old_new_indices[node_ids[j]];
			}

			switch (elem_type) {
				case MSH_CELL_TYPES::MSH_TETRA:
					this->add_tetra(*position, node_ids[0], node_ids[1], node_ids[2], node_ids[3],true);
					break;
				case MSH_CELL_TYPES::MSH_HEXA:
					this->add_hexa(*position, node_ids[0], node_ids[1], node_ids[2], node_ids[3], node_ids[4], node_ids[5], node_ids[6], node_ids[7],true);
					break;
				case MSH_CELL_TYPES::MSH_PRISM:
					this->add_triangular_prism(*position, node_ids[0], node_ids[1], node_ids[2], node_ids[3], node_ids[4], node_ids[5], true);
					break;
				case MSH_CELL_TYPES::MSH_PYRAMID:
					this->add_pyramid(*position, node_ids[0], node_ids[1], node_ids[2], node_ids[3], node_ids[4], true);
				default:
					std::cerr << "MSH Element type with index \"" << elem_type << "\" is not supported. Ignoring." << std::endl;
					break;
			}
		}
		return true;
	}

	inline bool import_binary_msh_file(std::istream& data_stream)
	{
		ChunkArray<VEC3>* position = this->vertex_attributes_.template add_attribute<VEC3>("position");
		std::map<uint32,uint32> old_new_indices;
		std::string line;
		line.reserve(512);

		do {
			line = this->skip_empty_lines(data_stream);
		} while (line.compare(0,6,"$Nodes") != 0 && data_stream.good());

		if (data_stream.bad())
			return false;

		std::getline(data_stream, line);
		this->nb_vertices_ = uint32(std::stoul(line));

		std::vector<char> buff;
		buff.resize(this->nb_vertices_*(4u + 3u* /*this->float_size_*/ sizeof(float64)));
		data_stream.read(&buff[0], buff.size());

		for (auto it = buff.begin(), end = buff.end() ; it != end ;)
		{
			const uint32 new_index = this->vertex_attributes_.template insert_lines<1>();
			auto& v = position->operator [](new_index);

			uint32 old_index = *reinterpret_cast<uint32*>(&(*it));
			it+=4;
			v[0] = *reinterpret_cast<float64*>(&(*it));
			it+=8;
			v[1] = *reinterpret_cast<float64*>(&(*it));
			it+=8;
			v[2] = *reinterpret_cast<float64*>(&(*it));
			it+=8;
			if (this->need_endianness_swap())
			{
				old_index = swap_endianness(old_index);
				v[0] = swap_endianness(v[0]);
				v[1] = swap_endianness(v[1]);
				v[2] = swap_endianness(v[2]);
			}
			old_new_indices[old_index] = new_index;
		}

		line = this->skip_empty_lines(data_stream);
		if (line.compare(0,9,"$EndNodes") != 0)
			return false;

		while (data_stream.good() && line.compare(0,9,"$Elements") != 0)
			line = this->skip_empty_lines(data_stream);

		line = this->skip_empty_lines(data_stream);
		this->nb_volumes_ = uint32(std::stoul(line));

		for (uint32 i = 0u; i < this->nb_volumes_;)
		{
			std::array<char,12> header_buff;
			data_stream.read(&header_buff[0], header_buff.size());

			int32 elem_type		= *reinterpret_cast<int32*>(&header_buff[0]);
			int32 nb_elements	= *reinterpret_cast<int32*>(&header_buff[4]);
			int32 nb_tags		= *reinterpret_cast<int32*>(&header_buff[8]);
			if (this->need_endianness_swap())
			{
				elem_type	= swap_endianness(elem_type);
				nb_elements	= swap_endianness(nb_elements);
				nb_tags		= swap_endianness(nb_tags);
			}


			uint32 number_of_nodes;
			switch(elem_type) {
				case MSH_CELL_TYPES::MSH_TETRA:		number_of_nodes = 4u; break;
				case MSH_CELL_TYPES::MSH_PYRAMID:	number_of_nodes = 5u; break;
				case MSH_CELL_TYPES::MSH_PRISM:		number_of_nodes = 6u; break;
				case MSH_CELL_TYPES::MSH_HEXA:		number_of_nodes = 8u; break;
				default:
					std::cerr << "MSH Element type with index \"" << elem_type << "\" is not supported. Ignoring." << std::endl;
					number_of_nodes = 0u;
					break;
			}
			std::vector<char> buff;
			const uint32 elem_size = 4u + nb_tags*4u + 4u*number_of_nodes;
			buff.resize(nb_elements*elem_size);
			data_stream.read(&buff[0], buff.size());

			std::vector<uint32> node_ids(number_of_nodes);
			for (uint32 j = 0u; j < nb_elements;++j)
			{
				const char* const ids = &buff[j* elem_size + 4u + nb_tags*4u];
				for (uint32 k = 0 ; k < number_of_nodes; ++k)
				{
					node_ids[k] = *reinterpret_cast<const uint32*>(&ids[4u*k]);
					if (this->need_endianness_swap())
						node_ids[k] = swap_endianness(node_ids[k]);
					node_ids[k] = old_new_indices[node_ids[k]];
				}
				switch (elem_type) {
					case MSH_CELL_TYPES::MSH_TETRA:
						this->add_tetra(*position, node_ids[0], node_ids[1], node_ids[2], node_ids[3],true);
						break;
					case MSH_CELL_TYPES::MSH_HEXA:
						this->add_hexa(*position, node_ids[0], node_ids[1], node_ids[2], node_ids[3], node_ids[4], node_ids[5], node_ids[6], node_ids[7],true);
						break;
					case MSH_CELL_TYPES::MSH_PRISM:
						this->add_triangular_prism(*position, node_ids[0], node_ids[1], node_ids[2], node_ids[3], node_ids[4], node_ids[5], true);
						break;
					case MSH_CELL_TYPES::MSH_PYRAMID:
						this->add_pyramid(*position, node_ids[0], node_ids[1], node_ids[2], node_ids[3], node_ids[4], true);
					default:
						std::cerr << "MSH Element type with index \"" << elem_type << "\" is not supported. Ignoring." << std::endl;
						break;
				}
			}
			i += nb_elements;
		}

		return true;
	}

	virtual bool import_file_impl(const std::string& filename) override
	{
		std::ifstream fp(filename.c_str(), std::ios::in | std::ios::binary);

		bool ok = this->read_header(fp);
		if (!ok)
			return false;

		if (this->get_version_number() == "1.0")
			return this->import_legacy_msh_file(fp);
		else
		{
			if (this->get_file_type() == 0)
				return this->import_ascii_msh_file(fp);
			else
			{
				if (this->get_file_type() == 1)
					return this->import_binary_msh_file(fp);
			}
		}

		return false;
	}
};


} // namespace io
} // namespace cgogn
#endif // IO_MSH_IO_H_
