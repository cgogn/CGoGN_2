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

#ifndef CGOGN_IO_MSH_IO_H_
#define CGOGN_IO_MSH_IO_H_

#include <map>

#include <cgogn/core/utils/logger.h>
#include <cgogn/io/dll.h>
#include <cgogn/io/data_io.h>
#include <cgogn/io/volume_import.h>
#include <cgogn/io/volume_export.h>

namespace cgogn
{

namespace io
{

template <uint32 CHUNK_SIZE, uint32 PRIM_SIZE, typename VEC3>
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

	inline const std::string& version_number() const
	{
		return version_number_;
	}

	inline int32 file_type() const
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
		}
		else
		{
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

template <typename MAP_TRAITS, typename VEC3>
class MshVolumeImport : public MshIO<MAP_TRAITS::CHUNK_SIZE, CMap3<MAP_TRAITS>::PRIM_SIZE, VEC3>, public VolumeImport<MAP_TRAITS>
{
public:

	using Self = MshVolumeImport<MAP_TRAITS, VEC3>;
	using Inherit_Msh = MshIO<MAP_TRAITS::CHUNK_SIZE, CMap3<MAP_TRAITS>::PRIM_SIZE, VEC3>;
	using Inherit_Import = VolumeImport<MAP_TRAITS>;
//	using DataInputGen = typename Inherit_Msh::DataInputGen;
//	template <typename T>
//	using DataInput = typename Inherit_Msh::template DataInput<T>;
	using MSH_CELL_TYPES = typename Inherit_Msh::MSH_CELL_TYPES;
	template <typename T>
	using ChunkArray = typename Inherit_Import::template ChunkArray<T>;

	inline MshVolumeImport()
	{}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(MshVolumeImport);

	virtual ~MshVolumeImport() override
	{}

	// MeshImportGen interface
protected:

	inline bool import_legacy_msh_file(std::istream& data_stream)
	{
		ChunkArray<VEC3>* position = this->template position_attribute<VEC3>();
		std::map<uint32,uint32> old_new_indices;
		std::string line;
		std::string word;
		line.reserve(512);
		word.reserve(128);
		line = this->skip_empty_lines(data_stream);
		this->set_nb_vertices(uint32(std::stoul(line)));

		for (uint32 i = 0u, end = this->nb_vertices(); i < end; ++i)
		{
			std::getline(data_stream,line);

			const uint32 new_index = this->insert_line_vertex_container();
			auto& v = position->operator[](new_index);
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
		this->set_nb_volumes(uint32(std::stoul(line)));

		for (uint32 i = 0u, end = this->nb_volumes(); i < end; ++i)
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

			switch (elem_type)
			{
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
					cgogn_log_warning("import_legacy_msh_file") << "MSH Element type with index \"" << elem_type << "\" is not supported. Ignoring.";
					break;
			}
		}
		return true;
	}

	inline bool import_ascii_msh_file(std::istream& data_stream)
	{
		ChunkArray<VEC3>* position = this->template position_attribute<VEC3>();
		std::map<uint32,uint32> old_new_indices;
		std::string line;
		std::string word;
		line.reserve(512);
		word.reserve(128);

		do
		{
			line = this->skip_empty_lines(data_stream);
		} while (line.compare(0,6,"$Nodes") != 0 && data_stream.good());

		if (data_stream.bad())
			return false;

		std::getline(data_stream, line);
		this->set_nb_vertices(uint32(std::stoul(line)));

		for (uint32 i = 0u, end = this->nb_vertices(); i < end ; ++i)
		{
			std::getline(data_stream,line);

			const uint32 new_index = this->insert_line_vertex_container();
			auto& v = position->operator[](new_index);
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
		this->set_nb_volumes(uint32(std::stoul(line)));

		for (uint32 i = 0u, end = this->nb_volumes(); i < end; ++i)
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
			switch(elem_type)
			{
				case MSH_CELL_TYPES::MSH_TETRA:		number_of_nodes = 4u; break;
				case MSH_CELL_TYPES::MSH_PYRAMID:	number_of_nodes = 5u; break;
				case MSH_CELL_TYPES::MSH_PRISM:		number_of_nodes = 6u; break;
				case MSH_CELL_TYPES::MSH_HEXA:		number_of_nodes = 8u; break;
				default:
					cgogn_log_warning("import_ascii_msh_file") << "MSH Element type with index \"" << elem_type << "\" is not supported. Ignoring.";
					number_of_nodes = 0u;
					break;
			}

			node_ids.resize(number_of_nodes);
			for (uint32 j = 0u; j < number_of_nodes; ++j)
			{
				iss >> node_ids[j];
				node_ids[j] = old_new_indices[node_ids[j]];
			}

			switch (elem_type)
			{
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
					cgogn_log_warning("import_ascii_msh_file") << "MSH Element type with index \"" << elem_type << "\" is not supported. Ignoring.";
					break;
			}
		}
		return true;
	}

	inline bool import_binary_msh_file(std::istream& data_stream)
	{
		ChunkArray<VEC3>* position = this->template position_attribute<VEC3>();
		std::map<uint32,uint32> old_new_indices;
		std::string line;
		line.reserve(512);

		do
		{
			line = this->skip_empty_lines(data_stream);
		} while (line.compare(0,6,"$Nodes") != 0 && data_stream.good());

		if (data_stream.bad())
			return false;

		std::getline(data_stream, line);
		this->set_nb_vertices(uint32(std::stoul(line)));

		std::vector<char> buff;
		buff.resize(this->nb_vertices()*(4u + 3u* /*this->float_size_*/ sizeof(float64)));
		data_stream.read(&buff[0], buff.size());

		for (auto it = buff.begin(), end = buff.end(); it != end ; )
		{
			const uint32 new_index = this->insert_line_vertex_container();
			auto& v = position->operator[](new_index);
			using Scalar = decltype(v[0]);
			uint32 old_index = *reinterpret_cast<uint32*>(&(*it));
			it+=4;
			v[0] = Scalar(*reinterpret_cast<float64*>(&(*it)));
			it+=8;
			v[1] = Scalar(*reinterpret_cast<float64*>(&(*it)));
			it+=8;
			v[2] = Scalar(*reinterpret_cast<float64*>(&(*it)));
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
		this->set_nb_volumes(uint32(std::stoul(line)));

		for (uint32 i = 0u, end = this->nb_volumes(); i < end;)
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
			switch(elem_type)
			{
				case MSH_CELL_TYPES::MSH_TETRA:		number_of_nodes = 4u; break;
				case MSH_CELL_TYPES::MSH_PYRAMID:	number_of_nodes = 5u; break;
				case MSH_CELL_TYPES::MSH_PRISM:		number_of_nodes = 6u; break;
				case MSH_CELL_TYPES::MSH_HEXA:		number_of_nodes = 8u; break;
				default:
					cgogn_log_warning("import_binary_msh_file") << "MSH Element type with index \"" << elem_type << "\" is not supported. Ignoring.";
					number_of_nodes = 0u;
					break;
			}
			//std::vector<char> buff;
			buff.clear();
			const uint32 elem_size = 4u + nb_tags*4u + 4u*number_of_nodes;
			buff.resize(nb_elements*elem_size);
			data_stream.read(&buff[0], buff.size());

			std::vector<uint32> node_ids(number_of_nodes);
			for (int32 j = 0u; j < nb_elements; ++j)
			{
				const char* const ids = &buff[j* elem_size + 4u + nb_tags*4u];
				for (uint32 k = 0; k < number_of_nodes; ++k)
				{
					node_ids[k] = *reinterpret_cast<const uint32*>(&ids[4u*k]);
					if (this->need_endianness_swap())
						node_ids[k] = swap_endianness(node_ids[k]);
					node_ids[k] = old_new_indices[node_ids[k]];
				}
				switch (elem_type)
				{
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
						cgogn_log_warning("cgogn_log_warning") << "MSH Element type with index \"" << elem_type << "\" is not supported. Ignoring.";
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

		if (this->version_number() == "1.0")
			return this->import_legacy_msh_file(fp);
		else
		{
			if (this->file_type() == 0)
				return this->import_ascii_msh_file(fp);
			else
			{
				if (this->file_type() == 1)
					return this->import_binary_msh_file(fp);
			}
		}

		return false;
	}
};

template <typename MAP>
class MshVolumeExport : public VolumeExport<MAP>
{
public:

	using Inherit = VolumeExport<MAP>;
	using Self = MshVolumeExport<MAP>;
	using Map = typename Inherit::Map;
	using Vertex = typename Inherit::Vertex;
	using Volume = typename Inherit::Volume;
	using ChunkArrayGen = typename Inherit::ChunkArrayGen;

protected:

	virtual void export_file_impl(const Map& map, std::ofstream& output, const ExportOptions& option) override
	{
		ChunkArrayGen const* pos = this->position_attribute();
		const std::string endianness = cgogn::internal::cgogn_is_little_endian ? "LittleEndian" : "BigEndian";
		const std::string format = (option.binary_?"binary" :"ascii");
		std::string scalar_type = pos->nested_type_name();
		scalar_type[0] = std::toupper(scalar_type[0], std::locale());

		// 1. vertices
		output << "$NOD" << std::endl;
		output << this->nb_vertices() << std::endl;
		uint32 vertices_counter = 1u;
		map.foreach_cell([&](Vertex v)
		{
			output << vertices_counter++ << " ";
			pos->export_element(map.embedding(v), output, false, false);
			output << std::endl;
		}, *(this->cell_cache_));
		output << "$ENDNOD" << std::endl;

		// 2. volumes
		output << "$ELM" << std::endl;
		const uint32 nb_vols = this->nb_volumes();
		output << nb_vols << std::endl;


		uint32 cell_counter = 1u;
		map.foreach_cell([&](Volume w)
		{
			const auto& vertices = this->vertices_of_volumes(w);
			const std::size_t nbv = this->number_of_vertices(w);
			const uint32 type = (nbv == 4u)?4u:(nbv == 5u)?7u:(nbv == 6u)?6u:5u;
			output << cell_counter++ << " " <<  type <<" 1 1 " <<  nbv <<" ";
			for (const auto i : vertices)
			{
				output << i + 1 << " ";
			}
			output << std::endl;
		}, *(this->cell_cache_));
		output << "$ENDELM" << std::endl;
	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_MSH_IO_CPP_))
extern template class CGOGN_IO_API MshIO<DefaultMapTraits::CHUNK_SIZE,1, Eigen::Vector3d>;
extern template class CGOGN_IO_API MshIO<DefaultMapTraits::CHUNK_SIZE,1, Eigen::Vector3f>;
extern template class CGOGN_IO_API MshIO<DefaultMapTraits::CHUNK_SIZE,1, geometry::Vec_T<std::array<float64,3>>>;
extern template class CGOGN_IO_API MshIO<DefaultMapTraits::CHUNK_SIZE,1, geometry::Vec_T<std::array<float32,3>>>;

extern template class CGOGN_IO_API MshVolumeImport<DefaultMapTraits, Eigen::Vector3d>;
extern template class CGOGN_IO_API MshVolumeImport<DefaultMapTraits, Eigen::Vector3f>;
extern template class CGOGN_IO_API MshVolumeImport<DefaultMapTraits, geometry::Vec_T<std::array<float64,3>>>;
extern template class CGOGN_IO_API MshVolumeImport<DefaultMapTraits, geometry::Vec_T<std::array<float32,3>>>;

extern template class CGOGN_IO_API MshVolumeExport<CMap3<DefaultMapTraits>>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_MSH_IO_CPP_))


} // namespace io
} // namespace cgogn
#endif // CGOGN_IO_MSH_IO_H_
