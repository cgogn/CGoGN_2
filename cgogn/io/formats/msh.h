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

#ifndef CGOGN_IO_FORMATS_MSH_H_
#define CGOGN_IO_FORMATS_MSH_H_

#include <map>

#include <cgogn/core/utils/logger.h>
#include <cgogn/io/dll.h>
#include <cgogn/io/data_io.h>
#include <cgogn/io/volume_import.h>
#include <cgogn/io/volume_export.h>
#include <cgogn/io/surface_import.h>
#include <cgogn/io/surface_export.h>

namespace cgogn
{

namespace io
{

template <typename VEC3>
class MshIO
{
public:

	using Self = MshIO<VEC3>;

	template <typename T>
	using ChunkArray = MapBaseData::ChunkArray<T>;

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

	inline MshIO() :
		version_number_(),
		file_type_(-1),
		float_size_(sizeof(float64)),
		swap_endianness_(false)
	{}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(MshIO);
	virtual ~MshIO() {}

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

	bool import_msh_file(const std::string& filename)
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

	virtual ChunkArray<VEC3>* msh_position_attribute() = 0;
	virtual uint32 msh_insert_line_vertex_container() = 0;
//	virtual void msh_clear() = 0;
	virtual void msh_reserve(uint32 n) = 0;
	virtual void msh_add_triangle(uint32 p0, uint32 p1, uint32 p2) { unused_parameters(p0, p1, p2); }
	virtual void msh_add_quad(uint32 p0, uint32 p1, uint32 p2, uint32 p3) { unused_parameters(p0, p1, p2, p3); }
	virtual void msh_add_face(const std::vector<uint32>& v_ids) { unused_parameters(v_ids); }
	virtual void msh_add_tetra(uint32 p0, uint32 p1, uint32 p2, uint32 p3, bool check_orientation) { unused_parameters(p0, p1, p2, p3, check_orientation); }
	virtual void msh_add_hexa(uint32 p0, uint32 p1, uint32 p2, uint32 p3, uint32 p4, uint32 p5, uint32 p6, uint32 p7, bool check_orientation) { unused_parameters(p0, p1, p2, p3, p4, p5, p6, p7, check_orientation); }
	virtual void msh_add_pyramid(uint32 p0, uint32 p1, uint32 p2, uint32 p3, uint32 p4, bool check_orientation) { unused_parameters(p0, p1, p2, p3, p4, check_orientation); }
	virtual void msh_add_triangular_prism(uint32 p0, uint32 p1, uint32 p2, uint32 p3, uint32 p4, uint32 p5, bool check_orientation) { unused_parameters(p0, p1, p2, p3, p4, p5, check_orientation); }
	virtual void msh_add_connector(uint32 p0, uint32 p1, uint32 p2, uint32 p3) { unused_parameters(p0, p1, p2, p3); }

	inline static std::string skip_empty_lines(std::istream& data_stream)
	{
		std::string line;
		line.reserve(1024ul);
		while(data_stream.good() && line.empty())
			getline_safe(data_stream, line);

		return line;
	}

	bool read_header(std::istream& data_stream)
	{
		std::string line;
		std::string word;
		line.reserve(512);
		word.reserve(128);
		getline_safe(data_stream, line);

		if (line == "$MeshFormat")
		{
			getline_safe(data_stream, line);
			std::istringstream iss(line);
			iss >> version_number_ >> file_type_ >> float_size_;
			if (file_type_ == 1)
			{
				std::array<char, 4> buff;
				data_stream.read(&buff[0],4);
				const int one = *reinterpret_cast<int32*>(&buff[0]);
				if (one != 1)
					swap_endianness_ = true;
			}
			getline_safe(data_stream, line); // $EndMeshFormat
		}
		else
		{
			if (line.compare(0, 4, "$NOD") == 0)
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

	bool import_legacy_msh_file(std::istream& data_stream)
	{
		position_ = this->msh_position_attribute();
		std::map<uint32,uint32> old_new_indices;
		std::string line;
		std::string word;
		line.reserve(512);
		word.reserve(128);
		line = this->skip_empty_lines(data_stream);
		const uint32 nb_vertices = uint32(std::stoul(line));

		for (uint32 i = 0u; i < nb_vertices; ++i)
		{
			getline_safe(data_stream,line);

			const uint32 new_index = this->msh_insert_line_vertex_container();
			auto& v = position_->operator[](new_index);
			uint32 old_index;
			std::istringstream iss(line);
			iss >> old_index >> v[0] >> v[1] >> v[2];
			old_new_indices[old_index] = new_index;
		}

		getline_safe(data_stream,line);
		if (line.compare(0, 7, "$ENDNOD") != 0)
			return false;
		getline_safe(data_stream,line);
		if (line.compare(0, 4, "$ELM") != 0)
			return false;

		getline_safe(data_stream,line);
		const uint32 nb_elements = uint32(std::stoul(line));
		this->msh_reserve(nb_elements);

		for (uint32 i = 0u; i < nb_elements; ++i)
		{
			getline_safe(data_stream,line);
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

			add_element(elem_type, node_ids);
		}
		return true;
	}

	inline bool import_ascii_msh_file(std::istream& data_stream)
	{
		position_ = this->msh_position_attribute();
		std::map<uint32,uint32> old_new_indices;
		std::string line;
		std::string word;
		line.reserve(512);
		word.reserve(128);

		do
		{
			line = this->skip_empty_lines(data_stream);
		} while (line.compare(0, 6, "$Nodes") != 0 && data_stream.good());

		if (data_stream.bad())
			return false;

		getline_safe(data_stream, line);
		const uint32 nb_vertices = uint32(std::stoul(line));

		for (uint32 i = 0u; i < nb_vertices ; ++i)
		{
			getline_safe(data_stream,line);

			const uint32 new_index = this->msh_insert_line_vertex_container();
			auto& v = position_->operator[](new_index);
			uint32 old_index;
			std::istringstream iss(line);
			iss >> old_index >> v[0] >> v[1] >> v[2];
			old_new_indices[old_index] = new_index;
		}

		line = this->skip_empty_lines(data_stream);
		if (line.compare(0, 9, "$EndNodes") != 0)
			return false;

		while (data_stream.good() && line.compare(0, 9, "$Elements") != 0)
			line = this->skip_empty_lines(data_stream);

		line = this->skip_empty_lines(data_stream);

		const uint32 nb_volumes = uint32(std::stoul(line));
		this->msh_reserve(nb_volumes);

		for (uint32 i = 0u; i < nb_volumes; ++i)
		{
			getline_safe(data_stream,line);
			int32 elem_number;
			int32 elem_type;
			uint32 nb_tags;
			int32 tags_trash;

			std::istringstream iss(line);
			iss >> elem_number >> elem_type >> nb_tags;
			for (uint32 j = 0u; j < nb_tags; ++j)
				iss >> tags_trash;

			const uint32 nb_nodes = number_of_nodes(MSH_CELL_TYPES(elem_type));
			std::vector<uint32> node_ids;

			node_ids.resize(nb_nodes);
			for (uint32 j = 0u; j < nb_nodes; ++j)
			{
				iss >> node_ids[j];
				node_ids[j] = old_new_indices[node_ids[j]];
			}

			add_element(elem_type, node_ids);
		}
		return true;
	}

	inline bool import_binary_msh_file(std::istream& data_stream)
	{
		position_ = this->msh_position_attribute();
		std::map<uint32,uint32> old_new_indices;
		std::string line;
		line.reserve(512);

		do
		{
			line = this->skip_empty_lines(data_stream);
		} while (line.compare(0, 6, "$Nodes") != 0 && data_stream.good());

		if (data_stream.bad())
			return false;

		getline_safe(data_stream, line);
		const uint32 nb_vertices = uint32(std::stoul(line));

		std::vector<char> buff;
		buff.resize(nb_vertices*(4u + 3u* /*this->float_size_*/ sizeof(float64)));
		data_stream.read(&buff[0], buff.size());

		for (auto it = buff.begin(), end = buff.end(); it != end ; )
		{
			const uint32 new_index = this->msh_insert_line_vertex_container();
			auto& v = position_->operator[](new_index);
			using LocScalar = decltype(v[0]);
			uint32 old_index = *reinterpret_cast<uint32*>(&(*it));
			it+=4;
			v[0] = LocScalar(*reinterpret_cast<float64*>(&(*it)));
			it+=8;
			v[1] = LocScalar(*reinterpret_cast<float64*>(&(*it)));
			it+=8;
			v[2] = LocScalar(*reinterpret_cast<float64*>(&(*it)));
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
		if (line.compare(0, 9, "$EndNodes") != 0)
			return false;

		while (data_stream.good() && line.compare(0, 9, "$Elements") != 0)
			line = this->skip_empty_lines(data_stream);

		line = this->skip_empty_lines(data_stream);

		const uint32 nb_volumes = uint32(std::stoul(line));
		this->msh_reserve(nb_volumes);

		for (uint32 i = 0u; i < nb_volumes;)
		{
			std::array<char, 12> header_buff;
			data_stream.read(&header_buff[0], header_buff.size());

			int32 elem_type	  = *reinterpret_cast<int32*>(&header_buff[0]);
			int32 nb_elements = *reinterpret_cast<int32*>(&header_buff[4]);
			int32 nb_tags     = *reinterpret_cast<int32*>(&header_buff[8]);

			if (this->need_endianness_swap())
			{
				elem_type   = swap_endianness(elem_type);
				nb_elements = swap_endianness(nb_elements);
				nb_tags     = swap_endianness(nb_tags);
			}

			const uint32 nb_nodes = number_of_nodes(MSH_CELL_TYPES(elem_type));
			//std::vector<char> buff;
			buff.clear();
			const uint32 elem_size = 4u + nb_tags*4u + 4u*nb_nodes;
			buff.resize(nb_elements*elem_size);
			data_stream.read(&buff[0], buff.size());

			std::vector<uint32> node_ids(nb_nodes);
			for (int32 j = 0u; j < nb_elements; ++j)
			{
				const char* const ids = &buff[j* elem_size + 4u + nb_tags*4u];
				for (uint32 k = 0; k < nb_nodes; ++k)
				{
					node_ids[k] = *reinterpret_cast<const uint32*>(&ids[4u*k]);
					if (this->need_endianness_swap())
						node_ids[k] = swap_endianness(node_ids[k]);
					node_ids[k] = old_new_indices[node_ids[k]];
				}
				this->add_element(elem_type, node_ids);
			}
			i += nb_elements;
		}

		return true;
	}

private:

	void add_element(int32 elem_type, const std::vector<uint32>& node_ids)
	{
		switch (elem_type)
		{
			case MSH_CELL_TYPES::MSH_TRIANGLE:
				this->msh_add_triangle(node_ids[0], node_ids[1], node_ids[2]);
				break;
			case MSH_CELL_TYPES::MSH_QUAD:
				this->msh_add_quad(node_ids[0], node_ids[1], node_ids[2], node_ids[3]);
				break;
			case MSH_CELL_TYPES::MSH_TETRA:
				this->msh_add_tetra(node_ids[0], node_ids[1], node_ids[2], node_ids[3],true);
				break;
			case MSH_CELL_TYPES::MSH_HEXA:
				this->msh_add_hexa(node_ids[0], node_ids[1], node_ids[2], node_ids[3], node_ids[4], node_ids[5], node_ids[6], node_ids[7],true);
				break;
			case MSH_CELL_TYPES::MSH_PRISM:
				this->msh_add_triangular_prism(node_ids[0], node_ids[1], node_ids[2], node_ids[3], node_ids[4], node_ids[5], true);
				break;
			case MSH_CELL_TYPES::MSH_PYRAMID:
				this->msh_add_pyramid(node_ids[0], node_ids[1], node_ids[2], node_ids[3], node_ids[4], true);
				break;
			default:
				cgogn_log_warning("add_element") << "MSH Element type with index \"" << elem_type << "\" is not supported. Ignoring.";
				break;
		}
	}

	inline static uint32 number_of_nodes(MSH_CELL_TYPES c)
	{
		switch(c)
		{
			case MSH_CELL_TYPES::MSH_TRIANGLE: return 3u;
			case MSH_CELL_TYPES::MSH_QUAD:
			case MSH_CELL_TYPES::MSH_TETRA:    return 4u;
			case MSH_CELL_TYPES::MSH_PYRAMID:  return 5u;
			case MSH_CELL_TYPES::MSH_PRISM:    return 6u;
			case MSH_CELL_TYPES::MSH_HEXA:     return 8u;
			default:
				cgogn_log_warning("number_of_nodes") << "MSH Element type with index \"" << c << "\" is not supported. Ignoring.";
				return 0;
		}
	}

private:

	std::string	version_number_;
	int32       file_type_;
	int32       float_size_; // sizeof(double) normally since the doc says "currently only data-size = sizeof(double) is supported"
	bool        swap_endianness_;

protected:
	ChunkArray<VEC3>* position_;
};


template <typename MAP, typename VEC3>
class MshSurfaceImport : public MshIO<VEC3>, public SurfaceFileImport<MAP>
{
public:

	using Self = MshSurfaceImport<MAP, VEC3>;
	using Inherit_Msh = MshIO<VEC3>;
	using Inherit_Import = SurfaceFileImport<MAP>;
	using MSH_CELL_TYPES = typename Inherit_Msh::MSH_CELL_TYPES;
	template <typename T>
	using ChunkArray = typename Inherit_Import::template ChunkArray<T>;

	inline MshSurfaceImport(MAP& map) : Inherit_Import(map) {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(MshSurfaceImport);
	virtual ~MshSurfaceImport() override {}

protected:

	// FileImport interface
	virtual bool import_file_impl(const std::string& filename) override
	{
		return this->import_msh_file(filename);
	}

	// MshIO interface
	virtual ChunkArray<VEC3>* msh_position_attribute() override
	{
		return this->template add_vertex_attribute<VEC3>("position");
	}

	virtual uint32 msh_insert_line_vertex_container() override
	{
		return this->insert_line_vertex_container();
	}

//	virtual void msh_clear() override
//	{
//		this->clear();
//	}

	virtual void msh_reserve(uint32 n) override
	{
		this->reserve(n);
	}

	virtual void msh_add_triangle(uint32 p0, uint32 p1, uint32 p2) override
	{
		this->add_triangle(p0, p1, p2);
	}

	virtual void msh_add_quad(uint32 p0, uint32 p1, uint32 p2, uint32 p3) override
	{
		this->add_quad(p0,p1,p2,p3);
	}

	virtual void msh_add_face(const std::vector<uint32>& v_ids) override
	{
		this->add_face(v_ids);
	}
};

template <typename MAP, typename VEC3>
class MshVolumeImport : public MshIO<VEC3>, public VolumeFileImport<MAP>
{
public:

	using Self = MshVolumeImport<MAP, VEC3>;
	using Inherit_Msh = MshIO<VEC3>;
	using Inherit_Import = VolumeFileImport<MAP>;
	using Scalar = typename VEC3::Scalar;
	using MSH_CELL_TYPES = typename Inherit_Msh::MSH_CELL_TYPES;
	template <typename T>
	using ChunkArray = typename Inherit_Import::template ChunkArray<T>;

	inline MshVolumeImport(MAP& map) : Inherit_Import(map) {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(MshVolumeImport);
	virtual ~MshVolumeImport() override {}

protected:

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

protected:

	// MshIO interface
	virtual ChunkArray<VEC3>* msh_position_attribute() override
	{
		return this->template add_vertex_attribute<VEC3>("position");
	}

	virtual uint32 msh_insert_line_vertex_container() override
	{
		return this->insert_line_vertex_container();
	}

//	virtual void msh_clear() override
//	{
//		this->clear();
//	}

	virtual void msh_reserve(uint32 n) override
	{
		this->reserve(n);
	}

	virtual void msh_add_tetra(uint32 p0, uint32 p1, uint32 p2, uint32 p3, bool check_orientation) override
	{
		if(check_orientation)
			this->template reorient_tetra<VEC3>(*(this->position_), p0, p1, p2, p3);
		this->add_tetra(p0,p1,p2,p3);
	}

	virtual void msh_add_hexa(uint32 p0, uint32 p1, uint32 p2, uint32 p3, uint32 p4, uint32 p5, uint32 p6, uint32 p7, bool check_orientation) override
	{
		if(check_orientation)
			this->template reorient_hexa<VEC3>(*(this->position_), p0, p1, p2, p3, p4, p5, p6, p7);
		this->add_hexa(p0,p1,p2,p3,p4,p5,p6,p7);
	}

	virtual void msh_add_pyramid(uint32 p0, uint32 p1, uint32 p2, uint32 p3, uint32 p4, bool check_orientation) override
	{
		if(check_orientation)
			this->template reorient_pyramid<VEC3>(*(this->position_), p0, p1, p2, p3, p4);
		this->add_pyramid(p0,p1,p2,p3,p4);
	}

	virtual void msh_add_triangular_prism(uint32 p0, uint32 p1, uint32 p2, uint32 p3, uint32 p4, uint32 p5, bool check_orientation) override
	{
		if(check_orientation)
			this->template reorient_triangular_prism<VEC3>(*(this->position_), p0, p1, p2, p3, p4, p5);
		this->add_triangular_prism(p0,p1,p2,p3,p4,p5);
	}

	virtual void msh_add_connector(uint32 p0, uint32 p1, uint32 p2, uint32 p3) override
	{
		this->add_connector(p0,p1,p2,p3);
	}
};

template <typename MAP>
class MshVolumeExport : public VolumeExport<MAP>
{
public:

	using Inherit = VolumeExport<MAP>;
	using Self = MshVolumeExport<MAP>;
	using Map = typename Inherit::Map;
	using MSH_CELL_TYPES = typename MshIO<Eigen::Vector3d>::MSH_CELL_TYPES;
	using Vertex = typename Inherit::Vertex;
	using Volume = typename Inherit::Volume;
	using ChunkArrayGen = typename Inherit::ChunkArrayGen;

protected:

	virtual void export_file_impl(const Map& map, std::ofstream& output, const ExportOptions& option) override
	{
		ChunkArrayGen const* position = this->position_attribute(Vertex::ORBIT);
		const std::string endianness = cgogn::internal::cgogn_is_little_endian ? "LittleEndian" : "BigEndian";
		const std::string format = (option.binary_?"binary" :"ascii");
		std::string scalar_type = position->nested_type_name();
		scalar_type[0] = std::toupper(scalar_type[0], std::locale());

		// 1. vertices
		output << "$NOD" << std::endl;
		output << this->nb_vertices() << std::endl;
		uint32 vertices_counter = 1u;
		map.foreach_cell([&](Vertex v)
		{
			output << vertices_counter++ << " ";
			position->export_element(map.embedding(v), output, false, false);
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
			const MSH_CELL_TYPES type = (nbv == 4u)?MSH_CELL_TYPES::MSH_TETRA:(nbv == 5u)?MSH_CELL_TYPES::MSH_PYRAMID:(nbv == 6u)?MSH_CELL_TYPES::MSH_PRISM:MSH_CELL_TYPES::MSH_HEXA;
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

template <typename MAP>
class MshSurfaceExport : public SurfaceExport<MAP>
{
public:

	using MSH_CELL_TYPES = typename MshIO<Eigen::Vector3d>::MSH_CELL_TYPES;
	using Inherit = SurfaceExport<MAP>;
	using Self = MshSurfaceExport<MAP>;
	using Map = typename Inherit::Map;
	using Vertex = typename Inherit::Vertex;
	using Face = typename Inherit::Face;
	using ChunkArrayGen = typename Inherit::ChunkArrayGen;

protected:

	virtual void export_file_impl(const Map& map, std::ofstream& output, const ExportOptions& option) override
	{
		unused_parameters(option);
		ChunkArrayGen const* position = this->position_attribute(Vertex::ORBIT);
//		const std::string endianness = cgogn::internal::cgogn_is_little_endian ? "LittleEndian" : "BigEndian";
//		const std::string format = (option.binary_?"binary" :"ascii");
//		std::string scalar_type = position->nested_type_name();
//		scalar_type[0] = std::toupper(scalar_type[0], std::locale());

		// 1. vertices
		output << "$NOD" << std::endl;
		output << this->nb_vertices() << std::endl;
		uint32 vertices_counter = 1u;
		map.foreach_cell([&](Vertex v)
		{
			output << vertices_counter++ << " ";
			position->export_element(map.embedding(v), output, false, false);
			output << std::endl;
		}, *(this->cell_cache_));
		output << "$ENDNOD" << std::endl;

		// 2. faces
		output << "$ELM" << std::endl;
		const uint32 nb_faces = this->nb_faces();
		output << nb_faces << std::endl;

		uint32 cell_counter = 1u;
		std::vector<uint32> vertices;
		map.foreach_cell([&](Face f)
		{
			vertices.reserve(4u);
			map.foreach_incident_vertex(f, [&] (Vertex v) {vertices.push_back(this->vindices_[v] + 1u);});
			const std::size_t nbv = vertices.size();
			const MSH_CELL_TYPES type = (nbv == 3u)? MSH_CELL_TYPES::MSH_TRIANGLE :(nbv == 4u)?MSH_CELL_TYPES::MSH_QUAD: MSH_CELL_TYPES(0);
			output << cell_counter++ << " " <<  type <<" 1 1 " <<  nbv <<" ";
			for (const auto i : vertices)
			{
				output << i << " ";
			}
			output << std::endl;
			vertices.clear();
		}, *(this->cell_cache_));
		output << "$ENDELM" << std::endl;
	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_FORMATS_MSH_CPP_))
extern template class CGOGN_IO_API MshIO<Eigen::Vector3d>;
extern template class CGOGN_IO_API MshIO<Eigen::Vector3f>;
extern template class CGOGN_IO_API MshIO<geometry::Vec_T<std::array<float64,3>>>;
extern template class CGOGN_IO_API MshIO<geometry::Vec_T<std::array<float32,3>>>;

extern template class CGOGN_IO_API MshSurfaceImport<CMap2, Eigen::Vector3d>;
extern template class CGOGN_IO_API MshSurfaceImport<CMap2, Eigen::Vector3f>;

extern template class CGOGN_IO_API MshVolumeImport<CMap3, Eigen::Vector3d>;
extern template class CGOGN_IO_API MshVolumeImport<CMap3, Eigen::Vector3f>;
extern template class CGOGN_IO_API MshVolumeImport<CMap3, geometry::Vec_T<std::array<float64,3>>>;
extern template class CGOGN_IO_API MshVolumeImport<CMap3, geometry::Vec_T<std::array<float32,3>>>;

extern template class CGOGN_IO_API MshSurfaceExport<CMap2>;
extern template class CGOGN_IO_API MshVolumeExport<CMap3>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_FORMATS_MSH_CPP_))

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_FORMATS_MSH_H_
