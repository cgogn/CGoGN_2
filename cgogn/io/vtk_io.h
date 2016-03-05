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

#ifndef IO_VTK_IO_H_
#define IO_VTK_IO_H_

#include <istream>
#include <sstream>

#include <io/data_io.h>
#include <io/surface_import.h>
#include <io/volume_import.h>

namespace cgogn
{

namespace io
{

template<unsigned int CHUNK_SIZE, unsigned int PRIM_SIZE, typename VEC3>
class VtkIO
{
public :
	enum VTK_CELL_TYPES
	{
		VTK_VERTEX = 1,
		VTK_POLY_VERTEX = 2,
		VTK_LINE = 3,
		VTK_POLY_LINE = 4,
		VTK_TRIANGLE = 5,
		VTK_TRIANGLE_STRIP = 6,
		VTK_POLYGON = 7,
		VTK_PIXEL = 8,
		VTK_QUAD = 9,

		VTK_TETRA = 10,
		VTK_VOXEL = 11,
		VTK_HEXAHEDRON = 12,
		VTK_WEDGE = 13,
		VTK_PYRAMID = 14,

		VTK_QUADRATIC_EDGE = 21,
		VTK_QUADRATIC_TRIANGLE = 22,
		VTK_QUADRATIC_QUAD = 23,
		VTK_QUADRATIC_TETRA = 24,
		VTK_QUADRATIC_HEXAHEDRON = 25
	};


	enum VTK_MESH_TYPE
	{
		UNKNOWN = 0,
		UNSTRUCTURED_GRID,
		POLYDATA
	};

	using Self = VtkIO<DEFAULT_CHUNK_SIZE, PRIM_SIZE, VEC3>;
	using DataInputGen = cgogn::io::DataInputGen<CHUNK_SIZE>;
	template<typename T>
	using DataInput = cgogn::io::DataInput<CHUNK_SIZE, PRIM_SIZE, T>;
	using Scalar = typename VEC3::Scalar;

	virtual ~VtkIO() {}

protected :
	std::unique_ptr<DataInputGen>	positions_;
	DataInput<unsigned int>			cells_;
	DataInput<int>					cell_types_;

protected :
	virtual void add_vertex_attribute(const DataInputGen& attribute_data, const std::string& attribute_name) = 0;
	virtual void add_cell_attribute(const DataInputGen& attribute_data, const std::string& attribute_name) = 0;

	bool parse_vtk_legacy_file(std::ifstream& fp)
	{
		VTK_MESH_TYPE vtk_type(VTK_MESH_TYPE::UNKNOWN);

		std::cout << "Opening a legacy vtk file" << std::endl;

		std::string line;
		std::string word;
		line.reserve(512);
		word.reserve(128);

		// printing the 2 first lines
		std::getline(fp, line);
		std::cout << line << std::endl;
		std::getline(fp, line);
		std::cout << line << std::endl;

		fp >> word;
		bool ascii_file = to_upper(word) == "ASCII";
		cgogn_assert(ascii_file || to_upper(word) == "BINARY");

		fp >> word;
		cgogn_assert(to_upper(word) == "DATASET");
		fp >> word;
		const std::string& dataset = to_upper(word);
		if (dataset == "UNSTRUCTURED_GRID")
			vtk_type = VTK_MESH_TYPE::UNSTRUCTURED_GRID;
		else {
			if (dataset == "POLYDATA")
				vtk_type = VTK_MESH_TYPE::POLYDATA;
		}

		unsigned int nb_vertices = 0u;
		unsigned int nb_cells = 0u;

		if (vtk_type == VTK_MESH_TYPE::UNSTRUCTURED_GRID || vtk_type == VTK_MESH_TYPE::POLYDATA)
		{
			while(!fp.eof())
			{
				std::getline(fp,line);
				word.clear();
				std::istringstream sstream(line);
				sstream >> word;
				word = to_upper(word);

				if (word == "POINTS")
				{
					std::string type_str;
					sstream >> nb_vertices >> type_str;
					type_str = to_lower(type_str);
					positions_ = DataInputGen::template newDataIO<PRIM_SIZE, VEC3>(type_str, 3);
					positions_->read_n(fp, nb_vertices, !ascii_file, false);
					this->add_vertex_attribute(*positions_,"position");
				} else {
					if (word == "CELLS" || word == "POLYGONS" || word == "TRIANGLE_STRIPS")
					{
						unsigned int size;
						sstream >> nb_cells >> size;
						cells_.read_n(fp, size, !ascii_file, false);

						std::vector<int>* cell_types_vec = static_cast<std::vector<int>*>(cell_types_.get_data());
						cgogn_assert(cell_types_vec != nullptr);
						if (word == "POLYGONS")
						{
							cell_types_vec->reserve(nb_cells);
							for (unsigned i = 0u; i < nb_cells ;++i)
							{
								cell_types_vec->push_back(VTK_CELL_TYPES::VTK_POLYGON);
							}
						} else if (word == "TRIANGLE_STRIPS")
						{
							cell_types_vec->reserve(nb_cells);
							for (unsigned i = 0u; i < nb_cells ;++i)
							{
								cell_types_vec->push_back(VTK_CELL_TYPES::VTK_TRIANGLE_STRIP);
							}
						}

					} else {
						if (word == "CELL_TYPES")
						{
							unsigned int nbc;
							sstream >> nbc;
							cell_types_.read_n(fp, nbc, !ascii_file, false);
						} else {
							if (word == "POINT_DATA" || word == "CELL_DATA")
							{
								const bool cell_data = (word == "CELL_DATA");
								unsigned int nb_data;
								sstream >> nb_data;

								if (!cell_data)
								{
									cgogn_assert( nb_vertices == 0u || nb_data == nb_vertices );
								}
								std::ifstream::pos_type previous_pos;
								do {
									previous_pos = fp.tellg();
									std::getline(fp, line);
									sstream.str(line);
									sstream.clear();
									word.clear();
									sstream >> word;
									word = to_upper(word);
									if (word == "SCALARS" || word == "VECTOR" || word == "NORMALS")
									{
										const bool is_vector = !(word == "SCALARS");
										std::string att_name;
										std::string att_type;
										unsigned int num_comp = is_vector? 3u : 1u;
										sstream >> att_name >> att_type >> num_comp;
										std::cout << "reading attribute \"" << att_name << "\" of type " << att_type << " (" << num_comp << " components)." << std::endl;

										const auto pos_before_lookup_table = fp.tellg(); // the lookup table might (or might not) be defined
										std::getline(fp,line);
										sstream.str(line);
										sstream.clear();
										std::string lookup_table;
										std::string lookup_table_name;
										sstream >> lookup_table >> lookup_table_name;
										if (to_upper(lookup_table) == "LOOKUP_TABLE")
										{
											std::cout << "VTK import : ignoring lookup table named \"" << lookup_table_name << "\"" << std::endl;
										} else {
											fp.seekg(pos_before_lookup_table); // if there wasn't a lookup table we go back and start reading the numerical values
										}

										std::unique_ptr<DataInputGen> att(DataInputGen::template newDataIO<PRIM_SIZE>(att_type, num_comp));
										att->read_n(fp, nb_data, !ascii_file, false);
										if (cell_data)
											this->add_cell_attribute(*att, att_name);
										else
											this->add_vertex_attribute(*att, att_name);
									} else {
										if (word == "FIELD")
										{
											std::string field_name;
											unsigned int num_arrays = 0u;
											sstream >> field_name >> num_arrays;
											for (unsigned int i = 0u ; i< num_arrays; ++i)
											{
												do {
													std::getline(fp,line);
												} while (line.empty());

												sstream.str(line);
												sstream.clear();
												std::string		data_name;
												unsigned int	nb_comp;
												unsigned int	nb_data;
												std::string		data_type;
												sstream >> data_name >> nb_comp >> nb_data >> data_type;
												std::cout << "reading field \"" << data_name << "\" of type " << data_type << " (" << nb_comp << " components)." << std::endl;
												std::unique_ptr<DataInputGen> att(DataInputGen::template newDataIO<PRIM_SIZE>(data_type, nb_comp));
												att->read_n(fp, nb_data, !ascii_file, false);
												if (cell_data)
													this->add_cell_attribute(*att, data_name);
												else
													this->add_vertex_attribute(*att, data_name);
											}
										} else
											if (word == "LOOKUP_TABLE")
											{
												std::string table_name;
												unsigned int nb_data = 0u;
												sstream >> table_name >> nb_data;
												std::cout << "ignoring the definition of the lookuptable named \"" << table_name << "\"" << std::endl;
												if (ascii_file)
												{
													DataInput<Eigen::Vector4f> trash;
													trash.skip_n(fp, nb_data, false);
												} else
												{
													DataInput<std::int32_t> trash;
													trash.skip_n(fp, nb_data, true);
												}
											}
									}
								} while (word != "POINT_DATA" && word != "CELL_DATA" && (!fp.eof()));
								if (!fp.eof())
								{
									fp.seekg(previous_pos);
									word.clear();
								} else
									break;
							} else
							{
								if (!word.empty())
									std::cerr << "VTK keyword \"" << word << "\" is not supported." << std::endl;
							}
						}
					}
				}
			}
		}
		return true;
	}

	static inline std::string vtk_data_type_to_cgogn_name_of_type(const std::string& vtk_type_str)
	{
		const std::string& data_type = to_lower(vtk_type_str);
		if (data_type == "char" || data_type == "int8")
			return name_of_type(std::int8_t());
		if (data_type == "unsigned_char" || data_type == "uint8")
			return name_of_type(std::uint8_t());
		if (data_type == "short" || data_type == "int16")
			return name_of_type(std::int16_t());
		if (data_type == "unsigned_short" || data_type == "uint16")
			return name_of_type(std::uint16_t());
		if (data_type == "int" || data_type == "int32")
			return name_of_type(std::int32_t());
		if (data_type == "unsigned_int" || data_type == "uint32")
			return name_of_type(std::uint32_t());
		if (data_type == "long" || data_type == "int64")
			return name_of_type(std::int64_t());
		if (data_type == "unsigned_long" || data_type == "uint64")
			return name_of_type(std::uint64_t());
		if (data_type == "float"  || data_type == "float32")
			return name_of_type(float());
		if (data_type == "double" || data_type == "float64")
			return name_of_type(double());

		std::cerr << "vtk_data_type_to_cgogn_name_of_type : unknown vtk type : " << vtk_type_str << std::endl;
		return std::string();
	}
};


template<typename MAP_TRAITS, typename VEC3>
class VtkSurfaceImport : public VtkIO<MAP_TRAITS::CHUNK_SIZE, CMap2<MAP_TRAITS>::PRIM_SIZE, VEC3>, public SurfaceImport<MAP_TRAITS>
{
public:
	using Self = VtkSurfaceImport<MAP_TRAITS, VEC3>;
	using Inherit_Vtk = VtkIO<MAP_TRAITS::CHUNK_SIZE, CMap2<MAP_TRAITS>::PRIM_SIZE, VEC3>;
	using Inherit_Import = SurfaceImport<MAP_TRAITS>;
	using DataInputGen = typename Inherit_Vtk::DataInputGen;
	template<typename T>
	using DataInput = typename Inherit_Vtk::template DataInput<T>;
	using VTK_CELL_TYPES = typename Inherit_Vtk::VTK_CELL_TYPES;

	virtual ~VtkSurfaceImport() override {}
protected:
	inline bool read_vtk_legacy_file(std::ifstream& fp)
	{
		if (!Inherit_Vtk::parse_vtk_legacy_file(fp))
			return false;

		this->nb_vertices_ = this->positions_->size();
		this->nb_faces_ = this->cell_types_.size();

		auto cells_it = static_cast<std::vector<unsigned int>*>(this->cells_.get_data())->begin();
		const std::vector<int>* cell_types_vec = static_cast<std::vector<int>*>(this->cell_types_.get_data());
		for(auto cell_types_it = cell_types_vec->begin(); cell_types_it != cell_types_vec->end() ; )
		{
			const std::size_t nb_vert = *(cells_it++);
			const int cell_type = *(cell_types_it++);

			if (cell_type != VTK_CELL_TYPES::VTK_TRIANGLE_STRIP)
			{
				this->faces_nb_edges_.push_back(nb_vert);
				for (std::size_t i = 0ul ; i < nb_vert;++i)
				{
					this->faces_vertex_indices_.push_back(*cells_it++);
				}
			} else {
				std::vector<unsigned int> vertexIDS;
				vertexIDS.reserve(nb_vert);
				for (std::size_t i = 0ul ; i < nb_vert;++i)
				{
					vertexIDS.push_back(*cells_it++);
				}

				for (unsigned int i = 0u ; i < nb_vert -2u; ++i)
				{
					if (i != 0u)
						++this->nb_faces_;
					this->faces_nb_edges_.push_back(3);
					this->faces_vertex_indices_.push_back(vertexIDS[i]);
					this->faces_vertex_indices_.push_back(vertexIDS[i+1]);
					this->faces_vertex_indices_.push_back(vertexIDS[i+2]);
					if (i % 2u == 0u)
						std::swap(this->faces_vertex_indices_.back(), *(this->faces_vertex_indices_.end()-2));
				}
			}
		}
		return true;
	}

	virtual void add_vertex_attribute(const DataInputGen& attribute_data, const std::string& attribute_name) override
	{
		attribute_data.to_chunk_array(attribute_data.add_attribute(this->vertex_attributes_, attribute_name));
	}
	virtual void add_cell_attribute(const DataInputGen& attribute_data, const std::string& attribute_name) override
	{
		attribute_data.to_chunk_array(attribute_data.add_attribute(this->face_attributes_, attribute_name));
	}
	virtual bool import_file_impl(const std::string& filename)
	{
		std::ifstream fp(filename.c_str(), std::ios::in | std::ios::binary);
		cgogn_assert(fp.good());
		const FileType file_type = get_file_type(filename);
		if (file_type == FileType::FileType_VTK_LEGACY)
			return this->read_vtk_legacy_file(fp);
	}
};


template<typename MAP_TRAITS, typename VEC3>
class VtkVolumeImport : public VtkIO<MAP_TRAITS::CHUNK_SIZE, CMap3<MAP_TRAITS>::PRIM_SIZE, VEC3>, public VolumeImport<MAP_TRAITS>
{
public:
	using Self = VtkVolumeImport<MAP_TRAITS, VEC3>;
	using Inherit_Vtk = VtkIO<MAP_TRAITS::CHUNK_SIZE, CMap3<MAP_TRAITS>::PRIM_SIZE, VEC3>;
	using Inherit_Import = VolumeImport<MAP_TRAITS>;
	using DataInputGen = typename Inherit_Vtk::DataInputGen;
	template<typename T>
	using DataInput = typename Inherit_Vtk::template DataInput<T>;
	using VTK_CELL_TYPES = typename Inherit_Vtk::VTK_CELL_TYPES;
	template <typename T>
	using ChunkArray = typename Inherit_Import::template ChunkArray<T>;

	virtual ~VtkVolumeImport() override {}

protected:
	inline bool read_vtk_legacy_file(std::ifstream& fp)
	{
		if (!Inherit_Vtk::parse_vtk_legacy_file(fp))
			return false;

		this->nb_vertices_ = this->positions_->size();
		this->nb_volumes_ = this->cell_types_.size();

		const std::vector<int>* cell_types_vec	= this->cell_types_.get_vec();
		const std::vector<unsigned int>* cells_vec	=this->cells_.get_vec();

		std::vector<unsigned int> cells_buffer;
		cells_buffer.reserve(cells_vec->size());

		// in the legacy file , the first number of each line is the number of vertices. We need to remove it.
		auto cells_it = cells_vec->begin();
		for (std::vector<int>::const_iterator type_it = cell_types_vec->begin(), end = cell_types_vec->end(); type_it != end; ++type_it)
		{
			++cells_it;
			unsigned int vol_nb_verts = 0u;
			if (*type_it == VTK_CELL_TYPES::VTK_TETRA)
				vol_nb_verts = 4u;
			else {
				if (*type_it == VTK_CELL_TYPES::VTK_HEXAHEDRON || *type_it == VTK_CELL_TYPES::VTK_VOXEL)
					vol_nb_verts = 8u;
				else {
					if (*type_it == VTK_CELL_TYPES::VTK_WEDGE)
						vol_nb_verts = 6u;
					else {
						if (*type_it == VTK_CELL_TYPES::VTK_PYRAMID)
							vol_nb_verts = 5u;
					}
				}
			}
			for (unsigned int i = 0u ; i < vol_nb_verts;++i)
			{
				cells_buffer.push_back(*cells_it++);
			}
		}


		add_vtk_volumes(cells_buffer,*cell_types_vec, *(this->vertex_attributes_.template get_attribute<VEC3>("position")));

		return true;
	}

	virtual void add_vertex_attribute(const DataInputGen& attribute_data, const std::string& attribute_name) override
	{
		attribute_data.to_chunk_array(attribute_data.add_attribute(this->vertex_attributes_, attribute_name));
	}
	virtual void add_cell_attribute(const DataInputGen& attribute_data, const std::string& attribute_name) override
	{
		attribute_data.to_chunk_array(attribute_data.add_attribute(this->volume_attributes_, attribute_name));
	}

	virtual bool import_file_impl(const std::string& filename)
	{
		const FileType file_type = get_file_type(filename);
		switch (file_type) {
			case FileType::FileType_VTK_LEGACY:
			{
				std::ifstream fp(filename.c_str(), std::ios::in | std::ios::binary);
				return this->read_vtk_legacy_file(fp);
			}
			case FileType::FileType_VTU: return this->import_VTU(filename);
			default:
				std::cerr << "VtkVolumeImport does not handle the files of type \"" << get_extension(filename) << "\"." << std::endl;
				return false;
		}
	}

	bool import_VTU(const std::string& filename)
	{
		using tinyxml2::XMLDocument;
		using tinyxml2::XMLError;
		using tinyxml2::XML_NO_ERROR;
		using tinyxml2::XMLElement;

		typename Inherit_Import::template ChunkArray<VEC3>* position =
				this->vertex_attributes_.template add_attribute<VEC3>("position");
		cgogn_assert(position != nullptr);

		XMLDocument doc;
		XMLError eResult = doc.LoadFile(filename.c_str());
		if (eResult != XML_NO_ERROR)
		{
			std::cerr << "unable loading file " << filename << std::endl;
			return false;
		}

		XMLElement* vtu_node = doc.RootElement();
		cgogn_assert(vtu_node != nullptr);
		XMLElement* grid_node = vtu_node->FirstChildElement("UnstructuredGrid");
		cgogn_assert(grid_node != nullptr);
		XMLElement* piece_node = grid_node->FirstChildElement("Piece");
		cgogn_assert(piece_node != nullptr);

		eResult = piece_node->QueryUnsignedAttribute("NumberOfPoints",&this->nb_vertices_);
		if (eResult != XML_NO_ERROR)
		{
			std::cerr << "unreadable VTU file: " << filename << std::endl;
			return false;
		}
		eResult = piece_node->QueryUnsignedAttribute("NumberOfCells",&this->nb_volumes_);
		if (eResult != XML_NO_ERROR)
		{
			std::cerr << "unreadable VTU file: " << filename << std::endl;
			return false;
		}

		std::cout << "reading file " << filename << std::endl;
		std::cout << "Number of vertices : " << this->nb_vertices_ << std::endl;
		std::cout << "Number of volumes : " << this->nb_volumes_ << std::endl;

		XMLElement* points_node = piece_node->FirstChildElement("Points");
		cgogn_assert(points_node != nullptr);
		XMLElement* array_node = points_node->FirstChildElement("DataArray");
		cgogn_assert(array_node != nullptr);

		std::stringstream ss(array_node->GetText());
		for (unsigned int i=0u; i< this->nb_vertices_; ++i)
		{
			VEC3 P;
			ss >> P[0];
			ss >> P[1];
			ss >> P[2];
			unsigned int id = this->vertex_attributes_.template insert_lines<1>();
			position->operator [](id) = P;
			cgogn_assert(id == i);
		}

		XMLElement* cell_node = piece_node->FirstChildElement("Cells");
		cgogn_assert(cell_node != nullptr);
		array_node = cell_node->FirstChildElement("DataArray");
		cgogn_assert(array_node != nullptr);

		std::vector<int> typeVols;
		typeVols.reserve(this->nb_volumes_);
		std::vector<unsigned int> offsets;
		offsets.reserve(this->nb_volumes_);
		std::vector<unsigned int> indices;
		indices.reserve(this->nb_volumes_*4u);

		while (array_node)
		{
			const std::string& propName = to_lower(std::string(array_node->Attribute("Name")));
			if (propName.empty())
			{
				std::cerr << "Error reading VTU unreadable file: "<< filename << std::endl;
				return false;
			}

			if (propName == "connectivity")
			{
				std::stringstream ss(array_node->GetText());
				while (!ss.eof())
				{
					unsigned int ind;
					ss >> ind;
					indices.push_back(ind);
				}
			}
			if (propName == "offsets")
			{
				std::stringstream ss(array_node->GetText());
				for (unsigned int i=0u; i< this->nb_volumes_; ++i)
				{
					unsigned int o;
					ss >> o;
					offsets.push_back(o);
				}
			}
			if (propName == "types")
			{
				bool unsupported = false;
				std::stringstream ss(array_node->GetText());
				for (unsigned int i=0u; i< this->nb_volumes_; ++i)
				{
					unsigned int t;
					ss >> t;
					if (!(t == 10u || t == 12u))
					{
						std::cerr << "error while parsing vtk file : volumes of type " << t << " are not supported" << std::endl;
						unsupported = true;
					}
					typeVols.push_back(t);
				}
				if (unsupported)
				{
					std::cerr << "warning, some unsupported volume cell types"<< std::endl;
				}

			}
			array_node = array_node->NextSiblingElement("DataArray");
		}

		add_vtk_volumes(indices, typeVols, *position);
		return true;
	}

	inline void add_vtk_volumes(std::vector<unsigned int>& ids, const std::vector<int>& type_vol, ChunkArray<VEC3> const& pos)
	{
		unsigned int curr_offset = 0;
		for (unsigned int i=0u; i< this->nb_volumes_; ++i)
		{
			if (type_vol[i]== VTK_CELL_TYPES::VTK_HEXAHEDRON || type_vol[i]== VTK_CELL_TYPES::VTK_VOXEL)
			{
				if (type_vol[i]== VTK_CELL_TYPES::VTK_VOXEL)
				{
					std::swap(ids[curr_offset+2],ids[curr_offset+3]);
					std::swap(ids[curr_offset+6],ids[curr_offset+7]);
				}
				this->add_hexa(pos, ids[curr_offset+0],ids[curr_offset+1],ids[curr_offset+2],ids[curr_offset+3],ids[curr_offset+4],ids[curr_offset+5],ids[curr_offset+6],ids[curr_offset+7]);
				curr_offset+=8u;
			}else {
				if (type_vol[i]== VTK_CELL_TYPES::VTK_TETRA)
				{
					this->add_tetra(pos, ids[curr_offset+0],ids[curr_offset+1],ids[curr_offset+2],ids[curr_offset+3]);
					curr_offset+=4u;
				} else {
					if (type_vol[i]== VTK_CELL_TYPES::VTK_PYRAMID)
					{
						this->add_pyramid(pos, ids[curr_offset+0],ids[curr_offset+1],ids[curr_offset+2],ids[curr_offset+3],ids[curr_offset+4]);
						curr_offset+=5u;
					} else {
						if (type_vol[i]== VTK_CELL_TYPES::VTK_WEDGE)
						{
							this->add_triangular_prism(pos, ids[curr_offset+0],ids[curr_offset+1],ids[curr_offset+2],ids[curr_offset+3],ids[curr_offset+4],ids[curr_offset+5]);
							curr_offset+=6u;
						}
					}
				}
			}
		}
	}
};
} // namespace io
} // namespace cgogn

#endif // IO_VTK_IO_H_
