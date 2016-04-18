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

#ifndef CGOGN_IO_VTK_IO_H_
#define CGOGN_IO_VTK_IO_H_

#include <istream>
#include <sstream>

#include <cgogn/core/utils/logger.h>

#include <cgogn/io/dll.h>
#include <cgogn/io/data_io.h>
#include <cgogn/io/surface_import.h>
#include <cgogn/io/volume_import.h>


namespace cgogn
{

namespace io
{

template<uint32 CHUNK_SIZE, uint32 PRIM_SIZE, typename VEC3>
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

	inline VtkIO() {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(VtkIO);
	virtual ~VtkIO() {}

protected :
	DataInput<VEC3>				positions_;
	DataInput<uint32>		cells_;
	DataInput<int>				cell_types_;
	DataInput<uint32>		offsets_;

protected :
	virtual void add_vertex_attribute(const DataInputGen& attribute_data, const std::string& attribute_name) = 0;
	virtual void add_cell_attribute(const DataInputGen& attribute_data, const std::string& attribute_name) = 0;

	inline std::vector<unsigned char> read_binary_xml_data(const char*data_str, bool is_compressed, DataType header_type)
	{
		if (!is_compressed)
		{
			std::vector<unsigned char> decode = base64_decode(data_str, 0);
			decode.erase(decode.begin(), decode.begin() + (header_type == DataType::UINT32 ? 4u : 8u));
			return decode;
		}
		else {
#ifdef CGOGN_WITH_ZLIB
			return zlib_decompress(data_str, header_type);
#else
			cgogn_log_error("read_binary_xml_data") <<  "read_binary_xml_data : unable to decompress the data : Zlib was not found.";
			std::exit(EXIT_FAILURE);
#endif
		}
	}
	/**
	 * @brief parse_vtk_legacy_file
	 * @param fp
	 * @return
	 * NOTE : by default binary data seems to be stored in big endian order.
	 */
	bool parse_vtk_legacy_file(std::ifstream& fp, bool big_endian = true)
	{
		VTK_MESH_TYPE vtk_type(VTK_MESH_TYPE::UNKNOWN);

		cgogn_log_info("parse_vtk_legacy_file") << "Opening a legacy vtk file";

		std::string line;
		std::string word;
		line.reserve(512);
		word.reserve(128);

		// printing the 2 first lines
		std::getline(fp, line);
//		cgogn_log_info("vtk_io") << line;
		std::getline(fp, line);
//		cgogn_log_info("vtk_io") << line;

		fp >> word;
		bool ascii_file = to_upper(word) == "ASCII";
		if (!(ascii_file || to_upper(word) == "BINARY"))
		{
			cgogn_log_warning("parse_vtk_legacy_file") << "Could not read the mesh file properly.";
			return false;
		}

		fp >> word;
		if (to_upper(word) != "DATASET")
		{
			cgogn_log_warning("parse_vtk_legacy_file") << "Could not read the mesh file properly.";
			return false;
		}
		fp >> word;
		const std::string& dataset = to_upper(word);
		if (dataset == "UNSTRUCTURED_GRID")
			vtk_type = VTK_MESH_TYPE::UNSTRUCTURED_GRID;
		else {
			if (dataset == "POLYDATA")
				vtk_type = VTK_MESH_TYPE::POLYDATA;
		}

		uint32 nb_vertices = 0u;
		uint32 nb_cells = 0u;

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
					auto pos = DataInputGen::template newDataIO<PRIM_SIZE, VEC3>(type_str, 3);
					pos->read_n(fp, nb_vertices, !ascii_file, big_endian);
					this->add_vertex_attribute(*pos,"position");
					this->positions_ = *dynamic_cast_unique_ptr<DataInput<VEC3>>(pos->simplify());
				} else {
					if (word == "CELLS" || word == "POLYGONS" || word == "TRIANGLE_STRIPS")
					{
						uint32 size;
						sstream >> nb_cells >> size;
						cells_.read_n(fp, size, !ascii_file, big_endian);

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
							uint32 nbc;
							sstream >> nbc;
							cell_types_.read_n(fp, nbc, !ascii_file, big_endian);
						} else {
							if (word == "POINT_DATA" || word == "CELL_DATA")
							{
								const bool cell_data = (word == "CELL_DATA");
								uint32 nb_data;
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
										uint32 num_comp = is_vector? 3u : 1u;
										sstream >> att_name >> att_type >> num_comp;
										att_type = vtk_data_type_to_cgogn_name_of_type(att_type);
										cgogn_log_info("parse_vtk_legacy_file") << "reading attribute \"" << att_name << "\" of type " << att_type << " (" << num_comp << " components).";

										const auto pos_before_lookup_table = fp.tellg(); // the lookup table might (or might not) be defined
										std::getline(fp,line);
										sstream.str(line);
										sstream.clear();
										std::string lookup_table;
										std::string lookup_table_name;
										sstream >> lookup_table >> lookup_table_name;
										if (to_upper(lookup_table) == "LOOKUP_TABLE")
										{
											cgogn_log_debug("parse_vtk_legacy_file") << "Ignoring lookup table named \"" << lookup_table_name << "\".";
										} else {
											fp.seekg(pos_before_lookup_table); // if there wasn't a lookup table we go back and start reading the numerical values
										}

										std::unique_ptr<DataInputGen> att(DataInputGen::template newDataIO<PRIM_SIZE>(att_type, num_comp));
										att->read_n(fp, nb_data, !ascii_file, big_endian);
										if (cell_data)
											this->add_cell_attribute(*att, att_name);
										else
											this->add_vertex_attribute(*att, att_name);
									} else {
										if (word == "FIELD")
										{
											std::string field_name;
											uint32 num_arrays = 0u;
											sstream >> field_name >> num_arrays;
											for (uint32 i = 0u ; i< num_arrays; ++i)
											{
												do {
													std::getline(fp,line);
												} while (line.empty());

												sstream.str(line);
												sstream.clear();
												std::string		data_name;
												uint32	nb_comp;
												//uint32	nb_data; already declared
												std::string	data_type;
												sstream >> data_name >> nb_comp >> nb_data >> data_type;
												data_type = vtk_data_type_to_cgogn_name_of_type(data_type);
												cgogn_log_info("parse_vtk_legacy_file") << "reading field \"" << data_name << "\" of type " << data_type << " (" << nb_comp << " components).";
												std::unique_ptr<DataInputGen> att(DataInputGen::template newDataIO<PRIM_SIZE>(data_type, nb_comp));
												att->read_n(fp, nb_data, !ascii_file, big_endian);
												if (cell_data)
													this->add_cell_attribute(*att, data_name);
												else
													this->add_vertex_attribute(*att, data_name);
											}
										} else
											if (word == "LOOKUP_TABLE")
											{
												std::string table_name;
												/*uint32*/ nb_data = 0u;
												sstream >> table_name >> nb_data;
												cgogn_log_debug("parse_vtk_legacy_file") << "Ignoring the definition of the lookuptable named \"" << table_name << "\".";
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
									cgogn_log_warning("parse_vtk_legacy_file") << "VTK keyword \"" << word << "\" is not supported.";
							}
						}
					}
				}
			}
		}
		return true;
	}


	bool parse_xml_vtu(const std::string& filename)
	{
		using tinyxml2::XMLDocument;
		using tinyxml2::XMLError;
		using tinyxml2::XML_NO_ERROR;
		using tinyxml2::XMLElement;

		XMLDocument doc;
		XMLError eResult = doc.LoadFile(filename.c_str());
		if (eResult != XML_NO_ERROR)
		{
			cgogn_log_warning("parse_xml_vtu")<< "Unable to load file \"" << filename << "\".";
			return false;
		}

		XMLElement* root_node = doc.RootElement();
		cgogn_assert(root_node != nullptr);
		const bool little_endian = (!root_node->Attribute("byte_order")) ||(to_lower(std::string(root_node->Attribute("byte_order"))) == "littleendian");

		std::string header_type("uint32");
		if (root_node->Attribute("header_type"))
			header_type = vtk_data_type_to_cgogn_name_of_type(root_node->Attribute("header_type"));

		const bool compressed = (root_node->Attribute("compressor") && (std::string(root_node->Attribute("compressor")) == "vtkZLibDataCompressor"));

		XMLElement* grid_node = root_node->FirstChildElement("UnstructuredGrid");
		if (grid_node == nullptr)
			grid_node = root_node->FirstChildElement("PolyData");
		cgogn_assert(grid_node != nullptr);
		XMLElement* piece_node = grid_node->FirstChildElement("Piece");
		cgogn_assert(piece_node != nullptr);

		uint32 nb_vertices = 0u;
		uint32 nb_cells = 0u;

		piece_node->QueryUnsignedAttribute("NumberOfPoints",&nb_vertices);
		piece_node->QueryUnsignedAttribute("NumberOfCells",&nb_cells);
		if (nb_cells== 0u)
			piece_node->QueryUnsignedAttribute("NumberOfPolys",&nb_cells);
		if (nb_vertices == 0u|| nb_cells == 0u)
			return false;

		XMLElement* points_node = piece_node->FirstChildElement("Points");
		cgogn_assert(points_node != nullptr);
		XMLElement* position_data_array_node = points_node->FirstChildElement("DataArray");
		for (XMLElement* elem = position_data_array_node; elem != nullptr ; elem = elem->NextSiblingElement("DataArray"))
		{
			if (elem->Attribute("Name") && to_lower(std::string(elem->Attribute("Name"))) == "points")
				position_data_array_node = elem;
		}
		cgogn_assert(position_data_array_node != nullptr);
		XMLElement* point_data_node = piece_node->FirstChildElement("PointData");
		XMLElement* point_data_array_node = point_data_node ? point_data_node->FirstChildElement("DataArray") : nullptr;
		std::vector<XMLElement*> vertices_nodes = {position_data_array_node};
		while (point_data_array_node)
		{
			vertices_nodes.push_back(point_data_array_node);
			point_data_array_node = point_data_array_node->NextSiblingElement("DataArray");
		}

		for (XMLElement* vertex_data : vertices_nodes)
		{
			std::string data_name("cgogn_unnamed_vertex_data");
			if (vertex_data->Attribute("Name"))
				data_name = to_lower(std::string(vertex_data->Attribute("Name")));
			const bool binary =  vertex_data->Attribute("format", nullptr) && (to_lower(std::string(vertex_data->Attribute("format", nullptr))) == "binary");
			uint32 nb_comp = 1u;
			vertex_data->QueryUnsignedAttribute("NumberOfComponents", &nb_comp);
			const std::string type = vtk_data_type_to_cgogn_name_of_type(std::string(vertex_data->Attribute("type", nullptr)));

			if (data_name.empty())
				cgogn_log_debug("parse_xml_vtu") << "Skipping a vertex DataArray without \"Name\" attribute.";
			else {
				const char*					ascii_data = vertex_data->GetText();
				std::vector<unsigned char>	binary_data;
				if (binary)
					binary_data = this->read_binary_xml_data(ascii_data,compressed, get_data_type(header_type));

				std::unique_ptr<IMemoryStream> mem_stream;
				if (binary)
					mem_stream = make_unique<IMemoryStream>(reinterpret_cast<char*>(&binary_data[0]), binary_data.size());
				else
					mem_stream = make_unique<IMemoryStream>(ascii_data);
				if (vertex_data == position_data_array_node)
				{
					cgogn_assert(nb_comp == 3);
					auto pos = DataInputGen::template newDataIO<PRIM_SIZE, VEC3>(type, nb_comp);
					pos->read_n(*mem_stream, nb_vertices,binary,!little_endian);
					this->add_vertex_attribute(*pos,"position");
					this->positions_ = *dynamic_cast_unique_ptr<DataInput<VEC3>>(pos->simplify());
				}
				else {
					std::unique_ptr<DataInputGen> vertex_att = DataInputGen::template newDataIO<PRIM_SIZE>(type, nb_comp);
					vertex_att->read_n(*mem_stream, nb_vertices,binary,!little_endian);
					this->add_vertex_attribute(*vertex_att, data_name);
				}
			}
		}


		XMLElement* const cell_node = piece_node->FirstChildElement("Cells");
		if (cell_node != nullptr)
		{
			XMLElement* cells_array_node = cell_node->FirstChildElement("DataArray");
			cgogn_assert(cells_array_node != nullptr);
			std::vector<XMLElement*> cell_nodes;
			while (cells_array_node)
			{
				cell_nodes.push_back(cells_array_node);
				cells_array_node = cells_array_node->NextSiblingElement("DataArray");
			}

			XMLElement* const cell_data_node = piece_node->FirstChildElement("CellData");
			cells_array_node = cell_data_node ? cell_data_node->FirstChildElement("DataArray") : nullptr;
			while (cells_array_node)
			{
				cell_nodes.push_back(cells_array_node);
				cells_array_node = cells_array_node->NextSiblingElement("DataArray");
			}


			for (XMLElement*& cell_data : cell_nodes)
			{
				if (to_lower(std::string(cell_data->Attribute("Name"))) == "connectivity" && (cell_data != cell_nodes.back()))
				{
					std::swap(cell_data, cell_nodes.back());
				}
			}

			for (XMLElement* cell_data : cell_nodes)
			{
				const std::string& data_name = to_lower(std::string(cell_data->Attribute("Name")));
				const bool binary = cell_data->Attribute("format", nullptr) && (to_lower(std::string(cell_data->Attribute("format", nullptr))) == "binary");
				uint32 nb_comp = 1u;
				cell_data->QueryUnsignedAttribute("NumberOfComponents", &nb_comp);
				std::string type = vtk_data_type_to_cgogn_name_of_type(std::string(cell_data->Attribute("type", nullptr)));

				if (data_name.empty())
					cgogn_log_debug("parse_xml_vtu") << "Skipping a cell DataArray without \"Name\" attribute.";
				else {
					const char*					ascii_data = cell_data->GetText();
					std::vector<unsigned char>	binary_data;
					if (binary)
						binary_data = this->read_binary_xml_data(ascii_data,compressed, get_data_type(header_type));

					std::unique_ptr<IMemoryStream> mem_stream;
					if (binary)
						mem_stream = make_unique<IMemoryStream>(reinterpret_cast<char*>(&binary_data[0]), binary_data.size());
					else
						mem_stream = make_unique<IMemoryStream>(ascii_data);
					if (data_name == "connectivity")
					{
						const uint32 last_offset = this->offsets_.get_vec()->back();
						auto cells = DataInputGen::template newDataIO<PRIM_SIZE, uint32>(type);
						cells->read_n(*mem_stream, last_offset,binary,!little_endian);
						this->cells_ = *dynamic_cast_unique_ptr<DataInput<uint32>>(cells->simplify());
					}
					else {
						if (data_name == "offsets")
						{
							auto offsets = DataInputGen::template newDataIO<PRIM_SIZE, uint32>(type);
							offsets->read_n(*mem_stream, nb_cells,binary,!little_endian);
							this->offsets_ = *dynamic_cast_unique_ptr<DataInput<uint32>>(offsets->simplify());
						}
						else {
							if (data_name == "types")
							{
								auto types = DataInputGen::template newDataIO<PRIM_SIZE, int>(type);
								types->read_n(*mem_stream, nb_cells,binary,!little_endian);
								this->cell_types_ = *dynamic_cast_unique_ptr<DataInput<int>>(types->simplify());
							}
							else {
								cgogn_log_info("parse_xml_vtu") << "Reading cell attribute \"" <<  data_name << "\" of type " << type << ".";
								auto cell_att = DataInputGen::template newDataIO<PRIM_SIZE>(type, nb_comp);
								cell_att->read_n(*mem_stream, nb_cells,binary,!little_endian);
								this->add_cell_attribute(*cell_att, data_name);
							}
						}
					}
				}
			}
		}

		XMLElement* const poly_node = piece_node->FirstChildElement("Polys");
		if (poly_node)
		{
			XMLElement* polys_array_node = poly_node->FirstChildElement("DataArray");
			cgogn_assert(polys_array_node != nullptr);
			std::vector<XMLElement*> poly_nodes;
			while (polys_array_node)
			{
				poly_nodes.push_back(polys_array_node);
				polys_array_node = polys_array_node->NextSiblingElement("DataArray");
			}

			for (XMLElement*& poly_data_array : poly_nodes)
			{
				if (poly_data_array->Attribute("Name") && to_lower(std::string(poly_data_array->Attribute("Name"))) == "connectivity" && (poly_data_array != poly_nodes.back()))
				{
					std::swap(poly_data_array, poly_nodes.back());
				}
			}

			for (XMLElement* poly_data_array : poly_nodes)
			{
				const std::string& data_name = to_lower(std::string(poly_data_array->Attribute("Name")));
				const bool binary = (poly_data_array->Attribute("format") && to_lower(std::string(poly_data_array->Attribute("format", nullptr))) == "binary");
				uint32 nb_comp = 1;
				poly_data_array->QueryUnsignedAttribute("NumberOfComponents", &nb_comp);
				std::string type;
				if (poly_data_array->Attribute("type", nullptr))
					type = vtk_data_type_to_cgogn_name_of_type(std::string(poly_data_array->Attribute("type", nullptr)));

				if (data_name.empty())
					cgogn_log_debug("parse_xml_vtu")<< "Skipping a cell DataArray without \"Name\" attribute.";
				else {
					const char*					ascii_data = poly_data_array->GetText();
					std::vector<unsigned char>	binary_data;
					if (binary)
						binary_data = this->read_binary_xml_data(ascii_data,compressed, get_data_type(header_type));

					std::unique_ptr<IMemoryStream> mem_stream;
					if (binary)
						mem_stream = make_unique<IMemoryStream>(reinterpret_cast<char*>(&binary_data[0]), binary_data.size());
					else
						mem_stream = make_unique<IMemoryStream>(ascii_data);
					if (data_name == "connectivity")
					{
						const uint32 last_offset = this->offsets_.get_vec()->back();
						auto cells = DataInputGen::template newDataIO<PRIM_SIZE, uint32>(type);
						cells->read_n(*mem_stream, last_offset,binary,!little_endian);
						this->cells_ = *dynamic_cast_unique_ptr<DataInput<uint32>>(cells->simplify());
					}
					else {
						if (data_name == "offsets")
						{
							auto offsets = DataInputGen::template newDataIO<PRIM_SIZE, uint32>(type);
							offsets->read_n(*mem_stream, nb_cells,binary,!little_endian);
							this->offsets_ = *dynamic_cast_unique_ptr<DataInput<uint32>>(offsets->simplify());
						}
						else
							cgogn_log_debug("parse_xml_vtu") << "Ignoring cell attribute \"" <<  data_name << "\" of type " << type << ".";
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
			return name_of_type(float32());
		if (data_type == "double" || data_type == "float64")
			return name_of_type(float64());

		cgogn_log_error("vtk_data_type_to_cgogn_name_of_type") << "Unknown vtk type \"" << vtk_type_str << "\".";
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

	inline bool read_xml_file(const std::string& filename)
	{
		if (!Inherit_Vtk::parse_xml_vtu(filename))
			return false;
		this->fill_surface_import();
		return true;
	}

	inline bool read_vtk_legacy_file(std::ifstream& fp)
	{
		if (!Inherit_Vtk::parse_vtk_legacy_file(fp))
			return false;
		this->fill_surface_import();
		return true;
	}

	inline bool read_vtp_file(const std::string& filename)
	{
		if (!Inherit_Vtk::parse_xml_vtu(filename))
			return false;
		this->fill_surface_import();

		this->nb_vertices_ = uint32(this->positions_.size());
		this->nb_faces_ = uint32(this->offsets_.size());

		auto cells_it = this->cells_.get_vec()->begin();
		uint32 last_offset = 0u;
		for(auto offset_it =this->offsets_.get_vec()->begin(), offset_end = this->offsets_.get_vec()->end() ; offset_it != offset_end; ++offset_it)
		{
			const uint32 curr_offset = *offset_it;
			const uint32 nb_vertices = curr_offset - last_offset;
			this->faces_nb_edges_.push_back(nb_vertices);
			for (uint32 i = 0u ; i < nb_vertices; ++i)
				this->faces_vertex_indices_.push_back(*cells_it++);
			last_offset = *offset_it;
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
	virtual bool import_file_impl(const std::string& filename) override
	{
		const FileType file_type = get_file_type(filename);
		switch (file_type)
		{
			case FileType::FileType_VTK_LEGACY:
			{
				std::ifstream fp(filename.c_str(), std::ios::in);
				cgogn_assert(fp.good());
				return this->read_vtk_legacy_file(fp);
			}
			case FileType::FileType_VTU:
				return this->read_xml_file(filename);
			case FileType::FileType_VTP:
				return this->read_vtp_file(filename);
			default:
				cgogn_log_warning("VtkSurfaceImport::import_file_impl")<< "VtkSurfaceImport does not handle the files of type \"" << get_extension(filename) << "\".";
				return false;
		}
	}

private:
	inline void fill_surface_import()
	{
		this->nb_vertices_ = uint32(this->positions_.size());
		this->nb_faces_ = uint32(this->cell_types_.size());

		auto cells_it = static_cast<std::vector<uint32>*>(this->cells_.get_data())->begin();
		const std::vector<int>* cell_types_vec = static_cast<std::vector<int>*>(this->cell_types_.get_data());
		for(auto cell_types_it = cell_types_vec->begin(); cell_types_it != cell_types_vec->end() ; )
		{
			const std::size_t nb_vert = *(cells_it++);
			const int cell_type = *(cell_types_it++);

			if (cell_type != VTK_CELL_TYPES::VTK_TRIANGLE_STRIP)
			{
				this->faces_nb_edges_.push_back(uint32(nb_vert));
				for (std::size_t i = 0ul ; i < nb_vert;++i)
				{
					this->faces_vertex_indices_.push_back(*cells_it++);
				}
			} else {
				std::vector<uint32> vertexIDS;
				vertexIDS.reserve(nb_vert);
				for (std::size_t i = 0ul ; i < nb_vert;++i)
				{
					vertexIDS.push_back(*cells_it++);
				}

				for (uint32 i = 0u ; i < nb_vert -2u; ++i)
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

	inline VtkVolumeImport() {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(VtkVolumeImport);
	virtual ~VtkVolumeImport() override {}

protected:
	inline bool read_vtk_legacy_file(std::ifstream& fp)
	{
		if (!Inherit_Vtk::parse_vtk_legacy_file(fp))
			return false;

		this->set_nb_vertices(uint32(this->positions_.size()));
		this->set_nb_volumes(uint32(this->cell_types_.size()));

		const std::vector<int>* cell_types_vec			= this->cell_types_.get_vec();
		const std::vector<uint32>* cells_vec		= this->cells_.get_vec();
		std::vector<uint32> cells_buffer;
		cells_buffer.reserve(cells_vec->size());

		// in the legacy file , the first number of each line is the number of vertices. We need to remove it.
		auto cells_it = cells_vec->begin();
		for (std::vector<int>::const_iterator type_it = cell_types_vec->begin(), end = cell_types_vec->end(); type_it != end; ++type_it)
		{
			++cells_it;
			uint32 vol_nb_verts = 0u;
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
			for (uint32 i = 0u ; i < vol_nb_verts;++i)
			{
				cells_buffer.push_back(*cells_it++);
			}
		}


		add_vtk_volumes(cells_buffer,*cell_types_vec, *(this->template get_position_attribute<VEC3>()));

		return true;
	}

	inline bool read_vtk_xml_file(const std::string& filename)
	{
		if (!Inherit_Vtk::parse_xml_vtu(filename))
			return false;

		this->set_nb_vertices(uint32(this->positions_.size()));
		this->set_nb_volumes(uint32(this->cell_types_.size()));

		const std::vector<int>* cell_types_vec			= this->cell_types_.get_vec();
		const std::vector<uint32>* cells_vec		= this->cells_.get_vec();

		ChunkArray<VEC3>* pos = this->template get_position_attribute<VEC3>();
		cgogn_assert(pos != nullptr);
		add_vtk_volumes(*cells_vec,*cell_types_vec, *pos);

		return true;
	}

	virtual void add_vertex_attribute(const DataInputGen& attribute_data, const std::string& attribute_name) override
	{
		attribute_data.to_chunk_array(attribute_data.add_attribute(this->get_vertex_attributes_container(), attribute_name));
	}
	virtual void add_cell_attribute(const DataInputGen& attribute_data, const std::string& attribute_name) override
	{
		attribute_data.to_chunk_array(attribute_data.add_attribute(this->get_volume_attributes_container(), attribute_name));
	}

	virtual bool import_file_impl(const std::string& filename) override
	{
		const FileType file_type = get_file_type(filename);
		switch (file_type) {
			case FileType::FileType_VTK_LEGACY:
			{
				std::ifstream fp(filename.c_str(), std::ios::in | std::ios::binary);
				return this->read_vtk_legacy_file(fp);
			}
			case FileType::FileType_VTU: return this->read_vtk_xml_file(filename);
			default:
				cgogn_log_warning("VtkVolumeImport::import_file_impl")<< "VtkSurfaceImport does not handle the files of type \"" << get_extension(filename) << "\".";
				return false;
		}
	}


	inline void add_vtk_volumes(std::vector<uint32> ids, const std::vector<int>& type_vol, ChunkArray<VEC3> const& pos)
	{
		uint32 curr_offset = 0;
		for (uint32 i=0u, end = this->get_nb_volumes(); i< end; ++i)
		{
			if (type_vol[i]== VTK_CELL_TYPES::VTK_HEXAHEDRON || type_vol[i]== VTK_CELL_TYPES::VTK_VOXEL)
			{
				if (type_vol[i]== VTK_CELL_TYPES::VTK_VOXEL)
				{
					std::swap(ids[curr_offset+2],ids[curr_offset+3]);
					std::swap(ids[curr_offset+6],ids[curr_offset+7]);
				}
				this->add_hexa(pos, ids[curr_offset+0],ids[curr_offset+1],ids[curr_offset+2],ids[curr_offset+3],ids[curr_offset+4],ids[curr_offset+5],ids[curr_offset+6],ids[curr_offset+7], true);
				curr_offset+=8u;
			}else {
				if (type_vol[i]== VTK_CELL_TYPES::VTK_TETRA)
				{
					this->add_tetra(pos, ids[curr_offset+0],ids[curr_offset+1],ids[curr_offset+2],ids[curr_offset+3], true);
					curr_offset+=4u;
				} else {
					if (type_vol[i]== VTK_CELL_TYPES::VTK_PYRAMID)
					{
						this->add_pyramid(pos, ids[curr_offset+0],ids[curr_offset+1],ids[curr_offset+2],ids[curr_offset+3],ids[curr_offset+4],true);
						curr_offset+=5u;
					} else {
						if (type_vol[i]== VTK_CELL_TYPES::VTK_WEDGE)
						{
							this->add_triangular_prism(pos, ids[curr_offset+0],ids[curr_offset+1],ids[curr_offset+2],ids[curr_offset+3],ids[curr_offset+4],ids[curr_offset+5],true);
							curr_offset+=6u;
						}
					}
				}
			}
		}
	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(IO_VTK_IO_CPP_))
extern template class CGOGN_IO_API VtkIO<DefaultMapTraits::CHUNK_SIZE,1, Eigen::Vector3d>;
extern template class CGOGN_IO_API VtkIO<DefaultMapTraits::CHUNK_SIZE,1, Eigen::Vector3f>;
extern template class CGOGN_IO_API VtkIO<DefaultMapTraits::CHUNK_SIZE,1, geometry::Vec_T<std::array<float64,3>>>;
extern template class CGOGN_IO_API VtkIO<DefaultMapTraits::CHUNK_SIZE,1, geometry::Vec_T<std::array<float32,3>>>;

extern template class CGOGN_IO_API VtkVolumeImport<DefaultMapTraits, Eigen::Vector3d>;
extern template class CGOGN_IO_API VtkVolumeImport<DefaultMapTraits, Eigen::Vector3f>;
extern template class CGOGN_IO_API VtkVolumeImport<DefaultMapTraits, geometry::Vec_T<std::array<float64,3>>>;
extern template class CGOGN_IO_API VtkVolumeImport<DefaultMapTraits, geometry::Vec_T<std::array<float32,3>>>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(IO_VTK_IO_CPP_))

} // namespace io
} // namespace cgogn

#endif // CGOGN_IO_VTK_IO_H_
