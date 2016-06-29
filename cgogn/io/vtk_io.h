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
#include <ostream>
#include <iomanip>

#include <cgogn/core/utils/logger.h>

#include <cgogn/io/dll.h>
#include <cgogn/io/data_io.h>
#include <cgogn/io/surface_import.h>
#include <cgogn/io/surface_export.h>
#include <cgogn/io/volume_import.h>
#include <cgogn/io/volume_export.h>

namespace cgogn
{

namespace io
{

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

CGOGN_IO_API std::string vtk_data_type_to_cgogn_name_of_type(const std::string& vtk_type_str);
CGOGN_IO_API std::string cgogn_name_of_type_to_vtk_xml_data_type(const std::string& cgogn_type);
CGOGN_IO_API std::string cgogn_name_of_type_to_vtk_legacy_data_type(const std::string& cgogn_type);
CGOGN_IO_API std::vector<unsigned char> read_binary_xml_data(const char*data_str, bool is_compressed, DataType header_type);
CGOGN_IO_API void write_binary_xml_data(std::ostream& output, const char* data_str, std::size_t size, bool compress = false);

template <typename T>
inline std::string vtk_name_of_type(const T& t)
{
	static_assert(std::is_arithmetic<T>::value, "T must be a scalar.");
	if (std::is_same<T, int8>::value)
		return "Int8";
	if (std::is_same<T,uint8>::value)
		return "UInt8";
	if (std::is_same<T, int16>::value)
		return "Int16";
	if (std::is_same<T,uint16>::value)
		return "UInt16";
	if (std::is_same<T, int32>::value)
		return "Int32";
	if (std::is_same<T,uint32>::value)
		return "UInt32";
	if (std::is_same<T, int64>::value)
		return "Int64";
	if (std::is_same<T,uint64>::value)
		return "UInt64";
	if (std::is_same<T,float32>::value)
		return "Float32";
	if (std::is_same<T,float64>::value)
		return "Float64";

	cgogn_log_error("vtk_name_of_type") << "Cannot convert to VTK the type \"" << cgogn::name_of_type(t) << "\".";
	return std::string();
}

template<typename MAP>
class VtkSurfaceExport : public SurfaceExport<MAP>
{
public:

	using Inherit = SurfaceExport<MAP>;
	using Self = VtkSurfaceExport<MAP>;
	using Map = typename Inherit::Map;
	using Vertex = typename Inherit::Vertex;
	using Face = typename Inherit::Face;
	using ChunkArrayGen = typename Inherit::ChunkArrayGen;
	template<typename T>
	using VertexAttribute = typename Inherit::template VertexAttribute<T>;
	using ChunkArrayContainer = typename Inherit::ChunkArrayContainer;

	virtual void export_file_impl(const Map& map, std::ofstream& output, const ExportOptions& option) override
	{
		if (to_lower(extension(option.filename_)) == "vtp")
			this->export_vtp_xml(map, output, option);
		if (to_lower(extension(option.filename_)) == "vtk")
			this->export_legacy_vtk(map, output, option);
	}
private:
	void export_legacy_vtk(const Map& map, std::ofstream& output, const ExportOptions& option)
	{
		const bool bin = option.binary_;
		const uint32 nbv = map.template nb_cells<Vertex::ORBIT>();
		const uint32 nbf = map.template nb_cells<Face::ORBIT>();
		std::string scalar_type = cgogn_name_of_type_to_vtk_legacy_data_type(this->position_attribute_->nested_type_name());

		output << "# vtk DataFile Version 2.0" << std::endl;
		output << "Mesh exported with CGoGN : github.com/cgogn/CGoGN_2";
		if (bin)
			output << " --- endianness = BigEndian";
		output << std::endl;
		output << (bin?"BINARY":"ASCII") << std::endl;
		output << "DATASET UNSTRUCTURED_GRID" << std::endl;

		{// point section
			output << "POINTS " << nbv << " " << scalar_type << std::endl;
			map.foreach_cell([&](Vertex v)
			{
				this->position_attribute_->export_element(map.embedding(v), output, bin, false);
				if (!bin)
					output << std::endl;
			}, *(this->cell_cache_));
			output << std::endl;
		} // end point section

		{ // cell section
			std::vector<uint32> buffer_cells;
			buffer_cells.reserve(4u*nbf);
			uint32 cell_section_size{0u};
			map.foreach_cell([&](Face f)
			{
				buffer_cells.push_back(map.codegree(f));
				cell_section_size += buffer_cells.back();
				Dart it = f.dart;
				do {
					buffer_cells.push_back(this->indices_[Vertex(it)]);
					it = map.phi1(it);
				} while (it != f.dart);
			}, *(this->cell_cache_));

			cell_section_size += nbf; // we add an integer for each face (the nb of vertices)

			output << "CELLS " << nbf << " " << cell_section_size << std::endl;

			if (bin)
			{
				for (auto& i : buffer_cells)
					i = swap_endianness_native_big(i);
				output.write(reinterpret_cast<char*>(&buffer_cells[0]), buffer_cells.size() * sizeof(uint32));
				for (auto& i : buffer_cells)
					i = swap_endianness_native_big(i);
				output << std::endl;
			} else {
					for(std::size_t i = 0u, end = buffer_cells.size(); i < end;)
					{
						const uint32 nb_vert = buffer_cells[i++];
						output << nb_vert << " ";
						for (uint32 j = 0u; j < nb_vert; ++j)
						{
							output << buffer_cells[i++] << " ";
						}
						output << std::endl;
					}
				}

			output << "CELL_TYPES " << nbf << std::endl;
			if (bin)
			{
				std::vector<int32> buffer_cell_type;
				buffer_cell_type.reserve(nbf);
				for (auto it = buffer_cells.begin(), end = buffer_cells.end() ; it != end ;)
				{
					const uint32 nb_vert = *it;
					switch (nb_vert) {
						case 3u: buffer_cell_type.push_back(VTK_TRIANGLE); break;
						case 4u: buffer_cell_type.push_back(VTK_QUAD); break;
						default: buffer_cell_type.push_back(VTK_POLYGON); break;
					}
					it += nb_vert + 1u;
				}
				for (auto& i : buffer_cell_type)
					i = swap_endianness_native_big(i);
				output.write(reinterpret_cast<char*>(&buffer_cell_type[0]), buffer_cell_type.size() * sizeof(int32));
				output << std::endl;
			} else {
				for (auto it = buffer_cells.begin(), end = buffer_cells.end() ; it != end ;)
				{
					const uint32 nb_vert = *it;
					switch (nb_vert) {
						case 3u: output << VTK_TRIANGLE; break;
						case 4u: output << VTK_QUAD; break;
						default: output << VTK_POLYGON; break;
					}
					output << std::endl;
					it += nb_vert + 1u;
				}
			}
		} // end cell section

		{ // point data section
			if (!this->vertex_attributes().empty())
			{
				const auto& vertex_attributes = this->vertex_attributes();
				output << "POINT_DATA " << nbv << std::endl;
				for(ChunkArrayGen const* vatt : vertex_attributes)
				{
					output << "SCALARS " << vatt->name() << " " << cgogn_name_of_type_to_vtk_legacy_data_type(vatt->nested_type_name()) << " " << vatt->nb_components() << std::endl;
					output << "LOOKUP_TABLE default" << std::endl;
					map.foreach_cell([&](Vertex v)
					{
						vatt->export_element(map.embedding(v), output, bin, false);
						if (!bin)
							output << std::endl;
					}, *(this->cell_cache_));
					if (bin)
						output << std::endl;
				}
			}
		} // end point data section

		{ // cell data section
			if (!this->face_attributes().empty())
			{
				const auto& face_attributes = this->face_attributes();
				output << "CELL_DATA " << nbf << std::endl;
				for(ChunkArrayGen const* fatt : face_attributes)
				{
					output << "SCALARS " << fatt->name() << " " << cgogn_name_of_type_to_vtk_legacy_data_type(fatt->nested_type_name()) << " " << fatt->nb_components() << std::endl;
					output << "LOOKUP_TABLE default" << std::endl;
					map.foreach_cell([&](Face f)
					{
						fatt->export_element(map.embedding(f), output, bin, false);
						if (!bin)
							output << std::endl;
					}, *(this->cell_cache_));
					if (bin)
						output << std::endl;
				}
			}
		} // end cell data section
	}

	void export_vtp_xml(const Map& map, std::ofstream& output, const ExportOptions& option)
	{
		ChunkArrayGen const* pos = this->position_attribute();
		const std::string endianness = cgogn::internal::cgogn_is_little_endian ? "LittleEndian" : "BigEndian";
		const std::string format = (option.binary_?"binary" :"ascii");
		std::string scalar_type = cgogn_name_of_type_to_vtk_xml_data_type(pos->nested_type_name());
		const uint32 nbv = map.template nb_cells<Vertex::ORBIT>();
		const uint32 nbf = map.template nb_cells<Face::ORBIT>();
		const bool bin = option.binary_;
		const bool compress = option.compress_;
		// set precision for real output
		output << std::setprecision(12);

		output << "<?xml version=\"1.0\"?>" << std::endl;
		output << "<VTKFile type=\"PolyData\" header_type=\"UInt32\" version=\"0.1\" byte_order=\"" << endianness << "\"";
		if (compress)
			output << " compressor=\"vtkZLibDataCompressor\"";
		output << ">" << std::endl;
		output << "<PolyData>" <<  std::endl;
		output << "<Piece NumberOfPoints=\"" << nbv << "\" NumberOfPolys=\"" << nbf << "\">" << std::endl;
		// 1st step : vertices
		output << "      <Points>" << std::endl;
		// 1.a : positions
		output << "        <DataArray type=\""<< scalar_type  << "\" Name=\"" << pos->name() << "\" NumberOfComponents=\"" << pos->nb_components() << "\" format=\"" << format << "\">" << std::endl;

		std::vector<char> buffer_char;

		if (bin)
		{
			const uint32 elem_size{pos->element_size()};
			buffer_char.clear();
			buffer_char.reserve(nbv * elem_size);

			map.foreach_cell([&](Vertex v)
			{
				const char* elem =  static_cast<const char*>(pos->element_ptr(map.embedding(v)));
				for(uint32 i = 0u; i < elem_size; ++i)
					buffer_char.push_back(elem[i]);
			}, *(this->cell_cache_));

			write_binary_xml_data(output,&buffer_char[0], buffer_char.size(), option.compress_);
			output << std::endl;
		} else {
			map.foreach_cell([&](Vertex v)
			{
					output << "          ";
					pos->export_element(map.embedding(v), output, false, false);
					output << std::endl;
			}, *(this->cell_cache_));
		}

		output << "        </DataArray>" << std::endl;
		output << "      </Points>" << std::endl;


		if (!this->vertex_attributes().empty())
		{
			output << "      <PointData>" << std::endl;
			// 1.B : other vertices attributes
			for (const auto& att : this->vertex_attributes())
			{
				scalar_type = cgogn_name_of_type_to_vtk_xml_data_type(att->nested_type_name());
				output << "        <DataArray type=\""<< scalar_type  <<"\" Name=\"" << att->name() << "\" NumberOfComponents=\"" << att->nb_components() << "\" format=\"" << format << "\">" << std::endl;

				if (bin)
				{
					const uint32 elem_size{pos->element_size()};
					buffer_char.clear();
					buffer_char.reserve(nbv * elem_size);

					map.foreach_cell([&](Vertex v)
					{
						const char* elem =  static_cast<const char*>(att->element_ptr(map.embedding(v)));
						for(uint32 i = 0u; i < elem_size; ++i)
							buffer_char.push_back(elem[i]);
					}, *(this->cell_cache_));
					write_binary_xml_data(output,&buffer_char[0], buffer_char.size(), option.compress_);
					output << std::endl;
				} else {
					map.foreach_cell([&](Vertex v)
					{
							output << "          ";
							att->export_element(map.embedding(v), output, false, false);
							output << std::endl;
					}, *(this->cell_cache_));
				}

				output << "        </DataArray>" << std::endl;
			}
			output << "      </PointData>" << std::endl;
		}
		// end vertices


		if (!this->face_attributes().empty())
		{
			output << "<CellData>" << std::endl;
			for (const auto& att : this->face_attributes())
			{
				scalar_type = cgogn_name_of_type_to_vtk_xml_data_type(att->nested_type_name());
				output << "<DataArray type=\""<< scalar_type  <<"\" Name=\"" << att->name() << "\" NumberOfComponents=\"" << att->nb_components() << "\" format=\"" << format << "\">" << std::endl;

				if (bin)
				{
					const uint32 elem_size{att->element_size()};
					buffer_char.clear();
					buffer_char.resize(nbf * elem_size);
					char* buffer_ptr = &buffer_char[0];

					map.foreach_cell([&](Face f)
					{
						const char* elem =  static_cast<const char*>(att->element_ptr(map.embedding(f)));
						std::memcpy(buffer_ptr, elem, elem_size);
						buffer_ptr += elem_size;
					}, *(this->cell_cache_));
					write_binary_xml_data(output,&buffer_char[0], buffer_char.size(), option.compress_);
					output << std::endl;
				} else {
					map.foreach_cell([&](Face f)
					{
						att->export_element(map.embedding(f), output, false, false);
						output << std::endl;
					}, *(this->cell_cache_));
				}

				output << "</DataArray>" << std::endl;
			}
			output << "</CellData>" << std::endl;
		}

		output << "<Polys>" << std::endl;


		output << "<DataArray type=\"Int32\" Name=\"connectivity\" format=\"" << format << "\">" << std::endl;

		if (bin)
		{
			buffer_char.clear();
			std::vector<int32> buffer_vertices;
			buffer_vertices.reserve(3 * nbf);
			map.foreach_cell([&](Face f)
			{
				Dart it = f.dart;
				do {
					buffer_vertices.push_back(this->indices_[Vertex(it)]);
					it = map.phi1(it);
				} while (it != f.dart);
			}, *(this->cell_cache_));
			write_binary_xml_data(output,reinterpret_cast<char*>(&buffer_vertices[0]), buffer_vertices.size() * sizeof(int32), compress);
			output << std::endl;
		} else {
			map.foreach_cell([&](Face f)
			{
				Dart it = f.dart;
				do {
					output << this->indices_[Vertex(it)] << " ";
					it = map.phi1(it);
				} while (it != f.dart);
				output << std::endl;
			}, *(this->cell_cache_));
		}


		output << "</DataArray>" << std::endl;

		output << "<DataArray type=\"Int32\" Name=\"offsets\" format=\"" << format << "\">" << std::endl;

		int32 offset{0};
		std::vector<int32> buffer_offset;
		buffer_offset.reserve(nbf);
		map.foreach_cell([&](Face f)
		{
			offset+= int32(map.codegree(f));
			buffer_offset.push_back(offset);
		}, *(this->cell_cache_));

		if (bin)
		{
			write_binary_xml_data(output,reinterpret_cast<const char*>(&buffer_offset[0]),  buffer_offset.size() * sizeof(int32), compress);
		} else {
			output << "         ";
			for (auto o : buffer_offset)
				output << " " << o;
		}
		output << std::endl << "</DataArray>" << std::endl;

		output << "</Polys>" << std::endl;
		output << "</Piece>" << std::endl;
		output << "</PolyData>" << std::endl;
		output << "</VTKFile>" << std::endl;
	}
};

template <typename MAP>
class VtkVolumeExport : public VolumeExport<MAP>
{
public:
	using Inherit = VolumeExport<MAP>;
	using Self = VtkVolumeExport<MAP>;
	using Map = typename Inherit::Map;
	using Vertex = typename Inherit::Vertex;
	using Volume = typename Inherit::Volume;
	using ChunkArrayGen = typename Inherit::ChunkArrayGen;


protected:
	virtual void export_file_impl(const Map& map, std::ofstream& output, const ExportOptions& option) override
	{
		if (to_lower(extension(option.filename_)) == "vtu")
			this->export_vtu(map, output, option);
		if (to_lower(extension(option.filename_)) == "vtk")
			this->export_legacy_vtk(map, output, option);
	}

private:
	void export_legacy_vtk(const Map& map, std::ofstream& output, const ExportOptions& option)
	{
		const bool bin = option.binary_;
		const uint32 nbv = this->nb_vertices();
		const uint32 nbw = this->nb_volumes();
		std::string scalar_type = cgogn_name_of_type_to_vtk_legacy_data_type(this->position_attribute_->nested_type_name());

		output << "# vtk DataFile Version 2.0" << std::endl;
		output << "Mesh exported with CGoGN : github.com/cgogn/CGoGN_2";
		if (bin)
			output << " --- endianness = BigEndian";
		output << std::endl;
		output << (bin?"BINARY":"ASCII") << std::endl;
		output << "DATASET UNSTRUCTURED_GRID" << std::endl;

		{// point section
			output << "POINTS " << nbv << " " << scalar_type << std::endl;
			map.foreach_cell([&](Vertex v)
			{
				this->position_attribute_->export_element(map.embedding(v), output, bin, false);
				if (!bin)
					output << std::endl;
			}, *(this->cell_cache_));
			output << std::endl;
		} // end point section

		{ // cell section
			std::vector<uint32> buffer_cells;
			buffer_cells.reserve(5u*nbw);
			uint32 cell_section_size{0u};
			map.foreach_cell([&](Volume w)
			{
				const auto& indices = this->vertices_of_volumes(w);
				buffer_cells.push_back(static_cast<uint32>(indices.size()));
				cell_section_size += buffer_cells.back();
				for (auto i : indices)
					buffer_cells.push_back(static_cast<uint32>(i));
			}, *(this->cell_cache_));

			cell_section_size += nbw; // we add an integer for each volume (the nb of vertices)

			output << "CELLS " << nbw << " " << cell_section_size << std::endl;

			if (bin)
			{
				for (auto& i : buffer_cells)
					i = swap_endianness_native_big(i);
				output.write(reinterpret_cast<char*>(&buffer_cells[0]), buffer_cells.size() * sizeof(uint32));
				output << std::endl;
			} else {
					for(std::size_t i = 0u, end = buffer_cells.size(); i < end;)
					{
						const uint32 nb_vert = buffer_cells[i++];
						output << nb_vert << " ";
						for (uint32 j = 0u; j < nb_vert; ++j)
						{
							output << buffer_cells[i++] << " ";
						}
						output << std::endl;
					}
				}

			output << "CELL_TYPES " << nbw << std::endl;
			if (bin)
			{
				std::vector<int32> buffer_cell_type;
				buffer_cell_type.reserve(nbw);
				map.foreach_cell([&](Volume w)
				{
					const auto& indices = this->vertices_of_volumes(w);
					switch (indices.size()) {
						case 4u: buffer_cell_type.push_back(VTK_TETRA); break;
						case 5u: buffer_cell_type.push_back(VTK_PYRAMID); break;
						case 6u: buffer_cell_type.push_back(VTK_WEDGE); break;
						case 8u: buffer_cell_type.push_back(VTK_HEXAHEDRON); break;
						default: buffer_cell_type.push_back(INT32_MAX); break;
					}
				}, *(this->cell_cache_));

				for (auto& i : buffer_cell_type)
					i = swap_endianness_native_big(i);
				output.write(reinterpret_cast<char*>(&buffer_cell_type[0]), buffer_cell_type.size() * sizeof(int32));
				output << std::endl;
			} else {
				map.foreach_cell([&](Volume w)
				{
					const auto& indices = this->vertices_of_volumes(w);
					switch (indices.size()) {
						case 4u: output << VTK_TETRA; break;
						case 5u: output << VTK_PYRAMID; break;
						case 6u: output << VTK_WEDGE; break;
						case 8u: output << VTK_HEXAHEDRON; break;
						default: output << INT32_MAX; break;
					}
					output << std::endl;
				}, *(this->cell_cache_));
				output << std::endl;
			}
		} // end cell section

		{ // point data section
			if (!this->vertex_attributes().empty())
			{
				const auto& vertex_attributes = this->vertex_attributes();
				output << "POINT_DATA " << nbv << std::endl;
				for(ChunkArrayGen const* vatt : vertex_attributes)
				{
					output << "SCALARS " << vatt->name() << " " << cgogn_name_of_type_to_vtk_legacy_data_type(vatt->nested_type_name()) << " " << vatt->nb_components() << std::endl;
					output << "LOOKUP_TABLE default" << std::endl;
					map.foreach_cell([&](Vertex v)
					{
						vatt->export_element(map.embedding(v), output, bin, false);
						if (!bin)
							output << std::endl;
					}, *(this->cell_cache_));
					if (bin)
						output << std::endl;
				}
			}
		} // end point data section

		{ // cell data section
			if (!this->volume_attributes().empty())
			{
				const auto& volume_attributes = this->volume_attributes();
				output << "CELL_DATA " << nbw << std::endl;
				for(ChunkArrayGen const* watt : volume_attributes)
				{
					output << "SCALARS " << watt->name() << " " << cgogn_name_of_type_to_vtk_legacy_data_type(watt->nested_type_name()) << " " << watt->nb_components() << std::endl;
					output << "LOOKUP_TABLE default" << std::endl;
					map.foreach_cell([&](Volume w)
					{
						watt->export_element(map.embedding(w), output, bin, false);
						if (!bin)
							output << std::endl;
					}, *(this->cell_cache_));
					if (bin)
						output << std::endl;
				}
			}
		} // end cell data section
	}

	void export_vtu(const Map& map, std::ofstream& output, const ExportOptions& option)
	{
		ChunkArrayGen const* pos = this->position_attribute();
		const std::string endianness = cgogn::internal::cgogn_is_little_endian ? "LittleEndian" : "BigEndian";
		const std::string format = (option.binary_?"binary" :"ascii");
		std::string scalar_type = cgogn_name_of_type_to_vtk_xml_data_type(pos->nested_type_name());
		const uint32 nbv = this->nb_vertices();
		const uint32 nbw = this->nb_volumes();
		const bool bin = option.binary_;

		output << "<?xml version=\"1.0\"?>" << std::endl;
		output << "<VTKFile type=\"UnstructuredGrid\" header_type=\"UInt32\" version=\"0.1\" byte_order=\"" << endianness << "\"";
		if (option.compress_)
			output << " compressor=\"vtkZLibDataCompressor\"";
		output << ">" << std::endl;
		output << "  <UnstructuredGrid>" <<  std::endl;
		output << "    <Piece NumberOfPoints=\"" << nbv << "\" NumberOfCells=\""<< nbw << "\">" << std::endl;

		// 1st step : vertices
		output << "      <Points>" << std::endl;
		// 1.a : positions
		output << "        <DataArray type=\""<< scalar_type  << "\" Name=\"" << pos->name() << "\" NumberOfComponents=\"" << pos->nb_components() << "\" format=\"" << format << "\">" << std::endl;

		std::vector<char> buffer_char;

		if (bin)
		{
			const uint32 elem_size{pos->element_size()};
			buffer_char.clear();
			buffer_char.reserve(nbv * elem_size);

			map.foreach_cell([&](Vertex v)
			{
				const char* elem =  static_cast<const char*>(pos->element_ptr(map.embedding(v)));
				for(uint32 i = 0u; i < elem_size; ++i)
					buffer_char.push_back(elem[i]);
			}, *(this->cell_cache_));

			write_binary_xml_data(output,&buffer_char[0], buffer_char.size(), option.compress_);
			output << std::endl;
		} else {
			map.foreach_cell([&](Vertex v)
			{
					output << "          ";
					pos->export_element(map.embedding(v), output, false, false);
					output << std::endl;
			}, *(this->cell_cache_));
		}

		output << "        </DataArray>" << std::endl;
		output << "      </Points>" << std::endl;

		if (!this->vertex_attributes().empty())
		{
			output << "      <PointData>" << std::endl;
			// 1.B : other vertices attributes
			for (const auto& att : this->vertex_attributes())
			{
				scalar_type = cgogn_name_of_type_to_vtk_xml_data_type(att->nested_type_name());
				output << "        <DataArray type=\""<< scalar_type  <<"\" Name=\"" << att->name() << "\" NumberOfComponents=\"" << att->nb_components() << "\" format=\"" << format << "\">" << std::endl;

				if (bin)
				{
					const uint32 elem_size{pos->element_size()};
					buffer_char.clear();
					buffer_char.reserve(nbv * elem_size);

					map.foreach_cell([&](Vertex v)
					{
						const char* elem =  static_cast<const char*>(att->element_ptr(map.embedding(v)));
						for(uint32 i = 0u; i < elem_size; ++i)
							buffer_char.push_back(elem[i]);
					}, *(this->cell_cache_));
					write_binary_xml_data(output,&buffer_char[0], buffer_char.size(), option.compress_);
					output << std::endl;
				} else {
					map.foreach_cell([&](Vertex v)
					{
							output << "          ";
							att->export_element(map.embedding(v), output, false, false);
							output << std::endl;
					}, *(this->cell_cache_));
				}

				output << "        </DataArray>" << std::endl;
			}
			output << "      </PointData>" << std::endl;
		}
		// end vertices
		// begin volumes
		output << "      <Cells>" << std::endl;
		// 2.a. Connectivity
		output << "        <DataArray type=\"Int32\" Name=\"connectivity\" format=\"" << format << "\">" << std::endl;

		if (bin)
		{
			buffer_char.clear();
			buffer_char.reserve(4*nbw*sizeof(int32));
			map.foreach_cell([&](Volume w)
			{
				const auto& vertices = this->vertices_of_volumes(w);
				const char*data = reinterpret_cast<const char*>(&vertices[0]);
				for(uint32 i = 0u; i < vertices.size() * sizeof(int32) ; ++i)
					buffer_char.push_back(data[i]);
			}, *(this->cell_cache_));
			write_binary_xml_data(output,&buffer_char[0], buffer_char.size(), option.compress_);
			output << std::endl;
		} else {
			map.foreach_cell([&](Volume w)
			{
				const auto& vertices = this->vertices_of_volumes(w);
				output << "          ";
				for (auto i : vertices)
					output << i << " ";
				output << std::endl;
			}, *(this->cell_cache_));
		}

		output << "        </DataArray>" << std::endl;
		// 2.b. offsets
		output << "        <DataArray type=\"Int32\" Name=\"offsets\" format=\"" << format << "\">" << std::endl;


		int32 offset{0};
		std::vector<int32> buffer_offset;
		buffer_offset.reserve(nbw);
		map.foreach_cell([&](Volume w)
		{
			offset+= int32(this->number_of_vertices(w));
			buffer_offset.push_back(offset);
		}, *(this->cell_cache_));

		if (bin)
		{
			write_binary_xml_data(output,reinterpret_cast<const char*>(&buffer_offset[0]),  buffer_offset.size() * sizeof(int32), option.compress_);
		} else {
			output << "         ";
			for (auto o : buffer_offset)
				output << " " << o;
		}
		output << std::endl << "        </DataArray>" << std::endl;


		// 2.c cell types
		output << "        <DataArray type=\"UInt8\" Name=\"types\" format=\"" << format << "\">" << std::endl;
		std::vector<uint8> buffer_format;
		buffer_format.reserve(this->nb_volumes());
		map.foreach_cell([&](Volume w)
		{
			const int32 nbv = static_cast<int32>(this->number_of_vertices(w));
			switch (nbv) {
				case 4: buffer_format.push_back(VTK_TETRA); break;
				case 5: buffer_format.push_back(VTK_PYRAMID); break;
				case 6: buffer_format.push_back(VTK_WEDGE); break;
				case 8: buffer_format.push_back(VTK_HEXAHEDRON); break;
				default:
					break;
			}
		}, *(this->cell_cache_));

		if (bin)
		{
			write_binary_xml_data(output,reinterpret_cast<char*>(&buffer_format[0]),  buffer_format.size() * sizeof(uint8), option.compress_);
		} else {
			output << "         ";
			for (auto i : buffer_format)
				output << std::to_string(i) << " ";
		}
		output << std::endl << "        </DataArray>" << std::endl;
		output << "      </Cells>" << std::endl;

		//2.d other volumes attributes
		if (!this->volume_attributes().empty())
		{
			output << "      <CellData>" << std::endl;
			for (const auto& att : this->volume_attributes())
			{
				scalar_type = cgogn_name_of_type_to_vtk_xml_data_type(att->nested_type_name());
				output << "        <DataArray type=\""<< scalar_type  <<"\" Name=\"" << att->name() << "\" NumberOfComponents=\"" << att->nb_components() << "\" format=\"" << format << "\">" << std::endl;

				if (bin)
				{
					const uint32 elem_size{att->element_size()};
					buffer_char.clear();
					buffer_char.reserve(nbw * elem_size);

					map.foreach_cell([&](Volume w)
					{
						const char* elem =  static_cast<const char*>(att->element_ptr(map.embedding(w)));
						for(uint32 i = 0u; i < elem_size; ++i)
							buffer_char.push_back(elem[i]);
					}, *(this->cell_cache_));
					write_binary_xml_data(output,&buffer_char[0], buffer_char.size(), option.compress_);
					output << std::endl;
				} else {
					map.foreach_cell([&](Volume w)
					{
						output << "         ";
						att->export_element(map.embedding(w), output, false, false);
						output << std::endl;
					}, *(this->cell_cache_));
				}

				output << "         </DataArray>" << std::endl;
			}
			output << "      </CellData>" << std::endl;
		}
		output << "    </Piece>" << std::endl;
		output << "  </UnstructuredGrid>" << std::endl;
		output << "</VTKFile>" << std::endl;
	}
};

template <uint32 CHUNK_SIZE, uint32 PRIM_SIZE, typename VEC3>
class VtkIO
{
public:

	enum VTK_MESH_TYPE
	{
		UNKNOWN = 0,
		UNSTRUCTURED_GRID,
		POLYDATA
	};

	using Self = VtkIO<DEFAULT_CHUNK_SIZE, PRIM_SIZE, VEC3>;
	using DataInputGen = cgogn::io::DataInputGen<CHUNK_SIZE>;
	template <typename T>
	using DataInput = cgogn::io::DataInput<CHUNK_SIZE, PRIM_SIZE, T>;
	using Scalar = typename VEC3::Scalar;

	inline VtkIO() {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(VtkIO);
	virtual ~VtkIO() {}

protected:

	DataInput<VEC3>		positions_;
	DataInput<uint32>	cells_;
	DataInput<int>		cell_types_;
	DataInput<uint32>	offsets_;

protected:

	virtual void add_vertex_attribute(const DataInputGen& attribute_data, const std::string& attribute_name) = 0;
	virtual void add_cell_attribute(const DataInputGen& attribute_data, const std::string& attribute_name) = 0;


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
		else
		{
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
					type_str = vtk_data_type_to_cgogn_name_of_type(type_str);
					auto pos = DataInputGen::template newDataIO<PRIM_SIZE, VEC3>(type_str, 3);
					pos->read_n(fp, nb_vertices, !ascii_file, big_endian);
					this->add_vertex_attribute(*pos,"position");
					this->positions_ = *dynamic_cast_unique_ptr<DataInput<VEC3>>(pos->simplify());
				}
				else
				{
					if (word == "CELLS" || word == "POLYGONS" || word == "TRIANGLE_STRIPS")
					{
						uint32 size;
						sstream >> nb_cells >> size;
						cells_.read_n(fp, size, !ascii_file, big_endian);

						std::vector<int>* cell_types_vec = static_cast<std::vector<int>*>(cell_types_.buffer_vector());
						cgogn_assert(cell_types_vec != nullptr);
						if (word == "POLYGONS")
						{
							cell_types_vec->reserve(nb_cells);
							for (unsigned i = 0u; i < nb_cells ;++i)
							{
								cell_types_vec->push_back(VTK_CELL_TYPES::VTK_POLYGON);
							}
						}
						else if (word == "TRIANGLE_STRIPS")
						{
							cell_types_vec->reserve(nb_cells);
							for (unsigned i = 0u; i < nb_cells ;++i)
							{
								cell_types_vec->push_back(VTK_CELL_TYPES::VTK_TRIANGLE_STRIP);
							}
						}
					}
					else
					{
						if (word == "CELL_TYPES")
						{
							uint32 nbc;
							sstream >> nbc;
							cell_types_.read_n(fp, nbc, !ascii_file, big_endian);
						}
						else
						{
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
										if (word == "NORMALS" || to_upper(att_name) == "NORMAL" || to_upper(att_name) == "NORMALS")
											att_name = "normal";
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
										}
										else
										{
											fp.seekg(pos_before_lookup_table); // if there wasn't a lookup table we go back and start reading the numerical values
										}

										std::unique_ptr<DataInputGen> att(DataInputGen::template newDataIO<PRIM_SIZE>(att_type, num_comp));
										att->read_n(fp, nb_data, !ascii_file, big_endian);
										if (cell_data)
											this->add_cell_attribute(*att, att_name);
										else
											this->add_vertex_attribute(*att, att_name);
									}
									else
									{
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
												if (to_upper(data_name) == "NORMAL" || to_upper(data_name) == "NORMALS")
													data_name = "normal";
												cgogn_log_info("parse_vtk_legacy_file") << "reading field \"" << data_name << "\" of type " << data_type << " (" << nb_comp << " components).";
												std::unique_ptr<DataInputGen> att(DataInputGen::template newDataIO<PRIM_SIZE>(data_type, nb_comp));
												att->read_n(fp, nb_data, !ascii_file, big_endian);
												if (cell_data)
													this->add_cell_attribute(*att, data_name);
												else
													this->add_vertex_attribute(*att, data_name);
											}
										}
										else if (word == "LOOKUP_TABLE")
										{
											std::string table_name;
											/*uint32*/ nb_data = 0u;
											sstream >> table_name >> nb_data;
											cgogn_log_debug("parse_vtk_legacy_file") << "Ignoring the definition of the lookuptable named \"" << table_name << "\".";
											if (ascii_file)
											{
												DataInput<Eigen::Vector4f> trash;
												trash.skip_n(fp, nb_data, false);
											}
											else
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
								}
								else
									break;
							}
							else
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
		const bool little_endian = (!root_node->Attribute("byte_order")) || (to_lower(std::string(root_node->Attribute("byte_order"))) == "littleendian");

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

		piece_node->QueryUnsignedAttribute("NumberOfPoints", &nb_vertices);
		piece_node->QueryUnsignedAttribute("NumberOfCells", &nb_cells);
		if (nb_cells == 0u)
			piece_node->QueryUnsignedAttribute("NumberOfPolys", &nb_cells);
		if (nb_vertices == 0u || nb_cells == 0u)
			return false;

		XMLElement* points_node = piece_node->FirstChildElement("Points");
		cgogn_assert(points_node != nullptr);
		XMLElement* position_data_array_node = points_node->FirstChildElement("DataArray");
		for (XMLElement* elem = position_data_array_node; elem != nullptr; elem = elem->NextSiblingElement("DataArray"))
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
				data_name = std::string(vertex_data->Attribute("Name"));
			if (to_lower(data_name) == "normal" || to_lower(data_name) == "normals")
				data_name = "normal";
			const bool binary =  vertex_data->Attribute("format", nullptr) && (to_lower(std::string(vertex_data->Attribute("format", nullptr))) == "binary");
			uint32 nb_comp = 1u;
			vertex_data->QueryUnsignedAttribute("NumberOfComponents", &nb_comp);
			const std::string type = vtk_data_type_to_cgogn_name_of_type(std::string(vertex_data->Attribute("type", nullptr)));

			if (data_name.empty())
				cgogn_log_debug("parse_xml_vtu") << "Skipping a vertex DataArray without \"Name\" attribute.";
			else
			{
				const char*					ascii_data = vertex_data->GetText();
				std::vector<unsigned char>	binary_data;
				if (binary)
					binary_data = read_binary_xml_data(ascii_data,compressed, data_type(header_type));

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
				else
				{
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
					break;
				}
			}

			for (XMLElement* cell_data : cell_nodes)
			{
				std::string data_name;
				if (cell_data->Attribute("Name"))
					 data_name = std::string(cell_data->Attribute("Name"));
				if (to_lower(data_name) == "normal" || to_lower(data_name) == "normals")
					data_name = "normal";

				const bool binary = cell_data->Attribute("format", nullptr) && (to_lower(std::string(cell_data->Attribute("format", nullptr))) == "binary");
				uint32 nb_comp = 1u;
				cell_data->QueryUnsignedAttribute("NumberOfComponents", &nb_comp);
				std::string type = vtk_data_type_to_cgogn_name_of_type(std::string(cell_data->Attribute("type", nullptr)));

				if (data_name.empty())
					cgogn_log_debug("parse_xml_vtu") << "Skipping a cell DataArray without \"Name\" attribute.";
				else
				{
					const char*					ascii_data = cell_data->GetText();
					std::vector<unsigned char>	binary_data;
					if (binary)
						binary_data = read_binary_xml_data(ascii_data,compressed, data_type(header_type));

					std::unique_ptr<IMemoryStream> mem_stream;
					if (binary)
						mem_stream = make_unique<IMemoryStream>(reinterpret_cast<char*>(&binary_data[0]), binary_data.size());
					else
						mem_stream = make_unique<IMemoryStream>(ascii_data);
					if (data_name == "connectivity")
					{
						const uint32 last_offset = this->offsets_.vec()->back();
						auto cells = DataInputGen::template newDataIO<PRIM_SIZE, uint32>(type);
						cells->read_n(*mem_stream, last_offset,binary,!little_endian);
						this->cells_ = *dynamic_cast_unique_ptr<DataInput<uint32>>(cells->simplify());
					}
					else
					{
						if (data_name == "offsets")
						{
							auto offsets = DataInputGen::template newDataIO<PRIM_SIZE, uint32>(type);
							offsets->read_n(*mem_stream, nb_cells,binary,!little_endian);
							this->offsets_ = *dynamic_cast_unique_ptr<DataInput<uint32>>(offsets->simplify());
						}
						else
						{
							if (data_name == "types")
							{
								auto types = DataInputGen::template newDataIO<PRIM_SIZE, int>(type);
								types->read_n(*mem_stream, nb_cells,binary,!little_endian);
								this->cell_types_ = *dynamic_cast_unique_ptr<DataInput<int>>(types->simplify());
							}
							else
							{
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
				else
				{
					const char*					ascii_data = poly_data_array->GetText();
					std::vector<unsigned char>	binary_data;
					if (binary)
						binary_data = read_binary_xml_data(ascii_data,compressed, data_type(header_type));

					std::unique_ptr<IMemoryStream> mem_stream;
					if (binary)
						mem_stream = make_unique<IMemoryStream>(reinterpret_cast<char*>(&binary_data[0]), binary_data.size());
					else
						mem_stream = make_unique<IMemoryStream>(ascii_data);
					if (data_name == "connectivity")
					{
						const uint32 last_offset = this->offsets_.vec()->back();
						auto cells = DataInputGen::template newDataIO<PRIM_SIZE, uint32>(type);
						cells->read_n(*mem_stream, last_offset,binary,!little_endian);
						this->cells_ = *dynamic_cast_unique_ptr<DataInput<uint32>>(cells->simplify());
					}
					else
					{
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
};

template <typename MAP_TRAITS, typename VEC3>
class VtkSurfaceImport : public VtkIO<MAP_TRAITS::CHUNK_SIZE, CMap2<MAP_TRAITS>::PRIM_SIZE, VEC3>, public SurfaceImport<MAP_TRAITS>
{
public:

	using Self = VtkSurfaceImport<MAP_TRAITS, VEC3>;
	using Inherit_Vtk = VtkIO<MAP_TRAITS::CHUNK_SIZE, CMap2<MAP_TRAITS>::PRIM_SIZE, VEC3>;
	using Inherit_Import = SurfaceImport<MAP_TRAITS>;
	using DataInputGen = typename Inherit_Vtk::DataInputGen;
	template <typename T>
	using DataInput = typename Inherit_Vtk::template DataInput<T>;

	virtual ~VtkSurfaceImport() override
	{}

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

		auto cells_it = this->cells_.vec()->begin();
		uint32 last_offset = 0u;
		for(auto offset_it = this->offsets_.vec()->begin(), offset_end = this->offsets_.vec()->end(); offset_it != offset_end; ++offset_it)
		{
			const uint32 curr_offset = *offset_it;
			const uint32 nb_vertices = curr_offset - last_offset;
			this->faces_nb_edges_.push_back(nb_vertices);
			for (uint32 i = 0u; i < nb_vertices; ++i)
				this->faces_vertex_indices_.push_back(*cells_it++);
			last_offset = *offset_it;
		}

		return true;
	}

	virtual void add_vertex_attribute(const DataInputGen& attribute_data, const std::string& attribute_name) override
	{
		cgogn_log_info("VtkSurfaceImport::add_vertex_attribute") << "Adding a vertex attribute named \"" << attribute_name << "\".";
		attribute_data.to_chunk_array(attribute_data.add_attribute(this->vertex_attributes_, attribute_name));
	}

	virtual void add_cell_attribute(const DataInputGen& attribute_data, const std::string& attribute_name) override
	{
		cgogn_log_info("VtkSurfaceImport::add_cell_attribute") << "Adding a face attribute named \"" << attribute_name << "\".";
		attribute_data.to_chunk_array(attribute_data.add_attribute(this->face_attributes_, attribute_name));
	}

	virtual bool import_file_impl(const std::string& filename) override
	{
		const FileType ft = file_type(filename);
		switch (ft)
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
				cgogn_log_warning("VtkSurfaceImport::import_file_impl")<< "VtkSurfaceImport does not handle the files of type \"" << extension(filename) << "\".";
				return false;
		}
	}

private:

	inline void fill_surface_import()
	{
		this->nb_vertices_ = uint32(this->positions_.size());
		this->nb_faces_ = uint32(this->cell_types_.size());

		auto cells_it = static_cast<std::vector<uint32>*>(this->cells_.buffer_vector())->begin();
		const std::vector<int>* cell_types_vec = static_cast<std::vector<int>*>(this->cell_types_.buffer_vector());
		for(auto cell_types_it = cell_types_vec->begin(); cell_types_it != cell_types_vec->end(); )
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
			}
			else
			{
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

template <typename MAP_TRAITS, typename VEC3>
class VtkVolumeImport : public VtkIO<MAP_TRAITS::CHUNK_SIZE, CMap3<MAP_TRAITS>::PRIM_SIZE, VEC3>, public VolumeImport<MAP_TRAITS>
{
public:

	using Self = VtkVolumeImport<MAP_TRAITS, VEC3>;
	using Inherit_Vtk = VtkIO<MAP_TRAITS::CHUNK_SIZE, CMap3<MAP_TRAITS>::PRIM_SIZE, VEC3>;
	using Inherit_Import = VolumeImport<MAP_TRAITS>;
	using DataInputGen = typename Inherit_Vtk::DataInputGen;
	template <typename T>
	using DataInput = typename Inherit_Vtk::template DataInput<T>;
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

		const std::vector<int>* cell_types_vec	= this->cell_types_.vec();
		const std::vector<uint32>* cells_vec	= this->cells_.vec();
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
			else
			{
				if (*type_it == VTK_CELL_TYPES::VTK_HEXAHEDRON || *type_it == VTK_CELL_TYPES::VTK_VOXEL)
					vol_nb_verts = 8u;
				else
				{
					if (*type_it == VTK_CELL_TYPES::VTK_WEDGE)
						vol_nb_verts = 6u;
					else
					{
						if (*type_it == VTK_CELL_TYPES::VTK_PYRAMID)
							vol_nb_verts = 5u;
					}
				}
			}
			for (uint32 i = 0u; i < vol_nb_verts;++i)
			{
				cells_buffer.push_back(*cells_it++);
			}
		}

		add_vtk_volumes(cells_buffer, *cell_types_vec, *(this->template position_attribute<VEC3>()));

		return true;
	}

	inline bool read_vtk_xml_file(const std::string& filename)
	{
		if (!Inherit_Vtk::parse_xml_vtu(filename))
			return false;

		this->set_nb_vertices(uint32(this->positions_.size()));
		this->set_nb_volumes(uint32(this->cell_types_.size()));

		const std::vector<int>* cell_types_vec	= this->cell_types_.vec();
		const std::vector<uint32>* cells_vec	= this->cells_.vec();

		ChunkArray<VEC3>* pos = this->template position_attribute<VEC3>();
		cgogn_assert(pos != nullptr);
		add_vtk_volumes(*cells_vec,*cell_types_vec, *pos);

		return true;
	}

	virtual void add_vertex_attribute(const DataInputGen& attribute_data, const std::string& attribute_name) override
	{
		cgogn_log_info("VtkVolumeImport::add_vertex_attribute") << "Adding a vertex attribute named \"" << attribute_name << "\".";
		attribute_data.to_chunk_array(attribute_data.add_attribute(this->vertex_attributes_container(), attribute_name));
	}
	virtual void add_cell_attribute(const DataInputGen& attribute_data, const std::string& attribute_name) override
	{
		cgogn_log_info("VtkVolumeImport::add_cell_attribute") << "Adding a volume attribute named \"" << attribute_name << "\".";
		attribute_data.to_chunk_array(attribute_data.add_attribute(this->volume_attributes_container(), attribute_name));
	}

	virtual bool import_file_impl(const std::string& filename) override
	{
		const FileType ft = file_type(filename);
		switch (ft)
		{
			case FileType::FileType_VTK_LEGACY:
			{
				std::ifstream fp(filename.c_str(), std::ios::in | std::ios::binary);
				return this->read_vtk_legacy_file(fp);
			}
			case FileType::FileType_VTU:
				return this->read_vtk_xml_file(filename);
			default:
				cgogn_log_warning("VtkVolumeImport::import_file_impl")<< "VtkSurfaceImport does not handle the files of type \"" << extension(filename) << "\".";
				return false;
		}
	}

	inline void add_vtk_volumes(std::vector<uint32> ids, const std::vector<int>& type_vol, ChunkArray<VEC3> const& pos)
	{
		uint32 curr_offset = 0;
		for (uint32 i = 0u, end = this->nb_volumes(); i< end; ++i)
		{
			if (type_vol[i] == VTK_CELL_TYPES::VTK_HEXAHEDRON || type_vol[i] == VTK_CELL_TYPES::VTK_VOXEL)
			{
				if (type_vol[i] == VTK_CELL_TYPES::VTK_VOXEL)
				{
					std::swap(ids[curr_offset+2], ids[curr_offset+3]);
					std::swap(ids[curr_offset+6], ids[curr_offset+7]);
				}
				this->add_hexa(pos, ids[curr_offset+0], ids[curr_offset+1], ids[curr_offset+2], ids[curr_offset+3], ids[curr_offset+4], ids[curr_offset+5], ids[curr_offset+6], ids[curr_offset+7], true);
				curr_offset += 8u;
			}
			else
			{
				if (type_vol[i] == VTK_CELL_TYPES::VTK_TETRA)
				{
					this->add_tetra(pos, ids[curr_offset+0], ids[curr_offset+1], ids[curr_offset+2], ids[curr_offset+3], true);
					curr_offset += 4u;
				}
				else
				{
					if (type_vol[i] == VTK_CELL_TYPES::VTK_PYRAMID)
					{
						this->add_pyramid(pos, ids[curr_offset+0], ids[curr_offset+1], ids[curr_offset+2], ids[curr_offset+3], ids[curr_offset+4], true);
						curr_offset += 5u;
					}
					else
					{
						if (type_vol[i] == VTK_CELL_TYPES::VTK_WEDGE)
						{
							this->add_triangular_prism(pos, ids[curr_offset+0], ids[curr_offset+1], ids[curr_offset+2], ids[curr_offset+3], ids[curr_offset+4], ids[curr_offset+5], true);
							curr_offset += 6u;
						}
					}
				}
			}
		}
	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_VTK_IO_CPP_))
extern template class CGOGN_IO_API VtkIO<DefaultMapTraits::CHUNK_SIZE,1, Eigen::Vector3d>;
extern template class CGOGN_IO_API VtkIO<DefaultMapTraits::CHUNK_SIZE,1, Eigen::Vector3f>;
extern template class CGOGN_IO_API VtkIO<DefaultMapTraits::CHUNK_SIZE,1, geometry::Vec_T<std::array<float64,3>>>;
extern template class CGOGN_IO_API VtkIO<DefaultMapTraits::CHUNK_SIZE,1, geometry::Vec_T<std::array<float32,3>>>;

extern template class CGOGN_IO_API VtkVolumeImport<DefaultMapTraits, Eigen::Vector3d>;
extern template class CGOGN_IO_API VtkVolumeImport<DefaultMapTraits, Eigen::Vector3f>;
extern template class CGOGN_IO_API VtkVolumeImport<DefaultMapTraits, geometry::Vec_T<std::array<float64,3>>>;
extern template class CGOGN_IO_API VtkVolumeImport<DefaultMapTraits, geometry::Vec_T<std::array<float32,3>>>;

extern template class CGOGN_IO_API VtkVolumeExport<CMap3<DefaultMapTraits>>;
extern template class CGOGN_IO_API VtkSurfaceExport<CMap2<DefaultMapTraits>>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_VTK_IO_CPP_))

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_VTK_IO_H_
