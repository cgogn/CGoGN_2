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

#ifndef CGOGN_IO_VTK_G_IO_H_
#define CGOGN_IO_VTK_G_IO_H_

#include <cgogn/geometry/types/eigen.h>
#include <cgogn/geometry/types/vec.h>
#include <cgogn/geometry/types/geometry_traits.h>

#include <cgogn/io/graph_import.h>
#include <cgogn/io/graph_export.h>

#include <cgogn/io/vtk_io.h>

namespace cgogn
{

namespace io
{

template <typename VEC3>
class VtkGraphImport : public VtkIO<UndirectedGraph::PRIM_SIZE, VEC3>, public GraphFileImport<VEC3>
{
public:

	using Self = VtkGraphImport<VEC3>;
	using Inherit_Vtk = VtkIO<UndirectedGraph::PRIM_SIZE, VEC3>;
	using Inherit_Import = GraphFileImport<VEC3>;
	using DataInputGen = typename Inherit_Vtk::DataInputGen;
	template <typename T>
	using DataInput = typename Inherit_Vtk::template DataInput<T>;

	virtual ~VtkGraphImport() override
	{}

protected:
	inline bool read_vtk_legacy_file(std::ifstream& fp)
	{
		if (!Inherit_Vtk::parse_vtk_legacy_file(fp))
			return false;
		this->fill_graph_import();
		return true;
	}

	virtual void add_vertex_attribute(const DataInputGen& attribute_data, const std::string& attribute_name) override
	{
		cgogn_log_info("VtkGraphImport::add_vertex_attribute") << "Adding a vertex attribute named \"" << attribute_name << "\".";
		Inherit_Import::add_vertex_attribute(attribute_data, attribute_name);
	}

	virtual void add_cell_attribute(const DataInputGen& attribute_data, const std::string& attribute_name) override
	{
		cgogn_log_info("VtkGraphImport::add_cell_attribute") << "Adding an edge attribute named \"" << attribute_name << "\".";
		Inherit_Import::add_edge_attribute(attribute_data, attribute_name);
	}

	virtual bool import_file_impl(const std::string& filename) override
	{
		std::ifstream fp(filename.c_str(), std::ios::in | std::ios_base::binary);
		cgogn_assert(fp.good());
		return this->read_vtk_legacy_file(fp);
	}
private:
	inline void fill_graph_import()
	{
		const uint32 nb_edges = uint32(this->cell_types_.size());
		this->reserve(nb_edges + 2);

		auto cells_it = this->cells_.vec().begin();
		const std::vector<int32>& cell_types_vec = this->cell_types_.vec();
		const auto offsets_begin = this->offsets_.vec().begin();
		auto offset_it = offsets_begin;
		std::size_t last_offset(0);
		for(auto cell_types_it = cell_types_vec.begin(); cell_types_it != cell_types_vec.end(); )
		{
			const int cell_type = *(cell_types_it++);
			std::size_t nb_vert(0);
			if (this->vtk_file_type_ == FileType::FileType_VTK_LEGACY)
				nb_vert = *cells_it++;
			else
			{
				const std::size_t curr_offset = *offset_it++;
				nb_vert = curr_offset - last_offset;
				last_offset = curr_offset;
			}

			if (cell_type == VTK_CELL_TYPES::VTK_LINE)
			{
				this->edges_nb_vertices_.push_back(uint32(nb_vert));
				for(std::size_t i = 0ul ; i < nb_vert ; ++i)
				{
					this->edges_vertex_indices_.push_back(*cells_it++);
				}
			}
		}
	}
};

template <typename MAP>
class VtkGraphExport : public GraphExport<MAP>
{
public:
	using Inherit = GraphExport<MAP>;
	using Self = VtkGraphExport<MAP>;
	using Map = typename Inherit::Map;
	using Vertex = typename Inherit::Vertex;
	using Edge = typename Inherit::Edge;
	using ChunkArrayGen = typename Inherit::ChunkArrayGen;
	template <typename T>
	using VertexAttribute = typename Inherit::template VertexAttribute<T>;
	using ChunkArrayContainer = typename Inherit::ChunkArrayContainer;

	virtual void export_file_impl(const Map& map, std::ofstream& output, const ExportOptions& option) override
	{
		if(to_lower(extension(option.filename_)) == "vtk")
			this->export_legacy_vtk(map, output, option);
	}

private:
	void export_legacy_vtk(const Map& map, std::ofstream& output, const ExportOptions& option)
	{
		const bool bin = option.binary_;

		if(bin)
			cgogn_log_info("vtkGraphExport") << "binary export for graphs not implemented yet";

		const uint32 nbv = map.template nb_cells<Vertex::ORBIT>();
		const uint32 nbe = map.template nb_cells<Edge::ORBIT>();

		std::string scalar_type = cgogn_name_of_type_to_vtk_legacy_data_type(this->position_attribute(Vertex::ORBIT)->nested_type_name());

		output << "# vtk DataFile Version 3.0" << std::endl;
		output << "Mesh exported with CGoGN : github.com/cgogn/CGoGN_2" << std::endl;
		output << "ASCII" << std::endl;
		output << "DATASET UNSTRUCTURED_GRID" << std::endl;

		{// point section
			output << "POINTS " << nbv << " " << scalar_type << std::endl;
			map.foreach_cell([&](Vertex v)
			{
				this->position_attribute(Vertex::ORBIT)->export_element(map.embedding(v), output, false, false);
				output << std::endl;
			}, *(this->cell_cache_));
			output << std::endl;
		} // end point section

		{ // cell section
			std::vector<uint32> buffer_cells;
			buffer_cells.reserve(2u*nbe);
			uint32 cell_section_size{0u};
			map.foreach_cell([&](Edge e)
			{
				buffer_cells.push_back(2);
				cell_section_size += buffer_cells.back();
				Dart it = e.dart;
				do {
					buffer_cells.push_back(this->vindices_[Vertex(it)]);
					it = map.alpha0(it);
				} while (it != e.dart);
			}, *(this->cell_cache_));

			cell_section_size += nbe; // we add an integer for each face (the nb of vertices)

			output << "CELLS " << nbe << " " << cell_section_size << std::endl;

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

			output << std::endl ;

			output << "CELL_TYPES " << nbe << std::endl;

			for (auto it = buffer_cells.begin(), end = buffer_cells.end() ; it != end ;)
			{
				const uint32 nb_vert = *it;
				switch (nb_vert) {
					case 2u: output << VTK_LINE; break;
				}
				output << std::endl;
				it += nb_vert + 1u;
			}

			output << std::endl ;

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
						vatt->export_element(map.embedding(v), output, false, false);
						output << std::endl;
					}, *(this->cell_cache_));
				}

				output << std::endl ;
			}
		} // end point data section

		{ // cell data section
			if (!this->edge_attributes().empty())
			{
				const auto& edge_attributes = this->edge_attributes();
				output << "CELL_DATA " << nbe << std::endl;
				for(ChunkArrayGen const* eatt : edge_attributes)
				{
					output << "SCALARS " << eatt->name() << " " << cgogn_name_of_type_to_vtk_legacy_data_type(eatt->nested_type_name()) << " " << eatt->nb_components() << std::endl;
					output << "LOOKUP_TABLE default" << std::endl;
					map.foreach_cell([&](Edge e)
					{
						eatt->export_element(map.embedding(e), output, false, false);
						output << std::endl;
					}, *(this->cell_cache_));

				}
			}
		} // end cell data section
	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_VTK_G_IO_CPP_))
extern template class CGOGN_IO_API VtkGraphImport<Eigen::Vector3d>;
extern template class CGOGN_IO_API VtkGraphImport<Eigen::Vector3f>;
extern template class CGOGN_IO_API VtkExportExport<UndirectedGraph>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_VTK_G_IO_CPP_))

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_VTK_G_IO_H_
