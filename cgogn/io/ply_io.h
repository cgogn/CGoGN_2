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

#ifndef CGOGN_IO_PLY_IO_H_
#define CGOGN_IO_PLY_IO_H_

#include <cgogn/geometry/types/eigen.h>
#include <cgogn/geometry/types/vec.h>
#include <cgogn/geometry/types/geometry_traits.h>

#include <cgogn/io/surface_import.h>
#include <cgogn/io/import_ply_data.h>
#include <cgogn/io/surface_export.h>

namespace cgogn
{

namespace io
{

CGOGN_IO_API std::string cgogn_name_of_type_to_ply_data_type(const std::string& cgogn_type);

template <typename MAP, typename VEC3>
class PlySurfaceImport : public SurfaceFileImport<MAP, VEC3>
{
public:

	using Self = PlySurfaceImport<MAP, VEC3>;
	using Inherit = SurfaceFileImport<MAP, VEC3>;
	using Scalar = typename geometry::vector_traits<VEC3>::Scalar;
	template <typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

	inline PlySurfaceImport(MAP& map) : Inherit(map) {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(PlySurfaceImport);
	virtual ~PlySurfaceImport() override {}

protected:

	virtual bool import_file_impl(const std::string& filename) override
	{
		PlyImportData pid;

		if (! pid.read_file(filename) )
		{
			cgogn_log_error("PlySurfaceImport::import_file_impl") << "Unable to open the file \"" << filename << "\".";
			return false;
		}

		ChunkArray<VEC3>* position = this->position_attribute();
		ChunkArray<VEC3>* color = nullptr;
		if (pid.has_colors())
			color = this->template add_vertex_attribute<VEC3>("color");

		const uint32 nb_vertices = pid.nb_vertices();
		const uint32 nb_faces = pid.nb_faces();

		// read vertices position
		std::vector<uint32> vertices_id;
		vertices_id.reserve(nb_vertices);

		for (uint32 i = 0; i < nb_vertices; ++i)
		{
			VEC3 pos;
			pid.vertex_position(i, pos);

			uint32 vertex_id = this->insert_line_vertex_container();
			(*position)[vertex_id] = pos;

			vertices_id.push_back(vertex_id);

			if (pid.has_colors())
			{
				VEC3 rgb;
				pid.vertex_color_float32(i, rgb);
				(*color)[vertex_id] = pos;
			}
		}

		// read faces (vertex indices)
		this->reserve(nb_faces);
		for (uint32 i = 0; i < nb_faces; ++i)
		{
			uint32 n = pid.face_valence(i);
			this->faces_nb_edges_.push_back(n);
			int* indices = pid.face_indices(i);
			for (uint32 j = 0; j < n; ++j)
			{
				uint32 index = uint32(indices[j]);
				this->faces_vertex_indices_.push_back(vertices_id[index]);
			}
		}

		return true;
	}
};

template <typename MAP>
class PlySurfaceExport : public SurfaceExport<MAP>
{
public:

	using Inherit = SurfaceExport<MAP>;
	using Self = PlySurfaceExport<MAP>;
	using Map = typename Inherit::Map;
	using Vertex = typename Inherit::Vertex;
	using Face = typename Inherit::Face;
	using ChunkArrayGen = typename Inherit::ChunkArrayGen;
	template<typename T>
	using VertexAttribute = typename Inherit::template VertexAttribute<T>;
	using ChunkArrayContainer = typename Inherit::ChunkArrayContainer;

protected:

	virtual void export_file_impl(const Map& map, std::ofstream& output, const ExportOptions& option) override
	{
		if (option.binary_)
			this->export_binary(map, output, option);
		else
			this->export_ascii(map, output, option);
	}
private:
	void export_ascii(const Map& map, std::ofstream& output, const ExportOptions& /*option*/)
	{
		output << "ply" << std::endl ;
		output << "format ascii 1.0" << std::endl ;
		output << "comment File generated by the CGoGN library" << std::endl ;
		output << "comment See : http://cgogn.unistra.fr/" << std::endl ;
		output << "comment or contact : cgogn@unistra.fr" << std::endl ;
		output << "element vertex " <<  map.template nb_cells<Vertex::ORBIT>() << std::endl ;
		output << "property float x" << std::endl ;
		output << "property float y" << std::endl ;
		output << "property float z" << std::endl ;
		output << "element face " << map.template nb_cells<Face::ORBIT>() << std::endl ;
		output << "property list uint uint vertex_indices" << std::endl ;
		output << "end_header" << std::endl ;

		// set precision for real output
		output << std::setprecision(12);


		map.foreach_cell([&] (Vertex v)
		{
			this->position_attribute(Vertex::ORBIT)->export_element(map.embedding(v), output, false, false);
			output << std::endl;
		}, *(this->cell_cache_));

		std::vector<uint32> prim;
		prim.reserve(20);
		map.foreach_cell([&] (Face f)
		{
			uint32 valence = 0;
			prim.clear();

			map.foreach_incident_vertex(f, [&] (Vertex v)
			{
				prim.push_back(this->vindices_[v]);
				++valence;
			});
			output << valence;
			for(uint32 i: prim)
				output << " " << i;
			output << std::endl;

		}, *(this->cell_cache_));
	}

	void export_binary(const Map& map, std::ofstream& output, const ExportOptions& /*option*/)
	{

		output << "ply" << std::endl ;
		if (cgogn::internal::cgogn_is_big_endian)
			output << "format binary_big_endian 1.0" << std::endl ;
		else
			output << "format binary_little_endian 1.0" << std::endl ;
		output << "comment File generated by the CGoGN library" << std::endl ;
		output << "comment See : http://cgogn.unistra.fr/" << std::endl ;
		output << "comment or contact : cgogn@unistra.fr" << std::endl ;
		output << "element vertex " << map.template nb_cells<Vertex::ORBIT>() << std::endl ;
		output << "property " << cgogn_name_of_type_to_ply_data_type(this->position_attribute(Vertex::ORBIT)->nested_type_name()) << " x" << std::endl ;
		output << "property " << cgogn_name_of_type_to_ply_data_type(this->position_attribute(Vertex::ORBIT)->nested_type_name()) << " y" << std::endl ;
		output << "property " << cgogn_name_of_type_to_ply_data_type(this->position_attribute(Vertex::ORBIT)->nested_type_name()) << " z" << std::endl ;
		output << "element face " << map.template nb_cells<Face::ORBIT>() << std::endl ;
		output << "property list uint uint vertex_indices" << std::endl ;
		output << "end_header" << std::endl ;

		static const uint32 BUFFER_SZ = 1024 * 1024;


		map.foreach_cell([&] (Vertex v)
		{
			this->position_attribute(Vertex::ORBIT)->export_element(map.embedding(v), output, true, cgogn::internal::cgogn_is_little_endian);
		}, *(this->cell_cache_));

		// second pass to save primitives
		std::vector<uint32> buffer_prims;
		buffer_prims.reserve(BUFFER_SZ + 128);// + 128 to avoid re-allocations

		std::vector<uint32> prim;
		prim.reserve(20);
		map.foreach_cell([&] (Face f)
		{
			uint32 valence = 0;
			prim.clear();

			map.foreach_incident_vertex(f, [&] (Vertex v)
			{
				prim.push_back(this->vindices_[v]);
				++valence;
			});

			buffer_prims.push_back(valence);
			for(uint32 i: prim)
				buffer_prims.push_back(i);

			if (buffer_prims.size() >= BUFFER_SZ)
			{
				output.write(reinterpret_cast<char*>(&(buffer_prims[0])), buffer_prims.size()*sizeof(uint32));
				buffer_prims.clear();
			}
		}, *(this->cell_cache_));
		if (!buffer_prims.empty())
		{
			output.write(reinterpret_cast<char*>(&(buffer_prims[0])), buffer_prims.size()*sizeof(uint32));
			buffer_prims.clear();
		}
	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_PLY_IO_CPP_))
extern template class CGOGN_IO_API PlySurfaceImport<CMap2, Eigen::Vector3d>;
extern template class CGOGN_IO_API PlySurfaceImport<CMap2, Eigen::Vector3f>;
extern template class CGOGN_IO_API PlySurfaceImport<CMap2, geometry::Vec_T<std::array<float64, 3>>>;
extern template class CGOGN_IO_API PlySurfaceImport<CMap2, geometry::Vec_T<std::array<float32, 3>>>;

extern template class CGOGN_IO_API PlySurfaceExport<CMap2>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_PLY_IO_CPP_))

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_PLY_IO_H_
