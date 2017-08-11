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

#ifndef CGOGN_IO_TETMESH_IO_H_
#define CGOGN_IO_TETMESH_IO_H_

#include <map>

#include <cgogn/core/utils/logger.h>
#include <cgogn/io/dll.h>
#include <cgogn/io/volume_import.h>
#include <cgogn/io/volume_export.h>

namespace cgogn
{

namespace io
{

template <typename MAP, typename VEC3>
class TetMeshVolumeImport : public VolumeFileImport<MAP, VEC3>
{
public:

	using Self = TetMeshVolumeImport<MAP, VEC3>;
	using Inherit = VolumeFileImport<MAP, VEC3>;
	template <typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

	inline TetMeshVolumeImport(MAP& map) : Inherit(map) {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(TetMeshVolumeImport);
	virtual ~TetMeshVolumeImport() override {}

protected:

	virtual bool import_file_impl(const std::string& filename) override
	{
		ChunkArray<VEC3>* position = this->position_attribute();
		std::ifstream fp(filename, std::ios::in);

		std::string line;
		line.reserve(512);

		// reading number of vertices
		uint32 nb_vertices = 0u;
		{
			getline_safe(fp, line);
			std::stringstream sstream(line);
			sstream >> line;
			if (!i_equals(line,"vertices"))
			{
				cgogn_log_warning("TetMeshVolumeImport::import_file_impl") << "Error while reading the tetmesh file.";
				return false;
			}
			getline_safe(fp,line);
			nb_vertices = std::stoul(line);
		}

		//reading vertices
		for(uint32 i = 0u; i < nb_vertices; ++i)
		{
			do
			{
				getline_safe(fp, line);
			} while (line.empty());

			const uint32 new_id = this->insert_line_vertex_container();
			auto& v = position->operator[](new_id);
			std::istringstream iss(line);
			iss >> v[0];
			iss >> v[1];
			iss >> v[2];
		}

		uint32 nb_tetras = 0u;
		{
			getline_safe(fp, line);
			std::stringstream sstream(line);
			sstream >> line;
			if (!i_equals(line,"tetrahedra"))
			{
				cgogn_log_warning("TetMeshVolumeImport::import_file_impl") << "Error while reading the tetmesh file.";
				return false;
			}
			getline_safe(fp,line);
			nb_tetras = std::stoul(line);
		}

		this->reserve(nb_tetras);

		// reading volumes
		for (uint32 i = 0u; i < nb_tetras; ++i)
		{
			do
			{
				getline_safe(fp, line);
			} while (line.empty());

			std::istringstream iss(line);
			std::array<uint32, 4> ids;

			for (auto& id : ids)
				iss >> id;

			this->add_tetra(ids[0] - 1, ids[1] - 1, ids[2] - 1, ids[3] - 1, true);
		}

		return true;
	}
};

template <typename MAP>
class TetMeshVolumeExport : public VolumeExport<MAP>
{
public:

	using Self = TetMeshVolumeExport<MAP>;
	using Inherit = VolumeExport<MAP>;
	using Map = typename Inherit::Map;
	using Vertex = typename Inherit::Vertex;
	using Volume = typename Inherit::Volume;
	using ChunkArrayGen = typename Inherit::ChunkArrayGen;

protected:

	virtual void export_file_impl(const Map& map, std::ofstream& output, const ExportOptions& ) override
	{
		ChunkArrayGen const* pos = this->position_attribute(Vertex::ORBIT);
		const uint32 nb_vols = this->nb_volumes();
		const uint32 nb_vert = this->nb_vertices();

		// 1. vertices
		output << "Vertices" << std::endl;
		output << nb_vert << std::endl;

		map.foreach_cell([&](Vertex v)
		{
			pos->export_element(map.embedding(v), output, false, false);
			output << 0. << std::endl;
		}, *(this->cell_cache_));

		// 2. Tetrahedra
		output << "Tetrahedra" << std::endl;
		output << nb_vols << std::endl;

		map.foreach_cell([&](Volume w)
		{
			const auto& vertices = this->vertices_of_volumes(w);
			if (vertices.size() != 4)
			{
				cgogn_log_warning("TetMeshVolumeExport::VolumeExport") << "Tetmesh files only accept tetrahedra.";
				return;
			}
			output << vertices[0] + 1u << " " << vertices[1] + 1u << " " << vertices[2] + 1u << " " << vertices[3] + 1u << std::endl;
		}, *(this->cell_cache_));
	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_TETMESH_IO_CPP_))
extern template class CGOGN_IO_API TetMeshVolumeImport<CMap3, Eigen::Vector3d>;
extern template class CGOGN_IO_API TetMeshVolumeImport<CMap3, Eigen::Vector3f>;
extern template class CGOGN_IO_API TetMeshVolumeImport<CMap3, geometry::Vec_T<std::array<float64, 3>>>;
extern template class CGOGN_IO_API TetMeshVolumeImport<CMap3, geometry::Vec_T<std::array<float32, 3>>>;

extern template class CGOGN_IO_API TetMeshVolumeExport<CMap3>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_TETMESH_IO_CPP_))

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_TETMESH_IO_H_
