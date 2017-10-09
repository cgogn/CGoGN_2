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

#ifndef CGOGN_IO_FORMATS_TET_H_
#define CGOGN_IO_FORMATS_TET_H_

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
class TetVolumeImport : public VolumeFileImport<MAP>
{
public:

	using Self = TetVolumeImport<MAP, VEC3>;
	using Inherit = VolumeFileImport<MAP>;
	using Scalar = typename VEC3::Scalar;
	template <typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

	inline TetVolumeImport(MAP& map) : Inherit(map) {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(TetVolumeImport);
	virtual ~TetVolumeImport() override {}

protected:

	virtual bool import_file_impl(const std::string& filename) override
	{
		ChunkArray<VEC3>* position = this->template add_vertex_attribute<VEC3>("position");
		std::ifstream fp(filename, std::ios::in);

		std::string line;
		line.reserve(512);

		// reading number of vertices
		uint32 nb_vertices = 0u;
		{
		getline_safe(fp, line);
		std::istringstream iss(line);
		iss >> nb_vertices;
		}

		uint32 nb_volumes = 0u;
		// reading number of tetrahedra
		{
			getline_safe(fp, line);
			std::istringstream iss(line);
			iss >> nb_volumes;
		}
		this->reserve(nb_volumes);

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
			// TODO : if required read other vertices attributes here
		}

		// reading volumes
		for (uint32 i = 0u; i < nb_volumes; ++i)
		{
			do
			{
				getline_safe(fp, line);
			} while (line.empty());

			std::istringstream iss(line);
			uint32 nbv;
			iss >> nbv; // type of volumes

			if (!iss.good())
			{
				iss.clear();
				char connector;
				iss >> connector >> connector; // the line should be like this: # C id0 id1 id2 id3
				if (connector == 'C')
				{
					this->reserve(nb_volumes -1u);
					std::array<uint32,4> ids;
					iss >> ids[0] >> ids[1] >> ids[2] >> ids[3];
					this->add_connector(ids[0], ids[1], ids[2], ids[3]);
				}
					continue;
			}

			std::vector<uint32> ids;
			ids.resize(nbv);
			for (auto& id : ids)
				iss >> id;

			switch (nbv)
			{
				case 4: {
					this->template reorient_tetra<VEC3>(*position, ids[0], ids[1], ids[2], ids[3]);
					this->add_tetra(ids[1], ids[2], ids[3], ids[0]);
					break;
				}
				case 5: {
					this->template reorient_pyramid<VEC3>(*position, ids[0], ids[1], ids[2], ids[3],ids[4]);
					this->add_pyramid(ids[0], ids[1], ids[2], ids[3],ids[4]);
					break;
				}
				case 6: {
					this->template reorient_triangular_prism<VEC3>(*position, ids[0], ids[1], ids[2], ids[3], ids[4], ids[5]);
					this->add_triangular_prism(ids[0], ids[1], ids[2], ids[3], ids[4], ids[5]);
					break;
				}
				case 8: {
					this->template reorient_hexa<VEC3>(*position, ids[4], ids[5], ids[7], ids[6], ids[0], ids[1], ids[3], ids[2]);
					this->add_hexa(ids[4], ids[5], ids[7], ids[6], ids[0], ids[1], ids[3], ids[2]);
					break;
				}
				default:
					cgogn_log_warning("TetVolumeImport") << "Elements with " << nbv << " vertices are not handled. Ignoring.";
					break;
			}
		}

		return true;
	}
};

template <typename MAP>
class TetVolumeExport : public VolumeExport<MAP>
{
public:

	using Inherit = VolumeExport<MAP>;
	using Self = TetVolumeExport<MAP>;
	using Map = typename Inherit::Map;
	using Vertex = typename Inherit::Vertex;
	using Volume = typename Inherit::Volume;
	using ChunkArrayGen = typename Inherit::ChunkArrayGen;

protected:
	virtual void export_file_impl(const Map& map, std::ofstream& output, const ExportOptions& option) override
	{
		ChunkArrayGen const* pos = this->position_attribute(Vertex::ORBIT);
		const std::string endianness = cgogn::internal::cgogn_is_little_endian ? "LittleEndian" : "BigEndian";
		const std::string format = (option.binary_?"binary" :"ascii");
		std::string scalar_type = pos->nested_type_name();
		scalar_type[0] = std::toupper(scalar_type[0], std::locale());
		const uint32 nb_vols = this->nb_volumes();
		const uint32 nb_vert = this->nb_vertices();

		// 1. vertices
		output << nb_vert << " vertices" << std::endl;
		output << nb_vols << " cells" << std::endl;

		map.foreach_cell([&](Vertex v)
		{
			pos->export_element(map.embedding(v), output, false, false);
			output << std::endl;
		}, *(this->cell_cache_));

		map.foreach_cell([&](Volume w)
		{
			const auto& vertices = this->vertices_of_volumes(w);
			output << vertices.size() << " ";
			for (auto i : vertices)
				output << i  << " ";
			output << std::endl;
		}, *(this->cell_cache_));

	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_FORMATS_TET_CPP_))
extern template class CGOGN_IO_API TetVolumeImport<CMap3, Eigen::Vector3d>;
extern template class CGOGN_IO_API TetVolumeImport<CMap3, Eigen::Vector3f>;
extern template class CGOGN_IO_API TetVolumeImport<CMap3, geometry::Vec_T<std::array<float64, 3>>>;
extern template class CGOGN_IO_API TetVolumeImport<CMap3, geometry::Vec_T<std::array<float32, 3>>>;

extern template class CGOGN_IO_API TetVolumeExport<CMap3>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_FORMATS_TET_CPP_))

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_FORMATS_TET_H_
