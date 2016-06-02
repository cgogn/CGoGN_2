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

#ifndef CGOGN_IO_TETGEN_IO_H_
#define CGOGN_IO_TETGEN_IO_H_

#include <map>
#include <sstream>

#include <cgogn/core/utils/logger.h>
#include <cgogn/io/dll.h>
#include <cgogn/io/data_io.h>
#include <cgogn/io/volume_import.h>

namespace cgogn
{

namespace io
{

template <typename MAP_TRAITS, typename VEC3>
class TetgenVolumeImport : public VolumeImport<MAP_TRAITS>
{
public:
	using Inherit = VolumeImport<MAP_TRAITS>;
	using Self = TetgenVolumeImport<MAP_TRAITS, VEC3>;
	template <typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

protected:
	virtual bool import_file_impl(const std::string& filename) override
	{
		const std::string node_filename = filename.substr(0, filename.rfind('.')) + ".node";
		const std::string ele_filename = filename.substr(0, filename.rfind('.')) + ".ele";

		ChunkArray<VEC3>* position = this->template position_attribute<VEC3>();
		std::ifstream node_file(node_filename, std::ios::in);
		if (!node_file.good())
		{
			cgogn_log_warning("TetgenVolumeImport") << "Unable to open file \"" << node_filename << "\".";
			return false;
		}

		std::ifstream ele_file(ele_filename, std::ios::in);
		if (!ele_file.good())
		{
			cgogn_log_warning("TetgenVolumeImport") << "Unable to open file \"" << ele_filename << "\".";
			return false;
		}

		std::string line;
		line.reserve(512);

		//Reading NODE file
		//First line: [# of points] [dimension (must be 3)] [# of attributes] [# of boundary markers (0 or 1)]
		{
			do
			{
				std::getline(node_file,line);
			}while(line.empty());

			std::istringstream iss(line);
			uint32 nbv = 0u;
			iss >> nbv;
			this->set_nb_vertices(nbv);
		}

		//Reading number of tetrahedra in ELE file
		{
			do
			{
				std::getline(ele_file,line);
			}while(line.empty());

			std::istringstream iss(line);
			uint32 nbw = 0u;
			iss >> nbw;
			this->set_nb_volumes(nbw);
		}

		//Reading vertices
		std::map<uint32, uint32> old_new_ids_map;

		for(uint32 i = 0u, end = this->nb_vertices() ; i < end; ++i)
		{
			do
			{
				std::getline(node_file,line);
			}while(line.empty());

			std::istringstream iss(line);

			uint32 old_index;
			iss >> old_index;

			const uint32 new_index = this->insert_line_vertex_container();
			old_new_ids_map[old_index] = new_index;

			auto& v = position->operator[](new_index);
			iss >> v[0];
			iss >> v[1];
			iss >> v[2];
		}

		// reading tetrahedra
		for(uint32 i = 0u, end = this->nb_volumes(); i < end; ++i)
		{
			do
			{
				std::getline(ele_file,line);
			} while(line.empty());

			std::istringstream iss(line);
			std::array<uint32, 4u> ids;
			iss >> ids[0]; // index of the tetra is useless
			iss >> ids[0];
			iss >> ids[1];
			iss >> ids[2];
			iss >> ids[3];
			for (auto& id : ids)
				id = old_new_ids_map[id];

			this->add_tetra(*position, ids[0], ids[1], ids[2], ids[3], true);
		}
		return true;
	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_TETGEN_IO_CPP_))
extern template class CGOGN_IO_API TetgenVolumeImport<DefaultMapTraits, Eigen::Vector3d>;
extern template class CGOGN_IO_API TetgenVolumeImport<DefaultMapTraits, Eigen::Vector3f>;
extern template class CGOGN_IO_API TetgenVolumeImport<DefaultMapTraits, geometry::Vec_T<std::array<float64,3>>>;
extern template class CGOGN_IO_API TetgenVolumeImport<DefaultMapTraits, geometry::Vec_T<std::array<float32,3>>>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_TETGEN_IO_CPP_))

} // namespace io
} // namespace cgogn
#endif // CGOGN_IO_TETGEN_IO_H_
