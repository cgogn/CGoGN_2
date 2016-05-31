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

#ifndef CGOGN_IO_LM6_IO_H_
#define CGOGN_IO_LM6_IO_H_

#include <libmesh6.h>
#include <cgogn/io/volume_import.h>

namespace cgogn
{

namespace io
{

template <typename MAP_TRAITS, typename VEC3>
class LM6VolumeImport : public VolumeImport<MAP_TRAITS>
{
	using Inherit = VolumeImport<MAP_TRAITS>;
	using Self = LM6VolumeImport<MAP_TRAITS,VEC3>;
	template <typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

	// MeshImportGen interface
public:
	inline LM6VolumeImport() {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(LM6VolumeImport);
	virtual void clear() override
	{
		Inherit::clear();
	}

protected:
	virtual bool import_file_impl(const std::string& filename) override
	{
		int version = -1;
		int dimension = -1;
		int mesh_index = GmfOpenMesh(filename.c_str(), GmfRead, &version, &dimension);
		if (mesh_index == 0)
			return false;

		const int number_of_vertices = GmfStatKwd(mesh_index, GmfVertices);
		const int number_of_tetras = GmfStatKwd(mesh_index, GmfTetrahedra);
		const int number_of_hexas = GmfStatKwd(mesh_index, GmfHexahedra);
		const int number_of_prisms = GmfStatKwd(mesh_index, GmfPrisms);
		const int number_of_pyramids = GmfStatKwd(mesh_index, GmfPyramids);


		this->set_nb_vertices(number_of_vertices);
		this->set_nb_volumes(number_of_tetras + number_of_hexas + number_of_prisms + number_of_pyramids);

		if (this->nb_vertices() == 0 || this->nb_volumes()== 0u)
		{
			cgogn_log_warning("LM6VolumeImport") << "Error while reading the file \"" << filename << "\".";
			GmfCloseMesh(mesh_index);
			clear();
			return false;
		}

		ChunkArray<VEC3>* position = this->template position_attribute<VEC3>();
		int32 ref;

		GmfGotoKwd(mesh_index, GmfVertices);
		for (uint32 i = 0u, end = this->nb_vertices() ; i < end; ++i)
		{
			uint32 idx = this->insert_line_vertex_container();
			std::array<float32, 3> v;
			(void) GmfGetLin(mesh_index, GmfVertices, &v[0],&v[1], &v[2], &ref);
			position->operator[](idx)[0] = v[0];
			position->operator[](idx)[1] = v[1];
			position->operator[](idx)[2] = v[2];
		}

		if (number_of_tetras > 0)
		{
			GmfGotoKwd(mesh_index, GmfTetrahedra);
			std::array<int, 4> ids;
			for (int i = 0 ; i < number_of_tetras; ++i)
			{
				(void) GmfGetLin(mesh_index, GmfTetrahedra, &ids[0],&ids[1], &ids[2], &ids[3], &ref);
				for (auto& id : ids)
					--id;
				this->add_tetra(*position, ids[0],ids[1], ids[2], ids[3], false);
			}
		}


		if (number_of_hexas > 0)
		{
			GmfGotoKwd(mesh_index, GmfHexahedra);
			std::array<int, 8> ids;
			for (int i = 0 ; i < number_of_hexas; ++i)
			{
				(void) GmfGetLin(mesh_index, GmfHexahedra, &ids[0],&ids[1], &ids[2], &ids[3], &ids[4], &ids[5], &ids[6], &ids[7], &ref);
				for (auto& id : ids)
					--id;
				this->add_hexa(*position, ids[0],ids[1], ids[5], ids[4], ids[3],ids[2], ids[6], ids[7], false);
			}
		}


		if (number_of_prisms > 0)
		{
			GmfGotoKwd(mesh_index, GmfPrisms);
			std::array<int, 6> ids;
			for (int i = 0 ; i < number_of_prisms; ++i)
			{
				(void) GmfGetLin(mesh_index, GmfPrisms, &ids[0],&ids[1], &ids[2], &ids[3], &ids[4], &ids[5], &ref);
				for (auto& id : ids)
					--id;
				this->add_triangular_prism(*position, ids[0],ids[1], ids[2], ids[3], ids[4],ids[5], false);
			}
		}

		if (number_of_pyramids > 0)
		{
			GmfGotoKwd(mesh_index, GmfPyramids);
			std::array<int, 5> ids;
			for (int i = 0 ; i < number_of_pyramids; ++i)
			{
				(void) GmfGetLin(mesh_index, GmfPyramids, &ids[0],&ids[1], &ids[2], &ids[3], &ids[4], &ref);
				for (auto& id : ids)
					--id;
				this->add_pyramid(*position, ids[0],ids[1], ids[2], ids[3], ids[4], false);
			}
		}

		GmfCloseMesh(mesh_index);
		return true;
	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_LM6_IO_CPP_))
extern template class CGOGN_IO_API LM6VolumeImport<DefaultMapTraits, Eigen::Vector3d>;
extern template class CGOGN_IO_API LM6VolumeImport<DefaultMapTraits, Eigen::Vector3f>;
extern template class CGOGN_IO_API LM6VolumeImport<DefaultMapTraits, geometry::Vec_T<std::array<float64,3>>>;
extern template class CGOGN_IO_API LM6VolumeImport<DefaultMapTraits, geometry::Vec_T<std::array<float32,3>>>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_LM6_IO_CPP_))

} // namespace io
} // namespace cgogn


#endif // CGOGN_IO_LM6_IO_H_
