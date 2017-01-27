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
#include <cgogn/io/surface_import.h>

namespace cgogn
{

namespace io
{

template <typename VEC3>
class LM6VolumeImport : public VolumeFileImport<VEC3>
{
	using Inherit = VolumeFileImport<VEC3>;
	using Self = LM6VolumeImport<VEC3>;
	template <typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

public:

	inline LM6VolumeImport() {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(LM6VolumeImport);

protected:

	virtual bool import_file_impl(const std::string& filename) override
	{
		int version = -1;
		int dimension = -1;
		int mesh_index = GmfOpenMesh(filename.c_str(), GmfRead, &version, &dimension);
		if (mesh_index == 0)
			return false;

		const uint32 number_of_vertices = uint32(GmfStatKwd(mesh_index, GmfVertices));
		const uint32 number_of_tetras = uint32(GmfStatKwd(mesh_index, GmfTetrahedra));
		const uint32 number_of_hexas = uint32(GmfStatKwd(mesh_index, GmfHexahedra));
		const uint32 number_of_prisms = uint32(GmfStatKwd(mesh_index, GmfPrisms));
		const uint32 number_of_pyramids = uint32(GmfStatKwd(mesh_index, GmfPyramids));

		const uint32 nb_volumes = number_of_tetras + number_of_hexas + number_of_prisms + number_of_pyramids;
		this->reserve(nb_volumes);

		if (number_of_vertices == 0u || nb_volumes == 0u)
		{
			cgogn_log_warning("LM6VolumeImport") << "Error while reading the file \"" << filename << "\".";
			GmfCloseMesh(mesh_index);
			this->clear();
			return false;
		}

		ChunkArray<VEC3>* position = this->position_attribute();
		int32 ref;

		GmfGotoKwd(mesh_index, GmfVertices);
		for (uint32 i = 0u ; i < number_of_vertices; ++i)
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
			for (uint32 i = 0; i < number_of_tetras; ++i)
			{
				(void) GmfGetLin(mesh_index, GmfTetrahedra, &ids[0],&ids[1], &ids[2], &ids[3], &ref);
				for (auto& id : ids)
					--id;
				this->add_tetra(ids[0],ids[1], ids[2], ids[3], false);
			}
		}


		if (number_of_hexas > 0)
		{
			GmfGotoKwd(mesh_index, GmfHexahedra);
			std::array<int, 8> ids;
			for (uint32 i = 0 ; i < number_of_hexas; ++i)
			{
				(void) GmfGetLin(mesh_index, GmfHexahedra, &ids[0],&ids[1], &ids[2], &ids[3], &ids[4], &ids[5], &ids[6], &ids[7], &ref);
				for (auto& id : ids)
					--id;
				this->add_hexa(ids[0],ids[1], ids[5], ids[4], ids[3],ids[2], ids[6], ids[7], false);
			}
		}


		if (number_of_prisms > 0)
		{
			GmfGotoKwd(mesh_index, GmfPrisms);
			std::array<int, 6> ids;
			for (uint32 i = 0; i < number_of_prisms; ++i)
			{
				(void) GmfGetLin(mesh_index, GmfPrisms, &ids[0],&ids[1], &ids[2], &ids[3], &ids[4], &ids[5], &ref);
				for (auto& id : ids)
					--id;
				this->add_triangular_prism(ids[0],ids[1], ids[2], ids[3], ids[4],ids[5], false);
			}
		}

		if (number_of_pyramids > 0)
		{
			GmfGotoKwd(mesh_index, GmfPyramids);
			std::array<int, 5> ids;
			for (uint32 i = 0; i < number_of_pyramids; ++i)
			{
				(void) GmfGetLin(mesh_index, GmfPyramids, &ids[0],&ids[1], &ids[2], &ids[3], &ids[4], &ref);
				for (auto& id : ids)
					--id;
				this->add_pyramid(ids[0],ids[1], ids[2], ids[3], ids[4], false);
			}
		}

		GmfCloseMesh(mesh_index);
		return true;
	}
};

template <typename VEC3>
class LM6SurfaceImport : public SurfaceFileImport<VEC3>
{
	using Inherit = SurfaceFileImport<VEC3>;
	using Self = LM6SurfaceImport<VEC3>;
	template <typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

public:

	inline LM6SurfaceImport() {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(LM6SurfaceImport);

protected:

	virtual bool import_file_impl(const std::string& filename) override
	{
		int version = -1;
		int dimension = -1;
		int mesh_index = GmfOpenMesh(filename.c_str(), GmfRead, &version, &dimension);
		if (mesh_index == 0)
			return false;

		const uint32 number_of_vertices = uint32(GmfStatKwd(mesh_index, GmfVertices));
		const uint32 number_of_triangles = uint32(GmfStatKwd(mesh_index, GmfTriangles));
		const uint32 number_of_quads = uint32(GmfStatKwd(mesh_index, GmfQuadrilaterals));

		const uint32 nb_faces = number_of_triangles + number_of_quads;
		this->reserve(nb_faces);

		if (number_of_vertices == 0u || nb_faces == 0u)
		{
			cgogn_log_warning("LM6SurfaceImport") << "Error while reading the file \"" << filename << "\".";
			GmfCloseMesh(mesh_index);
			this->clear();
			return false;
		}

		ChunkArray<VEC3>* position = this->position_attribute();
		int32 ref;

		GmfGotoKwd(mesh_index, GmfVertices);
		for (uint32 i = 0u ; i < number_of_vertices; ++i)
		{
			const uint32 idx = this->insert_line_vertex_container();
			std::array<float32, 3> v;
			(void) GmfGetLin(mesh_index, GmfVertices, &v[0],&v[1], &v[2], &ref);
			position->operator[](idx)[0] = v[0];
			position->operator[](idx)[1] = v[1];
			position->operator[](idx)[2] = v[2];
		}

		if (number_of_triangles > 0)
		{
			GmfGotoKwd(mesh_index, GmfTriangles);
			std::array<int, 3> ids;
			for (uint32 i = 0; i < number_of_triangles; ++i)
			{
				(void) GmfGetLin(mesh_index, GmfTriangles, &ids[0],&ids[1], &ids[2], &ref);
				for (auto& id : ids)
					--id;
				this->add_triangle(ids[0],ids[1], ids[2]);
			}
		}


		if (number_of_quads > 0)
		{
			GmfGotoKwd(mesh_index, GmfQuadrilaterals);
			std::array<int, 4> ids;
			for (uint32 i = 0; i < number_of_quads; ++i)
			{
				(void) GmfGetLin(mesh_index, GmfQuadrilaterals, &ids[0],&ids[1], &ids[2], &ids[3], &ref);
				for (auto& id : ids)
					--id;
				this->add_quad(ids[0],ids[1], ids[2], ids[3]);
			}
		}

		GmfCloseMesh(mesh_index);
		return true;
	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_LM6_IO_CPP_))
extern template class CGOGN_IO_API LM6SurfaceImport<Eigen::Vector3d>;
extern template class CGOGN_IO_API LM6SurfaceImport<Eigen::Vector3f>;
extern template class CGOGN_IO_API LM6VolumeImport<Eigen::Vector3d>;
extern template class CGOGN_IO_API LM6VolumeImport<Eigen::Vector3f>;
extern template class CGOGN_IO_API LM6VolumeImport<geometry::Vec_T<std::array<float64,3>>>;
extern template class CGOGN_IO_API LM6VolumeImport<geometry::Vec_T<std::array<float32,3>>>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_LM6_IO_CPP_))

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_LM6_IO_H_
