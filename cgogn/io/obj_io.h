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

#ifndef CGOGN_IO_OBJ_IO_H_
#define CGOGN_IO_OBJ_IO_H_

#include <cgogn/geometry/types/eigen.h>
#include <cgogn/geometry/types/vec.h>
#include <cgogn/geometry/types/geometry_traits.h>

#include <cgogn/io/surface_import.h>
#include <cgogn/io/surface_export.h>

#include <iomanip>

namespace cgogn
{

namespace io
{

template <typename VEC3>
class ObjSurfaceImport : public SurfaceFileImport<VEC3>
{
public:

	using Self = ObjSurfaceImport<VEC3>;
	using Inherit = SurfaceFileImport<VEC3>;
	using Scalar = typename geometry::vector_traits<VEC3>::Scalar;
	template <typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

	inline ObjSurfaceImport() {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(ObjSurfaceImport);
	virtual ~ObjSurfaceImport() override
	{}

protected:

	virtual bool import_file_impl(const std::string& filename) override
	{
		std::ifstream fp(filename.c_str(), std::ios::in);
		ChunkArray<VEC3>* position = this->add_position_attribute();

		std::string line, tag;

		do
		{
			fp >> tag;
			std::getline(fp, line);
		} while (tag != std::string("v"));

		// lecture des sommets
		std::vector<uint32> vertices_id;
		vertices_id.reserve(102400);

		uint32 i = 0;
		do
		{
			if (tag == std::string("v"))
			{
				std::stringstream oss(line);

				float64 x, y, z;
				oss >> x;
				oss >> y;
				oss >> z;

				VEC3 pos{Scalar(x), Scalar(y), Scalar(z)};

				uint32 vertex_id = this->vertex_attributes_.template insert_lines<1>();
				(*position)[vertex_id] = pos;

				vertices_id.push_back(vertex_id);
				i++;
			}

			fp >> tag;
			std::getline(fp, line);
		} while (!fp.eof());

		fp.clear();
		fp.seekg(0, std::ios::beg);

		do
		{
			fp >> tag;
			std::getline(fp, line);
		} while (tag != std::string("vn") && (!fp.eof()));

		if (tag == "vn")
		{
			uint32 counter{0u};
			ChunkArray<VEC3>* normal = this->vertex_attributes_.template add_chunk_array<VEC3>("normal");
			do
			{
				if (tag == std::string("vn"))
				{
					std::stringstream oss(line);

					float64 x, y, z;
					oss >> x;
					oss >> y;
					oss >> z;

					VEC3 norm{Scalar(x), Scalar(y), Scalar(z)};
					(*normal)[vertices_id[counter++]] = norm;
				}

				fp >> tag;
				std::getline(fp, line);
			} while (!fp.eof());
		}

		fp.clear();
		fp.seekg(0, std::ios::beg);

		do
		{
			fp >> tag;
			std::getline(fp, line);
		} while (tag != std::string("f"));

		this->faces_nb_edges_.reserve(vertices_id.size() * 2);
		this->faces_vertex_indices_.reserve(vertices_id.size() * 8);

		std::vector<uint32> table;
		table.reserve(64);
		do
		{
			if (tag == std::string("f")) // lecture d'une face
			{
				std::stringstream oss(line);

				table.clear();
				while (!oss.eof())  // lecture de tous les indices
				{
					std::string str;
					oss >> str;

					uint32 ind = 0;

					while ((ind < str.length()) && (str[ind] != '/'))
						ind++;

					if (ind > 0)
					{
						uint32 index;
						std::stringstream iss(str.substr(0, ind));
						iss >> index;
						table.push_back(index);
					}
				}

				uint32 n = uint32(table.size());
				this->faces_nb_edges_.push_back(n);
				for (uint32 j = 0; j < n; ++j)
				{
					uint32 index = table[j] - 1; // indices start at 1
					this->faces_vertex_indices_.push_back(vertices_id[index]);
				}
			}
			fp >> tag;
			std::getline(fp, line);
		} while (!fp.eof());

		return true;
	}
};

template <typename MAP>
class ObjSurfaceExport : public SurfaceExport<MAP>
{
public:

	using Inherit = SurfaceExport<MAP>;
	using Self = ObjSurfaceExport<MAP>;
	using Map = typename Inherit::Map;
	using Vertex = typename Inherit::Vertex;
	using Face = typename Inherit::Face;
	using ChunkArrayGen = typename Inherit::ChunkArrayGen;
	template<typename T>
	using VertexAttribute = typename Inherit::template VertexAttribute<T>;
	using ChunkArrayContainer = typename Inherit::ChunkArrayContainer;

protected:

	virtual void export_file_impl(const Map& map, std::ofstream& output, const ExportOptions& /*option*/) override
	{
		const ChunkArrayGen* normal_attribute(nullptr);

		for (const ChunkArrayGen* vatt: this->vertex_attributes())
			if(to_lower(vatt->name()) == "normal" || to_lower(vatt->name()) == "normals")
				normal_attribute = vatt;

		// set precision for float output
		output << std::setprecision(12);

		// two passes of traversal to avoid huge buffer (with same performance);
		output << "# vertices" << std::endl;

		map.foreach_cell([&] (Vertex v)
		{
			output << "v ";
			this->position_attribute_->export_element(map.embedding(v), output, false, false);
			output << std::endl;
		}, *(this->cell_cache_));

		if (normal_attribute != nullptr)
		{
			output << std::endl << "# normals" << std::endl;

			map.foreach_cell([&] (Vertex v)
			{
				output << "vn ";
				normal_attribute->export_element(map.embedding(v), output, false, false);
				output << std::endl;
			}, *(this->cell_cache_));
		}

		output << std::endl << "# faces" << std::endl;
		// second pass to save primitives
		std::vector<uint32> prim;
		prim.reserve(20);
		map.foreach_cell([&] (Face f)
		{
			output << "f";
			map.foreach_incident_vertex(f, [&] (Vertex v)
			{
				output << " " << (this->indices_[v]+1u);
			});
			output << std::endl;
		}, *(this->cell_cache_));
	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_OBJ_IO_CPP_))
extern template class CGOGN_IO_API ObjSurfaceImport<Eigen::Vector3d>;
extern template class CGOGN_IO_API ObjSurfaceImport<Eigen::Vector3f>;
extern template class CGOGN_IO_API ObjSurfaceImport<geometry::Vec_T<std::array<float64,3>>>;
extern template class CGOGN_IO_API ObjSurfaceImport<geometry::Vec_T<std::array<float32,3>>>;

extern template class CGOGN_IO_API ObjSurfaceExport<CMap2>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_OBJ_IO_CPP_))

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_OBJ_IO_H_
