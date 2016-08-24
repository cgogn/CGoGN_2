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

#ifndef CGOGN_IO_OFF_IO_H_
#define CGOGN_IO_OFF_IO_H_

#include <cgogn/core/cmap/cmap3.h>

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

template <typename MAP_TRAITS, typename VEC3>
class OffSurfaceImport : public SurfaceFileImport<MAP_TRAITS>
{
public:

	using Self = OffSurfaceImport<MAP_TRAITS, VEC3>;
	using Inherit = SurfaceFileImport<MAP_TRAITS>;
	using Scalar = typename geometry::vector_traits<VEC3>::Scalar;
	template <typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

	inline OffSurfaceImport() {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(OffSurfaceImport);
	virtual ~OffSurfaceImport() override {}

protected:

	virtual bool import_file_impl(const std::string& filename) override
	{
		std::ifstream fp(filename.c_str(), std::ios::in);

		std::string line;
		line.reserve(512);

		// read OFF header
		std::getline(fp, line);
		if (line.rfind("OFF") == std::string::npos)
		{
			cgogn_log_error("OffSurfaceImport::import_file_impl") << "File \"" << filename << "\" is not a valid off file.";
			return false;
		}

		// check if binary file
		if (line.rfind("BINARY") != std::string::npos)
			return this->import_off_bin(fp);

		// read number of vertices, edges, faces
		const uint32 nb_vertices = this->read_uint(fp,line);
		const uint32 nb_faces = this->read_uint(fp,line);
		/*const uint32 nb_edges_ =*/ this->read_uint(fp,line);
		this->reserve(nb_faces);


		ChunkArray<VEC3>* position = this->vertex_attributes_.template add_chunk_array<VEC3>("position");

		// read vertices position
		std::vector<uint32> vertices_id;
		vertices_id.reserve(nb_vertices);

		for (uint32 i = 0; i < nb_vertices; ++i)
		{

			float64 x = this->read_double(fp,line);
			float64 y = this->read_double(fp,line);
			float64 z = this->read_double(fp,line);

			VEC3 pos{Scalar(x), Scalar(y), Scalar(z)};

			uint32 vertex_id = this->vertex_attributes_.template insert_lines<1>();
			(*position)[vertex_id] = pos;

			vertices_id.push_back(vertex_id);
		}

		// read faces (vertex indices)
		for (uint32 i = 0u; i < nb_faces ; ++i)
		{
			uint32 n = this->read_uint(fp,line);
			this->faces_nb_edges_.push_back(n);
			for (uint32 j = 0; j < n; ++j)
			{
				uint32 index = this->read_uint(fp,line);
				this->faces_vertex_indices_.push_back(vertices_id[index]);
			}
		}

		return true;
	}

	inline bool import_off_bin(std::istream& fp)
	{
		char buffer1[12];
		fp.read(buffer1,12);

		const uint32 nb_vertices = swap_endianness_native_big(*(reinterpret_cast<uint32*>(buffer1)));
		const uint32 nb_faces = swap_endianness_native_big(*(reinterpret_cast<uint32*>(buffer1+4)));

		ChunkArray<VEC3>* position = this->vertex_attributes_.template add_chunk_array<VEC3>("position");

		const uint32 BUFFER_SZ = 1024 * 1024;
		std::vector<float32> buff_pos(3*BUFFER_SZ);
		std::vector<uint32> vertices_id;
		vertices_id.reserve(nb_vertices);

		{
			uint32 j = BUFFER_SZ;
			for (uint32 i = 0u; i < nb_vertices; ++i, ++j)
			{
				if (j == BUFFER_SZ)
				{
					j = 0u;
					// read from file into buffer
					if (i + BUFFER_SZ < nb_vertices)
						fp.read(reinterpret_cast<char*>(&buff_pos[0]), buff_pos.size() * sizeof(float32));
					else
						fp.read(reinterpret_cast<char*>(&buff_pos[0]), 3u * sizeof(float32)*(nb_vertices - i));

					//endian
					for (auto& p : buff_pos)
						p = swap_endianness_native_big(p);
				}

				VEC3 pos{ buff_pos[3u * j], buff_pos[3u * j + 1u], buff_pos[3u * j + 2u] };

				uint32 vertex_id = this->vertex_attributes_.template insert_lines<1>();
				(*position)[vertex_id] = pos;

				vertices_id.push_back(vertex_id);
			}
		}

		// read faces (vertex indices)

		std::vector<uint32> buff_ind(BUFFER_SZ);
		this->reserve(nb_faces);

		uint32* ptr = &buff_ind[0];
		uint32 nb_read = BUFFER_SZ;
		for (uint32 i = 0u; i < nb_faces; ++i)
		{
			if (nb_read == BUFFER_SZ)
			{
				fp.read(reinterpret_cast<char*>(&buff_ind[0]),buff_ind.size()*sizeof(uint32));
				ptr = &buff_ind[0];
				for (uint32 k=0; k< BUFFER_SZ;++k)
				{
					*ptr = swap_endianness_native_big(*ptr);
					++ptr;
				}
				ptr = &buff_ind[0];
				nb_read =0;
			}

			uint32 n = *ptr++;
			nb_read++;

			this->faces_nb_edges_.push_back(n);
			for (uint32 j = 0u; j < n; ++j)
			{
				if (nb_read == BUFFER_SZ)
				{
					fp.read(reinterpret_cast<char*>(&buff_ind[0]),buff_ind.size()*sizeof(uint32));
					ptr = &buff_ind[0];
					for (uint32 k=0u; k< BUFFER_SZ;++k)
					{
						*ptr = swap_endianness_native_big(*ptr);
						++ptr;
					}
					ptr = &buff_ind[0];
					nb_read=0u;
				}
				uint32 index = *ptr++;
				nb_read++;
				this->faces_vertex_indices_.push_back(vertices_id[index]);
			}
		}

		return true;
	}

private:

	static inline float64 read_double(std::istream& fp, std::string& line)
	{
		fp >> line;
		while (line[0]=='#')
		{
			fp.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			fp >> line;
		}
		return std::stod(line);
	}

	static inline uint32 read_uint(std::istream& fp, std::string& line)
	{
		fp >> line;
		while (line[0]=='#')
		{
			fp.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			fp >> line;
		}
		return uint32((std::stoul(line)));
	}
};

template <typename MAP>
class OffSurfaceExport : public SurfaceExport<MAP>
{
public:

	using Inherit = SurfaceExport<MAP>;
	using Self = OffSurfaceExport<MAP>;
	using Map = typename Inherit::Map;
	using Vertex = typename Inherit::Vertex;
	using Face = typename Inherit::Face;
	using ChunkArrayGen = typename Inherit::ChunkArrayGen;
	template<typename T>
	using VertexAttribute = typename Inherit::template VertexAttribute<T>;

protected:

	virtual void export_file_impl(const Map& map, std::ofstream& output, const ExportOptions& option) override
	{
		if (option.binary_)
			this->export_binary(map,output,option);
		else
			this->export_ascii(map,output,option);
	}
private:
	void export_ascii(const Map& map, std::ofstream& output, const ExportOptions& /*option*/)
	{
		output << "OFF" << std::endl;
		output << map.template nb_cells<Vertex::ORBIT>() << " " << map.template nb_cells<Face::ORBIT>() << " 0" << std::endl;

		// set precision for real output
		output << std::setprecision(12);

		// first pass to save positions
		map.foreach_cell([&] (Vertex v)
		{
			this->position_attribute_->export_element(map.embedding(v), output, false, false);
			output << std::endl;
		}, *(this->cell_cache_));

		const auto& ids = this->indices_;
		// second pass to save primitives
		std::vector<uint32> prim;
		prim.reserve(20);
		map.foreach_cell([&] (Face f)
		{
			uint32 valence{0u};
			prim.clear();

			map.foreach_incident_vertex(f, [&] (Vertex v)
			{
				prim.push_back(ids[v]);
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
		output << "OFF BINARY"<< std::endl;

		uint32 nb_cells[3];
		nb_cells[0] = swap_endianness_native_big(map.template nb_cells<Vertex::ORBIT>());
		nb_cells[1] = swap_endianness_native_big(map.template nb_cells<Face::ORBIT>());
		nb_cells[2] = 0;

		output.write(reinterpret_cast<char*>(nb_cells),3*sizeof(uint32));

		// two pass of traversal to avoid huge buffer (with same performance);

		// first pass to save positions & store contiguous indices
		static const uint32 BUFFER_SZ = 1024 * 1024;

		std::vector<float32> buffer_pos;
		buffer_pos.reserve(BUFFER_SZ + 3);

		map.foreach_cell([&] (Vertex v)
		{
			this->position_attribute_->export_element(map.embedding(v), output, true, false,4ul);
		}, *(this->cell_cache_));


//		// second pass to save primitives
		std::vector<uint32> buffer_prims;
		buffer_prims.reserve(BUFFER_SZ + 128);// + 128 to avoid re-allocations

		std::vector<uint32> prim;
		prim.reserve(20);

		const auto& ids = this->indices_;

		map.foreach_cell([&] (Face f)
		{
			uint32 valence = 0;
			prim.clear();

			map.foreach_incident_vertex(f, [&] (Vertex v)
			{
				prim.push_back(ids[v]);
				++valence;
			});

			buffer_prims.push_back(swap_endianness_native_big(valence));
			for(uint32 i: prim)
				buffer_prims.push_back(swap_endianness_native_big(i));

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
			buffer_prims.shrink_to_fit();
		}

	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_OFF_IO_CPP_))
extern template class CGOGN_IO_API OffSurfaceImport<DefaultMapTraits, Eigen::Vector3d>;
extern template class CGOGN_IO_API OffSurfaceImport<DefaultMapTraits, Eigen::Vector3f>;
extern template class CGOGN_IO_API OffSurfaceImport<DefaultMapTraits, geometry::Vec_T<std::array<float64,3>>>;
extern template class CGOGN_IO_API OffSurfaceImport<DefaultMapTraits, geometry::Vec_T<std::array<float32,3>>>;
extern template class CGOGN_IO_API OffSurfaceExport<CMap2<DefaultMapTraits>>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_OFF_IO_CPP_))

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_OFF_IO_H_
