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

#ifndef CGOGN_IO_FORMATS_OBJ_H_
#define CGOGN_IO_FORMATS_OBJ_H_

#include <cgogn/geometry/types/eigen.h>
#include <cgogn/geometry/types/vec.h>
#include <cgogn/geometry/types/geometry_traits.h>

#include <cgogn/io/point_set_import.h>

#include <cgogn/io/polyline_import.h>
#include <cgogn/io/polyline_export.h>

#include <cgogn/io/surface_import.h>
#include <cgogn/io/surface_export.h>

#include <cgogn/io/graph_import.h>
#include <cgogn/io/graph_export.h>

#include <iomanip>

namespace cgogn
{

namespace io
{

template <typename MAP, typename VEC3>
class ObjPointSetImport : public PointSetFileImport<MAP>
{
public:

	using Self = ObjPointSetImport<MAP, VEC3>;
	using Inherit = PointSetFileImport<MAP>;
	using Scalar = typename geometry::vector_traits<VEC3>::Scalar;
	template <typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

	inline ObjPointSetImport(MAP& map) : Inherit(map) {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(ObjPointSetImport);
	virtual ~ObjPointSetImport() override {}

protected:

	virtual bool import_file_impl(const std::string& filename) override
	{
		std::ifstream fp(filename.c_str(), std::ios::in);

		ChunkArray<VEC3>* position = this->template add_vertex_attribute<VEC3>("position");
		ChunkArray<VEC3>* normal = nullptr;
		std::vector<VEC3> norm_buff;

		std::string line, tag;
		bool has_normals = false;

		do
		{
			fp >> tag;
			getline_safe(fp, line);
		} while (tag != std::string("vn") && (!fp.eof()));

		if (tag == "vn")
		{
			has_normals = true;
			norm_buff.reserve(1024);
			do
			{
				if (tag == std::string("vn"))
				{
					std::stringstream oss(line);

					float64 x, y, z;
					oss >> x;
					oss >> y;
					oss >> z;
					float64 n = std::sqrt(x*x+y*y+z*z);
					norm_buff.emplace_back(Scalar(x/n), Scalar(y/n), Scalar(z/n));
				}

				fp >> tag;
				getline_safe(fp, line);
			} while (!fp.eof());
		}

		fp.clear();
		fp.seekg(0, std::ios::beg);

		do
		{
			fp >> tag;
			getline_safe(fp, line);
		} while (tag != std::string("v"));

		// lecture des sommets
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

				uint32 vertex_id = this->insert_line_vertex_container();
				(*position)[vertex_id] = pos;

				++i;
			}

			fp >> tag;
			getline_safe(fp, line);
		} while (!fp.eof());


		this->nb_vertices_ = i;

		if (has_normals)
		{
			normal = this->template add_vertex_attribute<VEC3>("normal");
			for(uint32 j = 0 ; j < norm_buff.size() ; ++j)
			{
				(*normal)[j] = norm_buff[j];
			}
		}


		return true;
	}
};

template <typename MAP, typename VEC3>
class ObjPolylineImport : public PolylineFileImport<MAP>
{

public:
	using Self = ObjPolylineImport<MAP, VEC3>;
	using Inherit = PolylineFileImport<MAP>;
	using Scalar = typename geometry::vector_traits<VEC3>::Scalar;
	template <typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

	inline ObjPolylineImport(MAP& map) : Inherit(map) {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(ObjPolylineImport);
	virtual ~ObjPolylineImport() override {}

protected:
	virtual bool import_file_impl(const std::string& filename) override
	{
		std::ifstream fp(filename.c_str(), std::ios::in);

		ChunkArray<VEC3>* position = this->template add_vertex_attribute<VEC3>("position");
		std::string line, tag;

		do
		{
			fp >> tag;
			getline_safe(fp, line);
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

				uint32 vertex_id = this->insert_line_vertex_container();
				(*position)[vertex_id] = pos;

				vertices_id.push_back(vertex_id);
				i++;
			}

			fp >> tag;
			getline_safe(fp, line);
		} while (!fp.eof());

		fp.clear();
		fp.seekg(0, std::ios::beg);

		this->nb_vertices_ = vertices_id.size();

		do
		{
			fp >> tag;
			getline_safe(fp, line);
		} while (tag != std::string("l"));

		do
		{
			if (tag == std::string("l"))
			{
				std::stringstream oss(line);

				uint32  a, b;
				oss >> a;
				oss >> b;

				uint32 min = b - 1;
				uint32 max = a - 1;

				if(min > max)
					std::swap(min, max);

				this->edges_vertex_indices_.push_back(vertices_id[min]);
				this->edges_vertex_indices_.push_back(vertices_id[max]);

				std::cout << a << " - " << b << std::endl;
			}

			fp >> tag;
			getline_safe(fp, line);
		} while (!fp.eof());

		return true;
	}
};

template <typename MAP>
class ObjPolylineExport : public PolylineExport<MAP>
{
public:
	using Inherit = PolylineExport<MAP>;
	using Self = ObjPolylineExport<MAP>;

	using Map = typename Inherit::Map;

	using Vertex = typename Inherit::Vertex;
	using Edge = typename Inherit::Edge;

	using ChunkArrayGen = typename Inherit::ChunkArrayGen;
	template <typename T>
	using VertexAttribute = typename Inherit::template VertexAttribute<T>;
	using ChunkArrayContainer = typename Inherit::ChunkArrayContainer;

protected:
	virtual void export_file_impl(const Map& map, std::ofstream& output, const ExportOptions& ) override
	{
		// set precision for float output
		output << std::setprecision(12);

		map.foreach_cell([&] (Vertex v)
		{
			output << "v ";
			this->position_attribute(Vertex::ORBIT)->export_element(map.embedding(v), output, false, false);
			output << std::endl;
		}, *(this->cell_cache_));

		std::cout << "obj export" << std::endl;
		map.foreach_cell([&] (Edge e)
		{
			std::cout << e.dart;

			std::pair<Vertex, Vertex> vs = map.vertices(e);

			//todo erase this trick
			if(this->vindices_[vs.first] != this->vindices_[vs.second])
				output << "l " << (this->vindices_[vs.first]+1u) << " " << (this->vindices_[vs.second]+1u) << std::endl;
		}, *(this->cell_cache_));
	}
};

template <typename MAP, typename VEC3>
class ObjSurfaceImport : public SurfaceFileImport<MAP>
{
public:

	using Self = ObjSurfaceImport<MAP, VEC3>;
	using Inherit = SurfaceFileImport<MAP>;
	using Scalar = typename geometry::vector_traits<VEC3>::Scalar;
	template <typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

	inline ObjSurfaceImport(MAP& map) : Inherit(map) {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(ObjSurfaceImport);
	virtual ~ObjSurfaceImport() override {}

protected:

	virtual bool import_file_impl(const std::string& filename) override
	{
		std::ifstream fp(filename.c_str(), std::ios::in);

		ChunkArray<VEC3>* position = this->template add_vertex_attribute<VEC3>("position");
		ChunkArray<VEC3>* normal = nullptr;
		std::vector<VEC3> norm_buff;

		std::string line, tag;
		bool has_normals = false;

		do
		{
			fp >> tag;
			getline_safe(fp, line);
		} while (tag != std::string("vn") && (!fp.eof()));

		if (tag == "vn")
		{
			has_normals = true;
			norm_buff.reserve(1024);
			do
			{
				if (tag == std::string("vn"))
				{
					std::stringstream oss(line);

					float64 x, y, z;
					oss >> x;
					oss >> y;
					oss >> z;
					float64 n = std::sqrt(x*x+y*y+z*z);
					norm_buff.emplace_back(Scalar(x/n), Scalar(y/n), Scalar(z/n));
				}

				fp >> tag;
				getline_safe(fp, line);
			} while (!fp.eof());
		}

		fp.clear();
		fp.seekg(0, std::ios::beg);

		do
		{
			fp >> tag;
			getline_safe(fp, line);
		} while (tag != std::string("v"));

		// lecture des sommets
		std::vector<uint32> vertices_id;
		vertices_id.reserve(102400);
		uint32 max_id = 0;

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

				uint32 vertex_id = this->insert_line_vertex_container();
				(*position)[vertex_id] = pos;

				vertices_id.push_back(vertex_id);

				if (vertex_id > max_id)
					max_id = vertex_id;
				i++;
			}

			fp >> tag;
			getline_safe(fp, line);
		} while (!fp.eof());

		if (has_normals)
		{
			normal = this->template add_vertex_attribute<VEC3>("normal");
			normal->set_all_values(VEC3(0,0,0));
		}

		fp.clear();
		fp.seekg(0, std::ios::beg);

		do
		{
			fp >> tag;
			getline_safe(fp, line);
		} while (tag != std::string("f"));

		this->faces_nb_edges_.reserve(vertices_id.size() * 2);
		this->faces_vertex_indices_.reserve(vertices_id.size() * 8);

		std::vector<uint32> table;
		table.reserve(64);
		std::vector<uint32> tableN;
		table.reserve(64);
		do
		{
			if (tag == std::string("f")) // lecture d'une face
			{
				std::stringstream oss(line);

				table.clear();
				tableN.clear();
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
					if (has_normals)
					{
						// jump over /?/ (v/vt/vn)
						++ind;
						while ((ind < str.length()) && (str[ind] != '/'))
							++ind;

						if (ind < str.length())
						{
							uint32 index;
							std::stringstream iss(str.substr(ind+1, str.length()));
							iss >> index;
							tableN.push_back(index);
						}
						else
							tableN.push_back(table.back());
					}
				}

				uint32 n = uint32(table.size());
				this->faces_nb_edges_.push_back(n);
				for (uint32 j = 0; j < n; ++j)
				{
					uint32 index = table[j] - 1; // indices start at 1
					this->faces_vertex_indices_.push_back(vertices_id[index]);
					if (has_normals)
					{
						auto k = vertices_id[index];
						(*normal)[k] += norm_buff[tableN[j] - 1];
					}
				}
			}
			fp >> tag;
			getline_safe(fp, line);
		} while (!fp.eof());

		// normalize
		if (has_normals)
		{
			for(auto j: vertices_id)
				(*normal)[j].normalize();
		}
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
			this->position_attribute(Vertex::ORBIT)->export_element(map.embedding(v), output, false, false);
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
				output << " " << (this->vindices_[v]+1u);
			});
			output << std::endl;
		}, *(this->cell_cache_));
	}
};

///
///
///
template <typename MAP, typename VEC3>
class ObjGraphImport : public GraphFileImport<MAP>
{
public:

	using Self = ObjGraphImport<MAP, VEC3>;
	using Inherit = GraphFileImport<MAP>;
    using Scalar = typename geometry::vector_traits<VEC3>::Scalar;
    template <typename T>
    using ChunkArray = typename Inherit::template ChunkArray<T>;

	inline ObjGraphImport(MAP& map) : Inherit(map) {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(ObjGraphImport);
	virtual ~ObjGraphImport() override {}

protected:

    virtual bool import_file_impl(const std::string& filename) override
    {
        std::ifstream fp(filename.c_str(), std::ios::in);

		ChunkArray<VEC3>* position = this->template add_vertex_attribute<VEC3>("position");
		ChunkArray<Scalar>* radius = this->template add_vertex_attribute<Scalar>("radius");

		std::string line, tag;

		do
		{
			fp >> tag;
			getline_safe(fp, line);
		} while (tag != std::string("v"));

		// lecture des sommets
		std::vector<uint32> vertices_id;
		vertices_id.reserve(65536);

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

				uint32 vertex_id = this->insert_line_vertex_container();
				(*position)[vertex_id] = pos;
				(*radius)[vertex_id] = Scalar(0.001);

				vertices_id.push_back(vertex_id);
				i++;
			}

			fp >> tag;
			getline_safe(fp, line);
		} while (!fp.eof());

		fp.clear();
		fp.seekg(0, std::ios::beg);

		this->reserve(vertices_id.size() * 2);

		do
		{
			fp >> tag;
			getline_safe(fp, line);
		} while (tag != std::string("l"));

		do
		{
			if (tag == std::string("l"))
			{
				std::stringstream oss(line);

				uint32 a, b;
				oss >> a;
				oss >> b;

				this->add_edge(vertices_id[a], vertices_id[b]);
			}

			fp >> tag;
			getline_safe(fp, line);
		} while (!fp.eof());

		return true;
    }
};

template <typename MAP>
class ObjGraphExport : public GraphExport<MAP>
{
public:

    using Inherit = GraphExport<MAP>;
	using Self = ObjGraphExport<MAP>;
    using Map = typename Inherit::Map;
    using Vertex = typename Inherit::Vertex;
    using Edge = typename Inherit::Edge;
    using ChunkArrayGen = typename Inherit::ChunkArrayGen;
    template <typename T>
    using VertexAttribute = typename Inherit::template VertexAttribute<T>;
    using ChunkArrayContainer = typename Inherit::ChunkArrayContainer;

protected:

    virtual void export_file_impl(const Map& map, std::ofstream& output, const ExportOptions& ) override
    {
        // set precision for float output
        output << std::setprecision(12);

		map.foreach_cell([&] (Vertex v)
		{
			output << "v ";
			this->position_attribute(Vertex::ORBIT)->export_element(map.embedding(v), output, false, false);
			output << std::endl;
		}, *(this->cell_cache_));

		map.foreach_cell([&] (Edge e)
		{
			std::pair<Vertex, Vertex> vs = map.vertices(e);
			output << "l " << (this->vindices_[vs.first]+1u) << " " << (this->vindices_[vs.second]+1u) << std::endl;
		}, *(this->cell_cache_));
    }
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_EXTERNAL_TEMPLATES_CPP_))
extern template class CGOGN_IO_EXPORT ObjPointSetImport<CMap0, Eigen::Vector3d>;
extern template class CGOGN_IO_EXPORT ObjPointSetImport<CMap0, Eigen::Vector3f>;

extern template class CGOGN_IO_EXPORT ObjSurfaceImport<CMap2, Eigen::Vector3d>;
extern template class CGOGN_IO_EXPORT ObjSurfaceImport<CMap2, Eigen::Vector3f>;
extern template class CGOGN_IO_EXPORT ObjSurfaceImport<CMap2, geometry::Vec_T<std::array<float64, 3>>>;
extern template class CGOGN_IO_EXPORT ObjSurfaceImport<CMap2, geometry::Vec_T<std::array<float32, 3>>>;

extern template class CGOGN_IO_EXPORT ObjGraphImport<Eigen::Vector3d>;
extern template class CGOGN_IO_EXPORT ObjGraphImport<Eigen::Vector3f>;
extern template class CGOGN_IO_EXPORT ObjGraphImport<geometry::Vec_T<std::array<float64,3>>>;
extern template class CGOGN_IO_EXPORT ObjGraphImport<geometry::Vec_T<std::array<float32,3>>>;

extern template class CGOGN_IO_EXPORT ObjSurfaceExport<CMap2>;

extern template class CGOGN_IO_EXPORT ObjGraphExport<UndirectedGraph>;

#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_EXTERNAL_TEMPLATES_CPP_))

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_FORMATS_OBJ_H_
