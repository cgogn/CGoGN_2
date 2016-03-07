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

#ifndef IO_VOLUME_IMPORT_H_
#define IO_VOLUME_IMPORT_H_

#include <istream>

#include <core/utils/string.h>
#include <core/container/chunk_array_container.h>
#include <core/cmap/cmap3_builder.h>

#include <geometry/functions/orientation.h>

#include <io/c_locale.h>
#include <io/dll.h>

#include <tinyxml2.h>

namespace cgogn
{

namespace io
{

enum VolumeFileType
{
	VolumeFileType_UNKNOWN = 0,
};

template <typename MAP_TRAITS>
class VolumeImport
{
public:

	using Self = VolumeImport<MAP_TRAITS>;
	using Map = CMap3<MAP_TRAITS>;
	using Vertex = typename Map::Vertex;

	static const unsigned int CHUNK_SIZE = MAP_TRAITS::CHUNK_SIZE;

	template <typename T>
	using ChunkArray = ChunkArray<CHUNK_SIZE, T>;
	using ChunkArrayContainer = cgogn::ChunkArrayContainer<CHUNK_SIZE, unsigned int>;

	template <typename T, Orbit ORBIT>
	using AttributeHandler = AttributeHandler<MAP_TRAITS, T, ORBIT>;
	template <typename T>

	using VertexAttributeHandler = typename Map::template VertexAttributeHandler<T>;
	using MapBuilder = cgogn::CMap3Builder_T<typename Map::MapTraits>;
	unsigned int nb_vertices_;
	unsigned int nb_edges_;
	unsigned int nb_faces_;
	unsigned int nb_volumes_;

	std::vector<unsigned int> volumes_nb_vertices_;
	std::vector<unsigned int> volumes_vertex_indices_;

	ChunkArrayContainer vertex_attributes_;

	VolumeImport() :
		nb_vertices_(0u)
	  ,nb_edges_(0u)
	  ,nb_faces_(0u)
	  ,volumes_nb_vertices_()
	  ,volumes_vertex_indices_()
	{}

	~VolumeImport()
	{}

	VolumeImport(const Self&) = delete;
	VolumeImport(Self&&) = delete;
	Self& operator=(const Self&) = delete;
	Self& operator=(Self&&) = delete;

	void clear()
	{
		nb_vertices_ = 0;
		nb_edges_ = 0;
		nb_faces_ = 0;
		volumes_nb_vertices_.clear();
		volumes_vertex_indices_.clear();
		vertex_attributes_.remove_attributes();
	}

	template <typename VEC3>
	bool import_file(const std::string& filename)
	{
		Scoped_C_Locale loc;

		const std::string& extension = to_lower(get_extension(filename));
		if (extension.empty())
			return false;

		std::ifstream fp(filename.c_str(), std::ios::in);
		if (!fp.good())
		{
			std::cerr << "Unable to open file " << filename << std::endl;
			return false;
		}

		this->clear();
		bool res = false;

		if (extension == "vtu" || extension == "vtk")
		{
			res = this->import_VTU<VEC3>(filename);
		}
		if (!res)
		{
			this->clear();
			std::cerr << "Unable to read mesh from file " << filename << std::endl;
		}

		return res;
	}

	bool create_map(Map& map)
	{
		using Face = typename Map::Face;

		if (this->nb_vertices_ == 0u)
			return false;

		MapBuilder mbuild(map);
		map.clear_and_remove_attributes();

		mbuild.template create_embedding<Vertex::ORBIT>();
		mbuild.template swap_chunk_array_container<Vertex::ORBIT>(this->vertex_attributes_);
		VertexAttributeHandler<std::vector<Dart>> darts_per_vertex = map.template add_attribute<std::vector<Dart>, Vertex::ORBIT>("darts_per_vertex");

		unsigned int index = 0u;
		// buffer for tempo faces (used to remove degenerated edges)
		std::vector<unsigned int> edgesBuffer;
		edgesBuffer.reserve(16u);

		typename Map::DartMarkerStore m(map);

		//for each volume of table
		for(unsigned int i = 0u; i < this->nb_volumes_; ++i)
		{
			// store volume in buffer, removing degenated faces
			const unsigned int nbv = this->volumes_nb_vertices_[i];

			edgesBuffer.clear();
			unsigned int prec = std::numeric_limits<unsigned int>::max();
			for (unsigned int j = 0u; j < nbv; ++j)
			{
				unsigned int em = this->volumes_vertex_indices_[index++];
				if (em != prec)
				{
					prec = em;
					edgesBuffer.push_back(em);
				}
			}

			if(nbv == 4u) //tetrahedral case
			{
				const Dart d = mbuild.add_pyramid_topo(3u);

				const std::array<Dart, 4> vertices_of_tetra = {d,
									   map.phi1(d),
									   map.phi_1(d),
									   map.phi_1(map.phi2(map.phi_1(d)))
									  };

				std::size_t buffer_id = 0ul;
				for (const Dart dv : vertices_of_tetra)
				{
					const unsigned emb = edgesBuffer[buffer_id++];
					mbuild.init_parent_vertex_embedding(dv,emb);

					Dart dd = dv;
					do
					{
						m.mark(dd);
						darts_per_vertex[emb].push_back(dd);
						dd = map.phi1(map.phi2(dd));
					} while(dd != dv);
				}
			}
			else if(nbv == 5u) //pyramidal case
			{
				Dart d = mbuild.add_pyramid_topo(4u);

				const std::array<Dart, 5> vertices_of_pyramid = {d,
									   map.phi1(d),
									   map.phi1(map.phi1(d)),
									   map.phi_1(d),
									   map.phi_1(map.phi2(map.phi_1(d)))
									  };

				std::size_t buffer_id = 0ul;
				for (Dart dv : vertices_of_pyramid)
				{
					const unsigned emb = edgesBuffer[buffer_id++];
					mbuild.init_parent_vertex_embedding(dv,emb);

					Dart dd = dv;
					do
					{
						m.mark(dd);
						darts_per_vertex[emb].push_back(dd);
						dd = map.phi1(map.phi2(dd));
					} while(dd != dv);
				}
			}
			else if(nbv == 6u) //prism case
			{
				Dart d = mbuild.add_prism_topo(3u);
				const std::array<Dart, 6> vertices_of_prism = {
					d,
				   map.phi1(d),
				   map.phi_1(d),
				   map.phi2(map.phi1(map.phi1(map.phi2(map.phi_1(d))))),
				   map.phi2(map.phi1(map.phi1(map.phi2(d)))),
				   map.phi2(map.phi1(map.phi1(map.phi2(map.phi1(d))))),
				};

				std::size_t buffer_id = 0ul;
				for (Dart dv : vertices_of_prism)
				{
					const unsigned int emb = edgesBuffer[buffer_id++];
					mbuild.init_parent_vertex_embedding(dv,emb);

					Dart dd = dv;
					do
					{
						m.mark(dd);
						darts_per_vertex[emb].push_back(dd);
						dd = map.phi1(map.phi2(dd));
					} while(dd != dv);
				}
			}
			else if(nbv == 8u) //hexahedral case
			{
				Dart d = mbuild.add_prism_topo(4u);
				const std::array<Dart, 8> vertices_of_hexa = {
					d,
					map.phi1(d),
					map.phi1(map.phi1(d)),
					map.phi_1(d),

					map.phi2(map.phi1(map.phi1(map.phi2(map.phi_1(d))))),
					map.phi2(map.phi1(map.phi1(map.phi2(d)))),
					map.phi2(map.phi1(map.phi1(map.phi2(map.phi1(d))))),
					map.phi2(map.phi1(map.phi1(map.phi2(map.phi1(map.phi1(d))))))
				};

				std::size_t buffer_id = 0ul;
				for (Dart dv : vertices_of_hexa)
				{
					const unsigned emb = edgesBuffer[buffer_id++];
					mbuild.init_parent_vertex_embedding(dv,emb);

					Dart dd = dv;
					do
					{
						m.mark(dd);
						darts_per_vertex[emb].push_back(dd);
						dd = map.phi1(map.phi2(dd));
					} while(dd != dv);
				}
			}  //end of hexa

		}

		std::cout << " elements created " << std::endl;

		//reconstruct neighbourhood
		unsigned int nbBoundaryFaces = 0;
		for (Dart d : map)
		{
			if (m.is_marked(d))
			{
				std::vector<Dart>& vec = darts_per_vertex[map.phi1(d)];

				Dart good_dart;
				for(auto it = vec.begin(); it != vec.end() && good_dart.is_nil(); ++it)
				{
					if(map.get_embedding(Vertex(map.phi1(*it))) == map.get_embedding(Vertex(d)) &&
							map.get_embedding(Vertex(map.phi_1(*it))) == map.get_embedding(Vertex(map.phi1(map.phi1(d)))))
					{
						good_dart = *it;
					}
				}

				if (!good_dart.is_nil())
				{
					const unsigned int degD = map.degree(Face(d));
					const unsigned int degGD = map.degree(Face(good_dart));

					//std::cout << "degD = " << degD << " et degGD = " << degGD << std::endl;
					if(degD == degGD)
					{
//						map.sew_volumes(d, good_dart, false);
						Dart f1_it = d;
						Dart f2_it = good_dart;
						do {
							mbuild.phi3_sew(f1_it, f2_it);
							f1_it = map.phi1(f1_it);
							f2_it = map.phi_1(f2_it);
						} while (f1_it != d);
						m.template unmark_orbit<Orbit::PHI1_PHI3>(d);
					}
	//                else
	//                    std::cout << "erreur : degD != degGD" << std::endl;
				}
				else
				{
					m.template unmark_orbit<Orbit::PHI1>(d);
					++nbBoundaryFaces;
				}
			}
		}

		if (nbBoundaryFaces > 0)
		{
			unsigned int nbH = mbuild.close_map();
			std::cout << "Map closed (" << nbBoundaryFaces << " boundary faces / " << nbH << " holes)" << std::endl;
		}

		return true;
	}

protected:

	template <typename VEC3>
	bool import_VTU(const std::string& filename)
	{
		using tinyxml2::XMLDocument;
		using tinyxml2::XMLError;
		using tinyxml2::XML_NO_ERROR;
		using tinyxml2::XMLElement;

		ChunkArray<VEC3>* position =
				vertex_attributes_.template add_attribute<VEC3>("position");
		cgogn_assert(position != nullptr);

		XMLDocument doc;
		XMLError eResult = doc.LoadFile(filename.c_str());
		if (eResult != XML_NO_ERROR)
		{
			std::cerr << "unable loading file " << filename << std::endl;
			return false;
		}

		XMLElement* vtu_node = doc.RootElement();
		cgogn_assert(vtu_node != nullptr);
		XMLElement* grid_node = vtu_node->FirstChildElement("UnstructuredGrid");
		cgogn_assert(grid_node != nullptr);
		XMLElement* piece_node = grid_node->FirstChildElement("Piece");
		cgogn_assert(piece_node != nullptr);

		eResult = piece_node->QueryUnsignedAttribute("NumberOfPoints",&nb_vertices_);
		if (eResult != XML_NO_ERROR)
		{
			std::cerr << "unreadable VTU file: " << filename << std::endl;
			return false;
		}
		eResult = piece_node->QueryUnsignedAttribute("NumberOfCells",&nb_volumes_);
		if (eResult != XML_NO_ERROR)
		{
			std::cerr << "unreadable VTU file: " << filename << std::endl;
			return false;
		}

		std::cout << "reading file " << filename << std::endl;
		std::cout << "Number of vertices : " << this->nb_vertices_ << std::endl;
		std::cout << "Number of volumes : " << this->nb_volumes_ << std::endl;

		XMLElement* points_node = piece_node->FirstChildElement("Points");
		cgogn_assert(points_node != nullptr);
		XMLElement* array_node = points_node->FirstChildElement("DataArray");
		cgogn_assert(array_node != nullptr);

		std::vector<unsigned int> verticesID;
		verticesID.reserve(nb_vertices_);
		std::stringstream ss(array_node->GetText());
		for (unsigned int i=0u; i< nb_vertices_; ++i)
		{
			VEC3 P;
			ss >> P[0];
			ss >> P[1];
			ss >> P[2];
			unsigned int id = vertex_attributes_.template insert_lines<1>();
			position->operator [](id) = P;
			verticesID.push_back(id);
		}

		XMLElement* cell_node = piece_node->FirstChildElement("Cells");
		cgogn_assert(cell_node != nullptr);
		array_node = cell_node->FirstChildElement("DataArray");
		cgogn_assert(array_node != nullptr);

		std::vector<unsigned int> typeVols;
		typeVols.reserve(nb_volumes_);
		std::vector<unsigned int> offsets;
		offsets.reserve(nb_volumes_);
		std::vector<unsigned int> indices;
		indices.reserve(nb_volumes_*4u);

		while (array_node)
		{
			const std::string& propName = to_lower(std::string(array_node->Attribute("Name")));
			if (propName.empty())
			{
				std::cerr << "Error reading VTU unreadable file: "<< filename << std::endl;
				return false;
			}

			if (propName == "connectivity")
			{
				std::stringstream ss(array_node->GetText());
				while (!ss.eof())
				{
					unsigned int ind;
					ss >> ind;
					indices.push_back(ind);
				}
			}
			if (propName == "offsets")
			{
				std::stringstream ss(array_node->GetText());
				for (unsigned int i=0u; i< nb_volumes_; ++i)
				{
					unsigned int o;
					ss >> o;
					offsets.push_back(o);
				}
			}
			if (propName == "types")
			{
				bool unsupported = false;
				std::stringstream ss(array_node->GetText());
				for (unsigned int i=0u; i< nb_volumes_; ++i)
				{
					unsigned int t;
					ss >> t;
					if (!(t == 10u || t == 12u))
					{
						std::cerr << "error while parsing vtk file : volumes of type " << t << " are not supported" << std::endl;
						unsupported = true;
					}
					typeVols.push_back(t);
				}
				if (unsupported)
				{
					std::cerr << "warning, some unsupported volume cell types"<< std::endl;
				}

			}
			array_node = array_node->NextSiblingElement("DataArray");
		}

		unsigned int currentOffset = 0;
		for (unsigned int i=0u; i< nb_volumes_; ++i)
		{
			if (typeVols[i]==12u)
			{
				volumes_nb_vertices_.push_back(8u);

				std::array<unsigned int, 8> pt;
				VEC3 const& P = position->operator [](verticesID[indices[currentOffset+4]]);
				VEC3 const& A = position->operator [](verticesID[indices[currentOffset  ]]);
				VEC3 const& B = position->operator [](verticesID[indices[currentOffset+1]]);
				VEC3 const& C = position->operator [](verticesID[indices[currentOffset+2]]);

				if (geometry::test_orientation_3D(P,A,B,C) == geometry::Orientation3D::OVER)
				{
					pt[0] = indices[currentOffset+3];
					pt[1] = indices[currentOffset+2];
					pt[2] = indices[currentOffset+1];
					pt[3] = indices[currentOffset+0];
					pt[4] = indices[currentOffset+7];
					pt[5] = indices[currentOffset+6];
					pt[6] = indices[currentOffset+5];
					pt[7] = indices[currentOffset+4];
				}
				else
				{
					pt[0] = indices[currentOffset+0];
					pt[1] = indices[currentOffset+1];
					pt[2] = indices[currentOffset+2];
					pt[3] = indices[currentOffset+3];
					pt[4] = indices[currentOffset+4];
					pt[5] = indices[currentOffset+5];
					pt[6] = indices[currentOffset+6];
					pt[7] = indices[currentOffset+7];
				}

				volumes_vertex_indices_.push_back(verticesID[pt[0]]);
				volumes_vertex_indices_.push_back(verticesID[pt[1]]);
				volumes_vertex_indices_.push_back(verticesID[pt[2]]);
				volumes_vertex_indices_.push_back(verticesID[pt[3]]);
				volumes_vertex_indices_.push_back(verticesID[pt[4]]);
				volumes_vertex_indices_.push_back(verticesID[pt[5]]);
				volumes_vertex_indices_.push_back(verticesID[pt[6]]);
				volumes_vertex_indices_.push_back(verticesID[pt[7]]);

			}
			else if (typeVols[i]==10u)
			{
				volumes_nb_vertices_.push_back(4u);

				std::array<unsigned int, 4> pt;
				pt[0] = indices[currentOffset];
				pt[1] = indices[currentOffset+1];
				pt[2] = indices[currentOffset+2];
				pt[3] = indices[currentOffset+3];

				VEC3 const& P = position->operator [](verticesID[pt[0]]);
				VEC3 const& A = position->operator [](verticesID[pt[1]]);
				VEC3 const& B = position->operator [](verticesID[pt[2]]);
				VEC3 const& C = position->operator [](verticesID[pt[3]]);

				if (geometry::test_orientation_3D(P,A,B,C) == geometry::Orientation3D::OVER)
					std::swap(pt[1], pt[2]);

				volumes_vertex_indices_.push_back(verticesID[pt[0]]);
				volumes_vertex_indices_.push_back(verticesID[pt[1]]);
				volumes_vertex_indices_.push_back(verticesID[pt[2]]);
				volumes_vertex_indices_.push_back(verticesID[pt[3]]);
			}
			currentOffset = offsets[i];
		}

		return true;
	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(IO_VOLUME_IMPORT_CPP_))
extern template class CGOGN_IO_API VolumeImport<DefaultMapTraits>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(IO_VOLUME_IMPORT_CPP_))

} // namespace io

} // namespace cgogn

#endif // IO_VOLUME_IMPORT_H_
