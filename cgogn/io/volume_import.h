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
#include <io/mesh_io_gen.h>

#include <tinyxml2.h>

namespace cgogn
{

namespace io
{

template <typename MAP_TRAITS>
class VolumeImport : public MeshImportGen
{
public:

	using Self = VolumeImport<MAP_TRAITS>;
	using Inherit = MeshImportGen;
	using Map = CMap3<MAP_TRAITS>;
	using Vertex = typename Map::Vertex;
	using Volume = typename Map::Volume;
	using Face = typename Map::Face;
	using Face2 = typename Map::Face2;

	static const unsigned int CHUNK_SIZE = MAP_TRAITS::CHUNK_SIZE;

	template <typename T>
	using ChunkArray = ChunkArray<CHUNK_SIZE, T>;
	using ChunkArrayContainer = cgogn::ChunkArrayContainer<CHUNK_SIZE, unsigned int>;

	template <typename T, Orbit ORBIT>
	using AttributeHandler = AttributeHandler<MAP_TRAITS, T, ORBIT>;
	using MapBuilder = cgogn::CMap3Builder_T<typename Map::MapTraits>;

	virtual ~VolumeImport() override {}

protected:
	unsigned int nb_vertices_;
	unsigned int nb_volumes_;

	std::vector<unsigned int> volumes_nb_vertices_;
	std::vector<unsigned int> volumes_vertex_indices_;

	ChunkArrayContainer vertex_attributes_;
	ChunkArrayContainer volume_attributes_;

public:
	VolumeImport() :
		nb_vertices_(0u)
	  ,volumes_nb_vertices_()
	  ,volumes_vertex_indices_()
	{}

	VolumeImport(const Self&) = delete;
	VolumeImport(Self&&) = delete;
	Self& operator=(const Self&) = delete;
	Self& operator=(Self&&) = delete;

	virtual void clear() override
	{
		nb_vertices_ = 0;
		volumes_nb_vertices_.clear();
		volumes_vertex_indices_.clear();
		vertex_attributes_.remove_attributes();
		volume_attributes_.remove_attributes();
	}

	bool create_map(Map& map)
	{
		if (this->nb_vertices_ == 0u)
			return false;

		MapBuilder mbuild(map);
		map.clear_and_remove_attributes();

		mbuild.template create_embedding<Vertex::ORBIT>();
		mbuild.template swap_chunk_array_container<Vertex::ORBIT>(this->vertex_attributes_);

		typename Map::template VertexAttributeHandler<std::vector<Dart>> darts_per_vertex = map.template add_attribute<std::vector<Dart>, Vertex::ORBIT>("darts_per_vertex");

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

		//reconstruct neighbourhood
		unsigned int nbBoundaryFaces = 0;
		for (Dart d : map)
		{
			if (m.is_marked(d))
			{
				std::vector<Dart>& vec = darts_per_vertex[Vertex(map.phi1(d))];

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
						m.unmark_orbit(Face(d));
					}
	//                else
	//                    std::cout << "erreur : degD != degGD" << std::endl;
				}
				else
				{
					m.unmark_orbit(Face2(d));
					++nbBoundaryFaces;
				}
			}
		}

		if (nbBoundaryFaces > 0)
		{
			unsigned int nbH = mbuild.close_map();
			std::cout << "Map closed (" << nbBoundaryFaces << " boundary faces / " << nbH << " holes)" << std::endl;
		}

		if (this->volume_attributes_.get_nb_attributes() > 0)
		{
			mbuild.template create_embedding<Volume::ORBIT>();
			mbuild.template swap_chunk_array_container<Volume::ORBIT>(this->volume_attributes_);
		}

		return true;
	}

	template<typename VEC3>
	void add_hexa(ChunkArray<VEC3>const& pos,unsigned int& p0, unsigned int& p1, unsigned int& p2, unsigned int& p3, unsigned int& p4, unsigned int& p5, unsigned int& p6, unsigned int& p7)
	{
		this->reoriente_hexa(pos, p0, p1, p2, p3, p4, p5, p6, p7);
		this->volumes_nb_vertices_.push_back(8u);
		this->volumes_vertex_indices_.push_back(p0);
		this->volumes_vertex_indices_.push_back(p1);
		this->volumes_vertex_indices_.push_back(p2);
		this->volumes_vertex_indices_.push_back(p3);
		this->volumes_vertex_indices_.push_back(p4);
		this->volumes_vertex_indices_.push_back(p5);
		this->volumes_vertex_indices_.push_back(p6);
		this->volumes_vertex_indices_.push_back(p7);
	}
	template<typename VEC3>
	inline void reoriente_hexa(ChunkArray<VEC3>const& pos, unsigned int& p0, unsigned int& p1, unsigned int& p2, unsigned int& p3, unsigned int& p4, unsigned int& p5, unsigned int& p6, unsigned int& p7)
	{
		if (geometry::test_orientation_3D(pos[p4], pos[p0],pos[p1],pos[p2]) == geometry::Orientation3D::OVER)
		{
			std::swap(p0, p3);
			std::swap(p1, p2);
			std::swap(p4, p7);
			std::swap(p5, p6);
		}
	}

	template<typename VEC3>
	void add_tetra(ChunkArray<VEC3>const& pos,unsigned int& p0, unsigned int& p1, unsigned int& p2, unsigned int& p3)
	{
		this->reoriente_tetra(pos,p0,p1,p2,p3);
		this->volumes_nb_vertices_.push_back(4u);
		this->volumes_vertex_indices_.push_back(p0);
		this->volumes_vertex_indices_.push_back(p1);
		this->volumes_vertex_indices_.push_back(p2);
		this->volumes_vertex_indices_.push_back(p3);
	}

	template<typename VEC3>
	void add_pyramid(ChunkArray<VEC3>const& pos,unsigned int& p0, unsigned int& p1, unsigned int& p2, unsigned int& p3, unsigned int& p4)
	{
		this->volumes_nb_vertices_.push_back(5u);
		this->volumes_vertex_indices_.push_back(p0);
		this->volumes_vertex_indices_.push_back(p1);
		this->volumes_vertex_indices_.push_back(p2);
		this->volumes_vertex_indices_.push_back(p3);
		this->volumes_vertex_indices_.push_back(p4);
	}

	template<typename VEC3>
	void add_triangular_prism(ChunkArray<VEC3>const& pos,unsigned int& p0, unsigned int& p1, unsigned int& p2, unsigned int& p3, unsigned int& p4, unsigned int& p5)
	{
		this->volumes_nb_vertices_.push_back(6u);
		this->volumes_vertex_indices_.push_back(p0);
		this->volumes_vertex_indices_.push_back(p1);
		this->volumes_vertex_indices_.push_back(p2);
		this->volumes_vertex_indices_.push_back(p3);
		this->volumes_vertex_indices_.push_back(p4);
		this->volumes_vertex_indices_.push_back(p5);
	}

	template<typename VEC3>
	inline void reoriente_tetra(ChunkArray<VEC3>const& pos, unsigned int& p0, unsigned int& p1, unsigned int& p2, unsigned int& p3)
	{
		if (geometry::test_orientation_3D(pos[p0], pos[p1],pos[p2],pos[p3]) == geometry::Orientation3D::OVER)
			std::swap(p1, p2);
	}

};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(IO_VOLUME_IMPORT_CPP_))
extern template class CGOGN_IO_API VolumeImport<DefaultMapTraits>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(IO_VOLUME_IMPORT_CPP_))

} // namespace io

} // namespace cgogn

#endif // IO_VOLUME_IMPORT_H_
