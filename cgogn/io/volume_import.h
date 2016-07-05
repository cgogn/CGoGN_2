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

#ifndef CGOGN_IO_VOLUME_IMPORT_H_
#define CGOGN_IO_VOLUME_IMPORT_H_

#include <istream>

#include <cgogn/core/utils/string.h>
#include <cgogn/core/container/chunk_array_container.h>
#include <cgogn/core/cmap/cmap3_builder.h>

#include <cgogn/geometry/functions/orientation.h>

#include <cgogn/io/dll.h>
#include <cgogn/io/c_locale.h>
#include <cgogn/io/mesh_io_gen.h>
#include <cgogn/io/io_utils.h>

#include <tinyxml2.h>

/*******************************************************************************
* CGoGN convention for ordering the indices for volume cells in the VolumeImport class (prior to the map creation)
*
*-->Tetras: any order. The orientation is checked when calling add_tetra.
*           3
*         ,/|`\
*       ,/  |  `\
*     ,/    '.   `\
*   ,/       |     `\
* ,/         |       `\
*2-----------'.--------1
* `\.         |      ,/
*    `\.      |    ,/
*       `\.   '. ,/
*          `\. |/
*             `0
*
*-->Pyramids: First the indices of the quad face, then the top. The orientation is checked when calling add_pyramid.
*               4
*             ,/|\
*           ,/ .'|\
*         ,/   | | \
*       ,/    .' | `.
*     ,/      |  '.  \
*   ,/       .'   |   \
* ,/         |    |    \
*0----------.'----3    `.
* `\        |      `\    \
*   `\     .'        `\ - \
*     `\   |           `\  \
*       `\.'            `\`
*          1----------------2
*
*-->Triangular prisms: First the indices of one of the triangular face then the indices of the opposite face (same order). The orientation is checked when calling add_triangular_prism.
*       3
*     ,/|`\
*   ,/  |  `\
* ,/    |    `\
*5------+------4
*|      |      |
*|      |      |
*|      |      |
*|      |      |
*|      |      |
*|      0      |
*|    ,/ `\    |
*|  ,/     `\  |
*|,/         `\|
*2-------------1
*
*-->Hexas: First the indices of one face then the indices of the opposite face (same order). The orientation is checked when calling add_hexa.
*7----------6
*|\         |\
*| \        | \
*|  \       |  \
*|   3------+---2
*|   |      |-- |
*4---+------5   |
* \  |       \  |
*  \ |        \ |
*   \|         \|
*    0----------1
*******************************************************************************/

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

	static const uint32 CHUNK_SIZE = MAP_TRAITS::CHUNK_SIZE;

	template <typename T>
	using ChunkArray = cgogn::ChunkArray<CHUNK_SIZE, T>;
	using ChunkArrayContainer = cgogn::ChunkArrayContainer<CHUNK_SIZE, uint32>;

	template <typename T, Orbit ORBIT>
	using Attribute = Attribute<MAP_TRAITS, T, ORBIT>;

	virtual ~VolumeImport() override
	{}

private:

	uint32 nb_vertices_;
	uint32 nb_volumes_;

	std::vector<VolumeType>	volumes_types;
	std::vector<uint32>		volumes_vertex_indices_;

	ChunkArrayContainer vertex_attributes_;
	ChunkArrayContainer volume_attributes_;

protected:

	inline void set_nb_vertices(uint32 nbv)
	{
		nb_vertices_ = nbv;
	}

	inline uint32 nb_vertices() const
	{
		return nb_vertices_;
	}

	inline void set_nb_volumes(uint32 nbw)
	{
		nb_volumes_ = nbw;
		volumes_types.reserve(nbw);
		volumes_vertex_indices_.reserve(8u * nbw);
	}

	inline uint32 nb_volumes() const
	{
		return nb_volumes_;
	}

	template <typename VEC3>
	inline ChunkArray<VEC3>* position_attribute()
	{
		auto res = this->vertex_attributes_.template add_chunk_array<VEC3>("position");
		if (res != nullptr)
			return res;
		else
			return this->vertex_attributes_.template get_chunk_array<VEC3>("position");
	}

	inline uint32 insert_line_vertex_container()
	{
		return vertex_attributes_.template insert_lines<1>();
	}

	inline ChunkArrayContainer& vertex_attributes_container()
	{
		return vertex_attributes_;
	}

	inline ChunkArrayContainer& volume_attributes_container()
	{
		return volume_attributes_;
	}

public:

	VolumeImport() :
		nb_vertices_(0u)
	  ,volumes_types()
	  ,volumes_vertex_indices_()
	{}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(VolumeImport);

	template <typename Map>
	bool create_map(Map& map)
	{
		static_assert(Map::DIMENSION == 3, "must use map of dim 3 in volume import");

		using Vertex = typename Map::Vertex;
		using Volume = typename Map::Volume;
		using Face = typename Map::Face;
		using Face2 = typename Map::Face2;
		using MapBuilder = typename Map::Builder;

		if (this->nb_vertices_ == 0u || this->volumes_types.size() != this->nb_volumes_)
			return false;

		MapBuilder mbuild(map);
		map.clear_and_remove_attributes();

		mbuild.template create_embedding<Vertex::ORBIT>();
		mbuild.template swap_chunk_array_container<Vertex::ORBIT>(this->vertex_attributes_);

		typename Map::template VertexAttribute<std::vector<Dart>> darts_per_vertex = map.template add_attribute<std::vector<Dart>, Vertex::ORBIT>("darts_per_vertex");

		uint32 index = 0u;
		typename Map::DartMarkerStore m(map);

		//for each volume of table
		for (uint32 i = 0u; i < this->nb_volumes_; ++i)
		{
			// store volume in buffer, removing degenated faces
			const VolumeType vol_type = this->volumes_types[i];

			if (vol_type == VolumeType::Tetra) //tetrahedral case
			{
				const Dart d = mbuild.add_pyramid_topo(3u);

				// check if add ok (special maps)
				if (d.is_nil()) break;

				const std::array<Dart, 4> vertices_of_tetra = {
					d,
					map.phi1(d),
					map.phi_1(d),
					map.phi_1(map.phi2(map.phi_1(d)))
				};

				for (const Dart dv : vertices_of_tetra)
				{
					const uint32 emb = this->volumes_vertex_indices_[index++];
					mbuild.init_parent_vertex_embedding(dv, emb);

					Dart dd = dv;
					do
					{
						m.mark(dd);
						darts_per_vertex[emb].push_back(dd);
						dd = map.phi1(map.phi2(dd));
					} while(dd != dv);
				}
			}
			else if (vol_type == VolumeType::Pyramid) //pyramidal case
			{
				Dart d = mbuild.add_pyramid_topo(4u);

				// check if add ok (special maps)
				if (d.is_nil()) break;

				const std::array<Dart, 5> vertices_of_pyramid = {
					d,
					map.phi1(d),
					map.phi1(map.phi1(d)),
					map.phi_1(d),
					map.phi_1(map.phi2(map.phi_1(d)))
				};

				for (Dart dv : vertices_of_pyramid)
				{
					const uint32 emb = this->volumes_vertex_indices_[index++];
					mbuild.init_parent_vertex_embedding(dv, emb);

					Dart dd = dv;
					do
					{
						m.mark(dd);
						darts_per_vertex[emb].push_back(dd);
						dd = map.phi1(map.phi2(dd));
					} while(dd != dv);
				}
			}
			else if (vol_type == VolumeType::TriangularPrism) //prism case
			{
				Dart d = mbuild.add_prism_topo(3u);

				// check if add ok (special maps)
				if (d.is_nil()) break;

				const std::array<Dart, 6> vertices_of_prism = {
					d,
					map.phi1(d),
					map.phi_1(d),
					map.phi2(map.phi1(map.phi1(map.phi2(map.phi_1(d))))),
					map.phi2(map.phi1(map.phi1(map.phi2(d)))),
					map.phi2(map.phi1(map.phi1(map.phi2(map.phi1(d)))))
				};

				for (Dart dv : vertices_of_prism)
				{
					const uint32 emb = this->volumes_vertex_indices_[index++];
					mbuild.init_parent_vertex_embedding(dv, emb);

					Dart dd = dv;
					do
					{
						m.mark(dd);
						darts_per_vertex[emb].push_back(dd);
						dd = map.phi1(map.phi2(dd));
					} while(dd != dv);
				}
			}
			else if (vol_type == VolumeType::Hexa) //hexahedral case
			{
				Dart d = mbuild.add_prism_topo(4u);

				// check if add ok (special maps)
				if (d.is_nil()) break;

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

				for (Dart dv : vertices_of_hexa)
				{
					const uint32 emb = this->volumes_vertex_indices_[index++];
					mbuild.init_parent_vertex_embedding(dv, emb);

					Dart dd = dv;
					do
					{
						m.mark(dd);
						darts_per_vertex[emb].push_back(dd);
						dd = map.phi1(map.phi2(dd));
					} while(dd != dv);
				}
			}
			else
			{ //end of hexa
				if (vol_type == VolumeType::Connector)
				{
					index += 4u;
					// The second part of the code generates connectors automatically. We don't have to do anything here.
				}
			}
		}

		//reconstruct neighbourhood
		uint32 nb_boundary_faces = 0u;
		map.foreach_dart([&] (Dart d)
		{
			if (m.is_marked(d))
			{
				Dart good_dart;

				// 1st step : for every dart of the face we try to find a valid phi3 candidate. If we can't it's a boundary face.
				{
					Dart d_it = d;
					do
					{
						const std::vector<Dart>& vec = darts_per_vertex[Vertex(map.phi1(d_it))];
						for (auto it = vec.begin(); it != vec.end() && good_dart.is_nil(); ++it)
						{
							if (map.embedding(Vertex(map.phi1(*it))) == map.embedding(Vertex(d_it)) &&
									map.embedding(Vertex(map.phi_1(*it))) == map.embedding(Vertex(map.phi1(map.phi1(d_it)))))
							{
								good_dart = *it;
							}
						}
						d_it = map.phi1(d_it);
					} while (good_dart.is_nil() && (d_it != d));
					d = map.phi_1(d_it);
				}

				if (!good_dart.is_nil()) //not a boundary faces
				{
					const uint32 degD = map.codegree(Face(d));
					const uint32 degGD = map.codegree(Face(good_dart));

					if (degD == degGD) // normal case : the two opposite faces have the same degree
					{
						mbuild.sew_volumes(d, good_dart);
						m.unmark_orbit(Face(d));
					}
					else
					{
						// there is one face of degree 4 and one face of degree 3.
						if (degD > degGD) // face of d is quad
						{
							const Dart another_d = map.phi1(map.phi1(d));
							const std::vector<Dart>& vec = darts_per_vertex[Vertex(map.phi_1(d))];

							Dart another_good_dart;
							for (auto it = vec.begin(); it != vec.end() && another_good_dart.is_nil(); ++it)
							{
								if (map.embedding(Vertex(map.phi1(*it))) == map.embedding(Vertex(another_d)) &&
										map.embedding(Vertex(map.phi_1(*it))) == map.embedding(Vertex(map.phi1(map.phi1(another_d)))))
								{
									another_good_dart = *it ;
								}
							}

							// we add a stamp volume between the faces
							const Dart d_quad = mbuild.add_stamp_volume_topo();
							{
								Dart q1_it = d;
								Dart q2_it = map.phi_1(d_quad);
								do
								{
									mbuild.init_parent_vertex_embedding(q2_it, map.embedding(Vertex(q1_it)));
									q1_it = map.phi1(q1_it);
									q2_it = map.phi_1(q2_it);
								} while (q1_it != d);
							}

							mbuild.sew_volumes(d, map.phi1(map.phi1(d_quad)));
							m.unmark_orbit(Face(d));

							mbuild.sew_volumes(good_dart, map.phi2(map.phi1(map.phi1(d_quad))));
							m.unmark_orbit(Face(good_dart));

							if (!another_good_dart.is_nil())
							{
								mbuild.sew_volumes(another_good_dart, map.phi2(d_quad));
								m.unmark_orbit(Face(another_good_dart));
							}
							else
							{
								m.unmark_orbit(Face2(map.phi2(d_quad)));
								++nb_boundary_faces;
							}
						}
						else // face of d is tri
						{
							const Dart another_dart = map.phi_1(d);
							std::vector<Dart>& vec = darts_per_vertex[Vertex(d)];

							Dart another_good_dart;
							for (auto it = vec.begin(); it != vec.end() && another_good_dart.is_nil(); ++it)
							{
								if (map.embedding(Vertex(map.phi1(*it))) == map.embedding(Vertex(another_dart)) &&
										map.embedding(Vertex(map.phi_1(*it))) == map.embedding(Vertex(map.phi1(map.phi1(good_dart)))))
								{
									another_good_dart = *it ;
								}
							}

							const Dart d_quad = mbuild.add_stamp_volume_topo();
							{
								Dart q1_it = good_dart;
								Dart q2_it = d_quad;
								do
								{
									mbuild.init_parent_vertex_embedding(q2_it, map.embedding(Vertex(q1_it)));
									q1_it = map.phi1(q1_it);
									q2_it = map.phi_1(q2_it);
								} while (q1_it != good_dart);
							}

							mbuild.sew_volumes(d_quad, map.phi_1(good_dart));
							m.unmark_orbit(Face(good_dart));

							mbuild.sew_volumes(d, map.phi2(map.phi_1(d_quad)));
							m.unmark_orbit(Face(d));

							if (!another_good_dart.is_nil())
							{
								mbuild.sew_volumes(another_good_dart, map.phi1(map.phi2(map.phi1(d_quad))));
								m.unmark_orbit(Face(another_good_dart));
							}
							else
							{
								m.unmark_orbit(Face2(map.phi1(map.phi2(map.phi1(d_quad)))));
								++nb_boundary_faces;
							}
						}
					}
				}
				else
				{
					m.unmark_orbit(Face2(d));
					++nb_boundary_faces;
				}
			}
		});

		if (nb_boundary_faces > 0)
		{
			mbuild.close_map();
			cgogn_log_info("create_map") << "Map closed with " << nb_boundary_faces << " boundary face(s).";
		}

		uint32 nb_vert_dart_marking = 0u;
		map.template foreach_cell<FORCE_DART_MARKING>([&nb_vert_dart_marking](Vertex){++nb_vert_dart_marking;});

		if (this->nb_vertices_ != nb_vert_dart_marking)
			map.template enforce_unique_orbit_embedding<Vertex::ORBIT>();

		if (this->volume_attributes_.nb_chunk_arrays() > 0)
		{
			mbuild.template create_embedding<Volume::ORBIT>();
			mbuild.template swap_chunk_array_container<Volume::ORBIT>(this->volume_attributes_);
		}

		map.remove_attribute(darts_per_vertex);

		return true;
	}

protected:

	virtual void clear() override
	{
		set_nb_vertices(0u);
		set_nb_volumes(0u);
		volumes_types.clear();
		volumes_vertex_indices_.clear();
		vertex_attributes_.remove_chunk_arrays();
		volume_attributes_.remove_chunk_arrays();
	}

	template <typename VEC3>
	void add_hexa(ChunkArray<VEC3>const& pos, uint32 p0, uint32 p1, uint32 p2, uint32 p3, uint32 p4, uint32 p5, uint32 p6, uint32 p7, bool check_orientation)
	{
		if (check_orientation)
			this->reoriente_hexa(pos, p0, p1, p2, p3, p4, p5, p6, p7);
		this->volumes_types.push_back(VolumeType::Hexa);
		this->volumes_vertex_indices_.push_back(p0);
		this->volumes_vertex_indices_.push_back(p1);
		this->volumes_vertex_indices_.push_back(p2);
		this->volumes_vertex_indices_.push_back(p3);
		this->volumes_vertex_indices_.push_back(p4);
		this->volumes_vertex_indices_.push_back(p5);
		this->volumes_vertex_indices_.push_back(p6);
		this->volumes_vertex_indices_.push_back(p7);
	}

	template <typename VEC3>
	inline void reoriente_hexa(ChunkArray<VEC3>const& pos, uint32& p0, uint32& p1, uint32& p2, uint32& p3, uint32& p4, uint32& p5, uint32& p6, uint32& p7)
	{
		if (geometry::test_orientation_3D(pos[p4], pos[p0], pos[p1], pos[p2]) == geometry::Orientation3D::OVER)
		{
			std::swap(p0, p3);
			std::swap(p1, p2);
			std::swap(p4, p7);
			std::swap(p5, p6);
		}
	}

	template <typename VEC3>
	void add_tetra(ChunkArray<VEC3>const& pos, uint32 p0, uint32 p1, uint32 p2, uint32 p3, bool check_orientation)
	{
		if (check_orientation)
			this->reoriente_tetra(pos, p0, p1, p2, p3);
		this->volumes_types.push_back(VolumeType::Tetra);
		this->volumes_vertex_indices_.push_back(p0);
		this->volumes_vertex_indices_.push_back(p1);
		this->volumes_vertex_indices_.push_back(p2);
		this->volumes_vertex_indices_.push_back(p3);
	}

	template <typename VEC3>
	inline void reoriente_tetra(ChunkArray<VEC3>const& pos, uint32& p0, uint32& p1, uint32& p2, uint32& p3)
	{
		if (geometry::test_orientation_3D(pos[p0], pos[p1], pos[p2], pos[p3]) == geometry::Orientation3D::OVER)
			std::swap(p1, p2);
	}

	template <typename VEC3>
	void add_pyramid(ChunkArray<VEC3>const& pos, uint32 p0, uint32 p1, uint32 p2, uint32 p3, uint32 p4, bool check_orientation)
	{
		this->volumes_types.push_back(VolumeType::Pyramid);
		if (check_orientation)
			this->reoriente_pyramid(pos, p0, p1, p2, p3, p4);
		this->volumes_vertex_indices_.push_back(p0);
		this->volumes_vertex_indices_.push_back(p1);
		this->volumes_vertex_indices_.push_back(p2);
		this->volumes_vertex_indices_.push_back(p3);
		this->volumes_vertex_indices_.push_back(p4);
	}

	template <typename VEC3>
	inline void reoriente_pyramid(ChunkArray<VEC3>const& pos, uint32& p0, uint32& p1, uint32& p2, uint32& p3, uint32& p4)
	{
		if (geometry::test_orientation_3D(pos[p4], pos[p0], pos[p1], pos[p2]) == geometry::Orientation3D::OVER)
			std::swap(p1, p3);
	}

	template <typename VEC3>
	void add_triangular_prism(ChunkArray<VEC3>const& pos, uint32 p0, uint32 p1, uint32 p2, uint32 p3, uint32 p4, uint32 p5, bool check_orientation)
	{
		if (check_orientation)
			this->reoriente_triangular_prism(pos, p0, p1, p2, p3, p4, p5);
		this->volumes_types.push_back(VolumeType::TriangularPrism);
		this->volumes_vertex_indices_.push_back(p0);
		this->volumes_vertex_indices_.push_back(p1);
		this->volumes_vertex_indices_.push_back(p2);
		this->volumes_vertex_indices_.push_back(p3);
		this->volumes_vertex_indices_.push_back(p4);
		this->volumes_vertex_indices_.push_back(p5);
	}

	template <typename VEC3>
	inline void reoriente_triangular_prism(ChunkArray<VEC3>const& pos, uint32& p0, uint32& p1, uint32& p2, uint32& p3, uint32& p4, uint32& p5)
	{
		if (geometry::test_orientation_3D(pos[p3], pos[p0], pos[p1], pos[p2]) == geometry::Orientation3D::OVER)
		{
			std::swap(p1, p2);
			std::swap(p4, p5);
		}
	}

	inline void add_connector(uint32 p0, uint32 p1, uint32 p2, uint32 p3)
	{
		this->volumes_types.push_back(VolumeType::Connector);
		this->volumes_vertex_indices_.push_back(p0);
		this->volumes_vertex_indices_.push_back(p1);
		this->volumes_vertex_indices_.push_back(p2);
		this->volumes_vertex_indices_.push_back(p3);
	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_VOLUME_IMPORT_CPP_))
extern template class CGOGN_IO_API VolumeImport<DefaultMapTraits>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_VOLUME_IMPORT_CPP_))

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_VOLUME_IMPORT_H_
