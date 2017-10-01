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
#include <set>

#include <cgogn/core/utils/string.h>

#include <cgogn/core/cmap/cmap3.h>

#include <cgogn/geometry/functions/orientation.h>

#include <cgogn/io/dll.h>
#include <cgogn/io/c_locale.h>
#include <cgogn/io/mesh_io_gen.h>
#include <cgogn/io/io_utils.h>
#include <cgogn/io/data_io.h>

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

template <typename MAP>
class VolumeImport
{
public:

	static_assert(MAP::DIMENSION == 3, "Must use map of dimension 3 in volume import");

	using Self = VolumeImport<MAP>;

	using ChunkArrayContainer = MapBaseData::ChunkArrayContainer<uint32>;
	using ChunkArrayGen = MapBaseData::ChunkArrayGen;
	template <typename T>
	using ChunkArray = MapBaseData::ChunkArray<T>;

	using Vertex = typename MAP::Vertex;
	using Vertex2 = typename MAP::Vertex2;
	using Volume = typename MAP::Volume;
	using Face = typename MAP::Face;
	using Face2 = typename MAP::Face2;
	using MapBuilder = typename MAP::Builder;

	template <typename T, Orbit ORBIT>
	using Attribute = Attribute<T, ORBIT>;

	using DataInputGen = cgogn::io::DataInputGen;

	inline VolumeImport(MAP& map):
		volumes_types_(),
		volumes_vertex_indices_(),
		map_(map),
		mbuild_(map)
	{
		map_.clear_and_remove_attributes();
	}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(VolumeImport);
	virtual ~VolumeImport() {}

	inline ChunkArrayContainer& vertex_container()
	{
		return mbuild_.template attribute_container<Vertex::ORBIT>();
	}

	inline ChunkArrayContainer& volume_container()
	{
		return mbuild_.template attribute_container<Volume::ORBIT>();
	}

	inline uint32 insert_line_vertex_container()
	{
		return vertex_container().template insert_lines<1>();
	}

	inline uint32 insert_line_volume_container()
	{
		return volume_container().template insert_lines<1>();
	}

	inline ChunkArrayGen* add_vertex_attribute(const DataInputGen& in_data, const std::string& att_name)
	{
		ChunkArrayGen* att = in_data.add_attribute(vertex_container(), att_name);
		in_data.to_chunk_array(att);
		return att;
	}

	template<typename T>
	inline ChunkArray<T>* add_vertex_attribute(const std::string& att_name)
	{
		return vertex_container().template add_chunk_array<T>(att_name);
	}

	inline void add_volume_attribute(const DataInputGen& in_data, const std::string& att_name)
	{
		in_data.to_chunk_array(in_data.add_attribute(volume_container(), att_name));
	}

	template<typename T>
	inline ChunkArray<T>* add_volume_attribute(const std::string& att_name)
	{
		return volume_container().template add_chunk_array<T>(att_name);
	}

	inline void reserve(uint32 nb_volumes)
	{
		volumes_types_.reserve(nb_volumes);
		volumes_vertex_indices_.reserve(8u * nb_volumes);
	}

	void add_hexa(uint32 p0, uint32 p1, uint32 p2, uint32 p3, uint32 p4, uint32 p5, uint32 p6, uint32 p7)
	{
		this->volumes_types_.push_back(VolumeType::Hexa);
		this->volumes_vertex_indices_.push_back(p0);
		this->volumes_vertex_indices_.push_back(p1);
		this->volumes_vertex_indices_.push_back(p2);
		this->volumes_vertex_indices_.push_back(p3);
		this->volumes_vertex_indices_.push_back(p4);
		this->volumes_vertex_indices_.push_back(p5);
		this->volumes_vertex_indices_.push_back(p6);
		this->volumes_vertex_indices_.push_back(p7);
	}

	void add_tetra(uint32 p0, uint32 p1, uint32 p2, uint32 p3)
	{
		this->volumes_types_.push_back(VolumeType::Tetra);
		this->volumes_vertex_indices_.push_back(p0);
		this->volumes_vertex_indices_.push_back(p1);
		this->volumes_vertex_indices_.push_back(p2);
		this->volumes_vertex_indices_.push_back(p3);
	}

	void add_pyramid(uint32 p0, uint32 p1, uint32 p2, uint32 p3, uint32 p4)
	{
		this->volumes_types_.push_back(VolumeType::Pyramid);
		this->volumes_vertex_indices_.push_back(p0);
		this->volumes_vertex_indices_.push_back(p1);
		this->volumes_vertex_indices_.push_back(p2);
		this->volumes_vertex_indices_.push_back(p3);
		this->volumes_vertex_indices_.push_back(p4);
	}

	void add_triangular_prism(uint32 p0, uint32 p1, uint32 p2, uint32 p3, uint32 p4, uint32 p5)
	{
		this->volumes_types_.push_back(VolumeType::TriangularPrism);
		this->volumes_vertex_indices_.push_back(p0);
		this->volumes_vertex_indices_.push_back(p1);
		this->volumes_vertex_indices_.push_back(p2);
		this->volumes_vertex_indices_.push_back(p3);
		this->volumes_vertex_indices_.push_back(p4);
		this->volumes_vertex_indices_.push_back(p5);
	}

	void add_connector(uint32 p0, uint32 p1, uint32 p2, uint32 p3)
	{
		this->volumes_types_.push_back(VolumeType::Connector);
		this->volumes_vertex_indices_.push_back(p0);
		this->volumes_vertex_indices_.push_back(p1);
		this->volumes_vertex_indices_.push_back(p2);
		this->volumes_vertex_indices_.push_back(p3);
	}

	inline uint32 nb_volumes() const
	{
		return uint32(volumes_types_.size());
	}

	void create_map()
	{
		if (nb_volumes() == 0u)
			return;

		mbuild_.template create_embedding<Vertex::ORBIT>();
		if (volume_container().nb_chunk_arrays() > 0)
			mbuild_.template create_embedding<Volume::ORBIT>();

		auto darts_per_vertex = map_.template add_attribute<std::vector<Dart>, Vertex>("darts_per_vertex");

		uint32 index = 0u;
		typename MAP::DartMarker dart_marker(map_);
		Dart d;
		uint32 vol_emb = 0u;

		// for each volume of table
		for (uint32 i = 0u, end = this->nb_volumes(); i < end; ++i)
		{
			// store volume in buffer, removing degenated faces
			const VolumeType vol_type = this->volumes_types_[i];

			if (vol_type == VolumeType::Tetra) // tetrahedral case
			{
				d = mbuild_.add_pyramid_topo_fp(3u);

				// check if add ok (special maps)
				if (d.is_nil()) break;

				const std::array<Dart, 4> vertices_of_tetra = {
					d,
					map_.phi1(d),
					map_.phi_1(d),
					map_.phi_1(map_.phi2(map_.phi_1(d)))
				};

				for (const Dart& dv : vertices_of_tetra)
				{
					const uint32 emb = this->volumes_vertex_indices_[index++];
					mbuild_.template set_orbit_embedding<Vertex>(Vertex2(dv), emb);

					Dart dd = dv;
					do
					{
						dart_marker.mark(dd);
						darts_per_vertex[emb].push_back(dd);
						dd = map_.phi1(map_.phi2(dd));
					} while(dd != dv);
				}
			}
			else if (vol_type == VolumeType::Pyramid) // pyramidal case
			{
				d = mbuild_.add_pyramid_topo_fp(4u);

				// check if add ok (special maps)
				if (d.is_nil()) break;

				const std::array<Dart, 5> vertices_of_pyramid = {
					d,
					map_.phi1(d),
					map_.phi1(map_.phi1(d)),
					map_.phi_1(d),
					map_.phi_1(map_.phi2(map_.phi_1(d)))
				};

				for (Dart dv : vertices_of_pyramid)
				{
					const uint32 emb = this->volumes_vertex_indices_[index++];
					mbuild_.template set_orbit_embedding<Vertex>(Vertex2(dv), emb);

					Dart dd = dv;
					do
					{
						dart_marker.mark(dd);
						darts_per_vertex[emb].push_back(dd);
						dd = map_.phi1(map_.phi2(dd));
					} while(dd != dv);
				}
			}
			else if (vol_type == VolumeType::TriangularPrism) // prism case
			{
				d = mbuild_.add_prism_topo_fp(3u);

				// check if add ok (special maps)
				if (d.is_nil()) break;

				const std::array<Dart, 6> vertices_of_prism = {
					d,
					map_.phi1(d),
					map_.phi_1(d),
					map_.phi2(map_.phi1(map_.phi1(map_.phi2(map_.phi_1(d))))),
					map_.phi2(map_.phi1(map_.phi1(map_.phi2(d)))),
					map_.phi2(map_.phi1(map_.phi1(map_.phi2(map_.phi1(d)))))
				};

				for (Dart dv : vertices_of_prism)
				{
					const uint32 emb = this->volumes_vertex_indices_[index++];
					mbuild_.template set_orbit_embedding<Vertex>(Vertex2(dv), emb);

					Dart dd = dv;
					do
					{
						dart_marker.mark(dd);
						darts_per_vertex[emb].push_back(dd);
						dd = map_.phi1(map_.phi2(dd));
					} while(dd != dv);
				}
			}
			else if (vol_type == VolumeType::Hexa) // hexahedral case
			{
				d = mbuild_.add_prism_topo_fp(4u);

				// check if add ok (special maps)
				if (d.is_nil()) break;

				const std::array<Dart, 8> vertices_of_hexa = {
					d,
					map_.phi1(d),
					map_.phi1(map_.phi1(d)),
					map_.phi_1(d),
					map_.phi2(map_.phi1(map_.phi1(map_.phi2(map_.phi_1(d))))),
					map_.phi2(map_.phi1(map_.phi1(map_.phi2(d)))),
					map_.phi2(map_.phi1(map_.phi1(map_.phi2(map_.phi1(d))))),
					map_.phi2(map_.phi1(map_.phi1(map_.phi2(map_.phi1(map_.phi1(d))))))
				};

				for (Dart dv : vertices_of_hexa)
				{
					const uint32 emb = this->volumes_vertex_indices_[index++];
					mbuild_.template set_orbit_embedding<Vertex>(Vertex2(dv), emb);

					Dart dd = dv;
					do
					{
						dart_marker.mark(dd);
						darts_per_vertex[emb].push_back(dd);
						dd = map_.phi1(map_.phi2(dd));
					} while(dd != dv);
				}
			}
			else //end of hexa
			{
				if (vol_type == VolumeType::Connector)
				{
					index += 4u;
					// The second part of the code generates connectors automatically. We don't have to do anything here.
				}
			}
			if (map_.is_embedded(Volume::ORBIT))
				mbuild_.template set_orbit_embedding<Volume>(Volume(d), vol_emb++);
		}

		// reconstruct neighbourhood
		uint32 nb_boundary_faces = 0u;

		map_.foreach_dart([&] (Dart d)
		{
			if (dart_marker.is_marked(d))
			{
				Dart good_dart;

				// 1st step : for every dart of the face we try to find a valid phi3 candidate. If we can't it's a boundary face.
				{
					Dart d_it = d;
					do
					{
						const std::vector<Dart>& vec = darts_per_vertex[Vertex(map_.phi1(d_it))];
						for (auto it = vec.begin(); it != vec.end() && good_dart.is_nil(); ++it)
						{
							if (map_.embedding(Vertex(map_.phi1(*it))) == map_.embedding(Vertex(d_it)) &&
								map_.embedding(Vertex(map_.phi_1(*it))) == map_.embedding(Vertex(map_.phi1(map_.phi1(d_it)))))
							{
								good_dart = *it;
							}
						}
						d_it = map_.phi1(d_it);
					} while (good_dart.is_nil() && (d_it != d));
					d = map_.phi_1(d_it);
				}

				if (!good_dart.is_nil()) //not a boundary faces
				{
					const uint32 degD = map_.codegree(Face(d));
					const uint32 degGD = map_.codegree(Face(good_dart));

					if (degD == degGD) // normal case : the two opposite faces have the same degree
					{
						mbuild_.sew_volumes_fp(d, good_dart);
						dart_marker.unmark_orbit(Face(d));
					}
					else
					{
						// there is one face of degree 4 and one face of degree 3.
						if (degD > degGD) // face of d is quad
						{
							const Dart another_d = map_.phi1(map_.phi1(d));
							const std::vector<Dart>& vec = darts_per_vertex[Vertex(map_.phi_1(d))];

							Dart another_good_dart;
							for (auto it = vec.begin(); it != vec.end() && another_good_dart.is_nil(); ++it)
							{
								if (map_.embedding(Vertex(map_.phi1(*it))) == map_.embedding(Vertex(another_d)) &&
									map_.embedding(Vertex(map_.phi_1(*it))) == map_.embedding(Vertex(map_.phi1(map_.phi1(another_d)))))
								{
									another_good_dart = *it ;
								}
							}

							// we add a stamp volume between the faces
							const Dart d_quad = mbuild_.add_stamp_volume_topo_fp();
							{
								if (map_.is_embedded(Volume::ORBIT))
									mbuild_. new_orbit_embedding(Volume(d_quad));
								Dart q1_it = d;
								Dart q2_it = map_.phi_1(d_quad);
								do
								{
									mbuild_.template set_orbit_embedding<Vertex>(Vertex2(q2_it), map_.embedding(Vertex(q1_it)));
									q1_it = map_.phi1(q1_it);
									q2_it = map_.phi_1(q2_it);
								} while (q1_it != d);
							}

							mbuild_.sew_volumes_fp(d, map_.phi1(map_.phi1(d_quad)));
							dart_marker.unmark_orbit(Face(d));

							mbuild_.sew_volumes_fp(good_dart, map_.phi2(map_.phi1(map_.phi1(d_quad))));
							dart_marker.unmark_orbit(Face(good_dart));

							if (!another_good_dart.is_nil())
							{
								mbuild_.sew_volumes_fp(another_good_dart, map_.phi2(d_quad));
								dart_marker.unmark_orbit(Face(another_good_dart));
							}
							else
							{
								dart_marker.unmark_orbit(Face2(map_.phi2(d_quad)));
								++nb_boundary_faces;
							}
						}
						else // face of d is tri
						{
							const Dart another_dart = map_.phi_1(d);
							std::vector<Dart>& vec = darts_per_vertex[Vertex(d)];

							Dart another_good_dart;
							for (auto it = vec.begin(); it != vec.end() && another_good_dart.is_nil(); ++it)
							{
								if (map_.embedding(Vertex(map_.phi1(*it))) == map_.embedding(Vertex(another_dart)) &&
									map_.embedding(Vertex(map_.phi_1(*it))) == map_.embedding(Vertex(map_.phi1(map_.phi1(good_dart)))))
								{
									another_good_dart = *it ;
								}
							}

							const Dart d_quad = mbuild_.add_stamp_volume_topo_fp();
							{
								if (map_.is_embedded(Volume::ORBIT))
									mbuild_.new_orbit_embedding(Volume(d_quad));
								Dart q1_it = good_dart;
								Dart q2_it = d_quad;
								do
								{
									mbuild_.template set_orbit_embedding<Vertex>(Vertex2(q2_it), map_.embedding(Vertex(q1_it)));
									q1_it = map_.phi1(q1_it);
									q2_it = map_.phi_1(q2_it);
								} while (q1_it != good_dart);
							}

							mbuild_.sew_volumes_fp(d_quad, map_.phi_1(good_dart));
							dart_marker.unmark_orbit(Face(good_dart));

							mbuild_.sew_volumes_fp(d, map_.phi2(map_.phi_1(d_quad)));
							dart_marker.unmark_orbit(Face(d));

							if (!another_good_dart.is_nil())
							{
								mbuild_.sew_volumes_fp(another_good_dart, map_.phi1(map_.phi2(map_.phi1(d_quad))));
								dart_marker.unmark_orbit(Face(another_good_dart));
							}
							else
							{
								dart_marker.unmark_orbit(Face2(map_.phi1(map_.phi2(map_.phi1(d_quad)))));
								++nb_boundary_faces;
							}
						}
					}
				}
				else
				{
					dart_marker.unmark_orbit(Face2(d));
					++nb_boundary_faces;
				}
			}
		});

		if (nb_boundary_faces > 0)
		{
			mbuild_.close_map();
			cgogn_log_info("create_map") << "Map closed with " << nb_boundary_faces << " boundary face(s).";
		}

		map_.template enforce_unique_orbit_embedding<Vertex::ORBIT>();

		map_.remove_attribute(darts_per_vertex);

		cgogn_assert(map_.template is_well_embedded<Vertex>());
		if (map_.template is_embedded<Volume::ORBIT>())
		{
			cgogn_assert(map_.template is_well_embedded<Volume>());
		}
	}

protected:

	template <typename T>
	inline void reorient_hexa(const ChunkArray<T>& pos, uint32& p0, uint32& p1, uint32& p2, uint32& p3, uint32& p4, uint32& p5, uint32& p6, uint32& p7)
	{
		if (geometry::test_orientation_3D<T>(pos[p4], pos[p0], pos[p1], pos[p2]) == geometry::Orientation3D::OVER)
		{
			std::swap(p0, p3);
			std::swap(p1, p2);
			std::swap(p4, p7);
			std::swap(p5, p6);
		}
	}

	template <typename T>
	inline void reorient_tetra(const ChunkArray<T>& pos, uint32& p0, uint32& p1, uint32& p2, uint32& p3)
	{
		if (geometry::test_orientation_3D<T>(pos[p0], pos[p1], pos[p2], pos[p3]) == geometry::Orientation3D::OVER)
			std::swap(p1, p2);
	}

	template <typename T>
	inline void reorient_pyramid(const ChunkArray<T>& pos, uint32& p0, uint32& p1, uint32& p2, uint32& p3, uint32& p4)
	{
		if (geometry::test_orientation_3D<T>(pos[p4], pos[p0], pos[p1], pos[p2]) == geometry::Orientation3D::OVER)
			std::swap(p1, p3);
	}

	template <typename T>
	inline void reorient_triangular_prism(const ChunkArray<T>& pos, uint32& p0, uint32& p1, uint32& p2, uint32& p3, uint32& p4, uint32& p5)
	{
		if (geometry::test_orientation_3D<T>(pos[p3], pos[p0], pos[p1], pos[p2]) == geometry::Orientation3D::OVER)
		{
			std::swap(p1, p2);
			std::swap(p4, p5);
		}
	}

private:

	std::vector<VolumeType> volumes_types_;
	std::vector<uint32>     volumes_vertex_indices_;

	MAP&              map_;
	MapBuilder        mbuild_;

};

template <typename MAP>
class VolumeFileImport : public VolumeImport<MAP>, public FileImport
{
	using Self = VolumeFileImport<MAP>;
	using Inherit_Import = VolumeImport<MAP>;
	using Inherit_File = FileImport;

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(VolumeFileImport);

public:
	inline VolumeFileImport(MAP& map) : Inherit_Import(map), Inherit_File() {}
	virtual ~VolumeFileImport() {}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_VOLUME_IMPORT_CPP_))
extern template class CGOGN_IO_API VolumeImport<CMap3>;
extern template class CGOGN_IO_API VolumeFileImport<CMap3>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_VOLUME_IMPORT_CPP_))

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_VOLUME_IMPORT_H_
