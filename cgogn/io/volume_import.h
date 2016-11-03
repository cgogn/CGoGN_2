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

#include <cgogn/core/cmap/cmap3_builder.h>
#include <cgogn/core/cmap/map_base_data.h>

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

template <typename VEC3>
class VolumeImport
{
public:

	using Self = VolumeImport<VEC3>;

	using ChunkArrayContainer = MapBaseData::ChunkArrayContainer<uint32>;
	using ChunkArrayGen = MapBaseData::ChunkArrayGen;
	template <typename T>
	using ChunkArray = MapBaseData::ChunkArray<T>;

	template <typename T, Orbit ORBIT>
	using Attribute = Attribute<T, ORBIT>;

	using DataInputGen = cgogn::io::DataInputGen;

	inline VolumeImport():
		volumes_types_()
	  ,volumes_vertex_indices_()
	  ,vertex_attributes_()
	  ,volume_attributes_()
	  ,position_attribute_(nullptr)
	{}

	virtual ~VolumeImport()
	{}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(VolumeImport);

	inline void reserve(uint32 nb_volumes)
	{
		volumes_types_.reserve(nb_volumes);
		volumes_vertex_indices_.reserve(8u * nb_volumes);
	}

	inline void add_vertex_attribute(const DataInputGen& in_data, const std::string& att_name)
	{
		ChunkArrayGen* att = in_data.add_attribute(vertex_attributes_, att_name);
		in_data.to_chunk_array(att);
		if (att_name == "position")
			position_attribute_ = dynamic_cast<ChunkArray<VEC3>*>(att);
	}

	template<typename T>
	inline ChunkArray<T>* add_vertex_attribute(const std::string& att_name)
	{
		return vertex_attributes_.template add_chunk_array<T>(att_name);
	}

	inline void add_volume_attribute(const DataInputGen& in_data, const std::string& att_name)
	{
		in_data.to_chunk_array(in_data.add_attribute(volume_attributes_, att_name));
	}

	template<typename T>
	inline ChunkArray<T>* add_volume_attribute(const std::string& att_name)
	{
		return volume_attributes_.template add_chunk_array<T>(att_name);
	}

	inline ChunkArray<VEC3>* position_attribute()
	{
		if (position_attribute_ == nullptr)
			return (position_attribute_ =  add_vertex_attribute<VEC3>("position"));
		else
			return position_attribute_;
	}

	inline uint32 insert_line_vertex_container()
	{
		return vertex_attributes_.template insert_lines<1>();
	}

	inline uint32 insert_line_volume_container()
	{
		return volume_attributes_.template insert_lines<1>();
	}

	template <typename Map>
	bool create_map(Map& map)
	{
		static_assert(Map::DIMENSION == 3, "must use map of dim 3 in volume import");

		using Vertex = typename Map::Vertex;
		using Vertex2 = typename Map::Vertex2;
		using Volume = typename Map::Volume;
		using Face = typename Map::Face;
		using Face2 = typename Map::Face2;
		using MapBuilder = typename Map::Builder;

		if (nb_volumes() == 0u)
			return false;

		MapBuilder mbuild(map);
		map.clear_and_remove_attributes();

		mbuild.template create_embedding<Vertex::ORBIT>();
		mbuild.template swap_chunk_array_container<Vertex::ORBIT>(this->vertex_attributes_);

		if (this->volume_attributes_.nb_chunk_arrays() > 0)
		{
			mbuild.template create_embedding<Volume::ORBIT>();
			mbuild.template swap_chunk_array_container<Volume::ORBIT>(this->volume_attributes_);
		}

		auto darts_per_vertex = map.template add_attribute<std::vector<Dart>, Vertex>("darts_per_vertex");

		uint32 index = 0u;
		typename Map::DartMarker dart_marker(map);
		Dart d;
		uint32 vol_emb = 0u;

		//for each volume of table
		for (uint32 i = 0u, end = this->nb_volumes(); i < end; ++i)
		{
			// store volume in buffer, removing degenated faces
			const VolumeType vol_type = this->volumes_types_[i];

			if (vol_type == VolumeType::Tetra) //tetrahedral case
			{
				d = mbuild.add_pyramid_topo_fp(3u);

				// check if add ok (special maps)
				if (d.is_nil()) break;

				const std::array<Dart, 4> vertices_of_tetra = {
					d,
					map.phi1(d),
					map.phi_1(d),
					map.phi_1(map.phi2(map.phi_1(d)))
				};

				for (const Dart& dv : vertices_of_tetra)
				{
					const uint32 emb = this->volumes_vertex_indices_[index++];
					mbuild.template set_orbit_embedding<Vertex>(Vertex2(dv), emb);

					Dart dd = dv;
					do
					{
						dart_marker.mark(dd);
						darts_per_vertex[emb].push_back(dd);
						dd = map.phi1(map.phi2(dd));
					} while(dd != dv);
				}
			}
			else if (vol_type == VolumeType::Pyramid) //pyramidal case
			{
				d = mbuild.add_pyramid_topo_fp(4u);

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
					mbuild.template set_orbit_embedding<Vertex>(Vertex2(dv), emb);

					Dart dd = dv;
					do
					{
						dart_marker.mark(dd);
						darts_per_vertex[emb].push_back(dd);
						dd = map.phi1(map.phi2(dd));
					} while(dd != dv);
				}
			}
			else if (vol_type == VolumeType::TriangularPrism) //prism case
			{
				d = mbuild.add_prism_topo_fp(3u);

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
					mbuild.template set_orbit_embedding<Vertex>(Vertex2(dv), emb);

					Dart dd = dv;
					do
					{
						dart_marker.mark(dd);
						darts_per_vertex[emb].push_back(dd);
						dd = map.phi1(map.phi2(dd));
					} while(dd != dv);
				}
			}
			else if (vol_type == VolumeType::Hexa) //hexahedral case
			{
				d = mbuild.add_prism_topo_fp(4u);

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
					mbuild.template set_orbit_embedding<Vertex>(Vertex2(dv), emb);

					Dart dd = dv;
					do
					{
						dart_marker.mark(dd);
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
			if (map.is_embedded(Volume::ORBIT))
				mbuild. template set_orbit_embedding<Volume>(Volume(d), vol_emb++);
		}

		//reconstruct neighbourhood
		uint32 nb_boundary_faces = 0u;
		map.foreach_dart([&] (Dart d)
		{
			if (dart_marker.is_marked(d))
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
						mbuild.sew_volumes_fp(d, good_dart);
						dart_marker.unmark_orbit(Face(d));
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
							const Dart d_quad = mbuild.add_stamp_volume_topo_fp();
							{
								if (map.is_embedded(Volume::ORBIT))
									mbuild. new_orbit_embedding(Volume(d_quad));
								Dart q1_it = d;
								Dart q2_it = map.phi_1(d_quad);
								do
								{
									mbuild.template set_orbit_embedding<Vertex>(Vertex2(q2_it), map.embedding(Vertex(q1_it)));
									q1_it = map.phi1(q1_it);
									q2_it = map.phi_1(q2_it);
								} while (q1_it != d);
							}

							mbuild.sew_volumes_fp(d, map.phi1(map.phi1(d_quad)));
							dart_marker.unmark_orbit(Face(d));

							mbuild.sew_volumes_fp(good_dart, map.phi2(map.phi1(map.phi1(d_quad))));
							dart_marker.unmark_orbit(Face(good_dart));

							if (!another_good_dart.is_nil())
							{
								mbuild.sew_volumes_fp(another_good_dart, map.phi2(d_quad));
								dart_marker.unmark_orbit(Face(another_good_dart));
							}
							else
							{
								dart_marker.unmark_orbit(Face2(map.phi2(d_quad)));
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

							const Dart d_quad = mbuild.add_stamp_volume_topo_fp();
							{
								if (map.is_embedded(Volume::ORBIT))
									mbuild. new_orbit_embedding(Volume(d_quad));
								Dart q1_it = good_dart;
								Dart q2_it = d_quad;
								do
								{
									mbuild.template set_orbit_embedding<Vertex>(Vertex2(q2_it), map.embedding(Vertex(q1_it)));
									q1_it = map.phi1(q1_it);
									q2_it = map.phi_1(q2_it);
								} while (q1_it != good_dart);
							}

							mbuild.sew_volumes_fp(d_quad, map.phi_1(good_dart));
							dart_marker.unmark_orbit(Face(good_dart));

							mbuild.sew_volumes_fp(d, map.phi2(map.phi_1(d_quad)));
							dart_marker.unmark_orbit(Face(d));

							if (!another_good_dart.is_nil())
							{
								mbuild.sew_volumes_fp(another_good_dart, map.phi1(map.phi2(map.phi1(d_quad))));
								dart_marker.unmark_orbit(Face(another_good_dart));
							}
							else
							{
								dart_marker.unmark_orbit(Face2(map.phi1(map.phi2(map.phi1(d_quad)))));
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
			mbuild.close_map();
			cgogn_log_info("create_map") << "Map closed with " << nb_boundary_faces << " boundary face(s).";
		}

		map.template enforce_unique_orbit_embedding<Vertex::ORBIT>();

		map.remove_attribute(darts_per_vertex);

		return true;
	}

protected:

	virtual void clear()
	{
		volumes_types_.clear();
		volumes_vertex_indices_.clear();
		vertex_attributes_.remove_chunk_arrays();
		volume_attributes_.remove_chunk_arrays();
		position_attribute_ = nullptr;
	}

	inline void reoriente_hexa(uint32& p0, uint32& p1, uint32& p2, uint32& p3, uint32& p4, uint32& p5, uint32& p6, uint32& p7)
	{
		const auto& pos = *position_attribute_;
		if (geometry::test_orientation_3D(pos[p4], pos[p0], pos[p1], pos[p2]) == geometry::Orientation3D::OVER)
		{
			std::swap(p0, p3);
			std::swap(p1, p2);
			std::swap(p4, p7);
			std::swap(p5, p6);
		}
	}

	inline void reoriente_tetra(uint32& p0, uint32& p1, uint32& p2, uint32& p3)
	{
		const auto& pos = *position_attribute_;
		if (geometry::test_orientation_3D(pos[p0], pos[p1], pos[p2], pos[p3]) == geometry::Orientation3D::OVER)
			std::swap(p1, p2);
	}

	inline void reoriente_pyramid(uint32& p0, uint32& p1, uint32& p2, uint32& p3, uint32& p4)
	{
		const auto& pos = *position_attribute_;
		if (geometry::test_orientation_3D(pos[p4], pos[p0], pos[p1], pos[p2]) == geometry::Orientation3D::OVER)
			std::swap(p1, p3);
	}

	inline void reoriente_triangular_prism(uint32& p0, uint32& p1, uint32& p2, uint32& p3, uint32& p4, uint32& p5)
	{
		const auto& pos = *position_attribute_;
		if (geometry::test_orientation_3D(pos[p3], pos[p0], pos[p1], pos[p2]) == geometry::Orientation3D::OVER)
		{
			std::swap(p1, p2);
			std::swap(p4, p5);
		}
	}

public:

	void add_hexa(uint32 p0, uint32 p1, uint32 p2, uint32 p3, uint32 p4, uint32 p5, uint32 p6, uint32 p7, bool check_orientation)
	{
		if (check_orientation)
			this->reoriente_hexa(p0, p1, p2, p3, p4, p5, p6, p7);
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

	void add_tetra(uint32 p0, uint32 p1, uint32 p2, uint32 p3, bool check_orientation)
	{
		if (check_orientation)
			this->reoriente_tetra(p0, p1, p2, p3);
		this->volumes_types_.push_back(VolumeType::Tetra);
		this->volumes_vertex_indices_.push_back(p0);
		this->volumes_vertex_indices_.push_back(p1);
		this->volumes_vertex_indices_.push_back(p2);
		this->volumes_vertex_indices_.push_back(p3);
	}

	void add_pyramid(uint32 p0, uint32 p1, uint32 p2, uint32 p3, uint32 p4, bool check_orientation)
	{
		this->volumes_types_.push_back(VolumeType::Pyramid);
		if (check_orientation)
			this->reoriente_pyramid(p0, p1, p2, p3, p4);
		this->volumes_vertex_indices_.push_back(p0);
		this->volumes_vertex_indices_.push_back(p1);
		this->volumes_vertex_indices_.push_back(p2);
		this->volumes_vertex_indices_.push_back(p3);
		this->volumes_vertex_indices_.push_back(p4);
	}

	void add_triangular_prism(uint32 p0, uint32 p1, uint32 p2, uint32 p3, uint32 p4, uint32 p5, bool check_orientation)
	{
		if (check_orientation)
			this->reoriente_triangular_prism(p0, p1, p2, p3, p4, p5);
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

private:

	inline uint32 nb_volumes() const
	{
		return uint32(volumes_types_.size());
	}

private:

	std::vector<VolumeType>	volumes_types_;
	std::vector<uint32>		volumes_vertex_indices_;

	ChunkArrayContainer vertex_attributes_;
	ChunkArrayContainer volume_attributes_;
	ChunkArray<VEC3>*	position_attribute_;
};

template <typename VEC3>
class VolumeFileImport : public VolumeImport<VEC3>, public FileImport
{
	using Self = VolumeFileImport<VEC3>;
	using Inherit1 = VolumeImport<VEC3>;
	using Inherit2 = FileImport;

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(VolumeFileImport);

public:
	inline VolumeFileImport() : Inherit1(), Inherit2()
	{}

	virtual ~VolumeFileImport()
	{}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_VOLUME_IMPORT_CPP_))
extern template class CGOGN_IO_API VolumeImport<Eigen::Vector3d>;
extern template class CGOGN_IO_API VolumeImport<Eigen::Vector3f>;
extern template class CGOGN_IO_API VolumeFileImport<Eigen::Vector3d>;
extern template class CGOGN_IO_API VolumeFileImport<Eigen::Vector3f>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_VOLUME_IMPORT_CPP_))

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_VOLUME_IMPORT_H_
