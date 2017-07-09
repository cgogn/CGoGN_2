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

#ifndef CGOGN_IO_VOLUME_EXPORT_H_
#define CGOGN_IO_VOLUME_EXPORT_H_

#include <ostream>

#include <cgogn/core/utils/numerics.h>
#include <cgogn/core/utils/string.h>
#include <cgogn/core/cmap/cmap3.h>
#include <cgogn/io/io_utils.h>
#include <cgogn/io/mesh_io_gen.h>

namespace cgogn
{

namespace io
{

template <typename MAP>
class VolumeExport : public MeshExport<MAP>
{
public:

	using Inherit = MeshExport<MAP>;
	using Self = VolumeExport<MAP>;
	using Map = MAP;
	using Vertex = typename Map::Vertex;
	using Edge = typename Map::Edge;
	using Face   = typename Map::Face;
	using Volume = typename Map::Volume;	
	using ChunkArrayGen = typename Map::ChunkArrayGen;
	using ChunkArrayContainer = typename Map::template ChunkArrayContainer<uint32>;

	inline VolumeExport() :
		vertices_of_volumes_(),
		nb_tetras_(0u),
		nb_pyramids_(0u),
		nb_triangular_prisms_(0u),
		nb_hexas_(0u),
		volume_attributes_()
	{}

	virtual ~VolumeExport() override {}

protected:

	virtual void prepare_for_export(Map& map, const ExportOptions& options) override
	{
		const ChunkArrayContainer& ver_cac = map.template attribute_container<Vertex::ORBIT>();
		const ChunkArrayContainer& edge_cac = map.template attribute_container<Edge::ORBIT>();
		const ChunkArrayContainer& face_cac = map.template attribute_container<Face::ORBIT>();
		const ChunkArrayContainer& vol_cac = map.template attribute_container<Volume::ORBIT>();

		for(const auto& pair : options.position_attributes_)
		{
			const ChunkArrayGen* pos_cag = nullptr;

			if(pair.first == Edge::ORBIT)
				pos_cag = edge_cac.get_chunk_array(pair.second);
			else if(pair.first == Face::ORBIT)
				pos_cag = face_cac.get_chunk_array(pair.second);
			else if(pair.first == Volume::ORBIT)
				pos_cag = vol_cac.get_chunk_array(pair.second);

			if(pos_cag)
				this->position_attributes_.insert(std::make_pair(pair.first, pos_cag));
		}

		vertices_of_volumes_ = map.template add_attribute<std::vector<uint32>, Volume>("vertices_of_volume_volume_export");

		for (const auto& pair : options.attributes_to_export_)
		{
			if (pair.first == Vertex::ORBIT)
			{
				const ChunkArrayGen* ver_cag = ver_cac.get_chunk_array(pair.second);
				if (ver_cag)
					this->vertex_attributes_.push_back(ver_cag);
			}
			else
			{
				const ChunkArrayGen* vol_cag = vol_cac.get_chunk_array(pair.second);
				if (vol_cag)
					volume_attributes_.push_back(vol_cag);
			}
		}

		std::function<bool(Volume)> volume_validator = [&](Volume w) -> bool
		{
			// we want to ignore the "connector" cells that are sometime added. Their codegree is 3.
			return map.codegree(w) != 3u && (!options.cell_filter_ || options.cell_filter_(w.dart));
		};

		this->cell_cache_->template build<Volume>(volume_validator);
		this->cell_cache_->template build<Vertex>([&map, &volume_validator](Vertex v)
		{
			bool use_vertex = false;
			map.foreach_incident_volume(v, [&map, &use_vertex, &volume_validator](Volume w) -> bool
			{
				use_vertex = volume_validator(w);
				return !use_vertex; // stop when use_vertex == true
			});
			return use_vertex;
		});

		uint32 count{0u};
		map.foreach_cell(
			[&] (Vertex v) { this->vindices_[v] = count++; },
			*(this->cell_cache_)
		);

		const auto& ids = this->vindices_;
		map.foreach_cell([&] (Volume w)
		{
			uint32 nb_vert{0u};
			map.foreach_incident_vertex(w, [&nb_vert](Vertex) {++nb_vert;});
			Dart it = w.dart;

			std::vector<uint32>& vertices = vertices_of_volumes_[w];

			if (nb_vert == 4u)
			{
				++nb_tetras_;
				vertices.push_back(ids[Vertex(it)]);
				it = map.phi1(it);
				vertices.push_back(ids[Vertex(it)]);
				it = map.phi1(it);
				vertices.push_back(ids[Vertex(it)]);
				it = map.phi_1(map.phi2(it));
				vertices.push_back(ids[Vertex(it)]);
			}
			else
			{
				if (nb_vert == 5u)
				{
					++nb_pyramids_;
					Face fit(it);
					while(map.codegree(fit) != 4u)
					{
						it = map.phi1(it);
						fit = Face(map.phi2(it));
					}
					it = fit.dart;
					vertices.push_back(ids[Vertex(it)]);
					it = map.phi1(it);
					vertices.push_back(ids[Vertex(it)]);
					it = map.phi1(it);
					vertices.push_back(ids[Vertex(it)]);
					it = map.phi1(it);
					vertices.push_back(ids[Vertex(it)]);
					it = map.phi_1(map.phi2(it));
					vertices.push_back(ids[Vertex(it)]);
				}
				else
				{
					if (nb_vert == 6u)
					{
						++nb_triangular_prisms_;
						Face fit(it);
						while(map.codegree(fit) != 3u)
						{
							it = map.phi1(it);
							fit = Face(map.phi2(it));
						}
						it = fit.dart;
						w.dart = it;
						vertices.push_back(ids[Vertex(it)]);
						it = map.phi_1(it);
						vertices.push_back(ids[Vertex(it)]);
						it = map.phi_1(it);
						vertices.push_back(ids[Vertex(it)]);
						it = map.template phi<21121>(w.dart);
						vertices.push_back(ids[Vertex(it)]);
						it = map.phi1(it);
						vertices.push_back(ids[Vertex(it)]);
						it = map.phi1(it);
						vertices.push_back(ids[Vertex(it)]);
					}
					else
					{
						if (nb_vert == 8u)
						{
							++nb_hexas_;
							vertices.push_back(ids[Vertex(it)]);
							it = map.phi_1(it);
							vertices.push_back(ids[Vertex(it)]);
							it = map.phi_1(it);
							vertices.push_back(ids[Vertex(it)]);
							it = map.phi_1(it);
							vertices.push_back(ids[Vertex(it)]);
							it = map.template phi<21121>(w.dart);
							vertices.push_back(ids[Vertex(it)]);
							it = map.phi1(it);
							vertices.push_back(ids[Vertex(it)]);
							it = map.phi1(it);
							vertices.push_back(ids[Vertex(it)]);
							it = map.phi1(it);
							vertices.push_back(ids[Vertex(it)]);
						}
						else
								cgogn_log_warning("VolumeExport::prepare_for_export") << "Unknown volume with " << nb_vert << " vertices. Ignoring.";
					}
				}
			}
		},
		*(this->cell_cache_));
	}

	void clean_added_attributes(Map& map) override
	{
		Inherit::clean_added_attributes(map);
		map.remove_attribute(vertices_of_volumes_);
	}

	inline uint32 nb_tetras() const
	{
		return nb_tetras_;
	}

	inline uint32 nb_pyramids() const
	{
		return nb_pyramids_;
	}

	inline uint32 nb_triangular_prisms() const
	{
		return nb_triangular_prisms_;
	}

	inline uint32 nb_hexas() const
	{
		return nb_hexas_;
	}

	inline uint32 nb_volumes() const
	{
		return uint32(this->cell_cache_->template size<Volume>());
	}

	inline uint32 nb_vertices() const
	{
		return uint32(this->cell_cache_->template size<Vertex>());
	}

	inline std::vector<uint32> const & vertices_of_volumes(Volume w) const
	{
		return vertices_of_volumes_[w];
	}

	inline std::size_t number_of_vertices(Volume w) const
	{
		return vertices_of_volumes_[w].size();
	}

	inline std::vector<const ChunkArrayGen*> const & volume_attributes() const
	{
		return volume_attributes_;
	}

private:

	void reset() override
	{
		Inherit::reset();
		if (vertices_of_volumes_.is_valid())
			vertices_of_volumes_.set_all_values(std::vector<uint32>());
		nb_tetras_ = 0u;
		nb_pyramids_ = 0u;
		nb_triangular_prisms_ = 0u;
		nb_hexas_ = 0u;
		volume_attributes_.clear();
	}

	Attribute<std::vector<uint32>, Volume::ORBIT> vertices_of_volumes_;
	uint32 nb_tetras_;
	uint32 nb_pyramids_;
	uint32 nb_triangular_prisms_;
	uint32 nb_hexas_;
	std::vector<const ChunkArrayGen*>	volume_attributes_;
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_VOLUME_EXPORT_CPP_))
extern template class CGOGN_IO_API VolumeExport<CMap3>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_VOLUME_EXPORT_CPP_))

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_VOLUME_EXPORT_H_
