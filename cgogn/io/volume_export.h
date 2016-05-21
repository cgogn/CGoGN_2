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
#define CGOGN_IO_VOLUME_IMPORT_H_

#include <ostream>

#include <cgogn/core/utils/numerics.h>
#include <cgogn/core/utils/string.h>
#include <cgogn/core/cmap/attribute.h>

#include <cgogn/io/io_utils.h>
#pragma once
namespace cgogn
{

namespace io
{

struct ExportOptions
{
	inline ExportOptions(const std::string& filename, std::vector<std::pair<Orbit, std::string>> const& attributes, bool binary) :
		filename_(filename)
	  ,binary_(binary)
	  ,attributes_to_export_(attributes)
	{}

	std::string filename_;
	bool binary_;
	std::vector<std::pair<Orbit, std::string>> attributes_to_export_;
};

template<typename MAP>
class VolumeExport
{
public:
	using Self = VolumeExport<MAP>;
	using Map = MAP;
	using Vertex = typename Map::Vertex;
	using Volume = typename Map::Volume;
	using AttributeGen = typename Map::AttributeGen;

	inline VolumeExport() :
		vertices_of_volumes_()
	  ,number_of_vertices_()
	  ,nb_tetras_(0u)
	  ,nb_pyramids_(0u)
	  ,nb_triangular_prisms_(0u)
	  ,nb_hexas_(0u)
	  ,vertex_attributes()
	  ,volume_attributes()
	  ,position_attribute()
	{}


	virtual ~VolumeExport() {}

	void export_file(Map& map, const ExportOptions& options)
	{
		this->reset();
		for (const auto& pair : options.attributes_to_export_)
		{
			if (pair.first == Vertex::ORBIT)
			{
				auto v_att = map.get_attribute_gen(pair.first, pair.second);
				if (pair.second == "position")
					position_attribute = std::move(v_att);
				else {
					if (v_att->is_valid())
						vertex_attributes.push_back(std::move(v_att));
				}
			} else {
				auto w_att = map.get_attribute_gen(pair.first, pair.second);
				if (w_att->is_valid())
					volume_attributes.push_back(std::move(w_att));
			}
		}

		if (position_attribute == nullptr || !position_attribute->is_valid())
		{
			cgogn_log_warning("VolumeExport::export_file") << "The position attribute is invalid.";
			return;
		}

		auto output = io::create_file(options.filename_);
		indices_ = map.template add_attribute<uint32,Vertex::ORBIT>("indices_vert");
		this->prepare_for_export(map);
		this->export_file_impl(map,*output, options);
		map.remove_attribute(indices_);
	}

	protected:

	virtual void export_file_impl(const Map& map, std::ofstream& output, const ExportOptions& options) = 0;

	inline uint32 get_nb_tetras() const
	{
		return nb_tetras_;
	}

	inline uint32 get_nb_pyramids() const
	{
		return nb_pyramids_;
	}

	inline uint32 get_nb_triangular_prisms() const
	{
		return nb_triangular_prisms_;
	}

	inline uint32 get_nb_hexas() const
	{
		return nb_hexas_;
	}

	inline std::vector<uint32> const & get_vertices_of_volumes() const
	{
		return vertices_of_volumes_;
	}

	inline std::vector<uint32> const & get_number_of_vertices() const
	{
		return number_of_vertices_;
	}

	inline std::vector<std::unique_ptr<AttributeGen>> const & get_vertex_attributes() const
	{
		return vertex_attributes;
	}

	inline std::vector<std::unique_ptr<AttributeGen>> const & get_volume_attributes() const
	{
		return volume_attributes;
	}

	AttributeGen const * get_position_attribute() const
	{
		return position_attribute.get();
	}
private:
	void prepare_for_export(Map& map)
	{
		number_of_vertices_.reserve(map.template nb_cells<Volume::ORBIT>());
		vertices_of_volumes_.reserve(4u* number_of_vertices_.capacity());

		uint32 count{0u};
		map.foreach_cell([&](Vertex v) { indices_[v] = count++;} );

		map.foreach_cell([&](Volume w)
		{
			uint32 nb_vert{0u};
			map.foreach_incident_vertex(w, [&nb_vert](Vertex) {++nb_vert;});
			Dart it = w.dart;

			if (nb_vert == 4u)
			{
				number_of_vertices_.push_back(4u);
				++nb_tetras_;
				vertices_of_volumes_.push_back(indices_[Vertex(it)]);
				it = map.phi1(it);
				vertices_of_volumes_.push_back(indices_[Vertex(it)]);
				it = map.phi1(it);
				vertices_of_volumes_.push_back(indices_[Vertex(it)]);
				it = map.template phi<211>(it);
				vertices_of_volumes_.push_back(indices_[Vertex(it)]);
			} else {
				if (nb_vert == 5u)
				{
					number_of_vertices_.push_back(5u);
					++nb_pyramids_;
					vertices_of_volumes_.push_back(indices_[Vertex(it)]);
					it = map.phi1(it);
					vertices_of_volumes_.push_back(indices_[Vertex(it)]);
					it = map.phi1(it);
					vertices_of_volumes_.push_back(indices_[Vertex(it)]);
					it = map.phi1(it);
					vertices_of_volumes_.push_back(indices_[Vertex(it)]);
					it = map.template phi<212>(it);
					vertices_of_volumes_.push_back(indices_[Vertex(it)]);
				} else {
					if (nb_vert == 6u)
					{
						number_of_vertices_.push_back(6u);
						++nb_triangular_prisms_;
						vertices_of_volumes_.push_back(indices_[Vertex(it)]);
						it = map.phi1(it);
						vertices_of_volumes_.push_back(indices_[Vertex(it)]);
						it = map.phi1(it);
						vertices_of_volumes_.push_back(indices_[Vertex(it)]);
						it = map.template phi<21121>(w.dart);
						vertices_of_volumes_.push_back(indices_[Vertex(it)]);
						it = map.phi_1(it);
						vertices_of_volumes_.push_back(indices_[Vertex(it)]);
						it = map.phi_1(it);
						vertices_of_volumes_.push_back(indices_[Vertex(it)]);
					} else {
						if (nb_vert == 8u)
						{
							number_of_vertices_.push_back(8u);
							++nb_hexas_;
							vertices_of_volumes_.push_back(indices_[Vertex(it)]);
							it = map.phi_1(it);
							vertices_of_volumes_.push_back(indices_[Vertex(it)]);
							it = map.phi_1(it);
							vertices_of_volumes_.push_back(indices_[Vertex(it)]);
							it = map.phi_1(it);
							vertices_of_volumes_.push_back(indices_[Vertex(it)]);
							it = map.template phi<21121>(w.dart);
							vertices_of_volumes_.push_back(indices_[Vertex(it)]);
							it = map.phi1(it);
							vertices_of_volumes_.push_back(indices_[Vertex(it)]);
							it = map.phi1(it);
							vertices_of_volumes_.push_back(indices_[Vertex(it)]);
							it = map.phi1(it);
							vertices_of_volumes_.push_back(indices_[Vertex(it)]);
						} else {
							cgogn_log_warning("VolumeExport::prepare_for_export") << "Unknown volume with " << nb_vert << " vertices. Ignoring.";
						}
					}
				}
			}
		});
	}

	void reset()
	{
		vertices_of_volumes_.clear();
		number_of_vertices_.clear();
		nb_tetras_ = 0u;
		nb_pyramids_ = 0u;
		nb_triangular_prisms_ = 0u;
		nb_hexas_ = 0u;
		vertex_attributes.clear();
		volume_attributes.clear();
		position_attribute = nullptr;
	}

	std::vector<uint32>			vertices_of_volumes_;
	std::vector<uint32>			number_of_vertices_;
	typename Map::template Attribute<uint32, Vertex::ORBIT> indices_;
	uint32 nb_tetras_;
	uint32 nb_pyramids_;
	uint32 nb_triangular_prisms_;
	uint32 nb_hexas_;
	std::vector<std::unique_ptr<AttributeGen>>	vertex_attributes;
	std::vector<std::unique_ptr<AttributeGen>>	volume_attributes;
	std::unique_ptr<AttributeGen>				position_attribute;
};

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_VOLUME_EXPORT_H_
