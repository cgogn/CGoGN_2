/*******************************************************************************
 * CGoGN: Combinatorial and Geometric modeling with Generic N-dimensional Maps *
 * Copyright (C) 2015, IGG Group, ICube, University of Strasbourg, France *
 *                                                                              *
 * This library is free software; you can redistribute it and/or modify it *
 * under the terms of the GNU Lesser General Public License as published by the
 ** Free Software Foundation; either version 2.1 of the License, or (at your
 ** option) any later version.
 **
 *                                                                              *
 * This library is distributed in the hope that it will be useful, but WITHOUT *
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or *
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
 * for more details. *
 *                                                                              *
 * You should have received a copy of the GNU Lesser General Public License *
 * along with this library; if not, write to the Free Software Foundation, *
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA. *
 *                                                                              *
 * Web site: http://cgogn.unistra.fr/ * Contact information: cgogn@unistra.fr
 **
 *                                                                              *
 *******************************************************************************/
#ifndef CGOGN_MODELING_HEXA_VOLUME_TILING_H_
#define CGOGN_MODELING_HEXA_VOLUME_TILING_H_

#include <cgogn/modeling/tiling/tiling.h>
#include <cgogn/core/cmap/cmap3_builder.h>

namespace cgogn
{

namespace modeling
{
/**
 * @brief The TilingHexa class allow creation of unregular grids of hexa
 *
 */
class TilingHexa : public Tiling<CMap3>
{
protected:
	using CDart   = CMap3::CDart;
	using Vertex2 = CMap3::Vertex;
	using Edge2   = CMap3::Edge;
	using Face2   = CMap3::Face;
	using Volume  = CMap3::Volume;

	CMap3::Builder builder_;

	template <typename SCELL>
	void embed_sub_cell_of_volume(std::vector<SCELL>* store)
	{
		if (map_.is_embedded<SCELL>())
			for (Volume v : volume_table_)
				if (v.is_valid())
					map_.foreach_dart_of_orbit(v, [&](Dart e)
					{
						if (!map_.is_valid_embedding(SCELL(e)))
							builder_.new_orbit_embedding(SCELL(e));
						if (store)
							store->push_back(SCELL(e));
					});
	}

	template <>
	void embed_sub_cell_of_volume<Volume>(std::vector<Volume>*)
	{
		if (map_.is_embedded<Volume>())
			for (Volume v : volume_table_)
				if (!v.is_valid()) builder_.new_orbit_embedding(v);
	}

	void compact_volume_table()
	{
		if (volume_table_.size() == nx_*ny_*nz_)
			return;
		auto it = volume_table_.begin();
		auto jt = volume_table_.begin();
		while (it != volume_table_.end())
		{
			while (!it->is_valid())
				++it;
			if (it != jt)
				*jt = *it;
			++it;
			++jt;
		}
		volume_table_.resize(jt-volume_table_.begin());
		volume_table_.shrink_to_fit();
	}

public:
	std::vector<Volume> volume_table_;

	/**
	 * @brief TilingHexa constructor (do not create grid)
	 * @param map
	 * @param x nb of hexa in x
	 * @param y nb of hexa in y
	 * @param z nb of hexa in z
	 */
	TilingHexa(CMap3& map, uint32 x, uint32 y, uint32 z) : Tiling(map, x, y, z), builder_(map) {}

	/**
	 * @brief grid3d topo
	 * @param close close the created sub-object
	 * @param fmask boolean function(i,j,k) for keeping or not the hexahedrons
	 */
	template <typename MASK>
	void grid_topo(const MASK& fmask)
	{
		volume_table_.reserve(nx_ * ny_ * nz_);
		
		for (uint32 k = 0; k < nz_; ++k)
		{
			for (uint32 j = 0; j < ny_; ++j)
			{
				for (uint32 i = 0; i < nx_; ++i)
				{
					Dart d = fmask(i, j, k) ? builder_.add_prism_topo_fp(4u) : Dart();
					if (!d.is_nil())
					{
						if (i > 0)
						{
							Volume v = volume_table_.back();
							if (v.is_valid())
							{
								Dart e1 = map_.phi<112>(v.dart);
								Dart e2 = map_.phi2(d);
								builder_.sew_volumes_fp(e1, e2);
							}
						}
						if (j > 0)
						{
							Volume v = *(volume_table_.rbegin() + nx_ - 1);
							if (v.is_valid())
							{
								Dart e1 = map_.phi<12>(v.dart);
								Dart e2 = map_.phi2(map_.phi_1(d));
								builder_.sew_volumes_fp(e1, e2);
							}
						}
						if (k > 0)
						{
							Volume v = *(volume_table_.rbegin() + nx_ * ny_ - 1);
							if (v.is_valid())
							{
								Dart e1 = map_.phi<2112>(v.dart);
								Dart e2 = d;
								builder_.sew_volumes_fp(e1, e2);
							}
						}
					}
					volume_table_.push_back(Volume(d));
				}
			}
		}
	}


	/**
	 * @brief close_grid
	 */
	void close_grid()
	{
		for (Volume v : volume_table_)
			if (v.is_valid())
				map_.foreach_dart_of_orbit(v, [&](Dart e)
				{
					if (map_.phi3(e) == e)
						builder_.boundary_mark(Volume(builder_.close_hole_topo(e)));
				});
	}

	/**
	 * @brief sew the two Z boundaries
	 */
	inline void self_sew_z()
	{
		uint32 last = nx_*ny_*(nz_-1);
		for (uint32 j = 0; j < ny_; ++j)
		{
			for (uint32 i = 0; i < nx_; ++i)
			{
				Dart d1 = volume_table_[j*nx_+i].dart;
				Dart d2 = map_.phi<2112>(volume_table_[last+j*nx_+i].dart);
				if (!d1.is_nil() && !d2.is_nil())
				{
					builder_.sew_volumes_fp(d1, d2);
				}
			}
		}
	}

	/**
	 * @brief sew the two Y boundaries
	 */
	inline void self_sew_y()
	{
		uint32 last = nx_*(ny_-1);
		for (uint32 k = 0; k < nz_; ++k)
		{
			for (uint32 i = 0; i < nx_; ++i)
			{
				Dart d1 = map_.phi_1(volume_table_[k*nx_*ny_+i].dart);
				Dart d2 = map_.phi1(volume_table_[last+k*nx_*ny_+i].dart);
				if (!d1.is_nil() && !d2.is_nil())
				{
					builder_.sew_volumes_fp(d1, d2);
				}
			}
		}
	}

	/**
	 * @brief sew the two X boundaries
	 */
	inline void self_sew_x()
	{
		uint32 last = nx_-1;
		for (uint32 k = 0; k < nz_; ++k)
		{
			for (uint32 j = 0; j < ny_; ++j)
			{
				Dart d1 = map_.phi2(volume_table_[nx_*(ny_*k+j)].dart);
				Dart d2 = map_.phi<112>(volume_table_[last+nx_*(ny_*k+j)].dart);
				if (!d1.is_nil() && !d2.is_nil())
				{
					builder_.sew_volumes_fp(d1, d2);
				}
			}
		}
	}

	/**
	 * @brief embed the grid
	 */
	inline void embed()
	{
		embed_sub_cell_of_volume<CDart>(nullptr);
		embed_sub_cell_of_volume<Vertex2>(nullptr);
		embed_sub_cell_of_volume<Edge2>(nullptr);
		embed_sub_cell_of_volume<Face2>(nullptr);
		embed_sub_cell_of_volume<Vertex>(&this->vertex_table_);
		embed_sub_cell_of_volume<Edge>(&this->edge_table_);
		embed_sub_cell_of_volume<Face>(&this->face_table_);
		embed_sub_cell_of_volume<Volume>(nullptr);
	}


	/**
	 * @brief create a closed and embedded 3d grid
	 * @param fmask boolean function(i,j,k) for keeping or not the hexahedrons
	 */
	template <typename MASK>
	void embedded_grid3D(const MASK& fmask)
	{
		grid_topo(fmask);
		close_grid();
		embed();
	}

	void embedded_grid3D()
	{
		grid_topo( [](uint32,uint32,uint32){return true;} );
		close_grid();
		embed();
	}

	
	/**
	 * @brief update position of vertices
	 * @param embedding function f(vertex,r,s,t)->vec3 with r,s,t in [0,1]
	 */
	template <typename EMBEDDER>
	void update_positions(const EMBEDDER& femb)
	{
		CellMarker<CMap3, Vertex::ORBIT> marker(map_);
		
		auto f = [&](Dart d, uint32 i, uint32 j, uint32 k)
		{
			Vertex v(d);
			if (!marker.is_marked(v))
			{
				marker.mark(v);
				femb(v, double(i) / nx_, double(j) / ny_, double(k) / nz_);
			}
		};
		
		for (uint32 k = 0; k < nz_; ++k)
		{
			for (uint32 j = 0; j < ny_; ++j)
			{
				for (uint32 i = 0; i < nx_; ++i)
				{
					Dart d = volume_table_[i + nx_ * (j + ny_ * k)].dart;
					if (!d.is_nil())
					{
						f(d, i, j, k);
						f(map_.phi_1(d), i + 1, j, k);
						f(map_.phi1(d), i, j + 1, k);
						f(map_.phi<11>(d), i + 1, j + 1, k);
						d = map_.phi<2112>(d);
						f(d, i, j + 1, k + 1);
						f(map_.phi1(d), i, j, k + 1);
						f(map_.phi<11>(d), i + 1, j, k + 1);
						f(map_.phi_1(d), i + 1, j + 1, k + 1);
					}
				}
			}
		}
//		compact_volume_table(); // to discuss
	}
};

}  // namespace modeling

}  // namespace cgogn

#endif  // 
