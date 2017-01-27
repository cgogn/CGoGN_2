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

#include <random>

#include <gtest/gtest.h>

#include <cgogn/core/cmap/cmap2.h>
#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/modeling/tiling/square_tore.h>

namespace cgogn
{

class SquareTilingTest : public testing::Test
{
public:

	using Dart = cgogn::Dart;
	template <typename T>
	using VertexAttribute = CMap2::VertexAttribute<T>;
	using CDart = CMap2::CDart;
	using Vertex = CMap2::Vertex;
	using Edge = CMap2::Edge;
	using Face = CMap2::Face;
	using Volume = CMap2::Volume;

protected:

	CMap2 cmap_;
	uint32 x_,y_;

	SquareTilingTest()
	{
		std::random_device rd;
		std::mt19937 rng(rd());

		std::uniform_int_distribution<uint32> uni(4,100);

		x_ = uni(rng);
		y_ = uni(rng);

		//cmap_.add_attribute<int32, CDart>("darts");
		cmap_.add_attribute<int32, Vertex>("vertices");
		cmap_.add_attribute<int32, Edge>("edges");
		cmap_.add_attribute<int32, Face>("faces");
		cmap_.add_attribute<int32, Volume>("volumes");
	}
};

TEST_F(SquareTilingTest, SquareGrid)
{
	cgogn::modeling::SquareGrid<CMap2> g(cmap_, x_, y_);

	uint32 nb_vertices = (x_+1)*(y_+1);
	uint32 nb_edges = (x_+1)*y_+(y_+1)*x_;
	uint32 nb_faces = x_*y_;
	uint32 nb_volumes = 1;

	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), nb_vertices);
	EXPECT_EQ(cmap_.nb_cells<Edge::ORBIT>(), nb_edges);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), nb_faces);
	EXPECT_EQ(cmap_.nb_cells<Volume::ORBIT>(), nb_volumes);
	EXPECT_TRUE(cmap_.check_map_integrity());
}

TEST_F(SquareTilingTest, SquareCylinder)
{
	cgogn::modeling::SquareCylinder<CMap2> g(cmap_, x_, y_);

	uint32 nb_vertices = x_*(y_+1);
	uint32 genius = 0;
	uint32 nb_faces = x_*y_+2;
	uint32 nb_edges = nb_vertices + (nb_faces-2) - genius;
	uint32 nb_volumes = 1;

	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), nb_vertices);
	EXPECT_EQ(cmap_.nb_cells<Edge::ORBIT>(), nb_edges);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), nb_faces);
	EXPECT_EQ(cmap_.nb_cells<Volume::ORBIT>(), nb_volumes);
	EXPECT_TRUE(cmap_.check_map_integrity());
}

TEST_F(SquareTilingTest, SquareTore)
{
	cgogn::modeling::SquareTore<CMap2> g(cmap_, x_, y_);

	uint32 nb_vertices = (x_-1)*(y_-1)+(x_-1)+y_;
	uint32 genius = 0;
	uint32 nb_faces = x_*y_;
	uint32 nb_edges = nb_vertices + nb_faces - genius;
	uint32 nb_volumes = 1;

	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), nb_vertices);
	EXPECT_EQ(cmap_.nb_cells<Edge::ORBIT>(), nb_edges);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), nb_faces);
	EXPECT_EQ(cmap_.nb_cells<Volume::ORBIT>(), nb_volumes);
	EXPECT_TRUE(cmap_.check_map_integrity());
}

} // namespace cgogn
