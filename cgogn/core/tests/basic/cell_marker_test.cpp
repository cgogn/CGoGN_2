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

#include <gtest/gtest.h>

#include <cgogn/core/basic/cell.h>
#include <cgogn/core/cmap/cmap2_tri.h>
#include <cgogn/core/cmap/cmap2_quad.h>
#include <cgogn/core/cmap/cmap3.h>
#include <cgogn/core/cmap/cmap3_tetra.h>
#include <cgogn/core/cmap/cmap3_hexa.h>

namespace cell_marker_test
{

using namespace cgogn;

using MapTypes = ::testing::Types<CMap2, CMap2Tri, CMap2Quad, CMap3, CMap3Tetra, CMap3Hexa>;

template<typename Builder>
void setup(Builder& /*builder*/)
{
	cgogn_assert_not_reached("The setup() function has to be specialized.");
}

template<>
void setup(cgogn::CMap2Builder_T<CMap2> & builder)
{
	Dart d1 = builder.add_face_topo_fp(4u);
	Dart d2 = builder.add_face_topo_fp(3u);
	builder.phi2_sew(d1,d2);
	builder.close_map();
}

template<>
void setup(cgogn::CMap2Builder_T<CMap2Tri>& builder)
{
	Dart d1 = builder.add_face_topo_fp(3u);
	Dart d2 = builder.add_face_topo_fp(3u);
	builder.phi2_sew(d1,d2);
	builder.close_map();
}

template<>
void setup(cgogn::CMap2Builder_T<CMap2Quad>& builder)
{
	Dart d1 = builder.add_face_topo_fp(4u);
	Dart d2 = builder.add_face_topo_fp(4u);
	builder.phi2_sew(d1,d2);
	builder.close_map();
}

template<>
void setup(cgogn::CMap3Builder_T<CMap3>& builder)
{
	Dart p1 = builder.add_prism_topo_fp(3u);
	Dart p2 = builder.add_prism_topo_fp(3u);
	builder.sew_volumes_fp(p1, p2);

	Dart p3 = builder.add_pyramid_topo_fp(4u);
	Dart p4 = builder.add_pyramid_topo_fp(4u);
	builder.sew_volumes_fp(p3, p4);

	builder.close_map();
}

template<>
void setup(cgogn::CMap3Builder_T<CMap3Tetra>& builder)
{
	Dart p1 = builder.add_pyramid_topo_fp(3u);
	Dart p2 = builder.add_pyramid_topo_fp(3u);
	builder.sew_volumes_fp(p1, p2);

	Dart p3 = builder.add_pyramid_topo_fp(3u);
	Dart p4 = builder.add_pyramid_topo_fp(3u);
	builder.sew_volumes_fp(p3, p4);

	builder.close_map();
}

template<>
void setup(cgogn::CMap3Builder_T<CMap3Hexa>& builder)
{
	Dart p1 = builder.add_prism_topo_fp(4u);
	Dart p2 = builder.add_prism_topo_fp(4u);
	builder.sew_volumes_fp(p1, p2);

	Dart p3 = builder.add_prism_topo_fp(4u);
	Dart p4 = builder.add_prism_topo_fp(4u);
	builder.sew_volumes_fp(p3, p4);

	builder.close_map();
}


template<typename Map>
class CellMarkerTest : public ::testing::Test
{
public:
	using Inherit = ::testing::Test;
	using Vertex = typename Map::Vertex;
	using Edge = typename Map::Edge;
	using Face = typename Map::Face;
	using Volume = typename Map::Volume;

	template<Orbit ORB>
	using CellMarker = typename Map::template CellMarker<ORB>;

	template<Orbit ORB>
	using CellMarkerNoUnmark = typename Map::template CellMarkerNoUnmark<ORB>;

	template<Orbit ORB>
	using CellMarkerStore = typename Map::template CellMarkerStore<ORB>;

	using MapBuilder = typename std::conditional<Map::DIMENSION == 2, cgogn::CMap2Builder_T<Map>, cgogn::CMap3Builder_T<Map>>::type;

	inline CellMarkerTest() : Inherit(),
		builder(map)
	{}

	virtual void SetUp() override
	{
		setup(builder);
		init_markers();
	}


	void init_markers()
	{
		vmarker = make_unique<CellMarker<Vertex::ORBIT>>(map);
		emarker = make_unique<CellMarker<Edge::ORBIT>>(map);
		fmarker = make_unique<CellMarker<Face::ORBIT>>(map);
		wmarker = make_unique<CellMarker<Volume::ORBIT>>(map);

		vmarkernounmark = make_unique<CellMarkerNoUnmark<Vertex::ORBIT>>(map);
		emarkernounmark = make_unique<CellMarkerNoUnmark<Edge::ORBIT>>(map);
		fmarkernounmark = make_unique<CellMarkerNoUnmark<Face::ORBIT>>(map);
		wmarkernounmark = make_unique<CellMarkerNoUnmark<Volume::ORBIT>>(map);

		vmarkerstore = make_unique<CellMarkerStore<Vertex::ORBIT>>(map);
		emarkerstore = make_unique<CellMarkerStore<Edge::ORBIT>>(map);
		fmarkerstore = make_unique<CellMarkerStore<Face::ORBIT>>(map);
		wmarkerstore = make_unique<CellMarkerStore<Volume::ORBIT>>(map);
	}

	Map map;
	MapBuilder builder;
	std::unique_ptr<CellMarker<Vertex::ORBIT>> vmarker;
	std::unique_ptr<CellMarker<Edge::ORBIT>> emarker;
	std::unique_ptr<CellMarker<Face::ORBIT>> fmarker;
	std::unique_ptr<CellMarker<Volume::ORBIT>> wmarker;

	std::unique_ptr<CellMarkerNoUnmark<Vertex::ORBIT>> vmarkernounmark;
	std::unique_ptr<CellMarkerNoUnmark<Edge::ORBIT>> emarkernounmark;
	std::unique_ptr<CellMarkerNoUnmark<Face::ORBIT>> fmarkernounmark;
	std::unique_ptr<CellMarkerNoUnmark<Volume::ORBIT>> wmarkernounmark;

	std::unique_ptr<CellMarkerStore<Vertex::ORBIT>> vmarkerstore;
	std::unique_ptr<CellMarkerStore<Edge::ORBIT>> emarkerstore;
	std::unique_ptr<CellMarkerStore<Face::ORBIT>> fmarkerstore;
	std::unique_ptr<CellMarkerStore<Volume::ORBIT>> wmarkerstore;
};

TYPED_TEST_CASE(CellMarkerTest, MapTypes);

TYPED_TEST(CellMarkerTest, default_unmarked)
{
	using Vertex = typename TypeParam::Vertex;
	using Edge = typename TypeParam::Edge;
	using Face = typename TypeParam::Face;
	using Volume = typename TypeParam::Volume;

	this->map.foreach_dart([&](Dart d)
	{
		if (!this->map.is_boundary(d))
		{
			EXPECT_FALSE(this->vmarker->is_marked(Vertex(d)));
			EXPECT_FALSE(this->emarker->is_marked(Edge(d)));
			EXPECT_FALSE(this->fmarker->is_marked(Face(d)));
			EXPECT_FALSE(this->wmarker->is_marked(Volume(d)));

			EXPECT_FALSE(this->vmarkernounmark->is_marked(Vertex(d)));
			EXPECT_FALSE(this->emarkernounmark->is_marked(Edge(d)));
			EXPECT_FALSE(this->fmarkernounmark->is_marked(Face(d)));
			EXPECT_FALSE(this->wmarkernounmark->is_marked(Volume(d)));

			EXPECT_FALSE(this->vmarkerstore->is_marked(Vertex(d)));
			EXPECT_FALSE(this->emarkerstore->is_marked(Edge(d)));
			EXPECT_FALSE(this->fmarkerstore->is_marked(Face(d)));
			EXPECT_FALSE(this->wmarkerstore->is_marked(Volume(d)));
		}
	});
}

TYPED_TEST(CellMarkerTest, marking)
{
	using Vertex = typename TypeParam::Vertex;
	using Edge = typename TypeParam::Edge;
	using Face = typename TypeParam::Face;
	using Volume = typename TypeParam::Volume;

	this->map.foreach_dart([&](Dart d)
	{
		if (!this->map.is_boundary(d))
		{
			Vertex v(d);
			this->vmarker->mark(v);
			this->vmarkernounmark->mark(v);
			this->vmarkerstore->mark(v);
			this->map.foreach_dart_of_orbit(v, [&](Dart d2)
			{
					EXPECT_TRUE(this->vmarker->is_marked(Vertex(d2)));
					EXPECT_TRUE(this->vmarkernounmark->is_marked(Vertex(d2)));
					EXPECT_TRUE(this->vmarkerstore->is_marked(Vertex(d2)));
			});

			Edge e(d);
			this->emarker->mark(e);
			this->emarkernounmark->mark(e);
			this->emarkerstore->mark(e);
			this->map.foreach_dart_of_orbit(e, [&](Dart d2)
			{
					EXPECT_TRUE(this->emarker->is_marked(Edge(d2)));
					EXPECT_TRUE(this->emarkernounmark->is_marked(Edge(d2)));
					EXPECT_TRUE(this->emarkerstore->is_marked(Edge(d2)));
			});

			Face f(d);
			this->fmarker->mark(f);
			this->fmarkernounmark->mark(f);
			this->fmarkerstore->mark(f);
			this->map.foreach_dart_of_orbit(f, [&](Dart d2)
			{
					EXPECT_TRUE(this->fmarker->is_marked(Face(d2)));
					EXPECT_TRUE(this->fmarkernounmark->is_marked(Face(d2)));
					EXPECT_TRUE(this->fmarkerstore->is_marked(Face(d2)));
			});

			Volume w(d);
			this->wmarker->mark(w);
			this->wmarkernounmark->mark(w);
			this->wmarkerstore->mark(w);
			this->map.foreach_dart_of_orbit(w, [&](Dart d2)
			{
					EXPECT_TRUE(this->wmarker->is_marked(Volume(d2)));
					EXPECT_TRUE(this->wmarkernounmark->is_marked(Volume(d2)));
					EXPECT_TRUE(this->wmarkerstore->is_marked(Volume(d2)));
			});
		}
	});
}

TYPED_TEST(CellMarkerTest, unmarking)
{
	using Vertex = typename TypeParam::Vertex;
	using Edge = typename TypeParam::Edge;
	using Face = typename TypeParam::Face;
	using Volume = typename TypeParam::Volume;

	this->map.foreach_dart([&](Dart d)
	{
		if (!this->map.is_boundary(d))
		{
			Vertex v(d);
			this->vmarker->mark(v);
			this->vmarkerstore->mark(v);
			this->vmarker->unmark(v);
			this->vmarkerstore->unmark(v);
			this->map.foreach_dart_of_orbit(v, [&](Dart d2)
			{
					EXPECT_FALSE(this->vmarker->is_marked(Vertex(d2)));
					EXPECT_FALSE(this->vmarkerstore->is_marked(Vertex(d2)));
			});

			Edge e(d);
			this->emarker->mark(e);
			this->emarkerstore->mark(e);
			this->emarker->unmark(e);
			this->emarkerstore->unmark(e);
			this->map.foreach_dart_of_orbit(e, [&](Dart d2)
			{
					EXPECT_FALSE(this->emarker->is_marked(Edge(d2)));
					EXPECT_FALSE(this->emarkerstore->is_marked(Edge(d2)));
			});

			Face f(d);
			this->fmarker->mark(f);
			this->fmarkerstore->mark(f);
			this->fmarker->unmark(f);
			this->fmarkerstore->unmark(f);
			this->map.foreach_dart_of_orbit(f, [&](Dart d2)
			{
					EXPECT_FALSE(this->fmarker->is_marked(Face(d2)));
					EXPECT_FALSE(this->fmarkerstore->is_marked(Face(d2)));
			});

			Volume w(d);
			this->wmarker->mark(w);
			this->wmarkerstore->mark(w);
			this->wmarker->unmark(w);
			this->wmarkerstore->unmark(w);
			this->map.foreach_dart_of_orbit(w, [&](Dart d2)
			{
					EXPECT_FALSE(this->wmarker->is_marked(Volume(d2)));
					EXPECT_FALSE(this->wmarkerstore->is_marked(Volume(d2)));
			});
		}
	});
}

TYPED_TEST(CellMarkerTest, unmark_all)
{
	using Vertex = typename TypeParam::Vertex;
	using Edge = typename TypeParam::Edge;
	using Face = typename TypeParam::Face;
	using Volume = typename TypeParam::Volume;

	this->map.foreach_dart([&](Dart d)
	{
		if (!this->map.is_boundary(d))
		{
			Vertex v(d);
			this->vmarker->mark(v);
			this->vmarkerstore->mark(v);

			Edge e(d);
			this->emarker->mark(e);
			this->emarkerstore->mark(e);

			Face f(d);
			this->fmarker->mark(f);
			this->fmarkerstore->mark(f);

			Volume w(d);
			this->wmarker->mark(w);
			this->wmarkerstore->mark(w);
		}
	});

	this->vmarker->unmark_all();
	this->vmarkerstore->unmark_all();
	this->emarker->unmark_all();
	this->emarkerstore->unmark_all();
	this->fmarker->unmark_all();
	this->fmarkerstore->unmark_all();
	this->wmarker->unmark_all();
	this->wmarkerstore->unmark_all();

	this->map.foreach_dart([&](Dart d)
	{
		if (!this->map.is_boundary(d))
		{
			EXPECT_FALSE(this->vmarker->is_marked(Vertex(d)));
			EXPECT_FALSE(this->vmarkerstore->is_marked(Vertex(d)));
			EXPECT_FALSE(this->emarker->is_marked(Edge(d)));
			EXPECT_FALSE(this->emarkerstore->is_marked(Edge(d)));
			EXPECT_FALSE(this->fmarker->is_marked(Face(d)));
			EXPECT_FALSE(this->fmarkerstore->is_marked(Face(d)));
			EXPECT_FALSE(this->wmarker->is_marked(Volume(d)));
			EXPECT_FALSE(this->wmarkerstore->is_marked(Volume(d)));
		}
	});
}

} // namespace cell_marker_test
