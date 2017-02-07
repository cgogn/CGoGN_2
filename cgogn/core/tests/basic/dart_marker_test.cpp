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

#include <cgogn/core/cmap/cmap2_tri.h>
#include <cgogn/core/cmap/cmap2_quad.h>
#include <cgogn/core/cmap/cmap3.h>
#include <cgogn/core/cmap/cmap3_tetra.h>
#include <cgogn/core/cmap/cmap3_hexa.h>

namespace dart_marker_test
{

using namespace cgogn;

using MapTypes = ::testing::Types<CMap2, CMap2Tri, CMap2Quad, CMap3, CMap3Tetra, CMap3Hexa>;

template<typename Builder>
void setup(Builder& builder)
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
class DartMarkerTest : public ::testing::Test
{
public:
	using Inherit = ::testing::Test;
	using Vertex = typename Map::Vertex;
	using Edge = typename Map::Edge;
	using Face = typename Map::Face;
	using Volume = typename Map::Volume;

	using DartMarker = typename cgogn::DartMarker<Map>;
	using DartMarkerNoUnmark = typename cgogn::DartMarkerNoUnmark<Map>;
	using DartMarkerStore = typename cgogn::DartMarkerStore<Map>;

	using MapBuilder = typename std::conditional<Map::DIMENSION == 2, cgogn::CMap2Builder_T<Map>, cgogn::CMap3Builder_T<Map>>::type;

	inline DartMarkerTest() : Inherit(),
		builder(map)
	{}

	virtual void SetUp() override
	{
		setup(builder);
		init_markers();
	}

	void init_markers()
	{
		marker = make_unique<DartMarker>(map);
		markernounmark = make_unique<DartMarkerNoUnmark>(map);
		markerstore = make_unique<DartMarkerStore>(map);

		vmarker = make_unique<DartMarker>(map);
		emarker = make_unique<DartMarker>(map);
		fmarker = make_unique<DartMarker>(map);
		wmarker = make_unique<DartMarker>(map);

		vmarkernounmark = make_unique<DartMarkerNoUnmark>(map);
		emarkernounmark = make_unique<DartMarkerNoUnmark>(map);
		fmarkernounmark = make_unique<DartMarkerNoUnmark>(map);
		wmarkernounmark = make_unique<DartMarkerNoUnmark>(map);

		vmarkerstore = make_unique<DartMarkerStore>(map);
		emarkerstore = make_unique<DartMarkerStore>(map);
		fmarkerstore = make_unique<DartMarkerStore>(map);
		wmarkerstore = make_unique<DartMarkerStore>(map);
	}

	Map map;
	MapBuilder builder;
	std::unique_ptr<DartMarker> marker;
	std::unique_ptr<DartMarkerNoUnmark> markernounmark;
	std::unique_ptr<DartMarkerStore> markerstore;


	std::unique_ptr<DartMarker> vmarker;
	std::unique_ptr<DartMarker> emarker;
	std::unique_ptr<DartMarker> fmarker;
	std::unique_ptr<DartMarker> wmarker;

	std::unique_ptr<DartMarkerNoUnmark> vmarkernounmark;
	std::unique_ptr<DartMarkerNoUnmark> emarkernounmark;
	std::unique_ptr<DartMarkerNoUnmark> fmarkernounmark;
	std::unique_ptr<DartMarkerNoUnmark> wmarkernounmark;

	std::unique_ptr<DartMarkerStore> vmarkerstore;
	std::unique_ptr<DartMarkerStore> emarkerstore;
	std::unique_ptr<DartMarkerStore> fmarkerstore;
	std::unique_ptr<DartMarkerStore> wmarkerstore;
};

TYPED_TEST_CASE(DartMarkerTest, MapTypes);

TYPED_TEST(DartMarkerTest, default_unmarked)
{
	this->map.foreach_dart([&](Dart d)
	{
		if (!this->map.is_boundary(d))
		{
			EXPECT_FALSE(this->marker->is_marked(d));
			EXPECT_FALSE(this->markernounmark->is_marked(d));
			EXPECT_FALSE(this->markerstore->is_marked(d));
		}
	});
}

TYPED_TEST(DartMarkerTest, marking)
{
	using Vertex = typename TypeParam::Vertex;
	using Edge = typename TypeParam::Edge;
	using Face = typename TypeParam::Face;
	using Volume = typename TypeParam::Volume;

	this->map.foreach_dart([&](Dart d)
	{
		if (!this->map.is_boundary(d))
		{
			this->marker->mark(d);
			this->markernounmark->mark(d);
			this->markerstore->mark(d);
			EXPECT_TRUE(this->marker->is_marked(d));
			EXPECT_TRUE(this->markernounmark->is_marked(d));
			EXPECT_TRUE(this->markerstore->is_marked(d));

			Vertex v(d);
			this->vmarker->mark_orbit(v);
			this->vmarkernounmark->mark_orbit(v);
			this->vmarkerstore->mark_orbit(v);
			this->map.foreach_dart_of_orbit(v, [&](Dart d2)
			{
					EXPECT_TRUE(this->vmarker->is_marked(d2));
					EXPECT_TRUE(this->vmarkernounmark->is_marked(d2));
					EXPECT_TRUE(this->vmarkerstore->is_marked(d2));
			});

			Edge e(d);
			this->emarker->mark_orbit(e);
			this->emarkernounmark->mark_orbit(e);
			this->emarkerstore->mark_orbit(e);
			this->map.foreach_dart_of_orbit(e, [&](Dart d2)
			{
					EXPECT_TRUE(this->emarker->is_marked(d2));
					EXPECT_TRUE(this->emarkernounmark->is_marked(d2));
					EXPECT_TRUE(this->emarkerstore->is_marked(d2));
			});

			Face f(d);
			this->fmarker->mark_orbit(f);
			this->fmarkernounmark->mark_orbit(f);
			this->fmarkerstore->mark_orbit(f);
			this->map.foreach_dart_of_orbit(f, [&](Dart d2)
			{
					EXPECT_TRUE(this->fmarker->is_marked(d2));
					EXPECT_TRUE(this->fmarkernounmark->is_marked(d2));
					EXPECT_TRUE(this->fmarkerstore->is_marked(d2));
			});

			Volume w(d);
			this->wmarker->mark_orbit(w);
			this->wmarkernounmark->mark_orbit(w);
			this->wmarkerstore->mark_orbit(w);
			this->map.foreach_dart_of_orbit(w, [&](Dart d2)
			{
					EXPECT_TRUE(this->wmarker->is_marked(d2));
					EXPECT_TRUE(this->wmarkernounmark->is_marked(d2));
					EXPECT_TRUE(this->wmarkerstore->is_marked(d2));
			});
		}
	});
}

TYPED_TEST(DartMarkerTest, unmarking)
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
			this->vmarker->mark_orbit(v);
			this->vmarkerstore->mark_orbit(v);
			this->vmarker->unmark_orbit(v);
			this->vmarkerstore->unmark_orbit(v);
			this->map.foreach_dart_of_orbit(v, [&](Dart d2)
			{
					EXPECT_FALSE(this->vmarker->is_marked(d2));
					EXPECT_FALSE(this->vmarkerstore->is_marked(d2));
			});

			Edge e(d);
			this->emarker->mark_orbit(e);
			this->emarkerstore->mark_orbit(e);
			this->emarker->unmark_orbit(e);
			this->emarkerstore->unmark_orbit(e);
			this->map.foreach_dart_of_orbit(e, [&](Dart d2)
			{
					EXPECT_FALSE(this->emarker->is_marked(d2));
					EXPECT_FALSE(this->emarkerstore->is_marked(d2));
			});

			Face f(d);
			this->fmarker->mark_orbit(f);
			this->fmarkerstore->mark_orbit(f);
			this->fmarker->unmark_orbit(f);
			this->fmarkerstore->unmark_orbit(f);
			this->map.foreach_dart_of_orbit(f, [&](Dart d2)
			{
					EXPECT_FALSE(this->fmarker->is_marked(d2));
					EXPECT_FALSE(this->fmarkerstore->is_marked(d2));
			});

			Volume w(d);
			this->wmarker->mark_orbit(w);
			this->wmarkerstore->mark_orbit(w);
			this->wmarker->unmark_orbit(w);
			this->wmarkerstore->unmark_orbit(w);
			this->map.foreach_dart_of_orbit(w, [&](Dart d2)
			{
					EXPECT_FALSE(this->wmarker->is_marked(d2));
					EXPECT_FALSE(this->wmarkerstore->is_marked(d2));
			});
		}
	});
}

TYPED_TEST(DartMarkerTest, unmark_all)
{
	using Vertex = typename TypeParam::Vertex;
	using Edge = typename TypeParam::Edge;
	using Face = typename TypeParam::Face;
	using Volume = typename TypeParam::Volume;

	this->map.foreach_dart([&](Dart d)
	{
		if (!this->map.is_boundary(d))
		{
			this->marker->mark(d);
			this->markerstore->mark(d);

			Vertex v(d);
			this->vmarker->mark_orbit(v);
			this->vmarkerstore->mark_orbit(v);

			Edge e(d);
			this->emarker->mark_orbit(e);
			this->emarkerstore->mark_orbit(e);

			Face f(d);
			this->fmarker->mark_orbit(f);
			this->fmarkerstore->mark_orbit(f);

			Volume w(d);
			this->wmarker->mark_orbit(w);
			this->wmarkerstore->mark_orbit(w);
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
			EXPECT_FALSE(this->vmarker->is_marked(d));
			EXPECT_FALSE(this->vmarkerstore->is_marked(d));
			EXPECT_FALSE(this->emarker->is_marked(d));
			EXPECT_FALSE(this->emarkerstore->is_marked(d));
			EXPECT_FALSE(this->fmarker->is_marked(d));
			EXPECT_FALSE(this->fmarkerstore->is_marked(d));
			EXPECT_FALSE(this->wmarker->is_marked(d));
			EXPECT_FALSE(this->wmarkerstore->is_marked(d));
		}
	});
}

} // namespace dart_marker_test
