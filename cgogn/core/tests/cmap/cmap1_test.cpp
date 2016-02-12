#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <core/cmap/cmap1.h>
#include <core/cmap/map_traits.h>
#include <core/cmap/sanity_check.h>

namespace cgogn
{

class CMap1Test: public ::testing::Test
{

public:
	typedef CMap1<DefaultMapTraits> myCMap1;
	typedef myCMap1::Vertex Vertex;
	typedef myCMap1::Edge Edge;
	typedef myCMap1::Face Face;

protected:
	myCMap1 cmap_;

	CMap1Test()
	{}


};

TEST_F(CMap1Test, addFace)
{
	Face f = cmap_.add_face(10);

//	cmap_.cut_edge(Edge(f));

	EXPECT_TRUE(is_well_embedded<myCMap1::VERTEX>(cmap_));
	EXPECT_TRUE(is_well_embedded<myCMap1::EDGE>(cmap_));
	EXPECT_TRUE(is_well_embedded<myCMap1::FACE>(cmap_));
	EXPECT_TRUE(is_orbit_embedding_unique<myCMap1::VERTEX>(cmap_));
	EXPECT_TRUE(is_orbit_embedding_unique<myCMap1::EDGE>(cmap_));
	EXPECT_TRUE(is_orbit_embedding_unique<myCMap1::FACE>(cmap_));
	EXPECT_TRUE(is_container_well_referenced<myCMap1::VERTEX>(cmap_));
	EXPECT_TRUE(is_container_well_referenced<myCMap1::EDGE>(cmap_));
	EXPECT_TRUE(is_container_well_referenced<myCMap1::FACE>(cmap_));

}

} // namespace cgogn
