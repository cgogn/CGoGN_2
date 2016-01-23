#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <core/cmap/cmap1.h>

namespace cgogn
{


class CMap1TopoTest: public CMap1<DefaultMapTraits>, public ::testing::Test
{

protected:

	CMap1TopoTest()
	{}

	void SetUp()
	{ }

	void TearDown()
	{}
};


TEST_F(CMap1TopoTest, testAddDart)
{

}

TEST_F(CMap1TopoTest, testDeleteDart)
{

}

TEST_F(CMap1TopoTest, testFaceDegree)
{
	Dart d = this->add_face_topo(10);
	EXPECT_EQ(10, this->degree(d));
}

TEST_F(CMap1TopoTest, testCutEdge)
{
	Dart d = this->add_face_topo(10);
	Dart d1 = this->phi1(d);

	Dart e = this->cut_edge_topo(d);

	EXPECT_EQ(d1.index, this->phi1(e).index);
	EXPECT_EQ(d.index, this->phi_1(e).index);
	EXPECT_EQ(11, this->degree(d));
}

TEST_F(CMap1TopoTest, testUncutEdge)
{
	Dart d = this->add_face_topo(10);
	Dart d11 = this->phi1(this->phi1(d));

	this->uncut_edge_topo(d);

	EXPECT_EQ(d11.index, this->phi1(d).index);
	EXPECT_EQ(9, this->degree(d));
}

TEST_F(CMap1TopoTest, testCollapseEdge)
{
	Dart d = this->add_face_topo(10);
	Dart d_1 = this->phi_1(d);
	Dart d1 = this->phi1(d);
	
	this->collapse_edge_topo(d);

	EXPECT_EQ(d1.index, this->phi1(d_1).index);
	EXPECT_EQ(9, this->degree(d_1));
}

TEST_F(CMap1TopoTest, testSplitFace)
{
	Dart d = this->add_face_topo(10);

	Dart e = this->phi1(this->phi1(this->phi1(d)));
	this->split_face_topo(d, e);

	EXPECT_EQ(3, this->degree(d));
	EXPECT_EQ(7, this->degree(e));
	EXPECT_FALSE(this->same_cell(Face(d),Face(e)));
}

TEST_F(CMap1TopoTest, testMergeFaces)
{
	Dart d = this->add_face_topo(10);
	Dart e = this->add_face_topo(10);

	this->merge_faces_topo(d, e);

	EXPECT_EQ(20, this->degree(d));
	EXPECT_TRUE(this->same_cell(Face(d),Face(e)));
}

TEST_F(CMap1TopoTest, testLinkFaces)
{
	Dart d = this->add_face_topo(10);
	Dart e = this->add_face_topo(10);

	this->link_faces_topo(d, e);

	EXPECT_EQ(22, this->degree(d));
	EXPECT_TRUE(this->same_cell(Face(d),Face(e)));
}

TEST_F(CMap1TopoTest, testReverseFace)
{
	Dart d = this->add_face_topo(10);
	std::vector<Dart> successors;

	{
		Dart dit = d;
		do
		{
			successors.push_back(this->phi1(dit));
			dit = this->phi1(dit);
		}
		while(dit != d);
	}

	this->reverse_face_topo(d);

	{
		Dart dit = d;
		unsigned i = 0;
		do
		{
			EXPECT_EQ(this->phi_1(dit).index, successors[i].index);
			dit = this->phi_1(dit);
			++i;
		}
		while(dit != d);
	}
}

TEST_F(CMap1TopoTest, testForEachDartOfVertex)
{

}

TEST_F(CMap1TopoTest, testForEachDartOfEdge)
{
	
}

TEST_F(CMap1TopoTest, testForEachDartOfFace)
{
	
}


// class CMap1TopoMock : public CMap1<DefaultMapTraits> {
// public:
//     MOCK_METHOD0( add_dart, Dart() );
//     MOCK_METHOD1( cut_edge_topo, Dart(Dart d) );
//     MOCK_METHOD1( add_face_topo, Dart(unsigned int nb_edges) );
// };


// TEST_F(CMap1TopoTest, testDeleteFace)
// {
// 	this->delete_face_topo(d_);
//  	EXPECT_EQ(0, this->degree(d_));
// }

} // namespace cgogn
