#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <core/cmap/cmap1.h>

namespace cgogn
{


// class CMap1TopoMock : public CMap1<DefaultMapTraits> {
// public:
//     MOCK_METHOD0( add_dart, Dart() );
//     MOCK_METHOD1( cut_edge_topo, Dart(Dart d) );
//     MOCK_METHOD1( add_face_topo, Dart(unsigned int nb_edges) );
// };

class CMap1TopoTest: public CMap1<DefaultMapTraits>, public ::testing::Test
{

	using CMap1 = CMap1<DefaultMapTraits>;

public:
	CMap1 cmap_;
	Dart d_;

protected:

	CMap1TopoTest()
	{}

	void SetUp()
	{
		d_ = this->add_face_topo(10);
	}

	void TearDown()
  	{}
};

TEST_F(CMap1TopoTest, testFaceDegree)
{
	EXPECT_EQ(10, this->degree(d_));
}

TEST_F(CMap1TopoTest, testCutEdge)
{
	Dart d1 = this->phi1(d_);
	Dart e = this->cut_edge_topo(d_);

	EXPECT_EQ(d1.index, this->phi1(e).index);
 	EXPECT_EQ(d_.index, this->phi_1(e).index);
 	EXPECT_EQ(11, this->degree(d_));
}

TEST_F(CMap1TopoTest, testUncutEdge)
{
	Dart e = this->phi1(d_);
	Dart d1 = this->phi1(e);
	this->uncut_edge_topo(e);

	EXPECT_EQ(d1.index, this->phi1(d_).index);
 	EXPECT_EQ(10, this->degree(d_));
}

TEST_F(CMap1TopoTest, testSplitFace)
{
	Dart e = this->phi1(d_);
	Dart d1 = this->phi1(e);
	this->uncut_edge_topo(e);

	EXPECT_EQ(d1.index, this->phi1(d_).index);
 	EXPECT_EQ(10, this->degree(d_));
}

// TEST_F(CMap1TopoTest, testDeleteFace)
// {
// 	this->delete_face_topo(d_);
//  	EXPECT_EQ(0, this->degree(d_));
// }



} // namespace cgogn