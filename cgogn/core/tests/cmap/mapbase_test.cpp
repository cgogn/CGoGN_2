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

#include <cgogn/core/cmap/cmap3.h>

namespace cgogn
{

template<typename Map>
class MapBaseTest : public ::testing::Test
{
public:
	using Inherit = ::testing::Test;
	using VertexAttribute = typename Map::template VertexAttribute<int32>;
	using Vertex = typename Map::Vertex;
	using Face = typename Map::Face;

protected:
	Map cmap_;

	// Test interface
protected:
	virtual void SetUp() override
	{}
	virtual void TearDown() override
	{}

private:
	virtual void TestBody() override
	{}
};

using MapTypes = ::testing::Types<CMap1, CMap2, CMap3>;
TYPED_TEST_CASE(MapBaseTest, MapTypes);

/**
 * @brief TYPED_TEST
 * -we can add an int32 attribute and get a valid Attribute.
 * -when we add the exactly same attribute twice we get a valid attribute.
 * -when we add an attribute on the same orbit with the same name but a different type, we get an invalid attribute.
 */
TYPED_TEST(MapBaseTest, add_attribute)
{
	using Vertex = typename MapBaseTest<TypeParam>::Vertex;
	Attribute <int32, Vertex::ORBIT> vatt1 = this->cmap_.template add_attribute<int32, Vertex>("cool_attribute");
	EXPECT_TRUE(vatt1.is_valid());


	Attribute <int32, Vertex::ORBIT> vatt3 = this->cmap_.template add_attribute<int32, Vertex>("cool_attribute");
	EXPECT_TRUE(vatt1.is_valid());
	EXPECT_FALSE(vatt3.is_valid());

	Attribute <float32, Vertex::ORBIT> vatt4 = this->cmap_.template add_attribute<float32, Vertex>("cool_attribute");
	EXPECT_TRUE(vatt1.is_valid());
	EXPECT_FALSE(vatt3.is_valid());
	EXPECT_FALSE(vatt4.is_valid());
}



TYPED_TEST(MapBaseTest, has_attribute)
{
	using Vertex = typename MapBaseTest<TypeParam>::Vertex;
	using Face = typename MapBaseTest<TypeParam>::Face;
	Attribute <int32, Vertex::ORBIT> vatt1 = this->cmap_.template add_attribute<int32, Vertex>("cool_attribute");
	EXPECT_TRUE(this->cmap_.has_attribute(Vertex::ORBIT,"cool_attribute"));
	EXPECT_FALSE(this->cmap_.has_attribute(Face::ORBIT,"cool_attribute"));
}

TYPED_TEST(MapBaseTest, remove_attribute)
{
	using Vertex = typename MapBaseTest<TypeParam>::Vertex;

	Attribute <int32, Vertex::ORBIT> vatt1 = this->cmap_.template add_attribute<int32, Vertex>("cool_attribute");
	Attribute <int32, Vertex::ORBIT> vatt2 = this->cmap_.template add_attribute<int32, Vertex>("cool_attribute2");

	EXPECT_TRUE(this->cmap_.has_attribute(Vertex::ORBIT,"cool_attribute"));
	this->cmap_.remove_attribute(vatt1);
	EXPECT_FALSE(this->cmap_.has_attribute(Vertex::ORBIT,"cool_attribute"));
	EXPECT_FALSE(vatt1.is_valid());

	EXPECT_TRUE(this->cmap_.has_attribute(Vertex::ORBIT,"cool_attribute2"));
	this->cmap_.remove_attribute(Vertex::ORBIT,"cool_attribute2");
	EXPECT_FALSE(this->cmap_.has_attribute(Vertex::ORBIT,"cool_attribute2"));
	EXPECT_FALSE(vatt2.is_valid());
}

} // namespace cgogn
