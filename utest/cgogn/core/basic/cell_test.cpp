#include <gtest/gtest.h>

#include <core/basic/cell.h>

namespace cgogn
{

// TEST(CellTest, Equality)
// {
// 	Cell<VERTEX1> v1();
// 	Cell<VERTEX1> v2();

// 	v1 != v2;

// 	// EXPECT_EQ(10u, d.index);
// }

const Dart dglobal(10u);
const Dart dmax(std::numeric_limits<unsigned int>::max());


TEST(CellTest, DefaultConstructor)
{
	Cell<Orbit::DART> c;
	Dart d = c;
	EXPECT_EQ(std::numeric_limits<unsigned int>::max(), d.index);
}

TEST(CellTest, Constructor)
{
	Cell<Orbit::DART> c(dglobal);
	Dart d = c;
	EXPECT_EQ(10u, d.index);
}

TEST(CellTest, OutOfLimitConstructor)
{
	Cell<Orbit::DART> c1 = dmax;
	Dart d1 = c1;
	Cell<Orbit::DART> c2;
	Dart d2 = c2;
	EXPECT_EQ(d1.index, d2.index);
}

TEST(CellTest, CopyConstructor)
{
	Cell<Orbit::DART> c = dglobal;
	Dart d = c;
	Cell<Orbit::DART> ccopy(c);
	Dart dcopy = ccopy;
	EXPECT_EQ(d.index, dcopy.index);
}

TEST(CellTest, IsValid)
{
	Cell<Orbit::DART> c = dglobal;
	EXPECT_TRUE(c.is_valid());
}

TEST(CellTest, Assignation)
{
	Cell<Orbit::DART> c1 = dglobal;
	Cell<Orbit::DART> c2;
	c2 = c1;

	Dart d2 = c2;

	EXPECT_EQ(d2.index, dglobal.index);
}

TEST(CellTest, PrintingOut)
{
	Cell<Orbit::DART> c = dglobal;
	std::ostringstream s;
	s << "c=" << c;
	EXPECT_EQ(0, strcmp(s.str().c_str(), "c=10"));
}

TEST(CellTest, ReadingIn)
{
	Cell<Orbit::DART> c;
	std::istringstream s("10");
	s >> c;

	Dart d = c;

	EXPECT_EQ(10u, d.index);
}

} // namespace cgogn
