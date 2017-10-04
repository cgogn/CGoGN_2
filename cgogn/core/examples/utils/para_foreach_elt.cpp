
#include <cgogn/core/utils/parallel_foreach_element.h>

#include <list>
#include <vector>
#include <map>
#include <set>


using namespace cgogn::numerics;

int test1()
{
	std::list<uint32> l(1000000);

	std::vector<float> v(1000000);

	std::set<double> s;
	for (uint32 i=0; i<1000000; ++i)
		s.insert(0.1*i);

	std::map<int32,float> m;
	for (int32 i=0; i<1000000; ++i)
		m[i] = 0.1f*i;


	// with list and ref access
	cgogn::parallel_foreach_element(l, [](uint32& x)
	{
		x = 3;
	});

	// with list and cont ref access
	cgogn::parallel_foreach_element(l, [](const uint32& x)
	{
		if (x != 3)
			std::cout << "Nooo" <<std::endl;
	});

	// with list (const) and const ref access
	const std::list<uint32>& lc = l;
	cgogn::parallel_foreach_element(lc, [](/*const uint32&*/ uint32 x)
	{
		if (x != 3)
			std::cout << "Nooo" <<std::endl;
	});

//	 with list (const) and ref access DO NOT COMPILE
//	cgogn::parallel_foreach_element(lc, [](uint32& x)
//	{
//		if (x != 3)
//			std::cout << "Nooo" <<std::endl;
//	});

	

	// with vector
	cgogn::parallel_foreach_element(v, [](float& x)
	{
		x = 0.3f;
	});

	// with set, and const ref access (forbiden modif in set)
	cgogn::parallel_foreach_element(s, [](const double& x)
	{
		if (x>0.1 && x<0.5)
			std::cout << x <<std::endl;
	});


	// with map
	// param must be [const] std::pair<const K,T>&
	cgogn::parallel_foreach_element(m, [](std::pair<const int32,float>& x)
	{
		x.second += 0.003f;
	});

	return 0;
}

// traversal of 2 containers
int test2()
{
	std::vector<int32> v(1000000);

	std::map<int32, float> m;
	for (int32 i = 0; i < 900000; ++i)
		m[i] = 0.1f*i;


	const std::vector<int32>& vc = v;

	cgogn::parallel_foreach_element(vc, m, [](const int32& x, std::pair<const int32, float>& y)
	{
		y.second += 0.00001f*x;
	});

// DO NOT COMPILE : x must be const&
//	cgogn::parallel_foreach_element(vc, m, [](int32& x, std::pair<const int32,float>& y)
//	{
//		y.second += 0.00001f*x;
//	});

	cgogn::parallel_foreach_element(v, m, [](int32& x, std::pair<const int32,float>& y)
	{
		y.second += 0.00001f*x;
	});

	cgogn::parallel_foreach_element(vc, m, [](const int32& x, std::pair<const int32,float>& y)
	{
		y.second += 0.00001f*x;
	});

	return 0;
}


// traversal of 3 and 4 containers
int test3()
{
	std::vector<int32> v(1000000);

	std::list<double> l(800000);

	std::map<int32, float> m;
	for (int32 i = 0; i < 900000; ++i)
		m[i] = 0.1f*i;

	std::set<uint32> s;
	for (uint32 i = 0; i < 850000; ++i)
		s.insert(i * 2);

	cgogn::parallel_foreach_element(v, m, l, [](int32 x, std::pair<const int32, float>& y, double& z)
	{
		y.second += 0.00001f*x;
		z = y.second;
	});

	// warning function parameter for set element must be const& or val.
	cgogn::parallel_foreach_element(v, m, l, s, [](const int32& x, std::pair<const int32, float>& y, double& z, uint32 w)
	{
		y.second += 0.00001f*x;
		z = y.second;
		w = x;
	});
	return 0;
}



int main()
{
	test1();
	test2();
	test3();
	return 0;
}
