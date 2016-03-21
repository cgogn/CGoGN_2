
#include <core/container/chunk_array_container.h>

#include <list>

using namespace cgogn::numerics;


const uint32 SIZE = 32u;
template <class T>
using ChunkArray = cgogn::ChunkArray<SIZE, T>;
template <typename T>
using ChunkArrayContainer = cgogn::ChunkArrayContainer<SIZE, T>;

int test1();
int test2();
int test3();
int test4();

int test1()
{
	std::cout << "############### TEST 1 ###############" << std::endl;

	ChunkArrayContainer<uint32> container;
	ChunkArray<int>* att1 = container.add_attribute<int>("entier");
	ChunkArray<float>* att2 = container.add_attribute<float>("reel");

	for (uint32 i = 0; i < 41; ++i)
		container.insert_lines<1>();

	for(uint32 i = container.begin(); i != container.end(); container.next(i))
	{
		(*att1)[i] = 1+int(i);
		(*att2)[i] = 3.0f + 0.1f*float(i);
	}

	container.remove_lines<1>(3);
	container.remove_lines<1>(19);
	container.remove_lines<1>(35);


	for(uint32 i = container.begin(); i != container.end(); container.next(i))
	{
		std::cout << i << ": " << (*att1)[i] << " / " << (*att2)[i] << std::endl;
	}
	std::cout << "----------------------------------------" << std::endl;

	uint32 li = container.insert_lines<1>();

	(*att1)[li] = 110;
	(*att2)[li] = 123.1f;

	li = container.insert_lines<1>();

	(*att1)[li] = 111;
	(*att2)[li] = 223.1f;

	li = container.insert_lines<1>();

	(*att1)[li] = 112;
	(*att2)[li] = 323.1f;

	for(uint32 i = container.begin(); i != container.end(); container.next(i))
	{
		std::cout << i << ": " << (*att1)[i] << " / " << (*att2)[i] << std::endl;
	}
	std::cout << "----------------------------------------" << std::endl;

	container.remove_lines<1>(3);
	container.remove_lines<1>(19);
	container.remove_lines<1>(35);

	std::vector<uint32> mapOldNew;
	container.compact<1>(mapOldNew);

	for(uint32 i = container.begin(); i != container.end(); container.next(i))
	{
		std::cout << i << ": " << (*att1)[i] << " / " << (*att2)[i] << std::endl;
	}
	std::cout << "----------------------------------------" << std::endl;

	return 0;
}

int test2()
{
	std::cout << "############### TEST 2 ###############" << std::endl;

	ChunkArrayContainer<unsigned char> container;
	ChunkArray<int>* att1 = container.add_attribute<int>("entier");

	for (int i = 0; i < 13; ++i)
		container.insert_lines<3>();

	for(uint32 i = container.begin(); i != container.end(); container.next(i))
		(*att1)[i] = 1+int(i);

	for(uint32 i = container.begin(); i != container.end(); container.next(i))
	{
		std::cout << i << ": " << (*att1)[i] << std::endl;
	}
	std::cout << "----------------------------------------" << std::endl;

	container.remove_lines<3>(2);
	container.remove_lines<3>(35);

	uint32 li = container.insert_lines<3>();

	(*att1)[li] = 110;
	(*att1)[li+1] = 111;
	(*att1)[li+2] = 112;

	for(uint32 i = container.begin(); i != container.end(); container.next(i))
		std::cout << i << ": " << (*att1)[i] << std::endl;
	std::cout << "----------------------------------------" << std::endl;

	container.remove_lines<3>(8);
	container.remove_lines<3>(17);

	for(uint32 i = container.begin(); i != container.end(); container.next(i))
		std::cout << i << ": " << (*att1)[i] << std::endl;
	std::cout << "-Compact--------------------------------------" << std::endl;

	std::vector<uint32> mapOldNew;
	container.compact<3>(mapOldNew);

	for(uint32 i = container.begin(); i != container.end(); container.next(i))
		std::cout << i << ": " << (*att1)[i] << std::endl;
	std::cout << "----------------------------------------" << std::endl;

	li = container.insert_lines<3>();

	(*att1)[li] = 110;
	(*att1)[li+1] = 111;
	(*att1)[li+2] = 112;

	li = container.insert_lines<3>();

	(*att1)[li] = 210;
	(*att1)[li+1] = 211;
	(*att1)[li+2] = 212;

	for(uint32 i = container.begin(); i != container.end(); container.next(i))
		std::cout << i << ": " << (*att1)[i] << std::endl;
	std::cout << "----------------------------------------" << std::endl;

	ChunkArray<bool>* attB = container.add_attribute<bool>("bools");

	for(uint32 i = container.begin(); i != container.end(); container.next(i))
		std::cout << i << ": " << (*att1)[i]<< " / "<< (*attB)[i] << std::endl;
	std::cout << "----------------------------------------" << std::endl;

	return 0;
}

int test3()
{
	std::cout << "############### TEST 3 ###############" << std::endl;

	ChunkArrayContainer<bool> container;
	ChunkArray<int>* att1 = container.add_attribute<int>("entier");
	ChunkArray<std::vector<int> >* att2 = container.add_attribute<std::vector<int> >("V_entier");
	ChunkArray<std::list<int> >* att3 = container.add_attribute<std::list<int> >("L_entier");

	for (uint32 i = 0; i < 13; ++i)
		container.insert_lines<3>();

	std::vector<int> vect = (*att2)[0];

	for(uint32 i = container.begin(); i != container.end(); container.next(i))
	{
		(*att1)[i] = 1+int(i);
		for (uint32 j = 0; j < i; ++j)
			(*att2)[i].push_back(int(j));
		for (uint32 j = 0; j < i/2; ++j)
			(*att3)[i].push_front(int(j));
	}

	container.remove_lines<3>(3);
	container.remove_lines<3>(19);
	container.remove_lines<3>(35);

	container.insert_lines<3>();

	for(uint32 i = container.begin(); i != container.end(); container.next(i))
	{
		std::cout << i << ": " << (*att1)[i] << " // ";
		for (auto j : (*att2)[i])
			std::cout << j << ",";
		std::cout << " // ";
		for (auto j : (*att3)[i])
			std::cout << j << ",";
		std::cout << std::endl;
	}
	std::cout << "----------------------------------------" << std::endl;

	for(uint32 i = container.begin(); i != container.end(); container.next_primitive(i,3))
	{
		std::cout << i << ": " << (*att1)[i] << " // ";
		for (auto j : (*att2)[i])
			std::cout << j << ",";
		std::cout << " // ";
		for (auto j : (*att3)[i])
			std::cout << j << ",";
		std::cout << std::endl;
	}
	std::cout << "----------------------------------------" << std::endl;

	return 0;
}

int test4()
{
	std::cout << "############### TEST 4 ###############" << std::endl;
	using vecvecdouble = std::vector< std::vector< double > >;
	using veclistdouble = std::vector< std::list< double > >;
	ChunkArrayContainer<uint32> container;
	ChunkArray<int>* att1 = container.add_attribute<int>("entier");
	ChunkArray<float>* att2 = container.add_attribute<float>("reel");
	ChunkArray<bool>* att3 = container.add_attribute<bool>("bools");
	ChunkArray<vecvecdouble>* att4 = container.add_attribute<vecvecdouble>("vecvecdouble");
	ChunkArray<veclistdouble>* att5 = container.add_attribute<veclistdouble>("veclistdouble");

	for (uint32 i = 0u; i < 7u; ++i)
		container.insert_lines<3>();

	for(uint32 i = container.begin(); i != container.end(); container.next(i))
	{
		(*att1)[i] = 1+int(i);
		(*att2)[i] = 3.0f + 0.1f*float(i);
		(*att3).set_value(i, static_cast<bool>(i%2 != 0));
		(*att4)[i] = {{3.0 + 0.1*double(i),15.0 + 0.1*double(i)}, {103.0 + 0.1*double(i), 203.0 + 0.1*double(i), 303.0 + 0.1*double(i)}};
		(*att5)[i] = {{3.0 + 0.1*double(i),15.0 + 0.1*double(i)}, {103.0 + 0.1*double(i), 203.0 + 0.1*double(i), 303.0 + 0.1*double(i)}};
	}

	container.remove_lines<3>(3);
	container.remove_lines<3>(13);

	std::ofstream of("pipo.map", std::ios::binary);
	container.save(of);
	of.close();

	ChunkArrayContainer<uint32> cont2;
	std::ifstream ifi("pipo.map", std::ios::binary);
	cont2.load(ifi);
	ifi.close();

	ChunkArray<int>* load_att1 = cont2.get_attribute<int>("entier");
	ChunkArray<float>* load_att2 = cont2.get_attribute<float>("reel");
	ChunkArray<bool>* load_att3 = cont2.get_attribute<bool>("bools");
	ChunkArray<vecvecdouble>* load_att4 = cont2.get_attribute<vecvecdouble>("vecvecdouble");
	ChunkArray<veclistdouble>* load_att5 = cont2.get_attribute<veclistdouble>("veclistdouble");

	for(uint32 i = cont2.begin(); i != cont2.end(); cont2.next(i))
	{
		std::cout << i << ": " << (*load_att1)[i] << " / " << (*load_att2)[i] << " / " <<  (*load_att3)[i] << " / ";
		for (const auto& v : (*load_att4)[i])
			for (auto x : v)
				std::cout << x << " ";

		std::cout << " / ";
		for (const auto& v : (*load_att5)[i])
			for (auto x : v)
				std::cout << x << " ";
		std::cout << std::endl;

	}
	std::cout << "----------------------------------------" << std::endl;

	return 0;
}



int main()
{
	test1();
	test2();
	test3();
	test4();
}
