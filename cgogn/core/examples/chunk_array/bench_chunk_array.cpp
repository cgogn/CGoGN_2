
#include <core/container/chunk_array_container.h>
#include <core/utils/serialization.h>
#define BLK_SZ 4096

using namespace cgogn;

int test1();
int test2();
int test3();
int test4();
int test5();

/**
 * @brief The Vec3f class: just for the example
 */
class Vec3f
{
	float data_[3];

public:

	Vec3f()
	{}

	Vec3f(float x,float y, float z)
	{
		data_[0] = x;
		data_[1] = y;
		data_[2] = z;
	}

	static std::string cgogn_name_of_type()
	{
		return "Vec3f";
	}
};

const unsigned int NB_LINES = 20000000;

int test1()
{
	std::cout << "= TEST 1 = ref unsigned char" << std::endl;

	ChunkArrayContainer<BLK_SZ, unsigned char> container;
	ChunkArray<BLK_SZ,int>* att1 = container.add_attribute<int>("entier");
	ChunkArray<BLK_SZ,float>* att2 = container.add_attribute<float>("reel");
	ChunkArray<BLK_SZ,Vec3f>* att3 = container.add_attribute<Vec3f>("Vec3f");

	for (unsigned int i = 0; i < NB_LINES; ++i)
		container.insert_lines<1>();

	for(unsigned int i = container.begin(); i != container.end(); container.next(i))
	{
		(*att1)[i] = 1+int(i);
		(*att2)[i] = 3.0f + 0.1f*float(i);
		(*att3)[i] = Vec3f(float(i), float(i), float(i));
	}

	for (unsigned int j = 0; j < 100; ++j)
	{
		for (unsigned int i = 0; i < NB_LINES/10; ++i)
		{
			container.remove_lines<1>(j%2+1+i*10);
			container.remove_lines<1>(j%2+3+i*10);
			container.remove_lines<1>(j%2+8+i*10);
		}

		for (unsigned int i = 0; i < 3*NB_LINES/10; ++i)
			container.insert_lines<1>();
	}

	std::cout << "---> OK" << std::endl;
	return 0;
}

int test2()
{
	std::cout << "= TEST 2 = ref bool" << std::endl;

	ChunkArrayContainer<BLK_SZ, bool> container;
	ChunkArray<BLK_SZ,int>* att1 = container.add_attribute<int>("entier");
	ChunkArray<BLK_SZ,float>* att2 = container.add_attribute<float>("reel");
	ChunkArray<BLK_SZ,Vec3f>* att3 = container.add_attribute<Vec3f>("Vec3f");

	for (unsigned int i = 0; i < NB_LINES; ++i)
		container.insert_lines<1>();

	for(unsigned int i = container.begin(); i != container.end(); container.next(i))
	{
		(*att1)[i] = 1+int(i);
		(*att2)[i] = 3.0f + 0.1f*float(i);
		(*att3)[i] = Vec3f(float(i), float(i), float(i));
	}

	for (unsigned int j = 0; j < 100; ++j)
	{
		for (unsigned int i = 0; i < NB_LINES/10; ++i)
		{
			container.remove_lines<1>(j%2+1+i*10);
			container.remove_lines<1>(j%2+3+i*10);
			container.remove_lines<1>(j%2+8+i*10);
		}

		for (unsigned int i = 0; i < 3*NB_LINES/10; ++i)
			container.insert_lines<1>();
	}

	std::cout << "---> OK" << std::endl;
	return 0;
}

int test3()
{
	std::cout << "= TEST 3 = random bool cleaning" << std::endl;

	ChunkArrayContainer<BLK_SZ, bool> container;
	ChunkArray<BLK_SZ,bool>* att1 = container.add_attribute<bool>("bools");

	for (unsigned int i = 0; i < NB_LINES; ++i)
		container.insert_lines<1>();

	for(unsigned int i = container.begin(); i != container.end(); container.next(i))
	{
		att1->set_value(i, true);
	}

	for (unsigned int j = 0; j < 100; ++j)
	{
		for (unsigned int i = 0; i < NB_LINES/2; ++i)
		{
			att1->set_false(i);
			att1->set_false(NB_LINES-1-i);
		}
	}

	std::cout << "---> OK" << std::endl;
	return 0;
}

int test4()
{
	std::cout << "= TEST 4 = random bool cleaning with set_false_byte" << std::endl;

	ChunkArrayContainer<BLK_SZ,  bool> container;
	ChunkArray<BLK_SZ,bool>* att1 = container.add_attribute<bool>("bools");

	for (unsigned int i = 0; i < NB_LINES; ++i)
		container.insert_lines<1>();

	for(unsigned int i = container.begin(); i != container.end(); container.next(i))
	{
		att1->set_value(i, true);
	}

	for (unsigned int j = 0; j < 100; ++j)
	{
		for (unsigned int i = 0; i < NB_LINES/2; ++i)
		{
			att1->set_false_byte(i);
			att1->set_false_byte(NB_LINES-1-i);
		}
	}

	std::cout << "---> OK" << std::endl;
	return 0;
}

int test5()
{
	std::cout << "= TEST 5 = Traversal" << std::endl;

	ChunkArrayContainer<BLK_SZ, unsigned int> container;
	ChunkArray<BLK_SZ,unsigned int>* att1 = container.add_attribute<unsigned int>("uints");

	for (unsigned int i = 0; i < NB_LINES; ++i)
		container.insert_lines<1>();

	for(unsigned int i = container.begin(); i != container.end(); container.next(i))
		att1->operator[](i) = i;

	for(unsigned int i = container.begin(); i < container.end(); i += 9)
		container.remove_lines<1>(i);

	unsigned int  total = 0;
	for (unsigned int j = 0; j < 50; ++j)
	{
		for(unsigned int i = container.begin(); i != container.end(); container.next(i))
		{
			if (att1->operator[](i) % i != 0)
				total += att1->operator[](i);
		}
		total = - total;
	}

	std::cout << "---> OK " << total << std::endl;
	return 0;
}

int main(int argc, char **argv)
{
	if (argc == 1)
	{
		std::cout << " PARAMETER: 1/2 for uint/bool refs ; 3/4 for random clear bool; 5 for traversal";
		return 1;
	}

	int c = atoi(argv[1]);
	switch(c)
	{
		case 1: test1();
			break;
		case 2: test2();
			break;
		case 3: test3();
			break;
		case 4: test4();
			break;
		case 5: test5();
			break;
		default:
			break;
	}

	return 0;
}
