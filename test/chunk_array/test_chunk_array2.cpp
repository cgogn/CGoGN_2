
#include <core/container/chunk_array_container.h>

#include <list>

#define BLK_SZ 32

using namespace cgogn;

int test_save();
int test_load(bool with_register);


typedef std::list< std::vector< double > > ListVecDouble;
typedef std::vector< std::list < std::string > > VecListString;


int test_save()
{
	std::cout << "=============== TEST SAVE ===============" << std::endl;

	ChunkArrayContainer<BLK_SZ, unsigned int> container;

	ChunkArray<BLK_SZ,float>* att1 = container.add_attribute<float>("float");
	ChunkArray<BLK_SZ,std::string>* att4 = container.add_attribute<std::string>("std::string");
	ChunkArray<BLK_SZ,ListVecDouble>* att2 = container.add_attribute<ListVecDouble>("ListVecDouble");
	ChunkArray<BLK_SZ,VecListString>* att3 = container.add_attribute<VecListString>("VecListString");

	for (unsigned int i = 0u; i < 10u; ++i)
		container.insert_lines<1>();

	for(unsigned int i = container.begin(); i != container.end(); container.next(i))
	{
		(*att1)[i] = 0.1f*float(i);
		(*att4)[i] = std::string(3,char('Z'-i));

		(*att2)[i] = {{3.0 + 0.1*double(i),15.0 + 0.1*double(i)}, {103.0 + 0.1*double(i), 203.0 + 0.1*double(i), 303.0 + 0.1*double(i)}};

		(*att3)[i] = {{"riri","riri"},{"fifi","fifi"},{"loulou","loulou"}};

		if (i%2)
		{
			(*att2)[i].front().push_back(0.0);
			(*att2)[i].back().push_back(1.0);
		}
		else
		{
			(*att3)[i][1].push_front(std::string(3,char('A'+i)));
		}
	}
	(*att2)[9].push_back({9.0,8.,7.0});

	container.remove_lines<1>(5);


	for(unsigned int i = container.begin(); i != container.end(); container.next(i))
	{
		if (att1)
			std::cout << "FLOAT=" << (*att1)[i] << "/";

		if (att4)
			std::cout << "STR=" << (*att4)[i] << "/";

		if (att2)
		{
			std::cout << " ATT2 = ";
			for (const auto& v : (*att2)[i])
			{
				for (auto x : v)
					std::cout << x << " ";
				std::cout << "/ ";
			}
		}

		if (att3)
		{
			std::cout << " ATT3 = ";
			for (const auto& v : (*att3)[i])
			{
				for (auto x : v)
					std::cout << x << " ";
				std::cout << "/ ";
			}

		}
		std::cout << std::endl;
	}
	std::cout << "----------------------------------------" << std::endl;

	std::ofstream of("pipo.map");
	container.save(of);
	of.close();
	return 0;
}

int test_load(bool with_register)
{
	std::cout << "=============== TEST LOAD ===============" << std::endl;
	ChunkArrayContainer<BLK_SZ, unsigned int> cont2;

	if (with_register)
	{
		ChunkArrayFactory<BLK_SZ>::register_CA<ListVecDouble>();
		ChunkArrayFactory<BLK_SZ>::register_CA<VecListString>();
	}

	std::ifstream ifi("pipo.map");
	cont2.load(ifi);
	ifi.close();

	ChunkArray<BLK_SZ,float>* att1 = cont2.get_attribute<float>("float");
	ChunkArray<BLK_SZ,std::string>* att4 = cont2.get_attribute<std::string>("std::string");
	ChunkArray<BLK_SZ,ListVecDouble>* att2 = cont2.get_attribute<ListVecDouble>("ListVecDouble");
	ChunkArray<BLK_SZ,VecListString>* att3 = cont2.get_attribute<VecListString>("VecListString");

	for(unsigned int i = cont2.begin(); i != cont2.end(); cont2.next(i))
	{
		if (att1)
			std::cout << "FLOAT=" << (*att1)[i] << "/";

		if (att4)
			std::cout << "STR=" << (*att4)[i] << "/";

		if (att2)
		{
			std::cout << " ATT2 = ";
			for (const auto& v : (*att2)[i])
			{
				for (auto x : v)
					std::cout << x << " ";
				std::cout << "/ ";
			}
		}

		if (att3)
		{
			std::cout << " ATT3 = ";
			for (const auto& v : (*att3)[i])
			{
				for (auto x : v)
					std::cout << x << " ";
				std::cout << "/ ";
			}
		}
		std::cout << std::endl;
	}
	std::cout << "----------------------------------------" << std::endl;

	return 0;
}


int main(int argc, char** argv)
{
	if (argc==1)
	{
		test_save();
		test_load(false);
	}
	else if (argc==2)
	{
		if (std::string(argv[1])== "save")
			test_save();
		if (std::string(argv[1])== "load")
			test_load(false);
		if (std::string(argv[1])== "load_register")
			test_load(true);
	}
}
