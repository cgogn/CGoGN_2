
#include "core/container/chunk_array_container.h"

#define BLK_SZ 32

using namespace cgogn;


int test1()
{
	std::cout << "=============== TEST 1 ===============" << std::endl;

	ChunkArrayContainer<BLK_SZ, 1, unsigned int> container;
	ChunkArray<BLK_SZ,int>* att1 = container.addAttribute<int>("entier");
	ChunkArray<BLK_SZ,float>* att2 = container.addAttribute<float>("reel");


	for (int i=0;i<21;++i)
		container.insertLines();


	for(unsigned int i=container.begin(); i!=container.end(); container.next(i))
	{
		(*att1)[i] = 1+i;
		(*att2)[i] = 3.0f + 0.1f*float(i);
	}

	container.removeLines(3);
	container.removeLines(11);
	container.removeLines(19);


	for(unsigned int i=container.begin(); i!=container.end(); container.next(i))
	{
		std::cout << i << ": "<< (*att1)[i] << " / " << (*att2)[i] << std::endl;
	}
	std::cout << "----------------------------------------" << std::endl;

	unsigned int li = container.insertLines();

	(*att1)[li] = 110;
	(*att2)[li] = 123.1f;

	li = container.insertLines();

	(*att1)[li] = 111;
	(*att2)[li] = 223.1f;

	li = container.insertLines();

	(*att1)[li] = 112;
	(*att2)[li] = 323.1f;

	for(unsigned int i=container.begin(); i!=container.end(); container.next(i))
	{
		std::cout << i << ": "<< (*att1)[i] << " / " << (*att2)[i] << std::endl;
	}
	std::cout << "----------------------------------------" << std::endl;

	container.removeLines(3);
	container.removeLines(11);
	container.removeLines(19);

	std::vector<unsigned int> mapOldNew;
	container.compact(mapOldNew);

	for(unsigned int i=container.begin(); i!=container.end(); container.next(i))
	{
		std::cout << i << ": "<< (*att1)[i] << " / " << (*att2)[i] << std::endl;
	}
	std::cout << "----------------------------------------" << std::endl;


	return 0;
}



int test2()
{
	std::cout << "=============== TEST 2 ===============" << std::endl;

	ChunkArrayContainer<BLK_SZ, 3, unsigned char> container;
	ChunkArray<BLK_SZ,int>* att1 = container.addAttribute<int>("entier");

	for (int i=0;i<7;++i)
		container.insertLines();

	for(unsigned int i=container.begin(); i!=container.end(); container.next(i))
		(*att1)[i] = 1+i;

	for(unsigned int i=container.begin(); i!=container.end(); container.next(i))
	{
		std::cout << i << ": "<< (*att1)[i] << std::endl;
	}
	std::cout << "----------------------------------------" << std::endl;

	container.removeLines(2);
	container.removeLines(15);

	unsigned int li = container.insertLines();

	(*att1)[li] = 110;
	(*att1)[li+1] = 111;
	(*att1)[li+2] = 112;

	for(unsigned int i=container.begin(); i!=container.end(); container.next(i))
		std::cout << i << ": "<< (*att1)[i] << std::endl;
	std::cout << "----------------------------------------" << std::endl;


	container.removeLines(8);
	container.removeLines(17);

	for(unsigned int i=container.begin(); i!=container.end(); container.next(i))
		std::cout << i << ": "<< (*att1)[i] << std::endl;
	std::cout << "-Compact--------------------------------------" << std::endl;

	std::vector<unsigned int> mapOldNew;
	container.compact(mapOldNew);

	for(unsigned int i=container.begin(); i!=container.end(); container.next(i))
		std::cout << i << ": "<< (*att1)[i] << std::endl;
	std::cout << "----------------------------------------" << std::endl;

	li = container.insertLines();

	(*att1)[li] = 110;
	(*att1)[li+1] = 111;
	(*att1)[li+2] = 112;

	li = container.insertLines();

	(*att1)[li] = 210;
	(*att1)[li+1] = 211;
	(*att1)[li+2] = 212;

	for(unsigned int i=container.begin(); i!=container.end(); container.next(i))
		std::cout << i << ": "<< (*att1)[i] << std::endl;
	std::cout << "----------------------------------------" << std::endl;


	ChunkArray<BLK_SZ,bool>* attB = container.addAttribute<bool>("bools");

	for(unsigned int i=container.begin(); i!=container.end(); container.next(i))
		std::cout << i << ": "<< (*att1)[i]<< " / "<< (*attB)[i] << std::endl;
	std::cout << "----------------------------------------" << std::endl;


	return 0;

}


int test3()
{
	std::cout << "=============== TEST 3 ===============" << std::endl;

	ChunkArrayContainer<BLK_SZ, 3, bool> container;
	ChunkArray<BLK_SZ,int>* att1 = container.addAttribute<int>("entier");

	for (int i=0;i<7;++i)
		container.insertLines();

	for(unsigned int i=container.begin(); i!=container.end(); container.next(i))
		(*att1)[i] = 1+i;

	container.removeLines(3);
	container.removeLines(11);
	container.removeLines(19);

	for(unsigned int i=container.begin(); i!=container.end(); container.next(i))
	{
		std::cout << i << ": "<< (*att1)[i] << std::endl;
	}
	std::cout << "----------------------------------------" << std::endl;

	return 0;
}


int test4()
{
	std::cout << "=============== TEST 4 ===============" << std::endl;

	ChunkArrayFactory<BLK_SZ>::registerCA("float",new ChunkArray<BLK_SZ,float>());
	ChunkArrayFactory<BLK_SZ>::registerCA("int",new ChunkArray<BLK_SZ,int>());
	ChunkArrayFactory<BLK_SZ>::registerCA("char",new ChunkArray<BLK_SZ,char>());


	ChunkArrayContainer<BLK_SZ, 3, unsigned int> container;
	ChunkArray<BLK_SZ,float>* att2 = container.addAttribute<float>("reel");
	ChunkArray<BLK_SZ,int>* att1 = container.addAttribute<int>("entier");

//	ChunkArrayGen<BLK_SZ>* att3 = att2->clone();


	for (int i=0;i<7;++i)
		container.insertLines();


	for(unsigned int i=container.begin(); i!=container.end(); container.next(i))
	{
		(*att1)[i] = 1+i;
		(*att2)[i] = 3.0f + 0.1f*float(i);
	}

	container.removeLines(3);
	container.removeLines(13);


	std::ofstream of("pipo.map");
	container.save(of);
	of.close();

	ChunkArrayContainer<BLK_SZ, 1, unsigned int> cont2;
	std::ifstream ifi("pipo.map");
	cont2.load(ifi);
	ifi.close();

	ChunkArray<BLK_SZ,int>* load_att1 = cont2.getAttribute<int>("entier");
	ChunkArray<BLK_SZ,float>* load_att2 = cont2.getAttribute<float>("reel");


	for(unsigned int i=cont2.begin(); i!=cont2.end(); cont2.next(i))
	{
		std::cout << i << ": "<< (*load_att1)[i] << " / " << (*load_att2)[i] << std::endl;
	}
	std::cout << "----------------------------------------" << std::endl;


	ChunkArrayFactory<BLK_SZ>::clean();

	return 0;
}


int main()
{
	test1();
	test2();
	test3();
	test4();
}
