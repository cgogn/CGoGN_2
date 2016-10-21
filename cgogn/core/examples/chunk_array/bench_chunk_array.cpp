
#include <cgogn/core/container/chunk_array_container.h>
#include <cgogn/core/utils/serialization.h>
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
	float32 data_[3];

public:

	Vec3f()
	{}

	Vec3f(float32 x,float32 y, float32 z)
	{
		data_[0] = x;
		data_[1] = y;
		data_[2] = z;
	}

	static std::string cgogn_name_of_type()
	{
		return "Vec3f";
	}

	inline uint32 size() const
	{
		return 3u;
	}

	inline float32& operator[](std::size_t i)
	{
		return data_[i];
	}

	inline const float32& operator[](std::size_t i) const
	{
		return data_[i];
	}
};

const uint32 NB_LINES = 20000000;

int test1()
{
	cgogn_log_info("bench_chunk_array") << "= TEST 1 = ref unsigned char" ;

	ChunkArrayContainer<BLK_SZ, unsigned char> container;
	ChunkArray<BLK_SZ,int32>* att1 = container.add_chunk_array<int32>("entier");
	ChunkArray<BLK_SZ,float32>* att2 = container.add_chunk_array<float32>("reel");
	ChunkArray<BLK_SZ,Vec3f>* att3 = container.add_chunk_array<Vec3f>("Vec3f");

	for (uint32 i = 0; i < NB_LINES; ++i)
		container.insert_lines<1>();

	for(uint32 i = container.begin(); i != container.end(); container.next(i))
	{
		(*att1)[i] = 1+int32(i);
		(*att2)[i] = 3.0f + 0.1f*float32(i);
		(*att3)[i] = Vec3f(float32(i), float32(i), float32(i));
	}

	for (uint32 j = 0; j < 100; ++j)
	{
		for (uint32 i = 0; i < NB_LINES/10; ++i)
		{
			container.remove_lines<1>(j%2+1+i*10);
			container.remove_lines<1>(j%2+3+i*10);
			container.remove_lines<1>(j%2+8+i*10);
		}

		for (uint32 i = 0; i < 3*NB_LINES/10; ++i)
			container.insert_lines<1>();
	}

	cgogn_log_info("bench_chunk_array") << "---> OK" ;
	return 0;
}

int test2()
{
	cgogn_log_info("bench_chunk_array") << "= TEST 2 = ref bool" ;

	ChunkArrayContainer<BLK_SZ, bool> container;
	ChunkArray<BLK_SZ,int32>* att1 = container.add_chunk_array<int32>("entier");
	ChunkArray<BLK_SZ,float32>* att2 = container.add_chunk_array<float32>("reel");
	ChunkArray<BLK_SZ,Vec3f>* att3 = container.add_chunk_array<Vec3f>("Vec3f");

	for (uint32 i = 0; i < NB_LINES; ++i)
		container.insert_lines<1>();

	for(uint32 i = container.begin(); i != container.end(); container.next(i))
	{
		(*att1)[i] = 1+int32(i);
		(*att2)[i] = 3.0f + 0.1f*float32(i);
		(*att3)[i] = Vec3f(float32(i), float32(i), float32(i));
	}

	for (uint32 j = 0; j < 100; ++j)
	{
		for (uint32 i = 0; i < NB_LINES/10; ++i)
		{
			container.remove_lines<1>(j%2+1+i*10);
			container.remove_lines<1>(j%2+3+i*10);
			container.remove_lines<1>(j%2+8+i*10);
		}

		for (uint32 i = 0; i < 3*NB_LINES/10; ++i)
			container.insert_lines<1>();
	}

	cgogn_log_info("bench_chunk_array") << "---> OK" ;
	return 0;
}

int test3()
{
	cgogn_log_info("bench_chunk_array") << "= TEST 3 = random bool cleaning" ;

	ChunkArrayContainer<BLK_SZ, uint8> container;
	ChunkArrayBool<BLK_SZ>* att1 = container.add_marker_attribute();

	for (uint32 i = 0; i < NB_LINES; ++i)
		container.insert_lines<1>();

	for(uint32 i = container.begin(); i != container.end(); container.next(i))
	{
		att1->set_value(i, true);
	}

	for (uint32 j = 0; j < 100; ++j)
	{
		for (uint32 i = 0; i < NB_LINES/2; ++i)
		{
			att1->set_false(i);
			att1->set_false(NB_LINES-1-i);
		}
	}

	cgogn_log_info("bench_chunk_array") << "---> OK" ;
	return 0;
}

int test4()
{
	cgogn_log_info("bench_chunk_array") << "= TEST 4 = random bool cleaning with set_false_byte" ;

	ChunkArrayContainer<BLK_SZ, uint8> container;
	ChunkArrayBool<BLK_SZ>* att1 = container.add_marker_attribute();

	for (uint32 i = 0; i < NB_LINES; ++i)
		container.insert_lines<1>();

	for(uint32 i = container.begin(); i != container.end(); container.next(i))
	{
		att1->set_value(i, true);
	}

	for (uint32 j = 0; j < 100; ++j)
	{
		for (uint32 i = 0; i < NB_LINES/2; ++i)
		{
			att1->set_false_byte(i);
			att1->set_false_byte(NB_LINES-1-i);
		}
	}

	cgogn_log_info("bench_chunk_array") << "---> OK" ;
	return 0;
}

int test5()
{
	cgogn_log_info("bench_chunk_array") << "= TEST 5 = Traversal" ;

	ChunkArrayContainer<BLK_SZ, uint32> container;
	ChunkArray<BLK_SZ,uint32>* att1 = container.add_chunk_array<uint32>("uints");

	for (uint32 i = 0; i < NB_LINES; ++i)
		container.insert_lines<1>();

	for(uint32 i = container.begin(); i != container.end(); container.next(i))
		att1->operator[](i) = i;

	for(uint32 i = container.begin(); i < container.end(); i += 9)
		container.remove_lines<1>(i);

	uint32  total = 0;
	for (uint32 j = 0; j < 50; ++j)
	{
		for(uint32 i = container.begin(); i != container.end(); container.next(i))
		{
			if (att1->operator[](i) % i != 0)
				total += att1->operator[](i);
		}
		total = - total;
	}

	cgogn_log_info("bench_chunk_array") << "---> OK " << total ;
	return 0;
}

int main(int argc, char **argv)
{
	if (argc == 1)
	{
		cgogn_log_info("bench_chunk_array") << " PARAMETER: 1/2 for uint/bool refs; 3/4 for random clear bool; 5 for traversal";
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
