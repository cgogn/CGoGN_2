#include <iostream>
#include <fstream>
#include <vector>

inline unsigned int changeEndian(unsigned int x)
{
	return (x>>24) | ((x<<8) & 0x00FF0000) | ((x>>8) & 0x0000FF00) |  (x<<24);
}

int main(int argc, char **argv)
{
	if (argc < 3)
	{
		std::cerr << argv[0] << "input_ascii.off output_bin.off" << std::endl;
		return 1;
	}

	std::ifstream ifs(argv[1],std::ios::in);
	std::ofstream ofs(argv[2],std::ios::out|std::ofstream::binary);

	std::string str;
	unsigned int nv;
	unsigned int np;
	unsigned int ne;

	ifs >> str;

	if (str != "OFF")
	{
		std::cerr << "no OFF comment at begin"<< std::endl;
		return 1;
	}

	ifs >> nv;
	ifs >> np;
	ifs >> ne;

	unsigned int nv_be = changeEndian(nv);
	unsigned int np_be = changeEndian(np);
	unsigned int ne_be = changeEndian(ne);


	ofs << "OFF BINARY"<< std::endl;
	ofs.write(reinterpret_cast<char*>(&nv_be),4);
	ofs.write(reinterpret_cast<char*>(&np_be),4);
	ofs.write(reinterpret_cast<char*>(&ne_be),4);

	float* vertices = new float[3*nv];

	for (unsigned int i=0; i<nv; ++i)
	{
		ifs >> vertices[3*i]  >> vertices[3*i+1] >> vertices[3*i+2];
	}

	unsigned int* ptr = reinterpret_cast<unsigned int*>(vertices);
	for (unsigned int i=0; i<3*nv;++i)
	{
		*ptr = changeEndian(*ptr);
		ptr++;
	}


	ofs.write(reinterpret_cast<char*>(vertices),4*3*nv);

	delete[] vertices;

	std::vector<unsigned int> prim;
	prim.reserve(8*1024*1024);

	for (unsigned int i=0; i<np; ++i)
	{
		unsigned int nb;
		ifs >> nb;
		prim.push_back(nb);
		for (unsigned int j=0; j<nb; ++j)
		{
			unsigned int ind;
			ifs >> ind;
			prim.push_back(ind);
		}
	}

	ptr = reinterpret_cast<unsigned int*>(&(prim[0]));
	for (unsigned int i=0; i<prim.size();++i)
	{
		*ptr = changeEndian(*ptr);
		ptr++;
	}

	ofs.write(reinterpret_cast<char*>(&(prim[0])),4*prim.size());

	ofs.close();
	ifs.close();

	return 0;
}
