#include <cgogn/core/cmap/cmap2.h>
#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/modeling/tiling/square_tore.h>
//#include <cgogn/core/utils/numerics.h>

using namespace cgogn;

template <typename T>
using VertexAttribute = CMap2::VertexAttribute<T>;
using Dart = cgogn::Dart;
using CDart = CMap2::CDart;
using Vertex = CMap2::Vertex;
using Edge = CMap2::Edge;
using Face = CMap2::Face;
using Volume = CMap2::Volume;

using Vec3 = Eigen::Vector3d;
using Mat4 = Eigen::Matrix4d;



void bench_vec_iter(CMap2& cmap, VertexAttribute<Vec3>& vertex_position, int32 nba, int32 nbp)
{
	std::vector<VertexAttribute<Vec3>> va(nba);

	std::string att_name = "att_AA";
	for (auto& att : va)
	{
		att = cmap.get_attribute<Vec3, CMap2::Vertex>(att_name);
		if (!att.is_valid())
			att = cmap.add_attribute<Vec3, CMap2::Vertex>(att_name);
		if (att_name[5] == 'Z')
		{
			att_name[4]++;
			att_name[5]='A';
		}
		else
			att_name[5]++;
	}

	std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
	for (auto it = vertex_position.begin(); it != vertex_position.end();++it)
	{
		int32 vi = it.index();
		for (auto& att : va)
			att[vi] = vertex_position[vi];

		for (auto& att : va)
			vertex_position[vi] += att[vi];
		for (int i = 0; i < nbp; ++i)
		{
			for (auto& att : va)
				vertex_position[vi] -= att[vi];
			for (auto& att : va)
				vertex_position[vi] += att[vi];
		}
		vertex_position[vi] /= va.size() + 1;
	}

	std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();
	std::chrono::duration<float> elapsed_seconds = end - start;
	cgogn_log_info("vector access  iter optim") << nba << " attributes / " << nbp << " computing pass done in " << elapsed_seconds.count() << " s";
}


void bench_vec_get_index(CMap2& cmap, VertexAttribute<Vec3>& vertex_position, int32 nba, int32 nbp)
{
	std::vector<VertexAttribute<Vec3>> va(nba);

	std::string att_name = "att_AA";
	for (auto& att : va)
	{
		att = cmap.get_attribute<Vec3, CMap2::Vertex>(att_name);
		if (!att.is_valid())
			att = cmap.add_attribute<Vec3, CMap2::Vertex>(att_name);
		if (att_name[5] == 'Z')
		{
			att_name[4]++;
			att_name[5]='A';
		}
		else
			att_name[5]++;
	}

	std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
	cmap.foreach_cell([&](CMap2::Vertex v)
	{
		auto vi = cmap.embedding(v);
		for (auto& att : va)
			att[vi] = vertex_position[vi];

		for (auto& att : va)
			vertex_position[vi] += att[vi];
		for (int i = 0; i < nbp; ++i)
		{
			for (auto& att : va)
				vertex_position[vi] -= att[vi];
			for (auto& att : va)
				vertex_position[vi] += att[vi];
		}
		vertex_position[vi] /= va.size() + 1;
	});

	std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();
	std::chrono::duration<float> elapsed_seconds = end - start;
	cgogn_log_info("vector access  index optim") << nba << " attributes / " << nbp << " computing pass done in " << elapsed_seconds.count() << " s";
}

void bench_vec_direct(CMap2& cmap, VertexAttribute<Vec3>& vertex_position, int32 nba, int32 nbp)
{
	std::vector<VertexAttribute<Vec3>> va(nba);

	std::string att_name = "att_AA";
	for (auto& att : va)
	{
		att = cmap.get_attribute<Vec3, CMap2::Vertex>(att_name);
		if (!att.is_valid())
			att = cmap.add_attribute<Vec3, CMap2::Vertex>(att_name);
		if (att_name[5] == 'Z')
		{
			att_name[4]++;
			att_name[5]='A';
		}
		else
			att_name[5]++;
	}

	std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
	cmap.foreach_cell([&](CMap2::Vertex v)
	{
		for (auto& att : va)
			att[v] = vertex_position[v];

		for (auto& att : va)
			vertex_position[v] += att[v];
		for (int i = 0; i < nbp; ++i)
		{
			for (auto& att : va)
				vertex_position[v] -= att[v];
			for (auto& att : va)
				vertex_position[v] += att[v];
		}
		vertex_position[v] /= va.size() + 1;
	});

	std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();
	std::chrono::duration<float> elapsed_seconds = end - start;
	cgogn_log_info("vector access") << nba << " attributes / " << nbp << " computing pass done in " << elapsed_seconds.count() << " s";
}



void bench_mat_iter(CMap2& cmap, VertexAttribute<Mat4>& vertex_matrice, int32 nba, int32 nbp)
{
	std::vector<VertexAttribute<Mat4>> va(nba);

	std::string att_name = "mat_AA";
	for (auto& att : va)
	{
		att = cmap.get_attribute<Mat4, CMap2::Vertex>(att_name);
		if (!att.is_valid())
			att = cmap.add_attribute<Mat4, CMap2::Vertex>(att_name);
		if (att_name[5] == 'Z')
		{
			att_name[4]++;
			att_name[5]='A';
		}
		else
			att_name[5]++;
	}

	std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
	for (auto it = vertex_matrice.begin(); it != vertex_matrice.end();++it)
	{
		int32 vi = it.index();
		for (auto& att : va)
			att[vi] << 1,0,1,0, 2,1,2,1, 3,2,3,2, 4,3,4,3;

		for (auto& att : va)
			vertex_matrice[vi] += att[vi];
		for (int i = 0; i < nbp; ++i)
		{
			for (auto& att : va)
				vertex_matrice[vi] *= att[vi];
			for (auto& att : va)
				vertex_matrice[vi] -= att[vi];
		}
		vertex_matrice[vi] /= va.size() + 1;
	};

	std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();
	std::chrono::duration<float> elapsed_seconds = end - start;
	cgogn_log_info("matrice access iter optim") << nba << " attributes / " << nbp << " computing pass done in " << elapsed_seconds.count() << " s";
}


void bench_mat_get_index(CMap2& cmap, VertexAttribute<Mat4>& vertex_matrice, int32 nba, int32 nbp)
{
	std::vector<VertexAttribute<Mat4>> va(nba);

	std::string att_name = "mat_AA";
	for (auto& att : va)
	{
		att = cmap.get_attribute<Mat4, CMap2::Vertex>(att_name);
		if (!att.is_valid())
			att = cmap.add_attribute<Mat4, CMap2::Vertex>(att_name);
		if (att_name[5] == 'Z')
		{
			att_name[4]++;
			att_name[5]='A';
		}
		else
			att_name[5]++;
	}

	std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
	cmap.foreach_cell([&](CMap2::Vertex v)
	{
		auto vi = cmap.embedding(v);
		for (auto& att : va)
			att[vi] << 1,0,1,0, 2,1,2,1, 3,2,3,2, 4,3,4,3;

		for (auto& att : va)
			vertex_matrice[vi] += att[vi];
		for (int i = 0; i < nbp; ++i)
		{
			for (auto& att : va)
				vertex_matrice[vi] *= att[vi];
			for (auto& att : va)
				vertex_matrice[vi] -= att[vi];
		}
		vertex_matrice[vi] /= va.size() + 1;
	});

	std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();
	std::chrono::duration<float> elapsed_seconds = end - start;
	cgogn_log_info("matrice access index optim") << nba << " attributes / " << nbp << " computing pass done in " << elapsed_seconds.count() << " s";
}


void bench_mat_direct(CMap2& cmap, VertexAttribute<Mat4>& vertex_matrice, int32 nba, int32 nbp)
{
	std::vector<VertexAttribute<Mat4>> va(nba);

	std::string att_name = "mat_AA";
	for (auto& att : va)
	{
		att = cmap.get_attribute<Mat4, CMap2::Vertex>(att_name);
		if (!att.is_valid())
			att = cmap.add_attribute<Mat4, CMap2::Vertex>(att_name);
		if (att_name[5] == 'Z')
		{
			att_name[4]++;
			att_name[5]='A';
		}
		else
			att_name[5]++;
	}

	std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
	cmap.foreach_cell([&](CMap2::Vertex v)
	{
		for (auto& att : va)
			att[v] << 1,0,1,0, 2,1,2,1, 3,2,3,2, 4,3,4,3;

		for (auto& att : va)
			vertex_matrice[v] += att[v];
		for (int i = 0; i < nbp; ++i)
		{
			for (auto& att : va)
				vertex_matrice[v] *= att[v];
			for (auto& att : va)
				vertex_matrice[v] -= att[v];
		}
		vertex_matrice[v] /= va.size() + 1;
	});

	std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();
	std::chrono::duration<float> elapsed_seconds = end - start;
	cgogn_log_info("matrice access ") << nba << " attributes / " << nbp << " computing pass done in " << elapsed_seconds.count() << " s";
}


int main()
{

	CMap2 cmap;
	VertexAttribute<Vec3> vertex_position = cmap.add_attribute<Vec3, Vertex>("vertices");
	VertexAttribute<Mat4> vertex_matrice = cmap.add_attribute<Mat4, Vertex>("matrices");

	cgogn::modeling::SquareGrid<CMap2> grid(cmap, 500, 500);

	bench_vec_iter(cmap, vertex_position,100,10);
	bench_vec_get_index(cmap, vertex_position,100,10);
	bench_vec_direct(cmap, vertex_position,100,10);

	bench_vec_iter(cmap, vertex_position,4,200);
	bench_vec_get_index(cmap, vertex_position,4,200);
	bench_vec_direct(cmap, vertex_position,4,200);

	bench_mat_iter(cmap, vertex_matrice,40,20);
	bench_mat_get_index(cmap, vertex_matrice,40,20);
	bench_mat_direct(cmap, vertex_matrice,40,20);

	bench_mat_iter(cmap, vertex_matrice,4,200);
	bench_mat_get_index(cmap, vertex_matrice,4,200);
	bench_mat_direct(cmap, vertex_matrice,4,200);

	return 0;
}
