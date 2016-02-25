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

#include <QApplication>
#include <QMatrix4x4>

#include <qoglviewer.h>

#include <core/cmap/cmap2.h>

#include <io/map_import.h>

#include <geometry/algos/bounding_box.h>
#include <geometry/algos/normal.h>

#include <rendering/map_render.h>
#include <rendering/shaders/shader_simple_color.h>
#include <rendering/shaders/shader_flat.h>
#include <rendering/shaders/shader_phong.h>
#include <rendering/shaders/shader_vector_per_vertex.h>
#include <rendering/shaders/vbo.h>

#include <geometry/algos/ear_triangulation.h>

#include <rendering/drawer.h>

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

using Map2 = cgogn::CMap2<cgogn::DefaultMapTraits>;
using Vec3 = Eigen::Vector3d;
//using Vec3 = cgogn::geometry::Vec_T<std::array<double,3>>;

template<typename T>
using VertexAttributeHandler = Map2::VertexAttributeHandler<T>;


class Viewer : public QOGLViewer
{
public:
	Viewer();
	Viewer(const Viewer&) = delete;
	Viewer& operator=(const Viewer&) = delete;

	virtual void draw();
	virtual void init();
	void import(const std::string& surfaceMesh);
	virtual ~Viewer();

private:
	Map2 map_;
	VertexAttributeHandler<Vec3> vertex_position_;
	VertexAttributeHandler<Vec3> vertex_normal_;

	cgogn::geometry::BoundingBox<Vec3> bb_;

	cgogn::rendering::MapRender* render_;

	cgogn::rendering::VBO* vbo_pos_;
	cgogn::rendering::VBO* vbo_norm_;
	cgogn::rendering::VBO* vbo_color_;


	cgogn::rendering::ShaderSimpleColor* shader1_;
	cgogn::rendering::ShaderFlat* shader2_;
	cgogn::rendering::ShaderVectorPerVertex* shader3_;
	cgogn::rendering::ShaderPhong* shader_phong_;

	cgogn::rendering::Drawer* drawer_;
	cgogn::rendering::Drawer* drawer2_;

};


//
// IMPLEMENTATION
//


void Viewer::import(const std::string& surfaceMesh)
{
	cgogn::io::import_surface<Vec3>(map_, surfaceMesh);

	vertex_position_ = map_.get_attribute<Vec3, Map2::Vertex::ORBIT>("position");
	vertex_normal_ = map_.add_attribute<Vec3, Map2::Vertex::ORBIT>("normal");

	cgogn::geometry::compute_normal_vertices<Vec3>(map_, vertex_position_, vertex_normal_);
	cgogn::geometry::compute_bounding_box(vertex_position_, bb_);

	setSceneRadius(bb_.diag_size());
	Vec3 center = bb_.center();
	setSceneCenter(qoglviewer::Vec(center[0], center[1], center[2]));
	showEntireScene();
}

Viewer::~Viewer()
{
	delete render_;
	delete vbo_pos_;
	delete vbo_norm_;
	delete vbo_color_;
	delete shader1_;
	delete shader2_;
	delete shader3_;
	delete shader_phong_;
}

Viewer::Viewer() :
	map_(),
	vertex_position_(),
	vertex_normal_(),
	bb_(),
	render_(nullptr),
	vbo_pos_(nullptr),
	vbo_norm_(nullptr),
	shader1_(nullptr),
	shader2_(nullptr),
	shader3_(nullptr)
{}

void Viewer::draw()
{
	QMatrix4x4 proj;
	QMatrix4x4 view;
	camera()->getProjectionMatrix(proj);
	camera()->getModelViewMatrix(view);

//	shader1_->bind();
//	shader1_->set_matrices(proj,view);
//	shader1_->bind_vao(0);

//	glPointSize(5.0f);
//	shader1_->set_color(QColor(255,0,0));
//	render_->draw(cgogn::rendering::POINTS);

//	shader1_->set_color(QColor(255,255,0));
//	render_->draw(cgogn::rendering::LINES);

//	shader1_->release_vao(0);
//	shader1_->release();

//	glEnable(GL_POLYGON_OFFSET_FILL);
//	glPolygonOffset(1.0f, 1.0f);

//	shader2_->bind();
//	shader2_->set_matrices(proj,view);
//	shader2_->bind_vao(0);
//	render_->draw(cgogn::rendering::TRIANGLES);
//	shader2_->release_vao(0);
//	shader2_->release();

//	shader3_->bind();
//	shader3_->set_matrices(proj,view);
//	shader3_->bind_vao(0);
//	render_->draw(cgogn::rendering::POINTS);
//	shader3_->release_vao(0);
//	shader3_->release();

	shader_phong_->bind();
	shader_phong_->set_matrices(proj,view);
	shader_phong_->bind_vao(0);
	render_->draw(cgogn::rendering::TRIANGLES);
	shader_phong_->release_vao(0);
	shader_phong_->release();


	drawer_->callList(proj,view);
	drawer2_->callList(proj,view);
}

void Viewer::init()
{
	glClearColor(0.1,0.1,0.3,0.0);

	vbo_pos_ = new cgogn::rendering::VBO(3);
	cgogn::rendering::update_vbo(vertex_position_, *vbo_pos_);

	vbo_norm_ = new cgogn::rendering::VBO(3);
	cgogn::rendering::update_vbo(vertex_normal_, *vbo_norm_);

	vbo_color_ = new cgogn::rendering::VBO(3);
//	cgogn::rendering::update_vbo(vertex_normal_, *vbo_color_,[] (const Vec3& n) -> std::array<float,3>
//	{
//		return {float(std::abs(n[0])), float(std::abs(n[1])), float(std::abs(n[2]))};
//	});

	cgogn::rendering::update_vbo(vertex_normal_, vertex_normal_, *vbo_color_, [] (const Vec3& n, const Vec3& n2) -> std::array<float,3>
	{
		return {float(std::abs(n[0])), float(std::abs(n[1])), float(std::abs(n2[2]))};
	});


	render_ = new cgogn::rendering::MapRender();

	render_->init_primitives<Vec3>(map_, cgogn::rendering::POINTS, vertex_position_);
	render_->init_primitives<Vec3>(map_, cgogn::rendering::LINES, vertex_position_);
	render_->init_primitives<Vec3>(map_, cgogn::rendering::TRIANGLES, vertex_position_);

	shader1_ = new cgogn::rendering::ShaderSimpleColor;
	shader1_->add_vao();
	shader1_->set_vao(0, vbo_pos_);

	shader2_ = new cgogn::rendering::ShaderFlat;
	shader2_->add_vao();
	shader2_->set_vao(0, vbo_pos_);

	shader2_->bind();
	shader2_->set_front_color(QColor(0,200,0));
	shader2_->set_back_color(QColor(0,0,200));
	shader2_->set_ambiant_color(QColor(5,5,5));
	shader2_->release();

	shader3_ = new cgogn::rendering::ShaderVectorPerVertex;
	shader3_->add_vao();
	shader3_->set_vao(0, vbo_pos_, vbo_norm_);

	shader3_->bind();
	shader3_->set_color(QColor(200,0,0));
	shader3_->set_length(bb_.diag_size()/50);
	shader3_->release();


	shader_phong_ = new cgogn::rendering::ShaderPhong(true);
	shader_phong_->add_vao();
	shader_phong_->set_vao(0, vbo_pos_, vbo_norm_, vbo_color_);

	shader_phong_->bind();
	shader_phong_->set_front_color(QColor(0,200,0));
	shader_phong_->set_back_color(QColor(0,0,200));
	shader_phong_->set_ambiant_color(QColor(5,5,5));
	shader_phong_->set_double_side(true);
	shader_phong_->set_specular_color(QColor(255,255,255));
	shader_phong_->set_specular_coef(10.0);
	shader_phong_->release();


	// drawer for simple old-school g1 rendering
	drawer_ = new cgogn::rendering::Drawer;
	drawer_->new_list();
	drawer_->begin(GL_LINES);
	drawer_->color3f(0.5,0.5,0.5);
	drawer_->vertex3fv(bb_.min().data()); // fv work with float & double
	drawer_->vertex3fv(bb_.max().data());
	drawer_->end();
	drawer_->begin(GL_LINE_LOOP);
		drawer_->color3f(1.0,1.0,1.0);
		drawer_->vertex3f(bb_.min()[0],bb_.min()[1],bb_.min()[2]);
		drawer_->vertex3f(bb_.max()[0],bb_.min()[1],bb_.min()[2]);
		drawer_->vertex3f(bb_.max()[0],bb_.max()[1],bb_.min()[2]);
		drawer_->vertex3f(bb_.min()[0],bb_.max()[1],bb_.min()[2]);
		drawer_->vertex3f(bb_.min()[0],bb_.max()[1],bb_.max()[2]);
		drawer_->vertex3f(bb_.max()[0],bb_.max()[1],bb_.max()[2]);
		drawer_->vertex3f(bb_.max()[0],bb_.min()[1],bb_.max()[2]);
		drawer_->vertex3f(bb_.min()[0],bb_.min()[1],bb_.max()[2]);
	drawer_->end();
	drawer_->begin(GL_LINES);
		drawer_->vertex3f(bb_.min()[0],bb_.min()[1],bb_.min()[2]);
		drawer_->vertex3f(bb_.min()[0],bb_.max()[1],bb_.min()[2]);
		drawer_->vertex3f(bb_.min()[0],bb_.min()[1],bb_.max()[2]);
		drawer_->vertex3f(bb_.min()[0],bb_.max()[1],bb_.max()[2]);
		drawer_->vertex3f(bb_.max()[0],bb_.min()[1],bb_.min()[2]);
		drawer_->vertex3f(bb_.max()[0],bb_.min()[1],bb_.max()[2]);
		drawer_->vertex3f(bb_.max()[0],bb_.max()[1],bb_.min()[2]);
		drawer_->vertex3f(bb_.max()[0],bb_.max()[1],bb_.max()[2]);
	drawer_->end();
	drawer_->end_list();

	drawer2_ = new cgogn::rendering::Drawer;
	drawer2_->new_list();
	drawer2_->begin(GL_TRIANGLES);
		drawer2_->color3f(0.0,1.0,1.0);
		drawer2_->vertex3f(bb_.min()[0],bb_.min()[1],bb_.min()[2]);
		drawer2_->color3f(1.0,1.0,0.0);
		drawer2_->vertex3f(bb_.min()[0],bb_.max()[1],bb_.min()[2]);
		drawer2_->color3f(1.0,0.0,1.0);
		drawer2_->vertex3f(bb_.max()[0],bb_.max()[1],bb_.min()[2]);
	drawer2_->end();

	drawer2_->pointSize(5.0);
	drawer2_->begin(GL_POINTS);
		drawer2_->color3f(0.0,1.0,1.0);
		drawer2_->vertex3f(bb_.min()[0],bb_.min()[1],bb_.max()[2]);
		drawer2_->color3f(1.0,1.0,0.0);
		drawer2_->vertex3f(bb_.min()[0],bb_.max()[1],bb_.max()[2]);
		drawer2_->color3f(1.0,0.0,1.0);
		drawer2_->vertex3f(bb_.max()[0],bb_.max()[1],bb_.max()[2]);
	drawer2_->end();

	drawer2_->end_list();
}

int main(int argc, char** argv)
{
	std::string surfaceMesh;
	if (argc < 2)
	{
		std::cout << "USAGE: " << argv[0] << " [filename]" << std::endl;
		surfaceMesh = std::string(DEFAULT_MESH_PATH) + std::string("aneurysm3D_1.off");
		std::cout << "Using default mesh : " << surfaceMesh << std::endl;
	}
	else
		surfaceMesh = std::string(argv[1]);

	QApplication application(argc, argv);
	qoglviewer::init_ogl_context();

	// Instantiate the viewer.
	Viewer viewer;
	viewer.setWindowTitle("simpleViewer");
	viewer.import(surfaceMesh);
	viewer.show();

	// Run main loop.
	return application.exec();
}
