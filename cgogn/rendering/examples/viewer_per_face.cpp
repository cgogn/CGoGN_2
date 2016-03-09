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
#include <QKeyEvent>

#include <core/cmap/cmap2.h>

#include <io/map_import.h>

#include <geometry/algos/bounding_box.h>
#include <geometry/algos/normal.h>

#include <rendering/map_render.h>
//#include <rendering/shaders/shader_simple_color.h>
#include <rendering/shaders/shader_flat.h>
#include <rendering/shaders/shader_phong.h>
#include <rendering/shaders/vbo.h>

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

using Map2 = cgogn::CMap2<cgogn::DefaultMapTraits>;
using Vec3 = Eigen::Vector3d;
//using Vec3 = cgogn::geometry::Vec_T<std::array<double,3>>;

template<typename T>
using VertexAttributeHandler = Map2::VertexAttributeHandler<T>;

template<typename T>
using FaceAttributeHandler = Map2::FaceAttributeHandler<T>;


class Viewer : public QOGLViewer
{
public:
	Viewer();
	Viewer(const Viewer&) = delete;
	Viewer& operator=(const Viewer&) = delete;

	virtual void draw();
	virtual void init();

	virtual void keyPressEvent(QKeyEvent *);
	void import(const std::string& surfaceMesh);
	virtual ~Viewer();
	virtual void closeEvent(QCloseEvent *e);

private:
	Map2 map_;
	VertexAttributeHandler<Vec3> vertex_position_;
	FaceAttributeHandler<Vec3> face_normal_;

	cgogn::geometry::BoundingBox<Vec3> bb_;

	cgogn::rendering::VBO* vbo_pos_;
	cgogn::rendering::VBO* vbo_norm_;
	cgogn::rendering::VBO* vbo_color_;

	cgogn::rendering::ShaderFlat* shader_flat_;
	cgogn::rendering::ShaderPhong* shader_phong_;

	bool phong_rendering_;
	bool flat_rendering_;
};


//
// IMPLEMENTATION
//


void Viewer::import(const std::string& surfaceMesh)
{
	cgogn::io::import_surface<Vec3>(map_, surfaceMesh);

	vertex_position_ = map_.get_attribute<Vec3, Map2::Vertex::ORBIT>("position");
	face_normal_ = map_.add_attribute<Vec3, Map2::Face::ORBIT>("normal");

	cgogn::geometry::compute_normal_faces<Vec3>(map_, vertex_position_, face_normal_);
	cgogn::geometry::compute_bounding_box(vertex_position_, bb_);

	setSceneRadius(bb_.diag_size()/2.0);
	Vec3 center = bb_.center();
	setSceneCenter(qoglviewer::Vec(center[0], center[1], center[2]));
	showEntireScene();
}

Viewer::~Viewer()
{}

void Viewer::closeEvent(QCloseEvent*)
{
	delete vbo_pos_;
	delete vbo_norm_;
	delete vbo_color_;
	delete shader_flat_;
	delete shader_phong_;
}

Viewer::Viewer() :
	map_(),
	vertex_position_(),
	face_normal_(),
	bb_(),
	vbo_pos_(nullptr),
	vbo_norm_(nullptr),
	vbo_color_(nullptr),
	shader_flat_(nullptr),
	shader_phong_(nullptr),
	phong_rendering_(true),
	flat_rendering_(false)
{}

void Viewer::keyPressEvent(QKeyEvent *ev)
{
	switch (ev->key()) {
		case Qt::Key_P:
			phong_rendering_ = true;
			flat_rendering_ = false;
			break;
		case Qt::Key_F:
			flat_rendering_ = true;
			phong_rendering_ = false;
			break;
		default:
			break;
	}
	// enable QGLViewer keys
	QOGLViewer::keyPressEvent(ev);
	//update drawing
	update();
}

void Viewer::draw()
{
	QMatrix4x4 proj;
	QMatrix4x4 view;
	camera()->getProjectionMatrix(proj);
	camera()->getModelViewMatrix(view);


	if (flat_rendering_)
	{
		shader_flat_->bind();
		shader_flat_->set_matrices(proj,view);
		shader_flat_->bind_vao(0);
		glDrawArrays(GL_TRIANGLES,0,vbo_pos_->size());
		shader_flat_->release_vao(0);
		shader_flat_->release();
	}

	if (phong_rendering_)
	{
		shader_phong_->bind();
		shader_phong_->set_matrices(proj,view);
		shader_phong_->bind_vao(0);
		glDrawArrays(GL_TRIANGLES,0,vbo_pos_->size());
		shader_phong_->release_vao(0);
		shader_phong_->release();
	}
}

void Viewer::init()
{
	glClearColor(0.1f,0.1f,0.3f,0.0f);


	vbo_pos_ = new cgogn::rendering::VBO(3);
	vbo_norm_ = new cgogn::rendering::VBO(3);
	vbo_color_ = new cgogn::rendering::VBO(3);

	// indices of vertices emb (f1_v1,f1_v2,f1_v3, f2_v1,f2_v2,f2_v3, f3_v1...)
	std::vector<unsigned int> ind_v;
	// indices of faces emb (f1,f1,f1, f2,f2,f2, f3,f3,f3...)
	std::vector<unsigned int> ind_f;

	// create indices ( need to be done only after topo modifications
	cgogn::rendering::create_indices_vertices_faces<Vec3>(map_,vertex_position_,ind_v,ind_f);

	// generate VBO: positions
	cgogn::rendering::generate_vbo(vertex_position_, ind_v, *vbo_pos_, [] (const Vec3& v) -> std::array<float,3>
	{
		return {float(v[0]), float(v[1]), float(v[2])};
	});

	// generate VBO: normals
	cgogn::rendering::generate_vbo(face_normal_, ind_f, *vbo_norm_, [] (const Vec3& n) -> std::array<float,3>
	{
		return {float(n[0]), float(n[1]), float(n[2])};
	});

	// generate VBO: colors (here abs of normals)
	cgogn::rendering::generate_vbo(face_normal_, ind_f, *vbo_color_, [] (const Vec3& n) -> std::array<float,3>
	{
		return {float(std::abs(n[0])), float(std::abs(n[1])), float(std::abs(n[2]))};
	});


	shader_phong_ = new cgogn::rendering::ShaderPhong(true);
	shader_phong_->add_vao();
	shader_phong_->set_vao(0, vbo_pos_, vbo_norm_, vbo_color_);
	shader_phong_->bind();
	shader_phong_->set_ambiant_color(QColor(5,5,5));
	shader_phong_->set_double_side(true);
	shader_phong_->set_specular_color(QColor(255,255,255));
	shader_phong_->set_specular_coef(100.0);
	shader_phong_->release();


	shader_flat_ = new cgogn::rendering::ShaderFlat(true);
	shader_flat_->add_vao();
	shader_flat_->set_vao(0, vbo_pos_, vbo_color_);
	shader_flat_->bind();
	shader_flat_->set_ambiant_color(QColor(5,5,5));
	shader_flat_->release();



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
