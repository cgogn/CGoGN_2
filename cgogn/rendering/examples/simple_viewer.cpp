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

#include <qoglviewer.h>
#include <QApplication>
#include <QMatrix4x4>

#include <rendering/shaders/shader_simple_color.h>
#include <rendering/shaders/shader_flat.h>
#include <rendering/shaders/shader_vector_per_vertex.h>
#include <rendering/shaders/vbo.h>

#include <core/cmap/cmap2.h>
#include <io/map_import.h>
#include <geometry/algos/bounding_box.h>
#include <rendering/map_render.h>
#include <geometry/algos/normal.h>

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

using Map2 = cgogn::CMap2<cgogn::DefaultMapTraits>;
using Vec3 = Eigen::Vector3d;

template<typename T>
using VertexAttributeHandler = Map2::VertexAttributeHandler<T>;


class Viewer : public QOGLViewer
{
public:

	Map2 map;
	VertexAttributeHandler<Vec3> vertex_position_;
	VertexAttributeHandler<Vec3> vertex_normal_;

	cgogn::geometry::BoundingBox<Vec3> bb_;

	cgogn::rendering::MapRender* render_;

	virtual void draw();
	virtual void init();

	void import(const std::string& surfaceMesh);

	cgogn::rendering::VBO* vbo_pos_;
	cgogn::rendering::VBO* vbo_norm_;

	cgogn::rendering::ShaderSimpleColor* shader1_;
	cgogn::rendering::ShaderFlat* shader2_;
	cgogn::rendering::ShaderVectorPerVertex* shader3_;
};


//
// IMPLEMENTATION
//


void Viewer::import(const std::string& surfaceMesh)
{
	cgogn::io::import_surface<Vec3>(map, surfaceMesh);

	vertex_position_ = map.get_attribute<Vec3, Map2::VERTEX>("position");
	vertex_normal_ = map.add_attribute<Vec3, Map2::VERTEX>("normal");

	cgogn::geometry::compute_normal_vertices<Vec3>(map, vertex_position_, vertex_normal_);
	cgogn::geometry::compute_bounding_box(vertex_position_, bb_);

	setSceneRadius(bb_.diag_size());
	Vec3 center = bb_.center();
	setSceneCenter(qoglviewer::Vec(center[0], center[1], center[2]));
	showEntireScene();
}

void Viewer::draw()
{
	QMatrix4x4 proj;
	QMatrix4x4 view;
	camera()->getProjectionMatrix(proj);
	camera()->getModelViewMatrix(view);

	shader1_->bind();
	shader1_->set_matrices(proj,view);
	shader1_->bind_vao(0);

	glPointSize(5.0f);
	shader1_->set_color(QColor(255,0,0));
	render_->draw(cgogn::rendering::POINTS);

	shader1_->set_color(QColor(255,255,0));
	render_->draw(cgogn::rendering::LINES);

	shader1_->release_vao(0);
	shader1_->release();

	glEnable(GL_POLYGON_OFFSET_FILL) ;
	glPolygonOffset(1.0f, 1.0f) ;

	shader2_->bind();
	shader2_->set_matrices(proj,view);
	shader2_->bind_vao(0);
	render_->draw(cgogn::rendering::TRIANGLES);
	shader2_->release_vao(0);
	shader2_->release();

	shader3_->bind();
	shader3_->set_matrices(proj,view);
	shader3_->bind_vao(0);
	render_->draw(cgogn::rendering::POINTS);
	shader3_->release_vao(0);
	shader3_->release();
}

void Viewer::init()
{
	glClearColor(0.1,0.1,0.3,0.0);

	vbo_pos_ = new cgogn::rendering::VBO;
	cgogn::rendering::update_vbo(vertex_position_, *vbo_pos_);

	vbo_norm_ = new cgogn::rendering::VBO;
	cgogn::rendering::update_vbo(vertex_normal_, *vbo_norm_);

	render_ = new cgogn::rendering::MapRender();

	render_->init_primitives(map, cgogn::rendering::POINTS);
	render_->init_primitives(map, cgogn::rendering::LINES);
	render_->init_primitives(map, cgogn::rendering::TRIANGLES);

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

	shader2_->release();
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
	cgogn::thread_start();

	// Instantiate the viewer.
	Viewer viewer;
	viewer.setWindowTitle("simpleViewer");
	viewer.import(surfaceMesh);
	viewer.show();

	// Run main loop.
	return application.exec();
}
