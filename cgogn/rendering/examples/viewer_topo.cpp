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

#include <rendering/drawer.h>
#include <rendering/map_render.h>
#include <rendering/topo_render.h>
#include <rendering/shaders/vbo.h>
#include <rendering/shaders/shader_flat.h>
#include <rendering/shaders/shader_simple_color.h>

#include <modeling/algos/catmull_clark.h>
#include <modeling/algos/loop.h>
#include <modeling/algos/pliant_remeshing.h>

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

using Map2 = cgogn::CMap2<cgogn::DefaultMapTraits>;
using Vec3 = Eigen::Vector3d;
//using Vec3 = cgogn::geometry::Vec_T<std::array<float64,3>>;

template<typename T>
using VertexAttribute = Map2::VertexAttribute<T>;


class Viewer : public QOGLViewer
{
public:

	Viewer();
	Viewer(const Viewer&) = delete;
	Viewer& operator=(const Viewer&) = delete;

	virtual void draw();
	virtual void init();

	virtual void keyPressEvent(QKeyEvent *);
	void import(const std::string& surface_mesh);
	virtual ~Viewer();
	virtual void closeEvent(QCloseEvent *e);

private:

	Map2 map_;
	VertexAttribute<Vec3> vertex_position_;

	cgogn::geometry::BoundingBox<Vec3> bb_;

	cgogn::rendering::MapRender* render_;
	cgogn::rendering::VBO* vbo_pos_;
	cgogn::rendering::ShaderFlat* shader_flat_;

	cgogn::rendering::TopoRender* topo_render;

	bool flat_rendering_;
	bool topo_rendering_;
};


//
// IMPLEMENTATION
//


void Viewer::import(const std::string& surface_mesh)
{
	cgogn::io::import_surface<Vec3>(map_, surface_mesh);

	vertex_position_ = map_.get_attribute<Vec3, Map2::Vertex::ORBIT>("position");
	if (!vertex_position_.is_valid())
	{
		cgogn_log_error("Viewer::import") << "Missing attribute position. Aborting.";
		std::exit(EXIT_FAILURE);
	}

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
	delete render_;
	delete vbo_pos_;
	delete shader_flat_;
	delete topo_render;
}

Viewer::Viewer() :
	map_(),
	vertex_position_(),
	bb_(),
	render_(nullptr),
	vbo_pos_(nullptr),
	shader_flat_(nullptr),
	topo_render(nullptr),
	flat_rendering_(true),
	topo_rendering_(true)
{}

void Viewer::keyPressEvent(QKeyEvent *ev)
{
	switch (ev->key()) {
		case Qt::Key_F:
			flat_rendering_ = !flat_rendering_;
			break;
		case Qt::Key_T:
			topo_rendering_ = !topo_rendering_;
			break;
		case Qt::Key_C:
			cgogn::modeling::catmull_clark<Vec3>(map_, vertex_position_);
			cgogn::rendering::update_vbo(vertex_position_, *vbo_pos_);
			render_->init_primitives<Vec3>(map_, cgogn::rendering::TRIANGLES, vertex_position_);
			topo_render->update_map2<Vec3>(map_, vertex_position_);
			break;
		case Qt::Key_L:
			cgogn::modeling::loop<Vec3>(map_, vertex_position_);
			cgogn::rendering::update_vbo(vertex_position_, *vbo_pos_);
			render_->init_primitives<Vec3>(map_, cgogn::rendering::TRIANGLES, vertex_position_);
			topo_render->update_map2<Vec3>(map_, vertex_position_);
			break;
		case Qt::Key_R:
			cgogn::modeling::pliant_remeshing<Vec3>(map_,vertex_position_);
			cgogn::rendering::update_vbo(vertex_position_, *vbo_pos_);
			render_->init_primitives<Vec3>(map_, cgogn::rendering::TRIANGLES, vertex_position_);
			topo_render->update_map2<Vec3>(map_,vertex_position_);
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
		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(1.0f, 1.0f);
		shader_flat_->bind();
		shader_flat_->set_matrices(proj,view);
		shader_flat_->bind_vao(0);
		render_->draw(cgogn::rendering::TRIANGLES);
		shader_flat_->release_vao(0);
		shader_flat_->release();
		glDisable(GL_POLYGON_OFFSET_FILL);
	}

	if (topo_rendering_)
	{
		topo_render->draw(proj,view);
	}
}

void Viewer::init()
{
	glClearColor(0.1f,0.1f,0.3f,0.0f);

	vbo_pos_ = new cgogn::rendering::VBO(3);
	cgogn::rendering::update_vbo(vertex_position_, *vbo_pos_);

	render_ = new cgogn::rendering::MapRender();
	render_->init_primitives<Vec3>(map_, cgogn::rendering::TRIANGLES, vertex_position_);

	shader_flat_ = new cgogn::rendering::ShaderFlat;
	shader_flat_->add_vao();
	shader_flat_->set_vao(0, vbo_pos_);
	shader_flat_->bind();
	shader_flat_->set_front_color(QColor(0,150,0));
	shader_flat_->set_back_color(QColor(0,0,150));
	shader_flat_->set_ambiant_color(QColor(5,5,5));
	shader_flat_->release();

	topo_render = new cgogn::rendering::TopoRender(this);
	topo_render->update_map2<Vec3>(map_,vertex_position_);
}

int main(int argc, char** argv)
{
	std::string surface_mesh;
	if (argc < 2)
	{
		cgogn_log_info("viewer_topo")<< "USAGE: " << argv[0] << " [filename]";
		surface_mesh = std::string(DEFAULT_MESH_PATH) + std::string("off/aneurysm_3D.off");
		cgogn_log_info("viewer_topo") << "Using default mesh \"" << surface_mesh << "\".";
	}
	else
		surface_mesh = std::string(argv[1]);

	QApplication application(argc, argv);
	qoglviewer::init_ogl_context();

	// Instantiate the viewer.
	Viewer viewer;
	viewer.setWindowTitle("simpleViewer");
	viewer.import(surface_mesh);
	viewer.show();

	// Run main loop.
	return application.exec();
}
