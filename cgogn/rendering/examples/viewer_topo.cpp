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
#include <QKeyEvent>

#include <QOGLViewer/qoglviewer.h>

#include <cgogn/core/cmap/cmap2.h>

#include <cgogn/io/map_import.h>
#include <cgogn/io/map_export.h>

#include <cgogn/geometry/algos/bounding_box.h>

#include <cgogn/rendering/drawer.h>
#include <cgogn/rendering/map_render.h>
#include <cgogn/rendering/topo_drawer.h>
#include <cgogn/rendering/shaders/vbo.h>
#include <cgogn/rendering/shaders/shader_flat.h>
#include <cgogn/rendering/shaders/shader_simple_color.h>

#include <cgogn/modeling/algos/catmull_clark.h>
#include <cgogn/modeling/algos/loop.h>
#include <cgogn/modeling/algos/pliant_remeshing.h>

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

using Map2 = cgogn::CMap2;
using Vertex = Map2::Vertex;

using Vec3 = Eigen::Vector3d;
//using Vec3 = cgogn::geometry::Vec_T<std::array<float64,3>>;

template <typename T>
using VertexAttribute = Map2::VertexAttribute<T>;

class Viewer : public QOGLViewer
{
public:

	using MapRender = cgogn::rendering::MapRender;
	using TopoDrawer = cgogn::rendering::TopoDrawer;

	Viewer();
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(Viewer);

	virtual void draw();
	virtual void init();

	virtual void keyPressEvent(QKeyEvent *);
	void import(const std::string& surface_mesh);
	virtual ~Viewer();
	virtual void closeEvent(QCloseEvent *e);

private:

	Map2 map_;
	VertexAttribute<Vec3> vertex_position_;

	cgogn::geometry::AABB<Vec3> bb_;

	std::unique_ptr<MapRender> render_;

	std::unique_ptr<cgogn::rendering::VBO> vbo_pos_;

	std::unique_ptr<cgogn::rendering::ShaderFlat::Param> param_flat_;

	std::unique_ptr<TopoDrawer> topo_drawer_;
	std::unique_ptr<TopoDrawer::Renderer> topo_drawer_rend_;

	bool flat_rendering_;
	bool topo_drawering_;
};

//
// IMPLEMENTATION
//

void Viewer::import(const std::string& surface_mesh)
{
	cgogn::io::import_surface<Vec3>(map_, surface_mesh);

	vertex_position_ = map_.template get_attribute<Vec3, Map2::Vertex>("position");
	if (!vertex_position_.is_valid())
	{
		cgogn_log_error("Viewer::import") << "Missing attribute position. Aborting.";
		std::exit(EXIT_FAILURE);
	}

	cgogn::geometry::compute_AABB(vertex_position_, bb_);
	setSceneRadius(bb_.diag_size()/2.0);
	Vec3 center = bb_.center();
	setSceneCenter(qoglviewer::Vec(center[0], center[1], center[2]));
	showEntireScene();
}

Viewer::~Viewer()
{}

void Viewer::closeEvent(QCloseEvent*)
{
	render_.reset();
	vbo_pos_.reset();
	topo_drawer_.reset();
	topo_drawer_rend_.reset();
}

Viewer::Viewer() :
	map_(),
	vertex_position_(),
	bb_(),
	render_(nullptr),
	vbo_pos_(nullptr),
	topo_drawer_(nullptr),
	topo_drawer_rend_(nullptr),
	flat_rendering_(true),
	topo_drawering_(true)
{}

void Viewer::keyPressEvent(QKeyEvent *ev)
{
	switch (ev->key())
	{
		case Qt::Key_F:
			flat_rendering_ = !flat_rendering_;
			break;
		case Qt::Key_T:
			topo_drawering_ = !topo_drawering_;
			break;
		case Qt::Key_C:
			cgogn::modeling::catmull_clark<Vec3>(map_, vertex_position_);
			cgogn::rendering::update_vbo(vertex_position_, vbo_pos_.get());
			render_->init_primitives<Vec3>(map_, cgogn::rendering::TRIANGLES, &vertex_position_);
			topo_drawer_->update<Vec3>(map_, vertex_position_);
			break;
		case Qt::Key_L:
			cgogn::modeling::loop<Vec3>(map_, vertex_position_);
			cgogn::rendering::update_vbo(vertex_position_, vbo_pos_.get());
			render_->init_primitives<Vec3>(map_, cgogn::rendering::TRIANGLES, &vertex_position_);
			topo_drawer_->update<Vec3>(map_, vertex_position_);
			break;
		case Qt::Key_R:
			cgogn::modeling::pliant_remeshing<Vec3>(map_,vertex_position_);
			cgogn::rendering::update_vbo(vertex_position_, vbo_pos_.get());
			render_->init_primitives<Vec3>(map_, cgogn::rendering::TRIANGLES, &vertex_position_);
			topo_drawer_->update<Vec3>(map_,vertex_position_);
			break;
		case Qt::Key_E:
			cgogn::io::export_surface(map_,cgogn::io::ExportOptions("/tmp/pipo.vtp",{cgogn::Orbit(Map2::Vertex::ORBIT),"position"}));
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
		param_flat_->bind(proj,view);
		render_->draw(cgogn::rendering::TRIANGLES);
		param_flat_->release();
		glDisable(GL_POLYGON_OFFSET_FILL);
	}

	if (topo_drawering_)
	{
		topo_drawer_rend_->draw(proj,view,this);
	}
}

void Viewer::init()
{
	glClearColor(0.1f, 0.1f, 0.3f, 0.0f);

	vbo_pos_ = cgogn::make_unique<cgogn::rendering::VBO>(3);
	cgogn::rendering::update_vbo(vertex_position_, vbo_pos_.get());

	render_ = cgogn::make_unique<cgogn::rendering::MapRender>();
	render_->init_primitives(map_, cgogn::rendering::TRIANGLES);

	param_flat_ = cgogn::rendering::ShaderFlat::generate_param();
	param_flat_->set_position_vbo(vbo_pos_.get());
	param_flat_->front_color_ = QColor(0,150,0);
	param_flat_->back_color_ = QColor(0,0,150);
	param_flat_->ambiant_color_ = QColor(5,5,5);

	topo_drawer_ = cgogn::make_unique<cgogn::rendering::TopoDrawer>();
	topo_drawer_rend_ = topo_drawer_->generate_renderer();
	topo_drawer_->update<Vec3>(map_,vertex_position_);
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
	viewer.setWindowTitle("viewer_topo");
	viewer.import(surface_mesh);
	viewer.show();

	// Run main loop.
	return application.exec();
}
