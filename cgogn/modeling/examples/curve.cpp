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
#include <chrono>

#include <QOGLViewer/qoglviewer.h>

#include <cgogn/core/graph/undirected_graph.h>

#include <cgogn/rendering/map_render.h>
#include <cgogn/rendering/shaders/shader_point_sprite.h>
#include <cgogn/rendering/shaders/shader_vector_per_vertex.h>
#include <cgogn/rendering/transparency_drawer.h>
#include <cgogn/rendering/drawer.h>
#include <cgogn/rendering/topo_drawer.h>

#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/geometry/algos/length.h>
#include <cgogn/geometry/algos/bounding_box.h>
#include <cgogn/geometry/algos/picking.h>
#include <cgogn/geometry/functions/intersection.h>

#include <cgogn/modeling/algos/curves.h>
#include <cgogn/io/map_import.h>
#include <cgogn/io/map_export.h>

using namespace cgogn;

class Viewer : public QOGLViewer
{
public:

	using Vec3 = Eigen::Vector3d;
	using Scalar = Vec3::Scalar;
	template <typename T>
	using VertexAttribute = UndirectedGraph::VertexAttribute<T>;

	using Vertex = UndirectedGraph::Vertex;

	Viewer();
	virtual ~Viewer();
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(Viewer);

	virtual void draw();
	virtual void init();
	virtual void closeEvent(QCloseEvent *e);
	virtual void resizeGL(int width, int height);

	virtual void keyPressEvent(QKeyEvent *);

	void import(const std::string& filename);

private:

	//Curve
	UndirectedGraph map_;
	VertexAttribute<Vec3> vertex_position_;
	Vertex v0;

	//NOTE Drawing debug
	std::shared_ptr<rendering::DisplayListDrawer> drawer_;
	std::unique_ptr<rendering::DisplayListDrawer::Renderer> drawer_rend_;

	std::shared_ptr<rendering::DisplayListDrawer> frame_drawer_;
	std::unique_ptr<rendering::DisplayListDrawer::Renderer> frame_drawer_rend_;

	std::chrono::time_point<std::chrono::system_clock> start_fps_;
	int nb_fps_;
	bool imported_;

	geometry::AABB<Vec3> bb_;
};


Viewer::~Viewer()
{}

void Viewer::closeEvent(QCloseEvent*)
{
}

Viewer::Viewer() :
	map_(),
	vertex_position_(),
	imported_(false)
{}

void Viewer::keyPressEvent(QKeyEvent *ev)
{
	switch (ev->key())
	{
		case Qt::Key_E:
		{
			const Orbit orbv = Vertex::ORBIT;

			auto export_options = cgogn::io::ExportOptions::create()
					.filename("test.skc")
					.binary(false)
					.compress(false)
					.overwrite(true)
					.position_attribute(orbv, "position");

			cgogn::io::export_graph(map_, export_options);
			break;
		}
		case Qt::Key_0:
		{
			map_.clear_and_remove_attributes();
			vertex_position_ = map_.add_attribute<Vec3, Vertex>("position");

			v0 = cgogn::modeling::generate_curve<Vec3>(map_,
											 vertex_position_,
											 200,
											 0.,
											 10.*M_PI,
											 [&](float32 t){return Vec3(t*std::cos(t),t*std::sin(t),t); });
		}
			break;
		case Qt::Key_1:
		{
			map_.clear_and_remove_attributes();
			vertex_position_ = map_.add_attribute<Vec3, Vertex>("position");

			v0 = cgogn::modeling::generate_curve<Vec3>(map_,
											 vertex_position_,
											 200,
											 0.,
											 4.*M_PI,
											 [&](float32 t){return Vec3(std::sin(2*t),std::cos(t),t); });
		}
			break;
		case Qt::Key_2:
		{
			map_.clear_and_remove_attributes();
			vertex_position_ = map_.add_attribute<Vec3, Vertex>("position");

			v0 = cgogn::modeling::generate_curve<Vec3>(map_,
											 vertex_position_,
											 200,
											 0.,
											 10.*M_PI,
											 [&](float32 t){return Vec3(std::cos(t),std::sin(t),t); });
		}
			break;
		case Qt::Key_3:
		{
			map_.clear_and_remove_attributes();
			vertex_position_ = map_.add_attribute<Vec3, Vertex>("position");

			v0 = cgogn::modeling::generate_curve<Vec3>(map_,
											 vertex_position_,
											 200,
											 -1.,
											 1,
											 [&](float32 t){return Vec3(t,0,t); });
		}
			break;
		case Qt::Key_4:
		{
			map_.clear_and_remove_attributes();
			vertex_position_ = map_.add_attribute<Vec3, Vertex>("position");

			v0 = cgogn::modeling::generate_curve<Vec3>(map_,
											 vertex_position_,
											 200,
											 0.,
											 10.*M_PI,
											 [&](float32 t){return Vec3(t,t*std::cos(t),t*std::sin(t)); });
		}
			break;
		case Qt::Key_5:
		{
			map_.clear_and_remove_attributes();
			vertex_position_ = map_.add_attribute<Vec3, Vertex>("position");

			v0 = cgogn::modeling::generate_curve<Vec3>(map_,
											 vertex_position_,
											 200,
											 0.,
											 2.*M_PI,
											 [&](float32 t){return Vec3(3.*std::cos(t)+std::cos(10.*t)*std::cos(t),
																		3.*std::sin(t)+std::cos(10.*t)*std::sin(t),
																		std::sin(10.*t)); });
		}
			break;
		case Qt::Key_6:
		{
			map_.clear_and_remove_attributes();
			vertex_position_ = map_.add_attribute<Vec3, Vertex>("position");

			v0 = cgogn::modeling::generate_curve<Vec3>(map_,
											 vertex_position_,
											 200,
											 0.,
											 2.*M_PI,
											 [&](float32 t){return Vec3(t*std::cos(t), t, t*std::cos(t)); });
		}
			break;
		case Qt::Key_7:
		{
			map_.clear_and_remove_attributes();
			vertex_position_ = map_.add_attribute<Vec3, Vertex>("position");

			v0 = cgogn::modeling::generate_curve<Vec3>(map_,
											 vertex_position_,
											 200,
											 0.,
											 2.*M_PI,
											 [&](float32 t){return Vec3(5.*std::cos(t)-std::cos(5.*t), 5.*std::sin(t)-std::sin(5.*t), t); });
		}
			break;
		case Qt::Key_8:
		{
			map_.clear_and_remove_attributes();
			vertex_position_ = map_.add_attribute<Vec3, Vertex>("position");

			v0 = cgogn::modeling::generate_curve<Vec3>(map_,
											 vertex_position_,
											 200,
											 0.,
											 2.*M_PI,
											 [&](float32 t){return Vec3(2.*std::sin(3.*t)*std::cos(t), 2.*std::sin(3.*t)*std::sin(t), 0.); });
		}
			break;
		case Qt::Key_9:
		{
			map_.clear_and_remove_attributes();
			vertex_position_ = map_.add_attribute<Vec3, Vertex>("position");

			v0 = cgogn::modeling::generate_curve<Vec3>(map_,
											 vertex_position_,
											 200,
											 0.,
											 2.*M_PI,
											 [&](float32 t){return Vec3(2.*std::sin(3.*t)*std::cos(t), 2.*std::sin(3.*t)*std::sin(t), std::sin(3.*t)); });
		}
			break;
		default:
			break;
	}


	drawer_->new_list();
	drawer_->ball_size(0.05f);
	drawer_->color3f(1.0,0.0,0.0);
	drawer_->begin(GL_POINTS);
	map_.foreach_cell([&](UndirectedGraph::Vertex v)
	{
		drawer_->vertex3fv(vertex_position_[v]);
	});
	drawer_->end();
	drawer_->line_width_aa(0.8);
	drawer_->begin(GL_LINES);
	drawer_->color3f(0.0,0.0,0.0);
	map_.foreach_cell([&](UndirectedGraph::Edge e)
	{
		std::pair<Vertex, Vertex> vs = map_.vertices(e);
		drawer_->vertex3fv(vertex_position_[vs.first]);
		drawer_->vertex3fv(vertex_position_[vs.second]);
	});
	drawer_->end();
	drawer_->end_list();

	geometry::compute_AABB(vertex_position_, bb_);
	setSceneRadius(bb_.diag_size()/2.0);
	Vec3 center = bb_.center();
	setSceneCenter(qoglviewer::Vec(center[0], center[1], center[2]));

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

	drawer_rend_->draw(proj, view, this);
	frame_drawer_rend_->draw(proj, view, this);
}

void Viewer::init()
{
	setSceneRadius(10.0);
	setSceneCenter(qoglviewer::Vec(0.0,0.0,0.0));
	showEntireScene();
	glClearColor(1.0f,1.0f,1.0f,0.0f);

	drawer_ = std::make_shared<cgogn::rendering::DisplayListDrawer>();
	drawer_rend_ = drawer_->generate_renderer();

	frame_drawer_ = std::make_shared<cgogn::rendering::DisplayListDrawer>();
	frame_drawer_rend_ = frame_drawer_->generate_renderer();

	if(imported_)
	{
		drawer_->new_list();
		drawer_->ball_size(0.05f);
		drawer_->color3f(1.0,0.0,0.0);
		drawer_->begin(GL_POINTS);
		map_.foreach_cell([&](UndirectedGraph::Vertex v)
		{
			drawer_->vertex3fv(vertex_position_[v]);
		});
		drawer_->end();
		drawer_->line_width_aa(0.8);
		drawer_->begin(GL_LINES);
		drawer_->color3f(0.0,0.0,0.0);
		map_.foreach_cell([&](UndirectedGraph::Edge e)
		{
			std::pair<Vertex, Vertex> vs = map_.vertices(e);
			drawer_->vertex3fv(vertex_position_[vs.first]);
			drawer_->vertex3fv(vertex_position_[vs.second]);
		});
		drawer_->end();
		drawer_->end_list();

		geometry::compute_AABB(vertex_position_, bb_);
		setSceneRadius(bb_.diag_size()/2.0);
		Vec3 center = bb_.center();
		setSceneCenter(qoglviewer::Vec(center[0], center[1], center[2]));
		showEntireScene();
	}
}

void Viewer::resizeGL(int w ,int h)
{
	QOGLViewer::resizeGL(w,h);
}

void Viewer::import(const std::string& filename)
{
	cgogn::io::import_graph<Vec3>(map_, filename);

	if (!map_.check_map_integrity())
	{
		cgogn_log_error("Viewer::import") << "Integrity of map not respected. Aborting.";
		std::exit(EXIT_FAILURE);
	}
	vertex_position_ = map_.get_attribute<Vec3, Vertex>("position");
	imported_ = true;
}

int main(int argc, char** argv)
{

	QApplication application(argc, argv);
	qoglviewer::init_ogl_context();
	// Instantiate the viewer.
	Viewer viewer;
	viewer.setWindowTitle("skeleton_viewer");

	if (argc == 2)
		viewer.import(argv[1]);

	viewer.show();

	// Run main loop.
	return application.exec();
}
