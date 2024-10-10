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

#include <cgogn/core/cmap/cmap1.h>

//#include <cgogn/io/map_import.h>
#include <cgogn/io/map_export.h>

#include <cgogn/modeling/algos/curves.h>

#include <cgogn/geometry/algos/bounding_box.h>

#include <cgogn/rendering/map_render.h>
#include <cgogn/rendering/shaders/vbo.h>
#include <cgogn/rendering/shaders/shader_point_sprite.h>
#include <cgogn/rendering/shaders/shader_round_point.h>

using namespace cgogn;

class Viewer : public QOGLViewer
{
public:

	using Vec3 = Eigen::Vector3d;
	using Scalar = Vec3::Scalar;
	template <typename T>
	using VertexAttribute = CMap1::VertexAttribute<T>;

	using Vertex = CMap1::Vertex;

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
	CMap1 map_;
	VertexAttribute<Vec3> vertex_position_;
	Vertex v0;

	geometry::AABB<Vec3> bb_;

	std::unique_ptr<cgogn::rendering::MapRender> render_;
	std::unique_ptr<cgogn::rendering::VBO> vbo_pos_;
	std::unique_ptr<cgogn::rendering::VBO> vbo_color_;
	std::unique_ptr<cgogn::rendering::VBO> vbo_sphere_sz_;
	std::unique_ptr<cgogn::rendering::ShaderPointSpriteColorSize::Param> param_point_sprite_;

	std::unique_ptr<cgogn::rendering::ShaderBoldLine::Param> param_edge_;

	std::unique_ptr<cgogn::rendering::DisplayListDrawer> drawer_;
	std::unique_ptr<cgogn::rendering::DisplayListDrawer::Renderer> drawer_rend_;


	bool vertices_rendering_;
	bool edge_rendering_;
	bool bb_rendering_;

};

Viewer::Viewer() :
	map_(),
	vertex_position_(),
	bb_(),
	render_(nullptr),
	vbo_pos_(nullptr),
	vbo_color_(nullptr),
	vbo_sphere_sz_(nullptr),
	param_point_sprite_(nullptr),
	param_edge_(nullptr),
	drawer_(nullptr),
	drawer_rend_(nullptr),
	vertices_rendering_(false),
	edge_rendering_(false),
	bb_rendering_(true)
{
}


Viewer::~Viewer()
{
}

void Viewer::closeEvent(QCloseEvent*)
{
	render_.reset();
	vbo_pos_.reset();
	vbo_color_.reset();
	vbo_sphere_sz_.reset();
	param_point_sprite_.reset();
	param_edge_.reset();
	drawer_.reset();
	drawer_rend_.reset();

	cgogn::rendering::ShaderProgram::clean_all();
}


void Viewer::keyPressEvent(QKeyEvent *ev)
{
	bool changed = false;
	switch (ev->key())
	{
		case Qt::Key_O:
		{
			const Orbit orbv = Vertex::ORBIT;

			auto export_options = cgogn::io::ExportOptions::create()
					.filename("test.obj")
					.binary(false)
					.compress(false)
					.overwrite(true)
					.position_attribute(orbv, "position");

			cgogn::io::export_polyline(map_, export_options);
			break;
		}
		case Qt::Key_V:
			vertices_rendering_ = !vertices_rendering_;
			break;
		case Qt::Key_E:
			edge_rendering_ = !edge_rendering_;
			break;
		case Qt::Key_B:
			bb_rendering_ = !bb_rendering_;
			break;
		case Qt::Key_0:
		{
			changed = true;
			map_.clear_and_remove_attributes();
			vertex_position_ = map_.add_attribute<Vec3, Vertex>("position");

			v0 = cgogn::modeling::generate_curve<Vec3>(map_,
													   vertex_position_,
													   200,
													   0,
													   float32(10*M_PI),
													   [&](float32 t){return Vec3(t*std::cos(t),t*std::sin(t),t); });
		}
			break;
		case Qt::Key_1:
		{
			changed = true;
			map_.clear_and_remove_attributes();
			vertex_position_ = map_.add_attribute<Vec3, Vertex>("position");

			v0 = cgogn::modeling::generate_curve<Vec3>(map_,
													   vertex_position_,
													   200,
													   0,
													   float32(4*M_PI),
													   [&](float32 t){return Vec3(std::sin(2*t),std::cos(t),t); });
		}
			break;
		case Qt::Key_2:
		{
			changed = true;
			map_.clear_and_remove_attributes();
			vertex_position_ = map_.add_attribute<Vec3, Vertex>("position");

			v0 = cgogn::modeling::generate_curve<Vec3>(map_,
													   vertex_position_,
													   200,
													   0,
													   float32(10*M_PI),
													   [&](float32 t){return Vec3(std::cos(t),std::sin(t),t); });
		}
			break;
		case Qt::Key_3:
		{
			changed = true;
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
			changed = true;
			map_.clear_and_remove_attributes();
			vertex_position_ = map_.add_attribute<Vec3, Vertex>("position");

			v0 = cgogn::modeling::generate_curve<Vec3>(map_,
													   vertex_position_,
													   200,
													   0,
													   float32(10*M_PI),
													   [&](float32 t){return Vec3(t,t*std::cos(t),t*std::sin(t)); });
		}
			break;
		case Qt::Key_5:
		{
			changed = true;
			map_.clear_and_remove_attributes();
			vertex_position_ = map_.add_attribute<Vec3, Vertex>("position");

			v0 = cgogn::modeling::generate_curve<Vec3>(map_,
													   vertex_position_,
													   200,
													   0,
													   float32(2*M_PI),
													   [&](float32 t){return Vec3(3.*std::cos(t)+std::cos(10.*t)*std::cos(t),
																				  3.*std::sin(t)+std::cos(10.*t)*std::sin(t),
																				  std::sin(10.*t)); });
		}
			break;
		case Qt::Key_6:
		{
			changed = true;
			map_.clear_and_remove_attributes();
			vertex_position_ = map_.add_attribute<Vec3, Vertex>("position");

			v0 = cgogn::modeling::generate_curve<Vec3>(map_,
													   vertex_position_,
													   200,
													   0,
													   float32(2*M_PI),
													   [&](float32 t){return Vec3(t*std::cos(t), t, t*std::cos(t)); });
		}
			break;
		case Qt::Key_7:
		{
			changed = true;
			map_.clear_and_remove_attributes();
			vertex_position_ = map_.add_attribute<Vec3, Vertex>("position");

			v0 = cgogn::modeling::generate_curve<Vec3>(map_,
													   vertex_position_,
													   200,
													   0,
													   float32(2*M_PI),
													   [&](float32 t){return Vec3(5.*std::cos(t)-std::cos(5.*t), 5.*std::sin(t)-std::sin(5.*t), t); });
		}
			break;
		case Qt::Key_8:
		{
			changed = true;
			map_.clear_and_remove_attributes();
			vertex_position_ = map_.add_attribute<Vec3, Vertex>("position");

			v0 = cgogn::modeling::generate_curve<Vec3>(map_,
													   vertex_position_,
													   200,
													   0,
													   float32(2*M_PI),
													   [&](float32 t){return Vec3(2.*std::sin(3.*t)*std::cos(t), 2.*std::sin(3.*t)*std::sin(t), 0); });
		}
			break;
		case Qt::Key_9:
		{
			changed = true;
			map_.clear_and_remove_attributes();
			vertex_position_ = map_.add_attribute<Vec3, Vertex>("position");

			v0 = cgogn::modeling::generate_curve<Vec3>(map_,
													   vertex_position_,
													   200,
													   0,
													   float32(2*M_PI),
													   [&](float32 t){return Vec3(2.*std::sin(3.*t)*std::cos(t), 2.*std::sin(3.*t)*std::sin(t), std::sin(3.*t)); });
		}
			break;
		default:
			break;
	}

	if(changed)
	{
		cgogn::geometry::compute_AABB(vertex_position_, bb_);

		cgogn::rendering::update_vbo(vertex_position_, vbo_pos_.get());

		cgogn::rendering::update_vbo(vertex_position_, vbo_color_.get(), [] (const Vec3& n) -> std::array<float,3>
		{
			return {float(1.0), float(0.0), float(0.0)};
		});

		cgogn::rendering::update_vbo(vertex_position_, vbo_sphere_sz_.get(), [&] (const Vec3& n) -> float
		{
			return cgogn::geometry::diagonal(bb_).norm()/1000.0;
		});

		render_->init_primitives(map_, cgogn::rendering::POINTS);
		render_->init_primitives(map_, cgogn::rendering::LINES);

		drawer_->new_list();
		drawer_->line_width_aa(2.0);
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
		drawer_->color3f(1.0,1.0,1.0);
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


		setSceneRadius(cgogn::geometry::diagonal(bb_).norm()/2.0);
		Vec3 center = cgogn::geometry::center(bb_);
		setSceneCenter(qoglviewer::Vec(center[0], center[1], center[2]));

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

	if (vertices_rendering_)
	{
		param_point_sprite_->bind(proj,view);
		render_->draw(cgogn::rendering::POINTS);
		param_point_sprite_->release();
	}

	if (edge_rendering_)
	{
		param_edge_->bind(proj,view);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		render_->draw(cgogn::rendering::LINES);
		glDisable(GL_BLEND);
		param_edge_->release();
	}

	if (bb_rendering_)
		drawer_rend_->draw(proj,view);
}

void Viewer::init()
{
	glClearColor(1.0f,1.0f,1.0f,0.0f);

	// create and fill VBO for positions
	vbo_pos_ = cgogn::make_unique<cgogn::rendering::VBO>(3);


	// fill a color vbo with abs of normals
	vbo_color_ = cgogn::make_unique<cgogn::rendering::VBO>(3);


	// fill a sphere size vbo
	vbo_sphere_sz_ = cgogn::make_unique<cgogn::rendering::VBO>(1);

	render_ = cgogn::make_unique<cgogn::rendering::MapRender>();

	param_point_sprite_ = cgogn::rendering::ShaderPointSpriteColorSize::generate_param();
	// set vbo param (see param::set_vbo signature)
	param_point_sprite_->set_all_vbos(vbo_pos_.get(), vbo_color_.get(), vbo_sphere_sz_.get());

	param_edge_ = cgogn::rendering::ShaderBoldLine::generate_param();
	param_edge_->set_position_vbo(vbo_pos_.get());
	param_edge_->color_ = QColor(0,255,0);
	param_edge_->width_= 2.5f;

	// drawer for simple old-school g1 rendering
	drawer_ = cgogn::make_unique<cgogn::rendering::DisplayListDrawer>();
	drawer_rend_= drawer_->generate_renderer();

	setSceneRadius(0.);
	Vec3 center(0.,0.,0.);
	setSceneCenter(qoglviewer::Vec(center[0], center[1], center[2]));
	showEntireScene();
}

void Viewer::resizeGL(int w ,int h)
{
	QOGLViewer::resizeGL(w,h);
}

void Viewer::import(const std::string& filename)
{
	//	cgogn::io::import_graph<Vec3>(map_, filename);

	if (!map_.check_map_integrity())
	{
		cgogn_log_error("Viewer::import") << "Integrity of map not respected. Aborting.";
		std::exit(EXIT_FAILURE);
	}
	vertex_position_ = map_.get_attribute<Vec3, Vertex>("position");
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
