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

#include <cgogn/core/cmap/cmap2.h>

#include <cgogn/io/map_import.h>

#include <cgogn/rendering/map_render.h>
#include <cgogn/rendering/drawer.h>
#include <cgogn/rendering/shaders/vbo.h>
#include <cgogn/rendering/shaders/shader_simple_color.h>
#include <cgogn/rendering/shaders/shader_flat.h>
#include <cgogn/rendering/shaders/shader_phong.h>
#include <cgogn/rendering/shaders/shader_vector_per_vertex.h>
#include <cgogn/rendering/shaders/shader_bold_line.h>
#include <cgogn/rendering/shaders/shader_point_sprite.h>
#include <cgogn/rendering/shaders/shader_round_point.h>

#include <cgogn/geometry/algos/ear_triangulation.h>
#include <cgogn/geometry/algos/bounding_box.h>
#include <cgogn/geometry/algos/normal.h>
#include <cgogn/geometry/algos/filtering.h>

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

using Map2 = cgogn::CMap2<cgogn::DefaultMapTraits>;
using Vertex = typename Map2::Vertex;
using Edge = typename Map2::Edge;

using Vec3 = Eigen::Vector3d;
//using Vec3 = cgogn::geometry::Vec_T<std::array<double,3>>;

template <typename T>
using VertexAttribute = Map2::VertexAttribute<T>;


class CustomFilter : public cgogn::CellFilters
{
public:

	CustomFilter(const VertexAttribute<Vec3>& p) : position_(p) {}

	bool filter(Vertex v) const
	{
		return position_[v][0] > 0;
	}

protected:

	const VertexAttribute<Vec3>& position_;
};


class Viewer : public QOGLViewer
{
public:

	Viewer();
	Viewer(const Viewer&) = delete;
	Viewer& operator=(const Viewer&) = delete;

	virtual void draw();
	virtual void init();
	void update_bb();

	virtual void keyPressEvent(QKeyEvent *);
	void import(const std::string& surface_mesh);
	virtual ~Viewer();
	virtual void closeEvent(QCloseEvent *e);

private:

	Map2 map_;

	VertexAttribute<Vec3> vertex_position_;
	VertexAttribute<Vec3> vertex_position2_;
	VertexAttribute<Vec3> vertex_normal_;

	cgogn::CellCache<Map2> cell_cache_;
	CustomFilter* filter_;

	cgogn::geometry::BoundingBox<Vec3> bb_;

	cgogn::rendering::MapRender* render_;

	cgogn::rendering::VBO* vbo_pos_;
	cgogn::rendering::VBO* vbo_norm_;
	cgogn::rendering::VBO* vbo_color_;
	cgogn::rendering::VBO* vbo_sphere_sz_;

	cgogn::rendering::ShaderBoldLine* shader_edge_;
	cgogn::rendering::ShaderFlat* shader_flat_;
	cgogn::rendering::ShaderVectorPerVertex* shader_normal_;
	cgogn::rendering::ShaderPhongColor* shader_phong_;
	cgogn::rendering::ShaderPointSpriteSize* shader_point_sprite_;

	cgogn::rendering::ShaderBoldLine::Param* param_edge_;
	cgogn::rendering::ShaderFlat::Param* param_flat_;
	cgogn::rendering::ShaderVectorPerVertex::Param* param_normal_;
	cgogn::rendering::ShaderPhongColor::Param* param_phong_;
	cgogn::rendering::ShaderPointSpriteSize::Param* param_point_sprite_;

	cgogn::rendering::Drawer* drawer_;

	bool phong_rendering_;
	bool flat_rendering_;
	bool vertices_rendering_;
	bool edge_rendering_;
	bool normal_rendering_;
	bool bb_rendering_;
};



//
// IMPLEMENTATION
//


void Viewer::import(const std::string& surface_mesh)
{
	cgogn::io::import_surface<Vec3>(map_, surface_mesh);

	vertex_position_ = map_.get_attribute<Vec3, Vertex::ORBIT>("position");
	if (!vertex_position_.is_valid())
	{
		cgogn_log_error("Viewer::import") << "Missing attribute position. Aborting.";
		std::exit(EXIT_FAILURE);
	}

	vertex_position2_ = map_.add_attribute<Vec3, Vertex::ORBIT>("position2");
	map_.copy_attribute(vertex_position2_, vertex_position_);

	vertex_normal_ = map_.add_attribute<Vec3, Vertex::ORBIT>("normal");
	cgogn::geometry::compute_normal_vertices<Vec3>(map_, vertex_position_, vertex_normal_);

	cell_cache_.build<Vertex>();
	cell_cache_.build<Edge>();

	filter_ = new CustomFilter(vertex_position_);

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
	delete filter_;
	delete render_;
	delete vbo_pos_;
	delete vbo_norm_;
	delete vbo_color_;
	delete vbo_sphere_sz_;
	delete shader_edge_;
	delete shader_flat_;
	delete shader_normal_;
	delete shader_phong_;
	delete shader_point_sprite_;
	delete drawer_;
}

Viewer::Viewer() :
	map_(),
	vertex_position_(),
	vertex_normal_(),
	cell_cache_(map_),
	bb_(),
	render_(nullptr),
	vbo_pos_(nullptr),
	vbo_norm_(nullptr),
	vbo_color_(nullptr),
	vbo_sphere_sz_(nullptr),
	shader_edge_(nullptr),
	shader_flat_(nullptr),
	shader_normal_(nullptr),
	shader_phong_(nullptr),
	shader_point_sprite_(nullptr),
	drawer_(nullptr),
	phong_rendering_(true),
	flat_rendering_(false),
	vertices_rendering_(false),
	edge_rendering_(false),
	normal_rendering_(false),
	bb_rendering_(true)
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
		case Qt::Key_N:
			normal_rendering_ = !normal_rendering_;
			break;
		case Qt::Key_E:
			edge_rendering_ = !edge_rendering_;
			break;
		case Qt::Key_V:
			vertices_rendering_ = !vertices_rendering_;
			break;
//		case Qt::Key_B:
//			bb_rendering_ = !bb_rendering_;
//			break;
		case Qt::Key_A: {
			cgogn::geometry::filter_average<Vec3>(map_, *filter_, vertex_position_, vertex_position2_);
//			cgogn::geometry::filter_average<Vec3>(map_, cell_cache_, vertex_position_, vertex_position2_);
			map_.swap_attributes(vertex_position_, vertex_position2_);
			cgogn::geometry::compute_normal_vertices<Vec3>(map_, vertex_position_, vertex_normal_);
			cgogn::rendering::update_vbo(vertex_position_, vbo_pos_);
			cgogn::rendering::update_vbo(vertex_normal_, vbo_norm_);
			cgogn::rendering::update_vbo(vertex_normal_, vbo_color_, [] (const Vec3& n) -> std::array<float,3>
			{
				return {float(std::abs(n[0])), float(std::abs(n[1])), float(std::abs(n[2]))};
			});
			update_bb();
			setSceneRadius(bb_.diag_size()/2.0);
			break;
		}
		case Qt::Key_B:
			cgogn::geometry::filter_bilateral<Vec3>(map_, cell_cache_, vertex_position_, vertex_position2_, vertex_normal_);
			map_.swap_attributes(vertex_position_, vertex_position2_);
			cgogn::geometry::compute_normal_vertices<Vec3>(map_, vertex_position_, vertex_normal_);
			cgogn::rendering::update_vbo(vertex_position_, vbo_pos_);
			cgogn::rendering::update_vbo(vertex_normal_, vbo_norm_);
			cgogn::rendering::update_vbo(vertex_normal_, vbo_color_, [] (const Vec3& n) -> std::array<float,3>
			{
				return {float(std::abs(n[0])), float(std::abs(n[1])), float(std::abs(n[2]))};
			});
			update_bb();
			setSceneRadius(bb_.diag_size()/2.0);
			break;
		case Qt::Key_T:
			cgogn::geometry::filter_taubin<Vec3>(map_, cell_cache_, vertex_position_, vertex_position2_);
			cgogn::geometry::compute_normal_vertices<Vec3>(map_, vertex_position_, vertex_normal_);
			cgogn::rendering::update_vbo(vertex_position_, vbo_pos_);
			cgogn::rendering::update_vbo(vertex_normal_, vbo_norm_);
			cgogn::rendering::update_vbo(vertex_normal_, vbo_color_, [] (const Vec3& n) -> std::array<float,3>
			{
				return {float(std::abs(n[0])), float(std::abs(n[1])), float(std::abs(n[2]))};
			});
			update_bb();
			setSceneRadius(bb_.diag_size()/2.0);
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

	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.0f, 2.0f);
	if (flat_rendering_)
	{
		param_flat_->bind(proj, view);
		render_->draw(cgogn::rendering::TRIANGLES);
		param_flat_->release();
	}

	if (phong_rendering_)
	{
		param_phong_->bind(proj, view);
		render_->draw(cgogn::rendering::TRIANGLES);
		param_phong_->release();
	}
	glDisable(GL_POLYGON_OFFSET_FILL);

	if (vertices_rendering_)
	{
		param_point_sprite_->bind(proj, view);
		render_->draw(cgogn::rendering::POINTS);
		param_point_sprite_->release();
	}

	if (edge_rendering_)
	{
		param_edge_->bind(proj, view);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		render_->draw(cgogn::rendering::LINES);
		glDisable(GL_BLEND);
		param_edge_->release();
	}

	if (normal_rendering_)
	{
		param_normal_->bind(proj, view);
		render_->draw(cgogn::rendering::POINTS);
		param_normal_->release();
	}

	if (bb_rendering_)
		drawer_->call_list(proj, view, this);
}

void Viewer::init()
{
	glClearColor(0.1f,0.1f,0.3f,0.0f);

	vbo_pos_ = new cgogn::rendering::VBO();
	cgogn::rendering::update_vbo(vertex_position_, vbo_pos_);

	vbo_norm_ = new cgogn::rendering::VBO();
	cgogn::rendering::update_vbo(vertex_normal_, vbo_norm_);

	// fill a color vbo with abs of normals
	vbo_color_ = new cgogn::rendering::VBO();
	cgogn::rendering::update_vbo(vertex_normal_, vbo_color_, [] (const Vec3& n) -> std::array<float,3>
	{
		return {float(std::abs(n[0])), float(std::abs(n[1])), float(std::abs(n[2]))};
	});

	// fill a sphere size vbo
	vbo_sphere_sz_ = new cgogn::rendering::VBO(1);
	cgogn::rendering::update_vbo(vertex_normal_, vbo_sphere_sz_, [&] (const Vec3& n) -> float
	{
		return bb_.diag_size()/1000.0 * (1.0 + 2.0*std::abs(n[2]));
	});

	render_ = new cgogn::rendering::MapRender();

	render_->init_primitives<Vec3>(map_, cgogn::rendering::POINTS);
	render_->init_primitives<Vec3>(map_, cgogn::rendering::LINES);
	render_->init_primitives<Vec3>(map_, cgogn::rendering::TRIANGLES, &vertex_position_);

	shader_point_sprite_ = new cgogn::rendering::ShaderPointSpriteSize;
	param_point_sprite_ = shader_point_sprite_->generate_param();
	param_point_sprite_->set_vbo(vbo_pos_,vbo_sphere_sz_);
	param_point_sprite_->color_ = QColor(255, 0, 0);

	shader_edge_ = new cgogn::rendering::ShaderBoldLine() ;
	param_edge_ = shader_edge_->generate_param();
	param_edge_->set_vbo(vbo_pos_);
	param_edge_->color_ = QColor(255,255,0);
	param_edge_->width_= 2.5f;

	shader_flat_ = new cgogn::rendering::ShaderFlat;
	param_flat_ = shader_flat_->generate_param();
	param_flat_->set_vbo(vbo_pos_);
	param_flat_->front_color_ = QColor(0,200,0);
	param_flat_->back_color_ = QColor(0,0,200);
	param_flat_->ambiant_color_ = QColor(5,5,5);

	shader_normal_ = new cgogn::rendering::ShaderVectorPerVertex;
	param_normal_ = shader_normal_->generate_param();
	param_normal_->set_vbo(vbo_pos_, vbo_norm_);
	param_normal_->color_ = QColor(200,0,200);
	param_normal_->length_ = bb_.diag_size()/50;

	shader_phong_ = new cgogn::rendering::ShaderPhongColor;
	param_phong_ = shader_phong_->generate_param();
	param_phong_->set_vbo(vbo_pos_, vbo_norm_, vbo_color_);

	// drawer for simple old-school g1 rendering
	drawer_ = new cgogn::rendering::Drawer();
	update_bb();
}

void Viewer::update_bb()
{
	cgogn::geometry::compute_bounding_box(vertex_position_, bb_);

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
}

int main(int argc, char** argv)
{
	std::string surface_mesh;
	if (argc < 2)
	{
		cgogn_log_info("simple_viewer") << "USAGE: " << argv[0] << " [filename]";
		surface_mesh = std::string(DEFAULT_MESH_PATH) + std::string("off/aneurysm_3D.off");
		cgogn_log_info("simple_viewer") << "Using default mesh \"" << surface_mesh << "\".";
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
