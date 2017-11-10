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

using Map2 = cgogn::CMap2;
using Vertex = Map2::Vertex;
using Edge = Map2::Edge;

using Vec3 = Eigen::Vector3d;
//using Vec3 = cgogn::geometry::Vec_T<std::array<double,3>>;

template <typename T>
using VertexAttribute = Map2::VertexAttribute<T>;


class CustomFilter : public cgogn::CellFilters
{
public:

	CustomFilter(const VertexAttribute<Vec3>& p) : position_(p) {}

	inline bool filter(Vertex v) const
	{
		return position_[v][0] > 0;
	}

	inline cgogn::uint32 filtered_cells() const
	{
		return cgogn::orbit_mask<Vertex>();
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

	Map2::CellCache cell_cache_;
	cgogn::FilteredQuickTraversor<Map2> fqt_;
	std::unique_ptr<CustomFilter> filter_;

	cgogn::geometry::AABB<Vec3> bb_;

	std::unique_ptr<cgogn::rendering::MapRender> render_;

	std::unique_ptr<cgogn::rendering::VBO> vbo_pos_;
	std::unique_ptr<cgogn::rendering::VBO> vbo_norm_;
	std::unique_ptr<cgogn::rendering::VBO> vbo_color_;
	std::unique_ptr<cgogn::rendering::VBO> vbo_sphere_sz_;

	std::unique_ptr<cgogn::rendering::ShaderBoldLine::Param> param_edge_;
	std::unique_ptr<cgogn::rendering::ShaderFlat::Param> param_flat_;
	std::unique_ptr<cgogn::rendering::ShaderVectorPerVertex::Param> param_normal_;
	std::unique_ptr<cgogn::rendering::ShaderPhongColor::Param> param_phong_;
	std::unique_ptr<cgogn::rendering::ShaderPointSpriteSize::Param> param_point_sprite_;

	std::unique_ptr<cgogn::rendering::DisplayListDrawer> drawer_;
	std::unique_ptr<cgogn::rendering::DisplayListDrawer::Renderer> drawer_rend_;

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

	vertex_position_ = map_.template get_attribute<Vec3, Vertex>("position");
	if (!vertex_position_.is_valid())
	{
		cgogn_log_error("Viewer::import") << "Missing attribute position. Aborting.";
		std::exit(EXIT_FAILURE);
	}

	vertex_position2_ = map_.template add_attribute<Vec3, Vertex>("position2");
	map_.copy_attribute(vertex_position2_, vertex_position_);

	vertex_normal_ = map_.template add_attribute<Vec3, Vertex>("normal");
	cgogn::geometry::compute_normal<Vec3>(map_, vertex_position_, vertex_normal_);

	cell_cache_.build<Vertex>();
	cell_cache_.build<Edge>();

	fqt_.build<Vertex>();
	fqt_.set_filter<Vertex>([&] (cgogn::Dart d) { return vertex_position_[Vertex(d)][0] > 0; });

	filter_ = cgogn::make_unique<CustomFilter>(vertex_position_);

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
	filter_.reset();
	render_.reset();
	drawer_rend_.reset();
	vbo_pos_.reset();
	vbo_norm_.reset();
	vbo_color_.reset();
	vbo_sphere_sz_.reset();
	drawer_.reset();
	cgogn::rendering::ShaderProgram::clean_all();

}

Viewer::Viewer() :
	map_(),
	vertex_position_(),
	vertex_normal_(),
	cell_cache_(map_),
	fqt_(map_),
	bb_(),
	render_(nullptr),
	vbo_pos_(nullptr),
	vbo_norm_(nullptr),
	vbo_color_(nullptr),
	vbo_sphere_sz_(nullptr),
	drawer_(nullptr),
	drawer_rend_(nullptr),
	phong_rendering_(true),
	flat_rendering_(false),
	vertices_rendering_(false),
	edge_rendering_(false),
	normal_rendering_(false),
	bb_rendering_(true)
{}

void Viewer::keyPressEvent(QKeyEvent *ev)
{
	switch (ev->key())
	{
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
			cgogn::geometry::filter_average<Vec3>(map_, fqt_, vertex_position_, vertex_position2_);
//			cgogn::geometry::filter_average<Vec3>(map_, *filter_, vertex_position_, vertex_position2_);
//			cgogn::geometry::filter_average<Vec3>(map_, cell_cache_, vertex_position_, vertex_position2_);
//			cgogn::geometry::filter_average<Vec3>(map_, vertex_position_, vertex_position2_);
			map_.swap_attributes(vertex_position_, vertex_position2_);
			cgogn::geometry::compute_normal<Vec3>(map_, vertex_position_, vertex_normal_);
			cgogn::rendering::update_vbo(vertex_position_, vbo_pos_.get());
			cgogn::rendering::update_vbo(vertex_normal_, vbo_norm_.get());
			cgogn::rendering::update_vbo(vertex_normal_, vbo_color_.get(), [] (const Vec3& n) -> std::array<float,3>
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
			cgogn::geometry::compute_normal<Vec3>(map_, vertex_position_, vertex_normal_);
			cgogn::rendering::update_vbo(vertex_position_, vbo_pos_.get());
			cgogn::rendering::update_vbo(vertex_normal_, vbo_norm_.get());
			cgogn::rendering::update_vbo(vertex_normal_, vbo_color_.get(), [] (const Vec3& n) -> std::array<float,3>
			{
				return {float(std::abs(n[0])), float(std::abs(n[1])), float(std::abs(n[2]))};
			});
			update_bb();
			setSceneRadius(bb_.diag_size()/2.0);
			break;
		case Qt::Key_T:
			cgogn::geometry::filter_taubin<Vec3>(map_, cell_cache_, vertex_position_, vertex_position2_);
			cgogn::geometry::compute_normal<Vec3>(map_, vertex_position_, vertex_normal_);
			cgogn::rendering::update_vbo(vertex_position_, vbo_pos_.get());
			cgogn::rendering::update_vbo(vertex_normal_, vbo_norm_.get());
			cgogn::rendering::update_vbo(vertex_normal_, vbo_color_.get(), [] (const Vec3& n) -> std::array<float,3>
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
		drawer_rend_->draw(proj, view, this);
}

void Viewer::init()
{
	glClearColor(0.1f,0.1f,0.3f,0.0f);

	vbo_pos_ = cgogn::make_unique<cgogn::rendering::VBO>();
	cgogn::rendering::update_vbo(vertex_position_, vbo_pos_.get());

	vbo_norm_ = cgogn::make_unique<cgogn::rendering::VBO>();
	cgogn::rendering::update_vbo(vertex_normal_, vbo_norm_.get());

	// fill a color vbo with abs of normals
	vbo_color_ = cgogn::make_unique<cgogn::rendering::VBO>();
	cgogn::rendering::update_vbo(vertex_normal_, vbo_color_.get(), [] (const Vec3& n) -> std::array<float,3>
	{
		return {float(std::abs(n[0])), float(std::abs(n[1])), float(std::abs(n[2]))};
	});

	// fill a sphere size vbo
	vbo_sphere_sz_ = cgogn::make_unique<cgogn::rendering::VBO>(1);
	cgogn::rendering::update_vbo(vertex_normal_, vbo_sphere_sz_.get(), [&] (const Vec3& n) -> float
	{
		return bb_.diag_size()/1000.0 * (1.0 + 2.0*std::abs(n[2]));
	});

	render_ = cgogn::make_unique<cgogn::rendering::MapRender>();

	render_->init_primitives(map_, cgogn::rendering::POINTS);
	render_->init_primitives(map_, cgogn::rendering::LINES);
	render_->init_primitives<Vec3>(map_, cgogn::rendering::TRIANGLES, &vertex_position_);

	param_point_sprite_ = cgogn::rendering::ShaderPointSpriteSize::generate_param();
	param_point_sprite_->set_all_vbos(vbo_pos_.get(), vbo_sphere_sz_.get());
	param_point_sprite_->color_ = QColor(255, 0, 0);

//	shader_edge_ = new cgogn::rendering::ShaderBoldLine() ;
//	param_edge_ = shader_edge_->generate_param();
	param_edge_ = cgogn::rendering::ShaderBoldLine::generate_param();
	param_edge_->set_position_vbo(vbo_pos_.get());
	param_edge_->color_ = QColor(255,255,0);
	param_edge_->width_= 2.5f;

	param_flat_ =  cgogn::rendering::ShaderFlat::generate_param();
	param_flat_->set_position_vbo(vbo_pos_.get());
	param_flat_->front_color_ = QColor(0,200,0);
	param_flat_->back_color_ = QColor(0,0,200);
	param_flat_->ambiant_color_ = QColor(5,5,5);

	param_normal_ = cgogn::rendering::ShaderVectorPerVertex::generate_param();
	param_normal_->set_all_vbos(vbo_pos_.get(), vbo_norm_.get());
	param_normal_->color_ = QColor(200,0,200);
	param_normal_->length_ = bb_.diag_size()/50;

	param_phong_ = cgogn::rendering::ShaderPhongColor::generate_param();
	param_phong_->set_all_vbos(vbo_pos_.get(), vbo_norm_.get(), vbo_color_.get());

	// drawer for simple old-school g1 rendering
	drawer_ = cgogn::make_unique<cgogn::rendering::DisplayListDrawer>();
	drawer_rend_ = drawer_->generate_renderer();
	update_bb();
}

void Viewer::update_bb()
{
	cgogn::geometry::compute_AABB(vertex_position_, bb_);

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
