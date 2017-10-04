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
//#include <cgogn/core/cmap/cmap2_tri.h>
//#include <cgogn/core/cmap/cmap2_quad.h>

#include <cgogn/io/map_import.h>

#include <cgogn/geometry/algos/bounding_box.h>

#include <cgogn/rendering/map_render.h>
#include <cgogn/rendering/shaders/shader_flat.h>
#include <cgogn/rendering/shaders/vbo.h>
#include <cgogn/rendering/shaders/shader_bold_line.h>
#include <cgogn/rendering/shaders/shader_point_sprite.h>

#include <cgogn/modeling/algos/pliant_remeshing.h>
#include <cgogn/geometry/algos/length.h>

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

using Map2 = cgogn::CMap2;
//using Map2 = cgogn::CMap2Tri;
//using Map2 = cgogn::CMap2Quad;

using Vec3 = Eigen::Vector3d;
//using Vec3 = cgogn::geometry::Vec_T<std::array<double,3>>;
using Scalar = cgogn::geometry::vector_traits<Vec3>::Scalar;

using Vertex = Map2::Vertex;
template <typename T>
using VertexAttribute = Map2::VertexAttribute<T>;

class Viewer : public QOGLViewer
{
public:

	Viewer() :
		map_(),
		vertex_position_(),
		bb_(),
		render_(nullptr),
		vbo_pos_(nullptr),
		vertices_rendering_(false),
		edge_rendering_(false)
	{}

	virtual ~Viewer()
	{}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(Viewer);

	virtual void init()
	{
		glClearColor(0.1f,0.1f,0.3f,0.0f);

		vbo_pos_ = cgogn::make_unique<cgogn::rendering::VBO>(3);
		cgogn::rendering::update_vbo(vertex_position_, vbo_pos_.get());

		render_ = cgogn::make_unique<cgogn::rendering::MapRender>();
		render_->init_primitives(map_, cgogn::rendering::POINTS);
		render_->init_primitives(map_, cgogn::rendering::LINES);
		render_->init_primitives<Vec3>(map_, cgogn::rendering::TRIANGLES, &vertex_position_);

		param_flat_ = cgogn::rendering::ShaderFlat::generate_param();
		param_flat_->set_position_vbo(vbo_pos_.get());
		param_flat_->front_color_ = QColor(0,200,0);
		param_flat_->back_color_ = QColor(0,0,200);
		param_flat_->ambiant_color_ = QColor(5,5,5);

		param_edge_ = cgogn::rendering::ShaderBoldLine::generate_param();
		param_edge_->set_position_vbo(vbo_pos_.get());
		param_edge_->color_ = QColor(20,20,20);
		param_edge_->width_= 2.5f;

		param_point_sprite_ = cgogn::rendering::ShaderPointSprite::generate_param();
		param_point_sprite_->set_position_vbo(vbo_pos_.get());
		Scalar mel = cgogn::geometry::mean_edge_length<Vec3>(map_, vertex_position_);
		param_point_sprite_->size_ = mel / 6.0;
	}

	virtual void draw()
	{
		QMatrix4x4 proj;
		QMatrix4x4 view;
		camera()->getProjectionMatrix(proj);
		camera()->getModelViewMatrix(view);

		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(1.0f, 2.0f);

		param_flat_->bind(proj, view);
		render_->draw(cgogn::rendering::TRIANGLES);
		param_flat_->release();

		glDisable(GL_POLYGON_OFFSET_FILL);

		if (vertices_rendering_)
		{
			param_point_sprite_->bind(proj, view);
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
	}

	virtual void keyPressEvent(QKeyEvent* ev)
	{
		switch (ev->key())
		{
			case Qt::Key_E:
				edge_rendering_ = !edge_rendering_;
				break;
			case Qt::Key_V:
				vertices_rendering_ = !vertices_rendering_;
				break;
			case Qt::Key_R:
				cgogn::modeling::pliant_remeshing<Vec3>(map_, vertex_position_);
				cgogn::rendering::update_vbo(vertex_position_, vbo_pos_.get());
				render_->init_primitives(map_, cgogn::rendering::POINTS);
				render_->init_primitives(map_, cgogn::rendering::LINES);
				render_->init_primitives<Vec3>(map_, cgogn::rendering::TRIANGLES, &vertex_position_);
			default:
				break;
		}
		// enable QGLViewer keys
		QOGLViewer::keyPressEvent(ev);
		//update drawing
		update();
	}

	void import(const std::string& surface_mesh)
	{
		cgogn::io::import_surface<Vec3>(map_, surface_mesh);

		vertex_position_ = map_.template get_attribute<Vec3, Map2::Vertex>("position");
		if (!vertex_position_.is_valid())
		{
			cgogn_log_error("Viewer::import") << "Missing attribute position. Aborting.";
			std::exit(EXIT_FAILURE);
		}

		cgogn::geometry::compute_AABB(vertex_position_, bb_);

		setSceneRadius(bb_.diag_size() / 2.0);
		Vec3 center = bb_.center();
		setSceneCenter(qoglviewer::Vec(center[0], center[1], center[2]));
		showEntireScene();
	}

	virtual void closeEvent(QCloseEvent*)
	{
		render_.reset();
		vbo_pos_.reset();
	}

private:

	Map2 map_;
	VertexAttribute<Vec3> vertex_position_;

	cgogn::geometry::AABB<Vec3> bb_;

	std::unique_ptr<cgogn::rendering::MapRender> render_;

	std::unique_ptr<cgogn::rendering::VBO> vbo_pos_;

	std::unique_ptr<cgogn::rendering::ShaderFlat::Param> param_flat_;
	std::unique_ptr<cgogn::rendering::ShaderBoldLine::Param> param_edge_;
	std::unique_ptr<cgogn::rendering::ShaderPointSprite::Param> param_point_sprite_;

	bool vertices_rendering_;
	bool edge_rendering_;
};

int main(int argc, char** argv)
{
	std::string surface_mesh;
	if (argc < 2)
	{
		cgogn_log_info("cmap2_import") << "USAGE: " << argv[0] << " [filename]";
		surface_mesh = std::string(DEFAULT_MESH_PATH) + std::string("off/aneurysm_3D.off");
		cgogn_log_info("cmap2_import") << "Using default mesh : " << surface_mesh;
	}
	else
		surface_mesh = std::string(argv[1]);

	QApplication application(argc, argv);
	qoglviewer::init_ogl_context();

	// Instantiate the viewer.
	Viewer viewer;
	viewer.setWindowTitle("simple_viewer");
	viewer.import(surface_mesh);
	viewer.show();

	// Run main loop.
	return application.exec();
}
