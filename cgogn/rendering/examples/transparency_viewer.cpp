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

#include <cgogn/core/cmap/cmap2.h>

#include <cgogn/io/map_import.h>

#include <cgogn/geometry/algos/bounding_box.h>
#include <cgogn/geometry/algos/normal.h>

#include <cgogn/rendering/map_render.h>
#include <cgogn/rendering/shaders/shader_flat.h>
#include <cgogn/rendering/shaders/shader_point_sprite.h>

using namespace cgogn::numerics;

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

using Map2 = cgogn::CMap2<cgogn::DefaultMapTraits>;
//using Map2 = cgogn::CMap2Tri<cgogn::DefaultMapTraits>;
//using Map2 = cgogn::CMap2Quad<cgogn::DefaultMapTraits>;


using Vec3 = Eigen::Vector3d;
//using Vec3 = cgogn::geometry::Vec_T<std::array<double,3>>;

template <typename T>
using VertexAttribute = Map2::VertexAttribute<T>;


class ViewerTransparency : public QOGLViewer
{
public:

	ViewerTransparency();
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(ViewerTransparency);

	virtual void draw();
	virtual void init();

	virtual void keyPressEvent(QKeyEvent *);
	void import(const std::string& surface_mesh);
	virtual ~ViewerTransparency();
	virtual void closeEvent(QCloseEvent *e);

private:

	Map2 map_;
	VertexAttribute<Vec3> vertex_position_;
	VertexAttribute<Vec3> vertex_transfo_;
	cgogn::geometry::AABB<Vec3> bb_;
	std::unique_ptr<cgogn::rendering::MapRender> render_;
	std::unique_ptr<cgogn::rendering::VBO> vbo_pos_;
	std::unique_ptr<cgogn::rendering::ShaderFlat::Param> param_flat_;
	std::unique_ptr<cgogn::rendering::ShaderPointSprite::Param> param_point_sprite_;

	std::chrono::time_point<std::chrono::system_clock> start_fps_;
	int nb_fps_;

	bool draw_transparent_;
	bool transparent_method_2_;
};


//
// IMPLEMENTATION
//


void ViewerTransparency::import(const std::string& surface_mesh)
{
	cgogn::io::import_surface<Vec3>(map_, surface_mesh);

	vertex_position_ = map_.get_attribute<Vec3, Map2::Vertex::ORBIT>("position");
	if (!vertex_position_.is_valid())
	{
		cgogn_log_error("ViewerTransparency::import") << "Missing attribute position. Aborting.";
		std::exit(EXIT_FAILURE);
	}

	vertex_transfo_ = map_.add_attribute<Vec3, Map2::Vertex::ORBIT>("pos_tr");

	cgogn::geometry::compute_AABB(vertex_position_, bb_);
	setSceneRadius(bb_.diag_size()/2.0);
	Vec3 center = bb_.center();
	setSceneCenter(qoglviewer::Vec(center[0], center[1], center[2]));
	showEntireScene();
}


ViewerTransparency::~ViewerTransparency()
{}


void ViewerTransparency::closeEvent(QCloseEvent*)
{
	render_.reset();
	vbo_pos_.reset();
	param_flat_.reset();
	param_point_sprite_.reset();
}

ViewerTransparency::ViewerTransparency() :
	map_(),
	vertex_position_(),
	bb_(),
	render_(nullptr),
	vbo_pos_(nullptr),
	param_flat_(nullptr),
	param_point_sprite_(nullptr),
	draw_transparent_(true)
{}

void ViewerTransparency::keyPressEvent(QKeyEvent *ev)
{
	switch (ev->key())
	{
		case Qt::Key_T:
			draw_transparent_ = !draw_transparent_;
			break;
		case Qt::Key_Y:
			transparent_method_2_ = !transparent_method_2_;
		default:
			break;
	}
	// enable QGLViewer keys
	QOGLViewer::keyPressEvent(ev);
	//update drawing
	update();
}

void ViewerTransparency::draw()
{
	QMatrix4x4 proj;
	QMatrix4x4 view;
	camera()->getProjectionMatrix(proj);
	camera()->getModelViewMatrix(view);

	// begin with opaque objects
	param_point_sprite_->bind(proj,view);
	render_->draw(cgogn::rendering::POINTS);
	param_point_sprite_->release();

	if (draw_transparent_)
	{

		if (transparent_method_2_)
			render_->sort_triangles_center_z<Vec3>(vertex_position_, view);
		else
		{
			cgogn::rendering::transform_position<Vec3>(map_, vertex_position_, vertex_transfo_, view);
			//  can be done with custom projection method
			//	map_.const_attribute_container<Map2::Vertex::ORBIT>().parallel_foreach_index( [&] (uint32 i, uint32)
			//	{
			//		QVector3D P = view.map(QVector3D(vertex_position_[i][0],vertex_position_[i][1],vertex_position_[i][2]));
			//		vertex_transfo_[i]= Vec3(P[0],P[1],P[2]);
			//	});

			render_->sort_triangles_center_z(vertex_transfo_);
			//  can be done with custom compare function
			//	render_->sort_triangles([&] (const std::array<uint32,3>& ta, const std::array<uint32,3>& tb)
			//	{
			//		auto za = vertex_transfo_[ta[0]][2] + vertex_transfo_[ta[1]][2] + vertex_transfo_[ta[2]][2];
			//		auto zb = vertex_transfo_[tb[0]][2] + vertex_transfo_[tb[1]][2] + vertex_transfo_[tb[2]][2];
			//		return za < zb;
			//	});
		}

		param_flat_->bind(proj,view);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		render_->draw(cgogn::rendering::TRIANGLES);
		glDisable(GL_BLEND);
		param_flat_->release();
	}
	else
	{
		param_flat_->bind(proj,view);
		render_->draw(cgogn::rendering::TRIANGLES);
		param_flat_->release();
	}


	nb_fps_++;
	std::chrono::duration<float64> elapsed_seconds = std::chrono::system_clock::now() - start_fps_;
	if (elapsed_seconds.count()>= 5)
	{
		cgogn_log_info("fps") << double(nb_fps_)/elapsed_seconds.count();
		nb_fps_ = 0;
		start_fps_ = std::chrono::system_clock::now();
	}

}

void ViewerTransparency::init()
{
	glClearColor(0.1f,0.1f,0.1f,0.0f);

	vbo_pos_ = cgogn::make_unique<cgogn::rendering::VBO>(3);
	cgogn::rendering::update_vbo(vertex_position_, vbo_pos_.get());

	render_ = cgogn::make_unique<cgogn::rendering::MapRender>();
	render_->init_primitives<Vec3>(map_, cgogn::rendering::TRIANGLES, &vertex_position_);
	render_->init_primitives<Vec3>(map_, cgogn::rendering::POINTS, &vertex_position_);


	param_flat_ = cgogn::rendering::ShaderFlat::generate_param();
	param_flat_->set_position_vbo(vbo_pos_.get()); // use vbo out from sorting
	param_flat_->front_color_ = QColor(0,200,0,120);
	param_flat_->back_color_ = QColor(0,0,200,120);
	param_flat_->ambiant_color_ = QColor(0,0,0,0);

	param_point_sprite_ = cgogn::rendering::ShaderPointSprite::generate_param();
	param_point_sprite_->set_position_vbo(vbo_pos_.get());
	param_point_sprite_->size_ = bb_.diag_size()/1000.0f;
	param_point_sprite_->color_ = QColor(200,0,0);

	start_fps_ = std::chrono::system_clock::now();
	nb_fps_ = 0;
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
	ViewerTransparency viewer;
	viewer.setWindowTitle("simple_viewer");
	viewer.import(surface_mesh);
	viewer.show();

	// Run main loop.
	return application.exec();
}
