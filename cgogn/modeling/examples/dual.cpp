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
#include <cgogn/rendering/map_render.h>
#include <cgogn/rendering/shaders/shader_flat.h>
#include <cgogn/rendering/shaders/vbo.h>
#include <cgogn/rendering/shaders/shader_bold_line.h>
#include <cgogn/rendering/shaders/shader_point_sprite.h>

#include <cgogn/modeling/algos/dual.h>
#include <cgogn/geometry/algos/centroid.h>

#include <cgogn/modeling/tiling/square_tore.h>
using namespace cgogn::numerics;

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

using Map2   = cgogn::CMap2;
using Vec3   = Eigen::Vector3d;
using Scalar = cgogn::geometry::vector_traits<Vec3>::Scalar;
using Vertex = Map2::Vertex;
template <typename T> using VertexAttribute = Map2::VertexAttribute<T>;


class Viewer;

Viewer* viewer1;
Viewer* viewer2;

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
		edge_rendering_(true)
	{}

	Viewer(Viewer* view) :
		QOGLViewer(view),
		map_(),
		vertex_position_(),
		bb_(),
		render_(nullptr),
		vbo_pos_(nullptr),
		vertices_rendering_(false),
		edge_rendering_(true)
	{}
	virtual ~Viewer()
	{}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(Viewer);

	virtual void init()
	{
		makeCurrent();

		glClearColor(0.1f, 0.1f, 0.3f, 0.0f);

		vbo_pos_ = cgogn::make_unique<cgogn::rendering::VBO>(3);
		cgogn::rendering::update_vbo(vertex_position_, vbo_pos_.get());

		render_ = cgogn::make_unique<cgogn::rendering::MapRender>();
		render_->init_primitives(map_, cgogn::rendering::POINTS);
		render_->init_primitives(map_, cgogn::rendering::LINES);
		render_->init_primitives<Vec3>(map_, cgogn::rendering::TRIANGLES, &vertex_position_);

		param_flat_ = cgogn::rendering::ShaderFlat::generate_param();
		param_flat_->set_position_vbo(vbo_pos_.get());
		param_flat_->front_color_ = QColor(0, 100, 0);
		param_flat_->back_color_ = QColor(0, 0, 100);
		param_flat_->ambiant_color_ = QColor(5, 5, 5);

		param_edge_ = cgogn::rendering::ShaderBoldLine::generate_param();
		param_edge_->set_position_vbo(vbo_pos_.get());
		param_edge_->color_ = QColor(200, 200, 20);
		param_edge_->width_ = 2.0f;

		param_point_sprite_ = cgogn::rendering::ShaderPointSprite::generate_param();
		param_point_sprite_->set_position_vbo(vbo_pos_.get());
		param_point_sprite_->size_ = bb_.diag_size() / 500.0;
		param_point_sprite_->color_ = QColor(200, 20, 20);

		if (this == viewer2)
		{
			cam_ = this->camera();
			this->setCamera(viewer1->camera());
		}
	}

	virtual void draw()
	{
		makeCurrent();

		QMatrix4x4 proj;
		QMatrix4x4 view;
		camera()->getProjectionMatrix(proj);
		camera()->getModelViewMatrix(view);

		if (edge_rendering_)
		{
			glEnable(GL_POLYGON_OFFSET_FILL);
			glPolygonOffset(1.0f, 2.0f);
		}

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
			param_edge_->bind(proj, view);
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
		if (surface_mesh == "tore")
		{
			Map2::Builder mb(map_);

			vertex_position_ = map_.template add_attribute<Vec3, Map2::Vertex>("position");
			cgogn::modeling::SquareTore<Map2> g(map_, 16, 16);
			g.embed_into_tore(vertex_position_, 10.0f, 4.0f);

			map_.merge_incident_faces(Map2::Edge(cgogn::Dart(64)));
			map_.merge_incident_faces(Map2::Edge(cgogn::Dart(66)));

			map_.merge_incident_faces(Map2::Edge(cgogn::Dart(136)));
			mb.boundary_mark(Map2::Face(cgogn::Dart(137)));


			for(uint32 j=2; j<8; ++j)
			{
				for(uint32 i=0; i<8; ++i)
				{
					Map2::Face f = Map2::Face(cgogn::Dart(i*8+j*128));
					mb.boundary_mark(f);
				}
			}
		}
		else
		{
			cgogn::io::import_surface<Vec3>(map_, surface_mesh);
			vertex_position_ = map_.template get_attribute<Vec3, Map2::Vertex>("position");
		}

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
		if (this == viewer2)
			this->setCamera(cam_);

		render_.reset();
		vbo_pos_.reset();
//		cgogn::rendering::ShaderProgram::clean_all();
	}

	void compute_dual(Viewer& view)
	{
		std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();

		Map2::CellMarker<Map2::Vertex::ORBIT> cm(map_);
		cgogn::modeling::dual(view.map_, map_,&cm,{"position"},
		[&] (Map2::Face f )
		{
			return cgogn::geometry::centroid<Vec3>(view.map_, f, view.vertex_position_);
		});

		std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();
		std::chrono::duration<float> elapsed_seconds = end - start;
		cgogn_log_info("compute_dual") << "done in " << elapsed_seconds.count() << " s";

//		cgogn::modeling::dual(view.map_, map_,nullptr,{"position","face_centroid"},
//		[&] (Map2::Face f )
//		{
//			return cgogn::geometry::centroid<Vec3>(view.map_, f, view.vertex_position_);
//		},
//		[&] (Map2::Vertex f )
//		{
//			return view.vertex_position_[f];
//		});

//		cgogn::modeling::dual(view.map_, map_,nullptr,{"face_centroid","position","edge_middle"},
//		[&] (Map2::Vertex f )
//		{
//			return view.vertex_position_[f];
//		},
//		[&] (Map2::Face f )
//		{
//			return cgogn::geometry::centroid<Vec3>(view.map_, f, view.vertex_position_);
//		},
//		[&] (Map2::Edge e )
//		{
//			return view.vertex_position_[Map2::Vertex(e.dart)];//+view.vertex_position_[Map2::Vertex(view.map_.phi2(e.dart))];
//		});



		vertex_position_ = map_.get_attribute<Vec3, Vertex>("position");
		if (!vertex_position_.is_valid())
		{
			cgogn_log_error("ViewerDual::compute_dual") << "Missing attribute position. Aborting.";
			std::exit(EXIT_FAILURE);
		}

		cgogn::io::export_surface(map_, cgogn::io::ExportOptions::create().filename("dual.off").position_attribute(Vertex::ORBIT, "position").binary(false));

		cgogn::geometry::compute_AABB(vertex_position_, bb_);
		setSceneRadius(bb_.diag_size() / 2.0);
		Vec3 center = bb_.center();
		setSceneCenter(qoglviewer::Vec(center[0], center[1], center[2]));
		showEntireScene();
	}

	Map2 map_;
	VertexAttribute<Vec3> vertex_position_;
private:
	cgogn::geometry::AABB<Vec3> bb_;
	std::unique_ptr<cgogn::rendering::MapRender> render_;
	std::unique_ptr<cgogn::rendering::VBO> vbo_pos_;
	std::unique_ptr<cgogn::rendering::ShaderFlat::Param> param_flat_;
	std::unique_ptr<cgogn::rendering::ShaderBoldLine::Param> param_edge_;
	std::unique_ptr<cgogn::rendering::ShaderPointSprite::Param> param_point_sprite_;

	bool vertices_rendering_;
	bool edge_rendering_;
	qoglviewer::Camera* cam_;
};

int main(int argc, char** argv)
{
	std::string surface_mesh;
	if (argc < 2)
	{
		cgogn_log_info("cmap2_import") << "USAGE: " << argv[0] << " [filename]";
		surface_mesh = std::string(DEFAULT_MESH_PATH) + std::string("obj/salad_bowl.obj");
		cgogn_log_info("cmap2_import") << "Using default mesh : " << surface_mesh;
	}
	else
		surface_mesh = std::string(argv[1]);

	QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
	QApplication application(argc, argv);
	qoglviewer::init_ogl_context();

	// Instantiate the viewer.
	viewer1 = new Viewer();
	viewer1->setWindowTitle("dual: original");
	viewer1->import(surface_mesh);
	viewer1->show();

	viewer2 = new Viewer(viewer1);
	viewer2->setWindowTitle("dual: dual");
	viewer2->compute_dual(*viewer1);
	viewer2->show();


	// Run main loop.
	return application.exec();
}

