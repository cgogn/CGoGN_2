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
#include <QMouseEvent>

#include <core/cmap/cmap2.h>

#include <io/map_import.h>

#include <geometry/algos/bounding_box.h>

#include <rendering/map_render.h>
#include <rendering/shaders/shader_flat.h>
#include <rendering/shaders/vbo.h>


#include <geometry/algos/picking.h>

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

using Map2 = cgogn::CMap2<cgogn::DefaultMapTraits>;
using Vec3 = Eigen::Vector3d;
//using Vec3 = cgogn::geometry::Vec_T<std::array<double,3>>;

template<typename T>
using VertexAttributeHandler = Map2::VertexAttributeHandler<T>;


class Viewer : public QOGLViewer
{
public:
	Viewer();
	Viewer(const Viewer&) = delete;
	Viewer& operator=(const Viewer&) = delete;

	virtual void draw();
	virtual void init();
	virtual void mousePressEvent(QMouseEvent *e);
	virtual void resizeGL(int w, int h);

	void import(const std::string& surfaceMesh);

	virtual ~Viewer();



private:
	QRect viewport_;
	QMatrix4x4 proj_;
	QMatrix4x4 view_;

	Map2 map_;
	VertexAttributeHandler<Vec3> vertex_position_;

	cgogn::geometry::BoundingBox<Vec3> bb_;

	cgogn::rendering::MapRender* render_;

	cgogn::rendering::VBO* vbo_pos_;

	cgogn::rendering::ShaderFlat* shader2_;

};


//
// IMPLEMENTATION
//


void Viewer::import(const std::string& surfaceMesh)
{
	cgogn::io::import_surface<Vec3>(map_, surfaceMesh);

	vertex_position_ = map_.get_attribute<Vec3, Map2::Vertex::ORBIT>("position");

	cgogn::geometry::compute_bounding_box(vertex_position_, bb_);

	setSceneRadius(bb_.diag_size());
	Vec3 center = bb_.center();
	setSceneCenter(qoglviewer::Vec(center[0], center[1], center[2]));
	showEntireScene();
}

Viewer::~Viewer()
{
	delete render_;
	delete vbo_pos_;
	delete shader2_;
}

Viewer::Viewer() :
	map_(),
	vertex_position_(),
	bb_(),
	render_(nullptr),
	vbo_pos_(nullptr),
	shader2_(nullptr)
{}

void Viewer::draw()
{
//	QMatrix4x4 proj;
//	QMatrix4x4 view;
	camera()->getProjectionMatrix(proj_);
	camera()->getModelViewMatrix(view_);

	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.0f, 1.0f);

	shader2_->bind();
	shader2_->set_matrices(proj_,view_);
	shader2_->bind_vao(0);
	render_->draw(cgogn::rendering::TRIANGLES);
	shader2_->release_vao(0);
	shader2_->release();
}

void Viewer::init()
{
	glClearColor(0.1f,0.1f,0.3f,0.0f);

	vbo_pos_ = new cgogn::rendering::VBO;
	cgogn::rendering::update_vbo(vertex_position_, *vbo_pos_);
	render_ = new cgogn::rendering::MapRender();

	render_->init_primitives<Vec3>(map_, cgogn::rendering::TRIANGLES, vertex_position_);

	shader2_ = new cgogn::rendering::ShaderFlat;
	shader2_->add_vao();
	shader2_->set_vao(0, vbo_pos_);

	shader2_->bind();
	shader2_->set_front_color(QColor(0,200,0));
	shader2_->set_back_color(QColor(0,0,200));
	shader2_->set_ambiant_color(QColor(5,5,5));
	shader2_->release();
}

void Viewer::mousePressEvent(QMouseEvent* event)
{

//	camera()->getProjectionMatrix(proj_);
//	camera()->getModelViewMatrix(view_);


	unsigned int x = event->x()*devicePixelRatio();
	unsigned int y = this->height() - event->y()*devicePixelRatio();
	QVector3D wp(x,y,0);
	QVector3D wq(x,y,0.99);
	QVector3D P = wp.unproject(view_,proj_,viewport_);
	QVector3D Q = wq.unproject(view_,proj_,viewport_);

	Vec3 A(P[0],P[1],P[2]);
	Vec3 B(Q[0],Q[1],Q[2]);



//	std::cout<<A << std::endl<< "//////////////"<< std::endl<<B<<std::endl<<"//////////////"<< std::endl;

	std::vector<Map2::Face> selected;
	cgogn::geometry::picking_face<Vec3>(map_,vertex_position_,A,B,selected);

	std::cout << selected.size()<< std::endl;



	QOGLViewer::mousePressEvent(event);
}

void Viewer::resizeGL(int w, int h)
{
	int vp[4];
	glGetIntegerv(GL_VIEWPORT, vp);
	viewport_= QRect(vp[0],vp[1],vp[2],vp[3]);
	std::cout << "viewport"<< std::endl;
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

	// Instantiate the viewer.
	Viewer viewer;
	viewer.setWindowTitle("simpleViewer");
	viewer.import(surfaceMesh);
	viewer.show();

	// Run main loop.
	return application.exec();
}
