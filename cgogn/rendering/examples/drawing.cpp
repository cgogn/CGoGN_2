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

#include <cgogn/rendering/drawer.h>

#include <cgogn/rendering/wall_paper.h>

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

//using Vec3 = Eigen::Vector3d;
using Vec3 = cgogn::geometry::Vec_T<std::array<double,3>>;

class Drawing : public QOGLViewer
{
public:
	Drawing();
	Drawing(Drawing* ptr);
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(Drawing);

	virtual void draw();
	virtual void init();
	virtual void closeEvent(QCloseEvent *e);
	virtual ~Drawing();

//private:
	std::shared_ptr<cgogn::rendering::DisplayListDrawer> drawer_;
	std::shared_ptr<cgogn::rendering::DisplayListDrawer> drawer2_;
	std::unique_ptr<cgogn::rendering::DisplayListDrawer::Renderer> drawer_rend_;
	std::unique_ptr<cgogn::rendering::DisplayListDrawer::Renderer> drawer2_rend_;

	std::shared_ptr<cgogn::rendering::WallPaper> wp_;
	std::shared_ptr<cgogn::rendering::WallPaper> button_;
	std::unique_ptr<cgogn::rendering::WallPaper::Renderer> wp_rend_;
	std::unique_ptr<cgogn::rendering::WallPaper::Renderer> button_rend_;

	Drawing* m_first;
};



Drawing::~Drawing()
{}

void Drawing::closeEvent(QCloseEvent*)
{
	this->makeCurrent();

	drawer_rend_.reset();
	drawer2_rend_.reset();
	wp_rend_.reset();
	button_rend_.reset();

	drawer_.reset();
	drawer2_.reset();
	wp_.reset();
	button_.reset();
}

Drawing::Drawing() :
	drawer_(nullptr),
	drawer2_(nullptr),
	drawer_rend_(nullptr),
	drawer2_rend_(nullptr),
	wp_(nullptr),
	button_(nullptr),
	wp_rend_(nullptr),
	button_rend_(nullptr),
	m_first(nullptr)
{
	m_first = this;
}

Drawing::Drawing(Drawing* ptr) :
	QOGLViewer(ptr),
	drawer_(nullptr),
	drawer2_(nullptr),
	drawer_rend_(nullptr),
	drawer2_rend_(nullptr),
	wp_(nullptr),
	button_(nullptr),
	wp_rend_(nullptr),
	button_rend_(nullptr),
	m_first(ptr)
{}


void Drawing::draw()
{
	wp_rend_->draw(this);
	button_rend_->draw(this);

	QMatrix4x4 proj;
	QMatrix4x4 view;
	camera()->getProjectionMatrix(proj);
	camera()->getModelViewMatrix(view);

	drawer_rend_->draw(proj,view,this);
	drawer2_rend_->draw(proj,view,this);
}

void Drawing::init()
{
	setSceneRadius(5.0);
	setSceneCenter(qoglviewer::Vec(0.0,0.0,0.0));
	showEntireScene();
	glClearColor(0.1f,0.1f,0.3f,0.0f);

	this->makeCurrent();

	if (m_first!=this)
	{
		drawer_ = m_first->drawer_;
		drawer2_ = m_first->drawer2_;
		wp_ = m_first->wp_;
		button_ = m_first->button_;

		drawer_rend_ = drawer_->generate_renderer();
		drawer2_rend_ = drawer2_->generate_renderer();
		wp_rend_ = wp_->generate_renderer();
		button_rend_ = button_->generate_renderer();
		return;
	}

	wp_ = std::make_shared<cgogn::rendering::WallPaper>(QImage(QString(DEFAULT_MESH_PATH) + QString("../images/cgogn2.png")));
	button_ = std::make_shared<cgogn::rendering::WallPaper>(QImage(QString(DEFAULT_MESH_PATH) + QString("../images/igg.png")));
//	button_->set_local_position(this->width(),this->height(),10,10,50,50);
	button_->set_local_position(0.1f,0.1f,0.2f,0.2f);

	wp_rend_ = wp_->generate_renderer();
	button_rend_ = button_->generate_renderer();

	// drawer for simple old-school g1 rendering
	drawer_ = std::make_shared<cgogn::rendering::DisplayListDrawer>();
	drawer_rend_ = drawer_->generate_renderer();
	drawer_->new_list();
	drawer_->line_width(2.0);
	drawer_->begin(GL_LINE_LOOP);
		drawer_->color3f(1.0,0.0,0.0);
		drawer_->vertex3f(0.0,0.0,0.0);
		drawer_->color3f(0.0,1.0,1.0);
		drawer_->vertex3f(1,0,0);
		drawer_->color3f(1.0,0.0,1.0);
		drawer_->vertex3f(1.0f,1.0f,0.0f);
		drawer_->color3f(1.0,1.0,0.0);
		drawer_->vertex3f(0,1,0);
	drawer_->end();
	drawer_->line_width_aa(3.0);
	drawer_->begin(GL_LINES);
		drawer_->color3f(0.0,0.8,0.0);
		drawer_->vertex3fv(Vec3(-1,2,0));
		drawer_->color3f(0.0,0.0,0.8);
		drawer_->vertex3fv(Vec3(-1.3,0,0));
		drawer_->color3f(0.0,0.0,0.8);
		drawer_->vertex3fv(Vec3(-2,1,0));
		drawer_->color3f(0.8,0.0,0.0);
		drawer_->vertex3fv(Vec3(-2.3,3,0));
	drawer_->end();

	drawer_->begin(GL_TRIANGLES);
		drawer_->color3f(1.0,0.0,0.0);
		drawer_->vertex3fv({{2,2,0}});
		drawer_->color3f(0.0,1.0,0.0);
		drawer_->vertex3fv({{4,3,0}});
		drawer_->color3f(0.0,0.0,1.0);
		drawer_->vertex3fv({{2.5,1,0}});
	drawer_->end();

	drawer_->point_size_aa(7.0);
	drawer_->begin(GL_POINTS);
	for (float a=0.0f; a < 1.0f; a+= 0.1f)
	{
		Vec3 P(4.0+std::cos(6.28*a),-2.0+std::sin(6.28*a),0.0);
		Vec3 C(a,0.5,1.0-a);
		drawer_->color3fv(C);
		drawer_->vertex3fv(P);
	}
	drawer_->end();

	drawer_->ball_size(0.1f);
	drawer_->begin(GL_POINTS);
	for (float a=0.05f; a < 1.0f; a+= 0.1f)
	{
		Vec3 P(4.0+std::cos(6.28*a)*1.2,-2.0+ std::sin(6.28*a)*1.2, std::sin(6.28*a)*0.2 );
		Vec3 C(a,0.5,1.0-a);
		drawer_->color3fv(C);
		drawer_->vertex3fv(P);
	}

	drawer_->end();
	drawer_->end_list();

	drawer2_ = std::make_shared<cgogn::rendering::DisplayListDrawer>();
	drawer2_rend_ = drawer2_->generate_renderer();
	drawer2_->new_list();
	drawer2_->point_size_aa(5.0);
	drawer2_->begin(GL_POINTS);
	drawer2_->color3f(1.0,1.0,1.0);
	for (float z=-1.0f; z < 1.0f; z+= 0.1f)
		for (float y=-2.0f; y < 0.0f; y+= 0.1f)
			for (float x=0.0f; x < 2.0f; x+= 0.1f)
			{
				drawer2_->vertex3f(x,y,z);
			}
	drawer2_->end();

	drawer2_->ball_size(0.03f);
	drawer2_->begin(GL_POINTS);
	drawer2_->color3f(1.0,1.0,1.0);
	for (float z=-1.0f; z < 1.0f; z+= 0.2f)
		for (float y=-2.0f; y < 0.0f; y+= 0.2f)
			for (float x=-3.0f; x < -1.0f; x+= 0.2f)
			{
				drawer2_->vertex3f(x,y,z);
			}
	drawer2_->end();

	drawer2_->end_list();

}

int main(int argc, char** argv)
{
	qoglviewer::init_ogl_context();
	QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);

	QApplication application(argc, argv);


	// Instantiate the viewer.

	std::unique_ptr<Drawing> viewer = cgogn::make_unique<Drawing>();
	viewer->setWindowTitle("Drawing");
	viewer->show();

	std::unique_ptr<Drawing> viewer2 = cgogn::make_unique<Drawing>(viewer.get());
	viewer2->setWindowTitle("Drawing2");
	viewer2->show();

//	cgogn_log_info("are context shared ?") << std::boolalpha <<	QOpenGLContext::areSharing(viewer2->context(),viewer->context());

	// Run main loop.
	const int ret = application.exec();

	viewer.reset();
	viewer2.reset();
	return ret;
}
