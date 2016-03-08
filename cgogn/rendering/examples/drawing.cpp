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

#include <rendering/drawer.h>

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

//using Vec3 = Eigen::Vector3d;
using Vec3 = cgogn::geometry::Vec_T<std::array<double,3>>;

class Drawing : public QOGLViewer
{
public:
	Drawing();
	Drawing(const Drawing&) = delete;
	Drawing& operator=(const Drawing&) = delete;

	virtual void draw();
	virtual void init();
	virtual void closeEvent(QCloseEvent *e);
	virtual ~Drawing();

private:
	cgogn::rendering::Drawer* drawer_;
	cgogn::rendering::Drawer* drawer2_;
};



Drawing::~Drawing()
{}

void Drawing::closeEvent(QCloseEvent*)
{
	delete drawer_;
	delete drawer2_;
}

Drawing::Drawing() :
	drawer_(nullptr),
	drawer2_(nullptr)
{}


void Drawing::draw()
{
	QMatrix4x4 proj;
	QMatrix4x4 view;
	camera()->getProjectionMatrix(proj);
	camera()->getModelViewMatrix(view);

	drawer_->call_list(proj,view);
	drawer2_->call_list(proj,view);
}

void Drawing::init()
{
	setSceneRadius(5.0);
	setSceneCenter(qoglviewer::Vec(0.0,0.0,0.0));
	showEntireScene();

	glClearColor(0.1f,0.1f,0.3f,0.0f);

	// drawer for simple old-school g1 rendering
	drawer_ = new cgogn::rendering::Drawer(this);
	drawer_->new_list();
	drawer_->line_width(2.0);
	drawer_->begin(GL_LINE_LOOP);
		drawer_->color3f(1.0,0.0,0.0);
		drawer_->vertex3f(0,0,0);
		drawer_->color3f(0.0,1.0,1.0);
		drawer_->vertex3f(1,0,0);
		drawer_->color3f(1.0,0.0,1.0);
		drawer_->vertex3f(1,1,0);
		drawer_->color3f(1.0,1.0,0.0);
		drawer_->vertex3f(0,1,0);
	drawer_->end();
//	drawer_->point_size(10.0);
	drawer_->line_width_AA(3.0);
	drawer_->begin(GL_LINES);
		drawer_->color3f(1.0,1.0,1.0);
		drawer_->vertex3fv(Vec3(-1,-1,0));
		drawer_->vertex3fv(Vec3(-1.2,-2,0));
		drawer_->vertex3fv(Vec3(-2,-2,0));
		drawer_->vertex3fv(Vec3(-2.2,1,0));
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
	drawer_->end_list();

	drawer2_ = new cgogn::rendering::Drawer(this);
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
	drawer2_->end_list();

}

int main(int argc, char** argv)
{

	QApplication application(argc, argv);
	qoglviewer::init_ogl_context();

	// Instantiate the viewer.
	Drawing viewer;
	viewer.setWindowTitle("Drawing");
	viewer.show();

	// Run main loop.
	return application.exec();
}
