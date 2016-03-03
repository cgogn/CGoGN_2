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
//	virtual void keyPressEvent(QKeyEvent *);
	virtual ~Drawing();

private:
	cgogn::rendering::Drawer* drawer_;
	cgogn::rendering::Drawer* drawer2_;
};



Drawing::~Drawing()
{
	delete drawer_;
	delete drawer2_;
}

Drawing::Drawing() :
	drawer_(nullptr),
	drawer2_(nullptr)
{}

//void Drawing::keyPressEvent(QKeyEvent *ev)
//{
//	switch (ev->key()) {
//		case Qt::Key_P:
//			phong_rendering_ = true;
//			flat_rendering_ = false;
//			break;
//		case Qt::Key_Escape:
//			exit(0);
//			break;
//		default:
//			break;
//	}
//	update();
//}

void Drawing::draw()
{
	QMatrix4x4 proj;
	QMatrix4x4 view;
	camera()->getProjectionMatrix(proj);
	camera()->getModelViewMatrix(view);

	drawer_->callList(proj,view);
//	drawer2_->callList(proj,view);
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
	drawer_->lineWidth(2.0);
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
//	drawer_->pointSize(10.0);
	drawer_->lineWidthAA(3.0);
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

	drawer_->pointSizeAA(6.0);
	drawer_->begin(GL_POINTS);
	for (float a=0.0; a < 1.0; a+= 0.1)
	{
		Vec3 P(3.0+std::cos(6.28*a),-2.0+std::sin(6.28*a),0.0);
		Vec3 C(a,0.5,1.0-a);
		drawer_->color3fv(C);
		drawer_->vertex3fv(P);
	}

	drawer_->end();

//	drawer_->pointSizeAA(7.0);
//	drawer_->begin(GL_POINTS);
//	drawer_->color3f(1.0,1.0,1.0);
//	for (float z=-3.0; z < 3.0; z+= 0.2)
//		for (float y=-3.0; y < 3.0; y+= 0.2)
//			for (float x=-3.0; x < 3.0; x+= 0.2)
//			{
//				drawer_->vertex3f(x,y,z);
//			}
//	drawer_->end();
	drawer_->end_list();

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
