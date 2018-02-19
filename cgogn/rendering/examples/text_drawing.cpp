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

#include <cgogn/rendering/text_drawer.h>

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

//using Vec3 = Eigen::Vector3d;
using Vec3 = cgogn::geometry::Vec_T<std::array<double,3>>;

class TextDrawing : public QOGLViewer
{
public:
	TextDrawing();
	TextDrawing(TextDrawing* ptr);
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(TextDrawing);
	virtual void draw();
	virtual void init();
	std::unique_ptr<cgogn::rendering::TextDrawer> tdr_;
	std::unique_ptr<cgogn::rendering::TextDrawer::Renderer> tdr_rend_;
};



TextDrawing::TextDrawing() :
	tdr_(nullptr),
	tdr_rend_(nullptr)
{}


void TextDrawing::draw()
{
	QMatrix4x4 proj;
	QMatrix4x4 view;
	camera()->getProjectionMatrix(proj);
	camera()->getModelViewMatrix(view);

	tdr_rend_->draw(proj, view);
}


void TextDrawing::init()
{
	setSceneRadius(5.0);
	setSceneCenter(qoglviewer::Vec(0.0,0.0,0.0));
	showEntireScene();
	glClearColor(0.1f,0.1f,0.2f,0.0f);

	tdr_ = std::make_unique<cgogn::rendering::TextDrawer>();
	tdr_rend_ = tdr_->generate_renderer();

	for (float z=-4; z<4; z += 1)
		for (float y = -4; y<4; y += 1)
			for (float x = -4; x < 4; x += 1)
			{
				Vec3 P{ x,y,z };
				QColor col(rand()%255, rand() % 255, rand() % 255);
				float sz = 0.1f*rand() / RAND_MAX + 0.05f;
				*tdr_ << P << col << sz << "Pi=3.14" ;
			}
	*tdr_ << cgogn::rendering::TextDrawer::end; 
}


int main(int argc, char** argv)
{
	qoglviewer::init_ogl_context();
	QApplication application(argc, argv);
	TextDrawing viewer;
	viewer.setWindowTitle("TextDrawing");
	viewer.show();
	return  application.exec();
}
