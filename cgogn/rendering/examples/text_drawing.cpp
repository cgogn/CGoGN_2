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

#include <iomanip>
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

	void draw() override;
	void init() override;
	void resizeEvent(QResizeEvent *ev) override;
	void keyPressEvent(QKeyEvent *ev) override;

	std::unique_ptr<cgogn::rendering::TextDrawer> tdr_;
	std::unique_ptr<cgogn::rendering::TextDrawer::Renderer> tdr_rend_;

	std::unique_ptr<cgogn::rendering::TextDrawer> tdr2_;
	std::unique_ptr<cgogn::rendering::TextDrawer::Renderer> tdr_rend2_;

	std::chrono::time_point<std::chrono::system_clock> start_fps_;
	int nb_fps_;
	std::string fps_;
};



TextDrawing::TextDrawing() :
	tdr_(nullptr),
	tdr_rend_(nullptr),
	fps_("?????")
{}


void TextDrawing::draw()
{
	QMatrix4x4 proj;
	QMatrix4x4 view;
	camera()->getProjectionMatrix(proj);
	camera()->getModelViewMatrix(view);

	tdr_rend_->draw(proj, view);
	QMatrix4x4 Id;
	QMatrix4x4 ratio;
	ratio.translate(-1,-1,0);
	ratio.scale(0.5f, 0.5f*width()/height(),0.0f);
	tdr_rend2_->draw(ratio,Id);

	nb_fps_++;
	if (nb_fps_ == 50)
	{
		std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();
		std::chrono::duration<float> elapsed_seconds = end - start_fps_;
		float fps = 50.0f/elapsed_seconds.count();
		fps_ = std::to_string(fps).substr(0,5);
		tdr2_->update_text(0,fps_);
		start_fps_ = std::chrono::system_clock::now();
		nb_fps_ = 0;
	}
}


void TextDrawing::init()
{
	setSceneRadius(5.0);
	setSceneCenter(qoglviewer::Vec(0.0,0.0,0.0));
	showEntireScene();
	glClearColor(0.1f,0.1f,0.2f,0.0f);

	tdr_ = cgogn::make_unique<cgogn::rendering::TextDrawer>();
	tdr_rend_ = tdr_->generate_renderer();

	for (float z=-4; z<4; z += 1)
		for (float y = -4; y<4; y += 1)
			for (float x = -4; x < 4; x += 1)
			{
				Vec3 P{ x,y,z };
				QColor col(rand()%255, rand() % 255, rand() % 255);
				float sz = 0.1f*rand() / RAND_MAX + 0.05f;
				std::stringstream ss;
				ss << std::setprecision(2)<< "(" << x << "," << y << "," << z << ")";
				*tdr_ << P << col << sz << ss.str();
			}
	*tdr_ << cgogn::rendering::TextDrawer::end; 

	tdr2_ = cgogn::make_unique<cgogn::rendering::TextDrawer>();
	tdr_rend2_ = tdr2_->generate_renderer();

	float sz = 32.0f / (devicePixelRatio()*width());
	*tdr2_ <<Vec3{sz,sz,-1} << QColor("white") <<sz << fps_ << " fps"<< cgogn::rendering::TextDrawer::end ;
	tdr_rend2_->set_italic(0.2f);
	start_fps_ = std::chrono::system_clock::now();
	nb_fps_ = 0;

}

void TextDrawing::resizeEvent(QResizeEvent *ev)
{
	if (tdr2_)
	{
		float sz = 32.0f / (devicePixelRatio()*width());
		*tdr2_ <<Vec3{sz,sz,-1} << QColor("white") <<sz << fps_ << " fps"<< cgogn::rendering::TextDrawer::end ;
	}

	QOGLViewer::resizeEvent(ev);
}

void TextDrawing::keyPressEvent(QKeyEvent *ev)
{
	switch (ev->key())
	{
		case Qt::Key_A:
				tdr_->update_text(0,"XXXXXXXXXX");
			break;
		case Qt::Key_Minus:
				tdr_->scale_text(0.9f);
			break;
		case Qt::Key_Plus:
				tdr_->scale_text(1.1f);
			break;
		default:
			break;
	}
	// enable QGLViewer keys
	QOGLViewer::keyPressEvent(ev);
	//update drawing
	update();
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
