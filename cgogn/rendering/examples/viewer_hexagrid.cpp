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

#include <cgogn/core/utils/logger.h>
#include <cgogn/core/cmap/cmap3.h>
#include <cgogn/io/map_import.h>
#include <cgogn/geometry/algos/bounding_box.h>
#include <cgogn/rendering/hexagrid_drawer.h>
#include <cgogn/modeling/tiling/hexa_volume.h>
#include <cgogn/rendering/frame_manipulator.h>
#include <Eigen/Dense>

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

using namespace cgogn::numerics;
using Map3 = cgogn::CMap3;
using Vec3 = Eigen::Vector3d;

template <typename T>
using VertexAttribute = Map3::VertexAttribute<T>;
template <typename T>
using VolumeAttribute = Map3::VolumeAttribute<T>;



class Viewer : public QOGLViewer
{
public:
	static const uint32 NB = 27;
	using HexaGridDrawer = cgogn::rendering::HexaGridDrawer;

	Viewer();
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(Viewer);

	virtual void draw();
	virtual void init();
	virtual void keyPressEvent(QKeyEvent*);
	void keyReleaseEvent(QKeyEvent*);
	virtual void mousePressEvent(QMouseEvent*);
	virtual void mouseReleaseEvent(QMouseEvent*);
	virtual void mouseMoveEvent(QMouseEvent*);

	void import();
	virtual ~Viewer();
	virtual void closeEvent(QCloseEvent *e);

private:

	void rayClick(QMouseEvent* event, qoglviewer::Vec& P, qoglviewer::Vec& Q);
	void plane_clip_from_frame();

	Map3 map_;
	VertexAttribute<Vec3> vertex_position_;
	VolumeAttribute<uint32> vol_indI;
	VolumeAttribute<uint32> vol_indJ;
	VolumeAttribute<uint32> vol_indK;

	cgogn::geometry::AABB<Vec3> bb_;

	std::unique_ptr<cgogn::rendering::VBO> vbo_pos_;

	std::unique_ptr<HexaGridDrawer> HexaGrid_drawer_;
	std::unique_ptr<HexaGridDrawer::Renderer> HexaGrid_drawer_rend_;

	std::unique_ptr<cgogn::rendering::FrameManipulator> frame_manip_;

	bool vol_rendering_;
	bool edge_rendering_;

	float32 expl_;

	QVector4D plane_clipping1_;
	QVector3D clipping_topo_Front_;
	QVector3D clipping_topo_Back_;

};

//
// IMPLEMENTATION
//
void Viewer::rayClick(QMouseEvent* event, qoglviewer::Vec& P, qoglviewer::Vec& Q)
{
	P = camera()->unprojectedCoordinatesOf(qoglviewer::Vec(event->x(), event->y(), 0.0));
	Q = camera()->unprojectedCoordinatesOf(qoglviewer::Vec(event->x(), event->y(), 1.0));
}

bool menger(uint32 l, uint32 i, uint32 j, uint32 k)
{
	if (l == 1)
		return true;
	l /= 3;
	if ((i / l == 1) + (j / l == 1) + (k / l == 1) >= 2)
		return false;
	return menger(l, i % l, j % l, k % l);
}

void Viewer::import()
{
	vertex_position_ = map_.template add_attribute<Vec3, Map3::Vertex>("position");

	cgogn::modeling::TilingHexa tile2(map_,NB,NB,NB);
	tile2.embedded_grid3D( [&](uint32 i, uint32 j, uint32 k)
	{
		return menger(NB, i, j, k);
	});

	vol_indI = map_.template add_attribute<uint32, Map3::Volume>("indI");
	vol_indJ = map_.template add_attribute<uint32, Map3::Volume>("indJ");
	vol_indK = map_.template add_attribute<uint32, Map3::Volume>("indK");


	tile2.update_positions([&](cgogn::CMap3::Vertex v, double r, double s, double t) {
		vertex_position_[v] = Vec3(r,s,t);
	});

	map_.foreach_cell([&] (Map3::Volume v)
	{
		Vec3 CG = cgogn::geometry::centroid(map_, v, vertex_position_);
		vol_indI[v] = uint32(CG[0]*NB);
		vol_indJ[v] = uint32(CG[1]*NB);
		vol_indK[v] = uint32(CG[2]*NB);
	});

	tile2.update_positions([&](cgogn::CMap3::Vertex v, double r, double s, double t) {
		vertex_position_[v] = Vec3(std::pow(1+2*r,2)/9,std::pow(1+2*s,2)/9,std::pow(1+2*t,2)/9);
	});




	if (!map_.check_map_integrity())
	{
		cgogn_log_error("Viewer::import") << "Integrity of map not respected. Aborting.";
		std::exit(EXIT_FAILURE);
	}
	cgogn::geometry::compute_AABB(vertex_position_, bb_);
	setSceneRadius(cgogn::geometry::diagonal(bb_).norm()/2.0);
	Vec3 center = cgogn::geometry::center(bb_);
	setSceneCenter(qoglviewer::Vec(center[0], center[1], center[2]));
	showEntireScene();
	map_.check_map_integrity();
}

Viewer::~Viewer()
{}

void Viewer::closeEvent(QCloseEvent*)
{
	vbo_pos_.reset();
	HexaGrid_drawer_.reset();
	HexaGrid_drawer_rend_.reset();
	cgogn::rendering::ShaderProgram::clean_all();
}

Viewer::Viewer() :
	map_(),
	vertex_position_(),
	bb_(),
	vbo_pos_(nullptr),
	HexaGrid_drawer_(nullptr),
	HexaGrid_drawer_rend_(nullptr),
	vol_rendering_(true),
	edge_rendering_(true),
	expl_(0.8f),
	plane_clipping1_(0,0,0,0),
	clipping_topo_Front_(0,0,0),
	clipping_topo_Back_(NB,NB,NB)
{}

void Viewer::keyPressEvent(QKeyEvent *ev)
{
	if ((ev->modifiers() & Qt::ShiftModifier) && (ev->modifiers() & Qt::ControlModifier))
		setCursor(Qt::CrossCursor);

	switch (ev->key())
	{
		case Qt::Key_V:
			vol_rendering_ = !vol_rendering_;
			break;
		case Qt::Key_E:
			edge_rendering_ = !edge_rendering_;
			break;

		case Qt::Key_Plus:
			expl_ += 0.05f;
			HexaGrid_drawer_rend_->set_explode_volume(expl_);
			break;
		case Qt::Key_Minus:
			expl_ -= 0.05f;
			HexaGrid_drawer_rend_->set_explode_volume(expl_);
			break;
		case Qt::Key_X:
			if (ev->modifiers() & Qt::ShiftModifier)
				clipping_topo_Front_[0] -= 1.0;
			else
				clipping_topo_Front_[0] += 1.0;
			HexaGrid_drawer_rend_->set_clipping_plane_topo(clipping_topo_Front_);
			break;
		case Qt::Key_Y:
			if (ev->modifiers() & Qt::ShiftModifier)
				clipping_topo_Front_[1] -= 1.0;
			else
				clipping_topo_Front_[1] += 1.0;
			HexaGrid_drawer_rend_->set_clipping_plane_topo(clipping_topo_Front_);
			break;
		case Qt::Key_Z:
			if (ev->modifiers() & Qt::ShiftModifier)
				clipping_topo_Front_[2] -= 1.0;
			else
				clipping_topo_Front_[2] += 1.0;
			HexaGrid_drawer_rend_->set_clipping_plane_topo(clipping_topo_Front_);
			break;

		case Qt::Key_A:
			if (ev->modifiers() & Qt::ShiftModifier)
				clipping_topo_Back_[0] -= 1.0;
			else
				clipping_topo_Back_[0] += 1.0;
			HexaGrid_drawer_rend_->set_clipping_plane_topo2(clipping_topo_Back_);
			break;
		case Qt::Key_B:
			if (ev->modifiers() & Qt::ShiftModifier)
				clipping_topo_Back_[1] -= 1.0;
			else
				clipping_topo_Back_[1] += 1.0;
			HexaGrid_drawer_rend_->set_clipping_plane_topo2(clipping_topo_Back_);
			break;
		case Qt::Key_C:
			if (ev->modifiers() & Qt::ShiftModifier)
				clipping_topo_Back_[2] -= 1.0;
			else
				clipping_topo_Back_[2] += 1.0;
			HexaGrid_drawer_rend_->set_clipping_plane_topo2(clipping_topo_Back_);
			break;

		default:
			break;
	}
	// enable QGLViewer keys
	QOGLViewer::keyPressEvent(ev);
	//update drawing

	update();
}

void Viewer::keyReleaseEvent(QKeyEvent* ev)
{
	QOGLViewer::keyReleaseEvent(ev);
	unsetCursor();
}

void Viewer::mousePressEvent(QMouseEvent* event)
{
	qoglviewer::Vec P;
	qoglviewer::Vec Q;
	Vec3 A(P[0], P[1], P[2]);
	Vec3 B(Q[0], Q[1], Q[2]);


	if ((event->modifiers() & Qt::ControlModifier) && !(event->modifiers() & Qt::ShiftModifier))
		frame_manip_->pick(event->x(), event->y(),P,Q);


	QOGLViewer::mousePressEvent(event);
	update();
}

void Viewer::mouseReleaseEvent(QMouseEvent* event)
{
	if (event->modifiers() & Qt::ControlModifier)
		frame_manip_->release();

	QOGLViewer::mouseReleaseEvent(event);
	update();
}

void Viewer::mouseMoveEvent(QMouseEvent* event)
{
	if (event->modifiers() & Qt::ControlModifier)
	{
		bool local_manip = (event->buttons() & Qt::RightButton);
		frame_manip_->drag(local_manip, event->x(), event->y());

		plane_clip_from_frame();

		HexaGrid_drawer_rend_->set_clipping_plane(plane_clipping1_);
	}

	QOGLViewer::mouseMoveEvent(event);
	update();
}

void Viewer::draw()
{
	QMatrix4x4 proj;
	QMatrix4x4 view;
	camera()->getProjectionMatrix(proj);
	camera()->getModelViewMatrix(view);

	if (vol_rendering_)
	{
		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(1.0f, 1.0f);
		HexaGrid_drawer_rend_->draw_faces(proj,view);
		glDisable(GL_POLYGON_OFFSET_FILL);
	}

	if (edge_rendering_)
		HexaGrid_drawer_rend_->draw_edges(proj,view);

	frame_manip_->draw(true,true,proj, view);
}

void Viewer::init()
{
	glClearColor(0.1f,0.1f,0.3f,0.0f);

	vbo_pos_ = cgogn::make_unique<cgogn::rendering::VBO>(3);
	cgogn::rendering::update_vbo(vertex_position_, vbo_pos_.get());


	HexaGrid_drawer_ = cgogn::make_unique<cgogn::rendering::HexaGridDrawer>();

	cgogn::rendering::FakeAttribute<Map3::Volume::ORBIT, Vec3> fake_color(Vec3(0.0,0.0,1.0f));

	HexaGrid_drawer_->update_face(map_,vertex_position_,fake_color,vol_indI,vol_indJ,vol_indK);
	HexaGrid_drawer_->update_edge(map_,vertex_position_,vol_indI,vol_indJ,vol_indK);

	HexaGrid_drawer_rend_ = HexaGrid_drawer_->generate_renderer();
	HexaGrid_drawer_rend_->set_explode_volume(expl_);


	frame_manip_ = cgogn::make_unique<cgogn::rendering::FrameManipulator>();
	frame_manip_->set_size(cgogn::geometry::diagonal(bb_).norm()/4);
	frame_manip_->set_position(bb_.max());
	frame_manip_->z_plane_param(QColor(200,200,200),-1.5f,-1.5f, 2.0f);

	plane_clip_from_frame();

	HexaGrid_drawer_rend_->set_clipping_plane(plane_clipping1_);
	HexaGrid_drawer_rend_->set_clipping_plane_topo(clipping_topo_Front_);
	HexaGrid_drawer_rend_->set_clipping_plane_topo2(clipping_topo_Back_);


}

void Viewer::plane_clip_from_frame()
{
	Vec3 position;
	Vec3 axis_z;
	frame_manip_->get_position(position);
	frame_manip_->get_axis(cgogn::rendering::FrameManipulator::Zt,axis_z);
	float32 d = -(position.dot(axis_z));
	plane_clipping1_ = QVector4D(axis_z[0],axis_z[1],axis_z[2],d);
}

int main(int argc, char** argv)
{
	QApplication application(argc, argv);
	qoglviewer::init_ogl_context();

	// Instantiate the viewer.
	Viewer viewer;
	viewer.setWindowTitle("viewer_topo3");
	viewer.import();
	viewer.show();

	// Run main loop.
	return application.exec();
}
