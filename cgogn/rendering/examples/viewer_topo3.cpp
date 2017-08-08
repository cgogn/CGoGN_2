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
//#include <cgogn/core/cmap/cmap3_tetra.h>
//#include <cgogn/core/cmap/cmap3_hexa.h>
#include <cgogn/core/cmap/cmap3.h>
#include <cgogn/io/map_import.h>
#include <cgogn/geometry/algos/bounding_box.h>
#include <cgogn/rendering/shaders/vbo.h>
#include <cgogn/rendering/map_render.h>
#include <cgogn/rendering/drawer.h>
#include <cgogn/rendering/volume_drawer.h>
#include <cgogn/rendering/topo_drawer.h>
#include <cgogn/geometry/algos/picking.h>
#include <cgogn/rendering/frame_manipulator.h>


#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

using namespace cgogn::numerics;
//using Map3 = cgogn::CMap3Tetra;
//using Map3 = cgogn::CMap3Hexa;
using Map3 = cgogn::CMap3;
using Vec3 = Eigen::Vector3d;
//using Vec3 = cgogn::geometry::Vec_T<std::array<float64,3>>;

template <typename T>
using VertexAttribute = Map3::VertexAttribute<T>;


class Viewer : public QOGLViewer
{
public:

	using TopoDrawer = cgogn::rendering::TopoDrawer;
	using VolumeDrawer = cgogn::rendering::VolumeDrawer;
	using DisplayListDrawer = cgogn::rendering::DisplayListDrawer;

	Viewer();
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(Viewer);

	virtual void draw();
	virtual void init();
	virtual void keyPressEvent(QKeyEvent*);
	void keyReleaseEvent(QKeyEvent*);
	virtual void mousePressEvent(QMouseEvent*);
	virtual void mouseReleaseEvent(QMouseEvent*);
	virtual void mouseMoveEvent(QMouseEvent*);

	void import(const std::string& volumeMesh);
	virtual ~Viewer();
	virtual void closeEvent(QCloseEvent *e);

private:

	void rayClick(QMouseEvent* event, qoglviewer::Vec& P, qoglviewer::Vec& Q);

	void plane_clip_from_frame();

	Map3 map_;
	VertexAttribute<Vec3> vertex_position_;

	cgogn::geometry::AABB<Vec3> bb_;

	std::unique_ptr<cgogn::rendering::VBO> vbo_pos_;

	std::unique_ptr<TopoDrawer> topo_drawer_;
	std::unique_ptr<TopoDrawer::Renderer> topo_drawer_rend_;

	std::unique_ptr<VolumeDrawer> volume_drawer_;
	std::unique_ptr<VolumeDrawer::Renderer> volume_drawer_rend_;

	std::unique_ptr<DisplayListDrawer> drawer_;
	std::unique_ptr<DisplayListDrawer::Renderer> drawer_rend_;

	std::unique_ptr<cgogn::rendering::FrameManipulator> frame_manip_;

	bool vol_rendering_;
	bool edge_rendering_;
	bool topo_drawering_;

	float32 expl_;

	QVector4D plane_clipping1_;
	float32 plane_thickness_;
	bool thick_plane_mode_;
};

//
// IMPLEMENTATION
//

void Viewer::rayClick(QMouseEvent* event, qoglviewer::Vec& P, qoglviewer::Vec& Q)
{
	P = camera()->unprojectedCoordinatesOf(qoglviewer::Vec(event->x(), event->y(), 0.0));
	Q = camera()->unprojectedCoordinatesOf(qoglviewer::Vec(event->x(), event->y(), 1.0));
}

void Viewer::import(const std::string& volumeMesh)
{
	cgogn::io::import_volume<Vec3>(map_, volumeMesh);

	vertex_position_ = map_.template get_attribute<Vec3, Map3::Vertex>("position");
	if (!vertex_position_.is_valid())
	{
		cgogn_log_error("Viewer::import") << "Missing attribute position. Aborting.";
		std::exit(EXIT_FAILURE);
	}

	if (!map_.check_map_integrity())
	{
		cgogn_log_error("Viewer::import") << "Integrity of map not respected. Aborting.";
		std::exit(EXIT_FAILURE);
	}

	cgogn::geometry::compute_AABB(vertex_position_, bb_);

	setSceneRadius(bb_.diag_size()/2.0);
	Vec3 center = bb_.center();
	setSceneCenter(qoglviewer::Vec(center[0], center[1], center[2]));
	showEntireScene();

	map_.check_map_integrity();
}

Viewer::~Viewer()
{}

void Viewer::closeEvent(QCloseEvent*)
{
	vbo_pos_.reset();
	topo_drawer_.reset();
	topo_drawer_rend_.reset();
	volume_drawer_.reset();
	volume_drawer_rend_.reset();
	drawer_.reset();
	drawer_rend_.reset();
	cgogn::rendering::ShaderProgram::clean_all();
}

Viewer::Viewer() :
	map_(),
	vertex_position_(),
	bb_(),
	vbo_pos_(nullptr),
	topo_drawer_(nullptr),
	topo_drawer_rend_(nullptr),
	volume_drawer_(nullptr),
	volume_drawer_rend_(nullptr),
	drawer_(nullptr),
	drawer_rend_(nullptr),
	vol_rendering_(true),
	edge_rendering_(true),
	topo_drawering_(true),
	expl_(0.8f),
	plane_clipping1_(0,0,0,0),
	thick_plane_mode_(false)
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

		case Qt::Key_T:
			topo_drawering_ = !topo_drawering_;
			break;
		case Qt::Key_Plus:
			expl_ += 0.05f;
			volume_drawer_rend_->set_explode_volume(expl_);
			topo_drawer_->set_explode_volume(expl_);
			topo_drawer_->update<Vec3>(map_,vertex_position_);
			break;
		case Qt::Key_Minus:
			expl_ -= 0.05f;
			volume_drawer_rend_->set_explode_volume(expl_);
			topo_drawer_->set_explode_volume(expl_);
			topo_drawer_->update<Vec3>(map_,vertex_position_);
			break;
		case Qt::Key_X:
			frame_manip_->rotate(cgogn::rendering::FrameManipulator::Xr, 0.1507f);
			break;
		case Qt::Key_P:
			if (ev->modifiers() & Qt::ControlModifier)
			{
				thick_plane_mode_ = !thick_plane_mode_;
			}
			else if (thick_plane_mode_)
			{
				if (ev->modifiers() & Qt::ShiftModifier)
					plane_thickness_ += bb_.diag_size()/200;
				else
				{
					if (plane_thickness_>= bb_.diag_size()/200)
						plane_thickness_ -= bb_.diag_size()/200;
				}
			}
			if (thick_plane_mode_)
			{
				volume_drawer_rend_->set_thick_clipping_plane(plane_clipping1_,plane_thickness_);
				topo_drawer_rend_->set_thick_clipping_plane(plane_clipping1_,plane_thickness_);
			}
			else
			{
				volume_drawer_rend_->set_clipping_plane(plane_clipping1_);
				topo_drawer_rend_->set_clipping_plane(plane_clipping1_);
			}
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
	rayClick(event, P, Q);
	Vec3 A(P[0], P[1], P[2]);
	Vec3 B(Q[0], Q[1], Q[2]);


	if ((event->modifiers() & Qt::ControlModifier) && !(event->modifiers() & Qt::ShiftModifier))
		frame_manip_->pick(event->x(), event->y(),P,Q);

	if ((event->modifiers() & Qt::ShiftModifier) && !(event->modifiers() & Qt::ControlModifier))
	{
		drawer_->new_list();
		std::vector<Map3::Volume> selected;
		cgogn::geometry::picking<Vec3>(map_, vertex_position_, A, B, selected);
		cgogn_log_info("Viewer") << "Selected volumes: " << selected.size();
		if (!selected.empty())
		{
			drawer_->line_width(2.0);
			drawer_->begin(GL_LINES);
			// closest vol in red
			drawer_->color3f(1.0, 0.0, 0.0);
			cgogn::rendering::add_to_drawer<Vec3>(map_, selected[0], vertex_position_, drawer_.get());
			// others in yellow
			drawer_->color3f(1.0, 1.0, 0.0);
			for (uint32 i = 1u; i < selected.size(); ++i)
				cgogn::rendering::add_to_drawer<Vec3>(map_, selected[i], vertex_position_, drawer_.get());
			drawer_->end();
		}
		drawer_->line_width(4.0);
		drawer_->begin(GL_LINES);
		drawer_->color3f(1.0, 0.0, 1.0);
		drawer_->vertex3fv(A);
		drawer_->vertex3fv(B);
		drawer_->end();

		drawer_->end_list();
	}

	if ((event->modifiers() & Qt::ShiftModifier) && (event->modifiers() & Qt::ControlModifier))
	{
		cgogn::Dart da;
		if (thick_plane_mode_)
			da = topo_drawer_->pick(A,B,plane_clipping1_, plane_thickness_);
		else
			da = topo_drawer_->pick(A,B,plane_clipping1_);
		if (!da.is_nil())
		{
			topo_drawer_->update_color(da, Vec3(1.0,0.0,0.0));
		}
	}

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

		if (thick_plane_mode_)
		{
			volume_drawer_rend_->set_thick_clipping_plane(plane_clipping1_,plane_thickness_);
			topo_drawer_rend_->set_thick_clipping_plane(plane_clipping1_,plane_thickness_);
		}
		else
		{
			volume_drawer_rend_->set_clipping_plane(plane_clipping1_);
			topo_drawer_rend_->set_clipping_plane(plane_clipping1_);
		}
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
		volume_drawer_rend_->draw_faces(proj,view,this);
		glDisable(GL_POLYGON_OFFSET_FILL);
	}

	if (edge_rendering_)
		volume_drawer_rend_->draw_edges(proj,view,this);

	if (topo_drawering_)
		topo_drawer_rend_->draw(proj,view,this);

	drawer_rend_->draw(proj, view, this);

	frame_manip_->draw(true,true,proj, view, this);
}

void Viewer::init()
{
	glClearColor(0.1f,0.1f,0.3f,0.0f);

	vbo_pos_ = cgogn::make_unique<cgogn::rendering::VBO>(3);
	cgogn::rendering::update_vbo(vertex_position_, vbo_pos_.get());

	topo_drawer_ =  cgogn::make_unique<cgogn::rendering::TopoDrawer>();
	topo_drawer_rend_ = topo_drawer_->generate_renderer();
	topo_drawer_->set_explode_volume(expl_);
	topo_drawer_->update<Vec3>(map_,vertex_position_);

	volume_drawer_ = cgogn::make_unique<cgogn::rendering::VolumeDrawer>();
	volume_drawer_->update_face<Vec3>(map_,vertex_position_);
	volume_drawer_->update_edge<Vec3>(map_,vertex_position_);

	volume_drawer_rend_ = volume_drawer_->generate_renderer();
	volume_drawer_rend_->set_explode_volume(expl_);

	drawer_ = cgogn::make_unique<cgogn::rendering::DisplayListDrawer>();
	drawer_rend_ = drawer_->generate_renderer();


	frame_manip_ = cgogn::make_unique<cgogn::rendering::FrameManipulator>();
	frame_manip_->set_size(bb_.diag_size()/4);
	frame_manip_->set_position(bb_.max());
	frame_manip_->z_plane_param(QColor(200,200,200),-1.5f,-1.5f, 2.0f);

	plane_thickness_ = bb_.diag_size()/20;

	plane_clip_from_frame();

	if (thick_plane_mode_)
	{
		volume_drawer_rend_->set_thick_clipping_plane(plane_clipping1_,plane_thickness_);
		topo_drawer_rend_->set_thick_clipping_plane(plane_clipping1_,plane_thickness_);
	}
	else
	{
		volume_drawer_rend_->set_clipping_plane(plane_clipping1_);
		topo_drawer_rend_->set_clipping_plane(plane_clipping1_);
	}
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
	std::string volumeMesh;
	if (argc < 2)
	{
		cgogn_log_debug("viewer_topo3") << "USAGE: " << argv[0] << " [filename]";
		volumeMesh = std::string(DEFAULT_MESH_PATH) + std::string("vtk/nine_hexas.vtu");
		cgogn_log_debug("viewer_topo3") << "Using default mesh \"" << volumeMesh << "\".";
	}
	else
		volumeMesh = std::string(argv[1]);

	QApplication application(argc, argv);
	qoglviewer::init_ogl_context();

	// Instantiate the viewer.
	Viewer viewer;
	viewer.setWindowTitle("viewer_topo3");
	viewer.import(volumeMesh);
	viewer.show();

	// Run main loop.
	return application.exec();
}
