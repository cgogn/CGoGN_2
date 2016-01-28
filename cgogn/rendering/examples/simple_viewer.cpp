/****************************************************************************

 Copyright (C) 2002-2014 Gilles Debunne. All rights reserved.

 This file is part of the QGLViewer library version 2.6.3.

 http://www.libqglviewer.com - contact@libqglviewer.com

 This file may be used under the terms of the GNU General Public License 
 versions 2.0 or 3.0 as published by the Free Software Foundation and
 appearing in the LICENSE file included in the packaging of this file.
 In addition, as a special exception, Gilles Debunne gives you certain 
 additional rights, described in the file GPL_EXCEPTION in this package.

 libQGLViewer uses dual licensing. Commercial/proprietary software must
 purchase a libQGLViewer Commercial License.

 This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.

*****************************************************************************/

#include "simple_viewer.h"
#include <geometry/algos/normal.h>


void Viewer::import(const std::string& surfaceMesh)
{
	cgogn::thread_start();

	cgogn::io::import_surface<Vec3>(map, surfaceMesh);

	vertex_position_ = map.get_attribute<Vec3, Map2::VERTEX>("position");
	vertex_normal_ = map.add_attribute<Vec3, Map2::VERTEX>("normal");

	cgogn::geometry::compute_normal_vertices<Vec3>(map, vertex_position_, vertex_normal_);
//	for(Vec3& n:vertex_normal_)
//		n = Vec3(0,0,1);

	cgogn::geometry::algo::compute_bounding_box(vertex_position_,bb_);

	setSceneRadius(bb_.diagSize());
	Vec3 center = bb_.center();
	setSceneCenter(qglviewer::Vec(center[0],center[1],center[2]));
	showEntireScene();
}




void Viewer::draw()
{
	QMatrix4x4 proj;
	QMatrix4x4 view;
	camera()->getProjectionMatrix(proj);
	camera()->getModelViewMatrix(view);

	shader1_->bind();
	shader1_->set_matrices(proj,view);
	shader1_->bind_vao(0);

	glPointSize(5.0f);
	shader1_->set_color(QColor(255,0,0));
	render_->draw(cgogn::rendering::POINTS);

	shader1_->set_color(QColor(255,255,0));
	render_->draw(cgogn::rendering::LINES);

	shader1_->release_vao(0);
	shader1_->release();

	glEnable(GL_POLYGON_OFFSET_FILL) ;
	glPolygonOffset(1.0f, 1.0f) ;

	shader2_->bind();
	shader2_->set_matrices(proj,view);
	shader2_->bind_vao(0);
	render_->draw(cgogn::rendering::TRIANGLES);
	shader2_->release_vao(0);
	shader2_->release();

	shader3_->bind();
	shader3_->set_matrices(proj,view);
	shader3_->bind_vao(0);
	render_->draw(cgogn::rendering::POINTS);
	shader3_->release_vao(0);
	shader3_->release();
}

void Viewer::init()
{
	glClearColor(0.1,0.1,0.3,0.0);
	vbo1_ = new cgogn::rendering::VBO;
	cgogn::rendering::update_vbo(vertex_position_,*vbo1_);

	vbo_norm_ = new cgogn::rendering::VBO;
	cgogn::rendering::update_vbo(vertex_normal_,*vbo_norm_);


	render_ = new cgogn::rendering::MapRender();


	render_->init_primitives(map,cgogn::rendering::POINTS);
	render_->init_primitives(map,cgogn::rendering::LINES);
	render_->init_primitives(map,cgogn::rendering::TRIANGLES);

	//	vbo1_->allocate(buffer_.size()/3,3,&(buffer_[0]));

	shader1_ = new cgogn::rendering::ShaderSimpleColor;
	shader1_->add_vao();
	shader1_->set_vao(0,vbo1_);

	shader2_ = new cgogn::rendering::ShaderFlat;
	shader2_->add_vao();
	shader2_->set_vao(0,vbo1_);

	shader2_->bind();
	shader2_->set_front_color(QColor(0,200,0));
	shader2_->set_back_color(QColor(0,0,200));
	shader2_->set_ambiant_color(QColor(5,5,5));
	shader2_->release();


	shader3_ = new cgogn::rendering::ShaderVectorPerVertex;
	shader3_->add_vao();
	shader3_->set_vao(0,vbo1_,vbo_norm_);

	shader3_->bind();
	shader3_->set_color(QColor(200,0,0));
	shader3_->set_length(bb_.diagSize()/50);

	shader2_->release();

}

