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

#include "qglviewer.h"

#include <QMatrix4x4>
#include <rendering/shaders/shader_simple_color.h>
#include <rendering/shaders/shader_flat.h>
#include <rendering/shaders/shader_vector_per_vertex.h>
#include <rendering/shaders/vbo.h>

#include <core/cmap/cmap2.h>
#include <io/map_import.h>
#include <geometry/algos/bounding_box.h>
#include <rendering/map_render.h>


#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

struct MyMapTraits : public cgogn::DefaultMapTraits
{
	static const unsigned int CHUNK_SIZE = 8192;
};
using Map2 = cgogn::CMap2_T<MyMapTraits, cgogn::CMap2Type<MyMapTraits>>;
using Vec3 = Eigen::Vector3d;

template<typename T>
using VertexAttributeHandler = Map2::VertexAttributeHandler<T>;



class Viewer : public QGLViewer
{
public:

	Map2 map;
	VertexAttributeHandler<Vec3> vertex_position_;
	VertexAttributeHandler<Vec3> vertex_normal_;

	cgogn::geometry::BoundingBox<Vec3> bb_;

	cgogn::rendering::MapRender* render_;

	virtual void draw();
	virtual void init();

	void import(const std::string& surfaceMesh);


	cgogn::rendering::VBO* vbo1_;
	cgogn::rendering::VBO* vbo_norm_;
//	cgogn::rendering::VBO* m_vbo_col2;

	cgogn::rendering::ShaderSimpleColor* shader1_;
	cgogn::rendering::ShaderFlat* shader2_;
	cgogn::rendering::ShaderVectorPerVertex* shader3_;

};
