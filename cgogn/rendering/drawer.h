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

#ifndef RENDERING_DRAWER_H_
#define RENDERING_DRAWER_H_

#include <rendering/shaders/shader_color_per_vertex.h>
#include <rendering/shaders/shader_flat.h>
#include <rendering/shaders/shader_bold_line.h>
#include <rendering/shaders/shader_round_point.h>
#include <rendering/shaders/vbo.h>
#include <rendering/dll.h>
#include <QOpenGLFunctions_3_3_Core>

namespace cgogn
{

namespace rendering
{


class CGOGN_RENDERING_API Drawer
{
	struct PrimParam
	{
		unsigned int begin;
		GLenum mode;
		float width;
		unsigned int nb;
		bool aa;
		PrimParam(std::size_t b, GLenum m, float w, bool a) : begin(static_cast<unsigned int>(b)), mode(m), width(w), nb(0), aa(a){}
	};

	using Vec3f = std::array<float,3>;

protected:

	static ShaderColorPerVertex* shader_cpv_;
	static ShaderBoldLine* shader_bl_;
	static ShaderRoundPoint* shader_rp_;
	static int nb_instances_;

	VBO* vbo_pos_;
	VBO* vbo_col_;

	std::vector<Vec3f> data_pos_;
	std::vector<Vec3f> data_col_;

	std::vector<PrimParam> begins_point_;
	std::vector<PrimParam> begins_round_point_;
	std::vector<PrimParam> begins_line_;
	std::vector<PrimParam> begins_bold_line_;
	std::vector<PrimParam> begins_face_;
	std::vector<PrimParam>* current_begin_;

	unsigned int vao_cpv_;
	unsigned int vao_bl_;
	unsigned int vao_rp_;

	float current_size_;
	bool current_aa_;

	QOpenGLFunctions_3_3_Core* ogl33_;

public:

	/**
	 * constructor, init all buffers (data and OpenGL) and shader
	 * @Warning need OpenGL context
	 */
	Drawer(QOpenGLFunctions_3_3_Core* ogl33);

	/**
	 * release buffers and shader
	 */
	~Drawer();


	/**
	 * init the data structure
	 */
	void new_list();

	/**
	 * as glBegin, but need a newList call before
	 * @param mode: GL_POINTS, GL_LINES, GL_LINE_LOOP, GL_TRIANGLES,
	 */
	void begin(GLenum mode);

	/**
	 * as glEnd
	 */
	void end();

	/**
	 * finalize the data initialization
	 * drawn is done if newList called with GL_COMPILE_AND_EXECUTE
	 */
	void end_list();

	/**
	 * use as glVertex3f
	 */
	void vertex3f(float x, float y, float z);

	/**
	 * use as glColor3f
	 */
	void color3f(float r, float g, float b);


	inline void vertex3fv(const std::array<float,3>& xyz)
	{
		vertex3f(xyz[0],xyz[1],xyz[2]);
	}

	inline void color3fv(const std::array<float,3>& rgb)
	{
		color3f(rgb[0],rgb[1],rgb[2]);
	}

	template <typename SCAL>
	inline void vertex3fv(SCAL* xyz)
	{
		vertex3f(float(xyz[0]),float(xyz[1]),float(xyz[2]));
	}

	template <typename SCAL>
	inline void color3fv(SCAL* rgb)
	{
		color3f(float(rgb[0]),float(rgb[1]),float(rgb[2]));
	}

	template <typename VEC3>
	inline void vertex3fv(const VEC3& xyz)
	{
		vertex3f(float(xyz[0]),float(xyz[1]),float(xyz[2]));
	}

	template <typename VEC3>
	inline void color3fv(const VEC3& rgb)
	{
		color3f(float(rgb[0]),float(rgb[1]),float(rgb[2]));
	}


	/**
	 * use as a glCallList (draw the compiled drawing list)
	 * @param projection projection matrix
	 * @param modelview modelview matrix
	 */
	void call_list(const QMatrix4x4& projection, const QMatrix4x4& modelview);

	/**
	 * usr as glPointSize
	 */
	inline void point_size(float ps)
	{
		current_aa_ = false;
		current_size_ = ps;
	}

	inline void point_size_aa(float ps)
	{
		current_aa_ = true;
		current_size_ = ps;
	}

	/**
	 * usr as glLineWidth
	 */
	inline void line_width(float lw)
	{
		current_aa_ = false;
		current_size_ = lw;
	}

	inline void line_width_aa(float lw)
	{
		current_aa_ = true;
		current_size_ = 2.0*lw;
	}


};



} // namespace rendering

} // namespace cgogn

#endif // RENDERING_DRAWER_H_
