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

#ifndef CGOGN_RENDERING_DRAWER_H_
#define CGOGN_RENDERING_DRAWER_H_

#include <cgogn/rendering/dll.h>

#include <cgogn/rendering/shaders/shader_color_per_vertex.h>
#include <cgogn/rendering/shaders/shader_flat.h>
#include <cgogn/rendering/shaders/shader_bold_line.h>
#include <cgogn/rendering/shaders/shader_round_point.h>
#include <cgogn/rendering/shaders/shader_point_sprite.h>
#include <cgogn/rendering/shaders/vbo.h>

#include <QOpenGLFunctions_3_3_Core>

namespace cgogn
{

namespace rendering
{

class CGOGN_RENDERING_API Drawer
{
	struct PrimParam
	{
		uint32 begin;
		GLenum mode;
		float32 width;
		uint32 nb;
		bool aa;
		PrimParam(std::size_t b, GLenum m, float32 w, bool a) : begin(uint32(b)), mode(m), width(w), nb(0), aa(a){}
	};

	using Vec3f = std::array<float32,3>;

protected:

	static ShaderColorPerVertex* shader_cpv_;
	static ShaderBoldLine* shader_bl_;
	static ShaderRoundPoint* shader_rp_;
	static ShaderPointSprite* shader_ps_;
	static int32 nb_instances_;

	VBO* vbo_pos_;
	VBO* vbo_col_;

	std::vector<Vec3f> data_pos_;
	std::vector<Vec3f> data_col_;

	std::vector<PrimParam> begins_point_;
	std::vector<PrimParam> begins_round_point_;
	std::vector<PrimParam> begins_balls_;

	std::vector<PrimParam> begins_line_;
	std::vector<PrimParam> begins_bold_line_;
	std::vector<PrimParam> begins_face_;
	std::vector<PrimParam>* current_begin_;

	uint32 vao_cpv_;
	uint32 vao_bl_;
	uint32 vao_rp_;
	uint32 vao_ps_;

	float32 current_size_;
	bool current_aa_;
	bool current_ball_;

	QOpenGLFunctions_3_3_Core* ogl33_;

public:

	using Self = Drawer;

	/**
	 * constructor, init all buffers (data and OpenGL) and shader
	 * @Warning need OpenGL context
	 */
	Drawer(QOpenGLFunctions_3_3_Core* ogl33);

	/**
	 * release buffers and shader
	 */
	~Drawer();
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(Drawer);

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
	void vertex3f(float32 x, float32 y, float32 z);

	/**
	 * use as glColor3f
	 */
	void color3f(float32 r, float32 g, float32 b);


	inline void vertex3fv(const std::array<float32,3>& xyz)
	{
		vertex3f(xyz[0],xyz[1],xyz[2]);
	}

	inline void color3fv(const std::array<float32,3>& rgb)
	{
		color3f(rgb[0],rgb[1],rgb[2]);
	}

	template <typename SCAL>
	inline void vertex3fv(SCAL* xyz)
	{
		vertex3f(float32(xyz[0]),float32(xyz[1]),float32(xyz[2]));
	}

	template <typename SCAL>
	inline void color3fv(SCAL* rgb)
	{
		color3f(float32(rgb[0]),float32(rgb[1]),float32(rgb[2]));
	}

	template <typename VEC3>
	inline void vertex3fv(const VEC3& xyz)
	{
		vertex3f(float32(xyz[0]),float32(xyz[1]),float32(xyz[2]));
	}

	template <typename VEC3>
	inline void color3fv(const VEC3& rgb)
	{
		color3f(float32(rgb[0]),float32(rgb[1]),float32(rgb[2]));
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
	inline void point_size(float32 ps)
	{
		current_aa_ = false;
		current_size_ = ps;
		current_ball_ = false;
	}

	inline void point_size_aa(float32 ps)
	{
		current_aa_ = true;
		current_size_ = ps;
		current_ball_ = false;
	}

	inline void ball_size(float32 ps)
	{
		current_ball_ = true;
		current_aa_ = false;
		current_size_ = ps;
	}

	/**
	 * usr as glLineWidth
	 */
	inline void line_width(float32 lw)
	{
		current_aa_ = false;
		current_size_ = lw;
	}

	inline void line_width_aa(float32 lw)
	{
		current_aa_ = true;
		current_size_ = 2.0*lw;
	}
};

} // namespace rendering

} // namespace cgogn

#endif // CGOGN_RENDERING_DRAWER_H_
