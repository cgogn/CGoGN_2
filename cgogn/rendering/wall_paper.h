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

#ifndef CGOGN_RENDERING_WALL_PAPER_H_
#define CGOGN_RENDERING_WALL_PAPER_H_

#include <cgogn/rendering/shaders/shader_texture.h>
#include <cgogn/rendering/shaders/vbo.h>
#include <cgogn/rendering/dll.h>
#include <QOpenGLFunctions_3_3_Core>

namespace cgogn
{

namespace rendering
{

/**
 * @brief WallPaper: allow rendering of a texture, front or back, full-screen or not
 *
 * Typical usage:
 *
 *  std::unique_ptr<cgogn::rendering::WallPaper> wp_;	// can be shared between contexts
 *  std::unique_ptr<cgogn::rendering::WallPaper::Renderer> wp_rend_; // one by context,
 *
 * init:
 *  wp_ = cgogn::make_unique<cgogn::rendering::WallPaper>();
 *  wp_rend_ = wp_->generate_renderer();
 *  wp_->update<Vec3>(map_,vertex_position_);
 *
 * draw:
 *  wp_rend_->draw(proj,view,this);
 *
 */
class CGOGN_RENDERING_API WallPaper
{
protected:
	std::unique_ptr<VBO> vbo_pos_;
	std::unique_ptr<VBO> vbo_tc_;
	std::unique_ptr<QOpenGLTexture> texture_;

public:
	class CGOGN_RENDERING_API Renderer
	{
		friend class WallPaper;
		std::unique_ptr<ShaderTexture::Param> param_texture_;
		WallPaper* wall_paper_data_;
		Renderer(WallPaper* wp);
	public:
		~Renderer();
		void draw(QOpenGLFunctions_3_3_Core* ogl33);
	};

	using Self = WallPaper;
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(WallPaper);

	/**
	 * constructor, init all buffers (data and OpenGL) and shader
	 * @Warning need OpenGL context
	 */
	WallPaper(const QImage& img);

	/**
	 * release buffers and shader
	 */
	~WallPaper();

	/**
	 * @brief generate a renderer (one per context)
	 * @return pointer on renderer
	 */
	inline std::unique_ptr<Renderer> generate_renderer()
	{
		return std::unique_ptr<Renderer>(new Renderer(this));
	}

	/**
	 * @brief set the texture in full screen
	 * @param front if true draw with depth of 0 (front) else with depth of ~1 (back)
	 */
	void set_full_screen(bool front = false);

	/**
	 * @brief set a local position for the image in pixel
	 * @warning position & size are converted in % when set, and used even when window size has changed
	 * @param win_w width of window
	 * @param win_h height of window
	 * @param x x pos (0 is left)
	 * @param y y pos (0 is top)
	 * @param w width to draw
	 * @param h height to draw
	 * @param front (default is back drawing)
	 */
	void set_local_position(uint32 win_w, uint32 win_h, uint32 x, uint32 y, uint32 w, uint32 h, bool front = true);

	/**
	 * @brief set a local position for the image in ratio ([0-1]) of the viewport
	 * @param x x pos (0.0 is left)
	 * @param y y pos (0 is top)
	 * @param w width to draw
	 * @param h height to draw
	 * @param front (default is front drawing)
	 */
	void set_local_position(float x, float y, float w, float h, bool front = true);

	void draw(QOpenGLFunctions_3_3_Core* ogl33);
};



} // namespace rendering

} // namespace cgogn

#endif // CGOGN_RENDERING_WALL_PAPER_H_
