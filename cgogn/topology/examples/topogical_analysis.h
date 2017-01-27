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

#include <cgogn/core/utils/numerics.h>
#include <cgogn/core/cmap/cmap3.h>
//#include <cgogn/core/cmap/cmap3_tetra.h>
#include <cgogn/io/map_import.h>
#include <cgogn/io/map_export.h>

#include <cgogn/rendering/map_render.h>
#include <cgogn/rendering/drawer.h>

#include <cgogn/rendering/shaders/shader_scalar_per_vertex.h>
#include <cgogn/rendering/shaders/shader_bold_line.h>
#include <cgogn/rendering/shaders/shader_point_sprite.h>

#include <cgogn/geometry/algos/angle.h>
#include <cgogn/geometry/algos/length.h>
#include <cgogn/geometry/algos/curvature.h>
#include <cgogn/geometry/algos/bounding_box.h>

#include <cgogn/topology/types/adjacency_cache.h>
#include <cgogn/topology/algos/features.h>


#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

template <typename MAP>
class TopologicalAnalyser : public QOGLViewer
{
	using Vec3 = Eigen::Vector3d;
	using Scalar = Eigen::Vector3d::Scalar;

	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Face = typename MAP::Face;
	using Volume = typename MAP::Volume;
	template<typename T>
	using VertexAttribute = typename MAP::template VertexAttribute<T>;
	template<typename T>
	using EdgeAttribute = typename MAP::template EdgeAttribute<T>;

public:

	TopologicalAnalyser() :
		map_(),
		adjacency_cache_(map_),
		vertex_position_(),
		scalar_field_(),
		edge_metric_(),
		bb_(),
		vbo_pos_(nullptr),
		vbo_scalar_(nullptr),
		map_render_(nullptr),
		features_drawer_(nullptr),
		features_renderer_(nullptr),
		features_proximity(0.3),
		nb_(0u),
		map_rendering_(true),
		vertices_rendering_(false),
		edge_rendering_(false),
		feature_points_rendering_(true)
	{}

	TopologicalAnalyser(const TopologicalAnalyser&) = delete;
	TopologicalAnalyser& operator=(const TopologicalAnalyser&) = delete;

	virtual ~TopologicalAnalyser()
	{
		vbo_pos_.reset();
		vbo_scalar_.reset();
		map_render_.reset();
		features_drawer_.reset();
		features_renderer_.reset();
	}

	virtual void init()
	{
		glClearColor(0.1f,0.1f,0.3f,0.0f);

		vbo_pos_ = cgogn::make_unique<cgogn::rendering::VBO>(3);
		cgogn::rendering::update_vbo(vertex_position_, vbo_pos_.get());

		float size = float(bb_.max_size()) / 500.0f;

		param_point_sprite_ = cgogn::rendering::ShaderPointSprite::generate_param();
		param_point_sprite_->color_ = QColor(180,180,180);
		param_point_sprite_->size_ = size;
		param_point_sprite_->set_position_vbo(vbo_pos_.get());

		param_edge_ = cgogn::rendering::ShaderBoldLine::generate_param();
		param_edge_->color_ = QColor(10,10,80);
		param_edge_->width_= 1.5f;
		param_edge_->set_position_vbo(vbo_pos_.get());

		vbo_scalar_ = cgogn::make_unique<cgogn::rendering::VBO>(1);
		cgogn::rendering::update_vbo(scalar_field_, vbo_scalar_.get());

		param_scalar_ = cgogn::rendering::ShaderScalarPerVertex::generate_param();
		param_scalar_->color_map_ = cgogn::rendering::ShaderScalarPerVertex::ColorMap::BGR;
		param_scalar_->show_iso_lines_ = true;
		param_scalar_->min_value_ = 0.0f;
		param_scalar_->max_value_ = 0.0f;
		param_scalar_->set_all_vbos(vbo_pos_.get(), vbo_scalar_.get());
		update_color();

		map_render_ = cgogn::make_unique<cgogn::rendering::MapRender>();
		update_topology();

		features_drawer_ = cgogn::make_unique<cgogn::rendering::DisplayListDrawer>();
		features_renderer_ = features_drawer_->generate_renderer();

		lines_drawer_ = cgogn::make_unique<cgogn::rendering::DisplayListDrawer>();
		lines_renderer_ = lines_drawer_->generate_renderer();

		distance_to_center_function();
	}

	virtual void draw()
	{
		QMatrix4x4 proj;
		QMatrix4x4 view;
		camera()->getProjectionMatrix(proj);
		camera()->getModelViewMatrix(view);

		if(feature_points_rendering_)
		{
			features_renderer_->draw(proj, view, this);
			lines_renderer_->draw(proj, view, this);
		}

		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(1.0f, 2.0f);
		if (map_rendering_)
		{
			param_scalar_->bind(proj,view);
			map_render_->draw(cgogn::rendering::TRIANGLES);
			param_scalar_->release();
		}
		glDisable(GL_POLYGON_OFFSET_FILL);

		if (vertices_rendering_)
		{
			param_point_sprite_->bind(proj,view);
			map_render_->draw(cgogn::rendering::POINTS);
			param_point_sprite_->release();
		}

		if (edge_rendering_)
		{
			param_edge_->bind(proj,view);
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			map_render_->draw(cgogn::rendering::LINES);
			glDisable(GL_BLEND);
			param_edge_->release();
		}

	}

	void update_topology()
	{
		map_render_->init_primitives(map_, cgogn::rendering::POINTS);
		map_render_->init_primitives(map_, cgogn::rendering::LINES);
		map_render_->init_primitives(map_, cgogn::rendering::TRIANGLES);
	}

	/**
	 * @brief transform the scalar field so that its values lie between 0 and 1
	 * and inverse the min and max.
	 */
	void update_color()
	{
		// Search the maximal and minimal value of the scalar field
		Scalar min = std::numeric_limits<Scalar>::max();
		Scalar max = std::numeric_limits<Scalar>::lowest();
		for(auto& v : scalar_field_)
		{
			min = std::min(min, v);
			max = std::max(max, v);
		}
		// Update the shader parameters
		param_scalar_->min_value_ = float(min);
		param_scalar_->max_value_ = float(max);

		// Update de VBO
		cgogn::rendering::update_vbo(scalar_field_, vbo_scalar_.get());
	}

	void draw_segments(const std::vector<Edge>& edges, float r, float g, float b)
	{
		lines_drawer_->line_width(2.0f);
		lines_drawer_->begin(GL_LINES);
		lines_drawer_->color3f(r, g, b);

		for (auto& e: edges) {
			lines_drawer_->vertex3fv(vertex_position_[Vertex(e.dart)]);
			lines_drawer_->vertex3fv(vertex_position_[Vertex(map_.phi1(e.dart))]);
		}
		lines_drawer_->end();
	}

	void draw_vertices(const std::vector<Vertex>& vertices,
					   float r, float g, float b, float ratio, int shift=0)
	{
		float radius = ratio * float(bb_.max_size()) / 50.0f;
		features_drawer_->ball_size(radius);
		features_drawer_->begin(GL_POINTS);
		features_drawer_->color3f(r, g, b);

		for (auto& v: vertices) {
			switch(shift) {
				case 0 :
					features_drawer_->vertex3fv(vertex_position_[v]);
					break;
				case 1 :
					features_drawer_->vertex3fv(vertex_position_[v]+Vec3(radius/Scalar(2),-radius/Scalar(2),Scalar(0)));
					break;
				case 2 :
					features_drawer_->vertex3fv(vertex_position_[v]+Vec3(-radius/Scalar(2),radius/Scalar(2),Scalar(0)));
					break;
				case 3 :
					features_drawer_->vertex3fv(vertex_position_[v]+Vec3(radius/Scalar(2),radius/Scalar(2),Scalar(0)));
					break;
				default:
					break;
			}
		}
		features_drawer_->end();
	}

	void draw_scalar_field(bool level_sets = false, bool morse_complex = false)
	{
		update_color();

		// Draw the critical points
		features_drawer_->new_list();
		cgogn::topology::ScalarField<Scalar, MAP> scalar_field(map_, adjacency_cache_, scalar_field_);
		scalar_field.critical_vertex_analysis();
		draw_vertices(scalar_field.maxima(), 1.0f, 1.0f, 1.0f, 0.4f, 1u);
		draw_vertices(scalar_field.minima(), 1.0f, 0.0f, 0.0f, 0.4f, 1u);
		draw_vertices(scalar_field.saddles(), 1.0f, 1.0f, 0.0f, 0.4f, 1u);

		std::vector<Vertex> MS_complex_boundary;
		scalar_field.extract_ascending_3_manifold(MS_complex_boundary);
		draw_vertices(MS_complex_boundary, 1.0f, 0.0f, 1.0f, 0.2f);
		MS_complex_boundary.clear();
		scalar_field.extract_descending_3_manifold(MS_complex_boundary);
		draw_vertices(MS_complex_boundary, 1.0f, 0.6f, 0.0f, 0.2f);

		features_drawer_->end_list();

		// Draw the level sets
		lines_drawer_->new_list();
		if (level_sets)
		{
			std::vector<Edge> level_lines;
			scalar_field.extract_level_sets(level_lines);
			draw_segments(level_lines, 1.0f, 1.0f, 1.0f);
		}

		// Draw the ascending and descending manyfold of the morse complex
		if (morse_complex)
		{
			std::vector<Edge> morse_lines;
			scalar_field.extract_descending_1_manifold(morse_lines);
			draw_segments(morse_lines, 0.5f, 0.5f, 1.0f);
			morse_lines.clear();
			scalar_field.extract_ascending_1_manifold(morse_lines);
			draw_segments(morse_lines, 1.0f, 0.5f, 0.0f);
		}
		lines_drawer_->end_list();
	}

	void draw_scalar_field2(std::vector<Vertex>& features1, std::vector<Vertex>& features2)
	{
		update_color();

		// Draw the critical points
		features_drawer_->new_list();
		cgogn::topology::ScalarField<Scalar, MAP> scalar_field(map_, adjacency_cache_, scalar_field_);
		scalar_field.critical_vertex_analysis();
		draw_vertices(scalar_field.maxima(), 1.0f, 1.0f, 1.0f, 0.4f);
		draw_vertices(scalar_field.minima(), 1.0f, 0.0f, 0.0f, 0.1f);
		draw_vertices(scalar_field.saddles(), 1.0f, 1.0f, 0.0f, 0.2f);
		draw_vertices(features1, 1.0f, 0.0f, 1.0f, 0.6f, 1);
		draw_vertices(features2, 0.0f, 1.0f, 1.0f, 0.6f, 2);

		std::vector<Vertex> MS_complex_boundary;
		scalar_field.extract_ascending_3_manifold(MS_complex_boundary);
		draw_vertices(MS_complex_boundary, 1.0f, 0.0f, 1.0f, 0.2f);

		features_drawer_->end_list();

		// Draw the ascending and descending manyfold of the morse complex
		lines_drawer_->new_list();
		std::vector<Edge> morse_lines;
		scalar_field.extract_descending_1_manifold(morse_lines);
		draw_segments(morse_lines, 0.5f, 0.5f, 1.0f);
		morse_lines.clear();
		scalar_field.extract_ascending_1_manifold(morse_lines);
		draw_segments(morse_lines, 1.0f, 0.5f, 0.0f);
		lines_drawer_->end_list();
	}

	virtual void keyPressEvent(QKeyEvent *e)
	{
		switch (e->key()) {
			case Qt::Key_M:
				map_rendering_ = !map_rendering_;
				break;
			case Qt::Key_V:
				vertices_rendering_ = !vertices_rendering_;
				break;
			case Qt::Key_E:
				edge_rendering_ = !edge_rendering_;
				break;
			case Qt::Key_A:
				feature_points_rendering_ = !feature_points_rendering_;
				break;
			case Qt::Key_Plus:
				++nb_;
				draw_scalar_field(false, false);
				break;
			case Qt::Key_Minus:
				if (nb_>0) --nb_;
				draw_scalar_field(false, false);
				break;
			case Qt::Key_0:
			{
				if (dimension_ == 2u)
					two_features_geodesic_distance_function();
				else
					distance_to_boundary_function();
				break;
			}
			case Qt::Key_1:
			{
				distance_to_center_function();
				break;
			}
			case Qt::Key_2:
			{
				edge_length_weighted_geodesic_distance_function();
				break;
			}
			case Qt::Key_3:
			{
				edge_length_weighted_geodesic_distance_function2();
				//				curvature_weighted_geodesic_distance_function();
				break;
			}
			case Qt::Key_4:
			{
				edge_length_weighted_morse_function();
				break;
			}
			case Qt::Key_5:
			{
				curvature_weighted_morse_function();
				break;
			}
			case Qt::Key_7:
			{
				show_level_sets();
				break;
			}
		}
		QOGLViewer::keyPressEvent(e);
		update();
	}

	virtual void closeEvent(QCloseEvent*)
	{
	}

	template <typename T, typename std::enable_if<T::DIMENSION == 2>::type* = nullptr>
	void import_concrete(const std::string& filename)
	{
		cgogn::io::import_surface<Vec3>(map_, filename);
		cgogn_log_info("import") << "2D mesh imported";
	}

	template <typename T, typename std::enable_if<T::DIMENSION == 3>::type* = nullptr>
	void import_concrete(const std::string& filename)
	{
		cgogn::io::import_volume<Vec3>(map_, filename);
		cgogn_log_info("import") << "3D mesh imported";
	}

	void import(const std::string& filename)
	{
		import_concrete<MAP>(filename);

		vertex_position_ = map_.template get_attribute<Vec3, Vertex>("position");
		if (!vertex_position_.is_valid())
		{
			cgogn_log_error("import") << "Missing attribute position. Aborting.";
			std::exit(EXIT_FAILURE);
		}

		adjacency_cache_.init();
		scalar_field_ = map_.template add_attribute<Scalar, Vertex>("scalar_field_");
		edge_metric_ = map_.template add_attribute<Scalar, Edge>("edge_metric");
		cgogn::geometry::compute_AABB(vertex_position_, bb_);

		setSceneRadius(bb_.diag_size()/2.0);
		Vec3 center = bb_.center();
		setSceneCenter(qoglviewer::Vec(center[0], center[1], center[2]));
		showEntireScene();
	}

	void height_function()
	{
		map_.foreach_cell([&] (Vertex v)
		{
			scalar_field_[v] = vertex_position_[v][0];
		});

		draw_scalar_field();
	}

	void distance_to_basic_features()
	{
		// Find features for the edge_metric
		std::vector<Vertex> features;
		compute_length(edge_metric_);

		Vertex center = cgogn::geometry::central_vertex<Vec3, MAP>(map_, vertex_position_);
		cgogn::topology::FeaturesFinder<Scalar, MAP> features_finder(map_, adjacency_cache_, edge_metric_);
		features_finder.basic_features(center, features);

		// Build the scalar field from the selected features
		cgogn::topology::DistanceField<Scalar, MAP> distance_field(map_, adjacency_cache_, edge_metric_);
		distance_field.distance_to_features(features, scalar_field_);

		for (auto& s : scalar_field_) s = Scalar(1) - s;

		draw_scalar_field();
	}

	void distance_to_boundary_function()
	{
		compute_length(edge_metric_);

		std::vector<Vertex> boundary_vertices;
		map_.foreach_cell([&](Vertex v)
		{
			if (map_.is_incident_to_boundary(v))
				boundary_vertices.push_back(v);
		});

		VertexAttribute<Scalar> boundary_field;
		boundary_field = map_.template add_attribute<Scalar, Vertex>("boundary_field");

		cgogn::topology::DistanceField<Scalar, MAP> distance_field(map_, adjacency_cache_, edge_metric_);
		distance_field.distance_to_features(boundary_vertices, boundary_field);

		cgogn::topology::ScalarField<Scalar, MAP> scalar_field(map_, adjacency_cache_, boundary_field);
		scalar_field.critical_vertex_analysis();

		std::vector<Vertex> features;
		for (Vertex v : scalar_field.maxima())
			features.push_back(v);

		distance_field.morse_distance_to_features(features, scalar_field_);

		for (auto& s : scalar_field_) s = Scalar(1) - s;

		draw_scalar_field();

		map_.remove_attribute(boundary_field);
	}

	void distance_to_boundary_function2()
	{
		compute_length(edge_metric_);
		cgogn::topology::DistanceField<Scalar, MAP> distance_field(map_, adjacency_cache_, edge_metric_);

		VertexAttribute<Scalar> boundary_field;
		boundary_field = map_.template add_attribute<Scalar, Vertex>("boundary_field");

		std::vector<Vertex> boundary_vertices;
		map_.foreach_cell([&](Vertex v)
		{
			if (map_.is_incident_to_boundary(v))
				boundary_vertices.push_back(v);
		});
		distance_field.distance_to_features(boundary_vertices, boundary_field);

		cgogn::topology::ScalarField<Scalar, MAP> scalar_field(map_, adjacency_cache_, boundary_field);
		scalar_field.critical_vertex_analysis();

		std::vector<Vertex> features;
		for (Vertex v : boundary_vertices)
			features.push_back(v);

		std::vector<Vertex> features1;
		std::vector<Vertex> features2;
		Scalar min_max = std::numeric_limits<Scalar>::max();
		for (Vertex v : scalar_field.maxima())
		{
			min_max = std::min(min_max, boundary_field[v]);
			features.push_back(v);
			features1.push_back(v);
			cgogn_log_info("feature+1") << boundary_field[v];
		}
		Scalar target_distance = 0.9 * min_max;

		distance_field.distance_to_features(features, boundary_field);
		scalar_field.critical_vertex_analysis();
		for (Vertex v : scalar_field.maxima())
		{
			if (boundary_field[v] > target_distance)
			{
				features.push_back(v);
				features2.push_back(v);
				cgogn_log_info("feature+2") << boundary_field[v];
			}
		}

		distance_field.distance_to_features(features, scalar_field_);

		Scalar actual_distance;
		Vertex max_vertex;
		int i = 0;
		do {
			max_vertex = distance_field.find_maximum(scalar_field_);
			actual_distance = scalar_field_[max_vertex];
			cgogn_log_info("actual_distance") << actual_distance;
			if (i>0 && actual_distance > target_distance)
			{
				boundary_vertices.push_back(max_vertex);
				features.push_back(max_vertex);
				distance_field.distance_to_features(boundary_vertices, scalar_field_);
				--i;
			}

		} while (i>0 && actual_distance > target_distance);

//		distance_field.distance_to_features(features, scalar_field_);
//		for (auto& s : scalar_field_) s = Scalar(1) - s;

		draw_scalar_field2(features1, features2);

		map_.remove_attribute(boundary_field);
	}

	void distance_to_center_function()
	{
		compute_length(edge_metric_);
		cgogn::topology::DistanceField<Scalar, MAP> distance_field(map_, adjacency_cache_, edge_metric_);

		if (dimension_ == 2u)
		{
			Vertex center = cgogn::geometry::central_vertex<Vec3, MAP>(map_, vertex_position_);
			distance_field.distance_to_features({center}, scalar_field_);
		}
		else
		{
			cgogn::topology::FeaturesFinder<Scalar, MAP> features_finder(map_, adjacency_cache_, edge_metric_);
			Vertex center = features_finder.central_vertex();
			distance_field.distance_to_features({center}, scalar_field_);
//			distance_field.distance_to_boundary(scalar_field_);
//			Vertex center = distance_field.find_maximum(scalar_field_);
//			distance_field.distance_to_features({center}, scalar_field_);
//			edge_metric_normalize();
//			distance_field.distance_to_features({center}, scalar_field_);
		}

		draw_scalar_field(false, true);
	}

	void two_features_geodesic_distance_function()
	{
		// Find features for the edge_metric
		std::vector<Vertex> features;
		compute_length(edge_metric_);

		Vertex center = cgogn::geometry::central_vertex<Vec3, MAP>(map_, vertex_position_);
		cgogn::topology::FeaturesFinder<Scalar, MAP> features_finder(map_, adjacency_cache_, edge_metric_);
		features_finder.get_maximal_diameter(center, features);

		// Build the scalar field from the selected features
		cgogn::topology::DistanceField<Scalar, MAP> distance_field(map_, adjacency_cache_, edge_metric_);
		distance_field.distance_to_features(features, scalar_field_);

		for (auto& s : scalar_field_) s = Scalar(1) - s;

		draw_scalar_field();
	}

	void scalar_field_inverse_normalize()
	{
		Scalar min = std::numeric_limits<Scalar>::max();
		Scalar max = std::numeric_limits<Scalar>::lowest();

		for(auto& v : scalar_field_)
		{
			min = std::min(min, v);
			max = std::max(max, v);
		}
		Scalar delta = max - min;
		Scalar diff  = max - Scalar(2)*min;

		for (auto& s : scalar_field_) s = (diff - s) / delta;
	}

	void edge_metric_normalize()
	{
		scalar_field_inverse_normalize();

		map_.foreach_cell([&](Edge e)
		{
			std::pair<Vertex,Vertex> p = map_.vertices(e);
			Scalar weight = Scalar(0.5f)*(scalar_field_[p.first]+scalar_field_[p.second]);
			edge_metric_[e] *= weight;
		});
	}

	void edge_length_weighted_geodesic_distance_function()
	{
		// Find features for the edge_metric
		std::vector<Vertex> features;
		compute_length(edge_metric_);

		Vertex center = cgogn::geometry::central_vertex<Vec3, MAP>(map_, vertex_position_);
		cgogn::topology::FeaturesFinder<Scalar, MAP> features_finder(map_, adjacency_cache_, edge_metric_);
		features_finder.filtered_features(center, features_proximity, scalar_field_, features);


		// Build the scalar field from the selected features
		cgogn::topology::DistanceField<Scalar, MAP> distance_field(map_, adjacency_cache_, edge_metric_);
		distance_field.distance_to_features(features, scalar_field_);

		// scalar_field_inverse_normalize();
		// for (auto& s : scalar_field_) s = Scalar(1) - s;

		draw_scalar_field();
	}

	void edge_length_weighted_geodesic_distance_function2()
	{
		// Find features for the edge_metric
		std::vector<Vertex> features;
		compute_length(edge_metric_);

		Vertex center = cgogn::geometry::central_vertex<Vec3, MAP>(map_, vertex_position_);
		cgogn::topology::FeaturesFinder<Scalar, MAP> features_finder(map_, adjacency_cache_, edge_metric_);
		features_finder.filtered_features(center, features_proximity, scalar_field_, features);

		edge_metric_normalize();

		// Build the scalar field from the selected features
		cgogn::topology::DistanceField<Scalar, MAP> distance_field(map_, adjacency_cache_, edge_metric_);
		distance_field.distance_to_features(features, scalar_field_);

		scalar_field_inverse_normalize();
		//		for (auto& s : scalar_field_) s = Scalar(1) - s;

		draw_scalar_field();
	}

	template <typename T, typename std::enable_if<T::DIMENSION == 2>::type* = nullptr>
	void curvature_weighted_geodesic_distance_function()
	{
		// Find features for the edge_metric
		std::vector<Vertex> features;
		compute_curvature<MAP>(edge_metric_);

		Vertex center = cgogn::geometry::central_vertex<Vec3, MAP>(map_, vertex_position_);
		cgogn::topology::FeaturesFinder<Scalar, MAP> features_finder(map_, adjacency_cache_, edge_metric_);
		features_finder.filtered_features(center, features_proximity, scalar_field_, features);

		// Build the scalar field from the selected features
		cgogn::topology::DistanceField<Scalar, MAP> distance_field(map_, adjacency_cache_, edge_metric_);
		distance_field.distance_to_features(features, scalar_field_);

		for (auto& s : scalar_field_) s = Scalar(1) - s;

		draw_scalar_field();
	}

	template <typename T, typename std::enable_if<T::DIMENSION == 3>::type* = nullptr>
	void curvature_weighted_geodesic_distance_function()
	{
		// Find features for the edge_metric
		std::vector<Vertex> features;

		Vertex center = cgogn::geometry::central_vertex<Vec3, MAP>(map_, vertex_position_);
		cgogn::topology::FeaturesFinder<Scalar, MAP> features_finder(map_, adjacency_cache_);
		features_finder.filtered_features(center, features_proximity, scalar_field_, features);

		// Build the scalar field from the selected features
		cgogn::topology::DistanceField<Scalar, MAP> distance_field(map_, adjacency_cache_);
		distance_field.distance_to_features(features, scalar_field_);

		for (auto& s : scalar_field_) s = Scalar(1) - s;

		draw_scalar_field();
	}

	void curvature_weighted_geodesic_distance_function()
	{
		curvature_weighted_geodesic_distance_function<MAP>();
	}

	void edge_length_weighted_morse_function()
	{
		// Find features for the edge_metric
		std::vector<Vertex> features;
		compute_length(edge_metric_);

		Vertex center = cgogn::geometry::central_vertex<Vec3, MAP>(map_, vertex_position_);
		cgogn::topology::FeaturesFinder<Scalar, MAP> features_finder(map_, adjacency_cache_, edge_metric_);
		features_finder.filtered_features(center, features_proximity, scalar_field_, features);
		edge_metric_normalize();

		// Build the scalar field from the selected features
		cgogn::topology::DistanceField<Scalar, MAP> distance_field(map_, adjacency_cache_, edge_metric_);
		distance_field.morse_distance_to_features(features, scalar_field_);

		for (auto& s : scalar_field_) s = Scalar(1) - s;

		draw_scalar_field(false, true);
	}

	template <typename T, typename std::enable_if<T::DIMENSION == 2>::type* = nullptr>
	void curvature_weighted_morse_function()
	{
		// Find features for the edge_metric
		std::vector<Vertex> features;
		compute_curvature<MAP>(edge_metric_);

		Vertex center = cgogn::geometry::central_vertex<Vec3, MAP>(map_, vertex_position_);
		cgogn::topology::FeaturesFinder<Scalar, MAP> features_finder(map_, adjacency_cache_, edge_metric_);
		features_finder.filtered_features(center, features_proximity, scalar_field_, features);

		// Build the scalar field from the selected features
		cgogn::topology::DistanceField<Scalar, MAP> distance_field(map_, adjacency_cache_, edge_metric_);
		distance_field.morse_distance_to_features(features, scalar_field_);

		draw_scalar_field();
	}

	template <typename T, typename std::enable_if<T::DIMENSION == 3>::type* = nullptr>
	void curvature_weighted_morse_function()
	{
		// Find features for the edge_metric
		std::vector<Vertex> features;

		Vertex center = cgogn::geometry::central_vertex<Vec3, MAP>(map_, vertex_position_);
		cgogn::topology::FeaturesFinder<Scalar, MAP> features_finder(map_, adjacency_cache_, edge_metric_);
		features_finder.filtered_features(center, features_proximity, scalar_field_, features);

		// Build the scalar field from the selected features
		cgogn::topology::DistanceField<Scalar, MAP> distance_field(map_, adjacency_cache_);
		distance_field.morse_distance_to_features(features, scalar_field_);

		draw_scalar_field();
	}

	void curvature_weighted_morse_function()
	{
		curvature_weighted_morse_function<MAP>();
	}

	void show_level_sets()
	{
		draw_scalar_field(true);
	}

	void compute_length(EdgeAttribute<Scalar>& length)
	{
		map_.foreach_cell([&](Edge e)
		{
			length[e] = cgogn::geometry::length<Vec3>(map_, e, vertex_position_);
		});
	}

	template <typename CONCRETE_MAP, typename std::enable_if<CONCRETE_MAP::DIMENSION == 2>::type* = nullptr>
	void compute_curvature(EdgeAttribute<Scalar>& edge_metric)
	{
		EdgeAttribute<Scalar> length = map_.template add_attribute<Scalar, Edge>("lenght");
		EdgeAttribute<Scalar> edgeangle = map_.template add_attribute<Scalar, Edge>("edgeangle");
		EdgeAttribute<Scalar> edgeaera = map_.template add_attribute<Scalar, Edge>("edgeaera");

		VertexAttribute<Scalar> kmax = map_.template add_attribute<Scalar, Vertex>("kmax");
		VertexAttribute<Scalar> kmin = map_.template add_attribute<Scalar, Vertex>("kmin");
		VertexAttribute<Vec3> vertex_normal = map_.template add_attribute<Vec3, Vertex>("vertex_normal");
		VertexAttribute<Vec3> Kmax = map_.template add_attribute<Vec3, Vertex>("Kmax");
		VertexAttribute<Vec3> Kmin = map_.template add_attribute<Vec3, Vertex>("Kmin");
		VertexAttribute<Vec3> knormal = map_.template add_attribute<Vec3, Vertex>("knormal");

		compute_length(length);

		cgogn::geometry::compute_angle_between_face_normals<Vec3, MAP>(map_, vertex_position_, edgeangle);
		cgogn::geometry::compute_incident_faces_area<Vec3, Edge, MAP>(map_, vertex_position_, edgeaera);

		Scalar meanEdgeLength = cgogn::geometry::mean_edge_length<Vec3>(map_, vertex_position_);

		Scalar radius = Scalar(2.0) * meanEdgeLength;

		cgogn::geometry::compute_curvature<Vec3>(map_, radius, vertex_position_, vertex_normal, edgeangle,edgeaera,kmax,kmin,Kmax,Kmin,knormal);

		//compute kmean
		VertexAttribute<Scalar> kmean = map_.template add_attribute<Scalar, Vertex>("kmean");

		double min = std::numeric_limits<double>::max();
		double max = std::numeric_limits<double>::lowest();

		map_.foreach_cell([&](Vertex v)
		{
			kmean[v] = (kmin[v] + kmax[v]) / Scalar(2.0);
			min = std::min(min, kmean[v]);
			max = std::max(max, kmean[v]);
		});

		//compute kgaussian
		VertexAttribute<Scalar> kgaussian = map_.template add_attribute<Scalar, Vertex>("kgaussian");

		min = std::numeric_limits<double>::max();
		max = std::numeric_limits<double>::lowest();

		map_.foreach_cell([&](Vertex v)
		{
			kgaussian[v] = (kmin[v] * kmax[v]);
			min = std::min(min, kgaussian[v]);
			max = std::max(max, kgaussian[v]);
		});

		//compute kindex
		VertexAttribute<Scalar> k1 = map_.template add_attribute<Scalar, Vertex>("k1");
		VertexAttribute<Scalar> k2 = map_.template add_attribute<Scalar, Vertex>("k2");
		VertexAttribute<Scalar> kI = map_.template add_attribute<Scalar, Vertex>("kI");

		map_.foreach_cell([&](Vertex v)
		{
			k1[v] = kmean[v] + std::sqrt(kmean[v] * kmean[v] - kgaussian[v]);
			k2[v] = kmean[v] - std::sqrt(kmean[v] * kmean[v] - kgaussian[v]);

			if(k1[v] == k2[v])
				kI[v] = 0.0;
			else
				kI[v] = (2 / M_PI) * std::atan((k1[v] + k2[v]) / (k1[v] - k2[v]));
		});

		//build a metric to feed dijkstra
		Scalar avg_e(0);
		Scalar avg_ki(0);
		cgogn::numerics::uint32 nbe = 0;

		map_.foreach_cell([&](Edge e)
		{
			avg_e = length[e];
			avg_ki = kI[Vertex(e.dart)] + kI[Vertex(map_.phi1(e.dart))];
			++nbe;
		});
		avg_e /= nbe;
		avg_ki /= nbe;

		map_.foreach_cell([&](Edge e)
		{
			Scalar diffKI = kI[Vertex(e.dart)] - kI[Vertex(map_.phi1(e.dart))];

			Scalar w(0.0);
			if(kI[Vertex(e.dart)] < 0.0 && kI[Vertex(map_.phi1(e.dart))] < 0.0)
				w = 0.05;

			edge_metric[e] = (length[e] / avg_e) + (w * (diffKI / avg_ki));
		});

		map_.remove_attribute(length);
		map_.remove_attribute(edgeangle);
		map_.remove_attribute(edgeaera);
		map_.remove_attribute(kmax);
		map_.remove_attribute(kmin);
		map_.remove_attribute(vertex_normal);
		map_.remove_attribute(Kmax);
		map_.remove_attribute(Kmin);
		map_.remove_attribute(knormal);

		map_.remove_attribute(kmean);
		map_.remove_attribute(kgaussian);
		map_.remove_attribute(k1);
		map_.remove_attribute(k2);
		map_.remove_attribute(kI);
	}

private:
	MAP map_;
	static const unsigned int dimension_ = MAP::DIMENSION;

	cgogn::topology::AdjacencyCache<MAP> adjacency_cache_;

	VertexAttribute<Vec3> vertex_position_;
	VertexAttribute<Scalar> scalar_field_;

	EdgeAttribute<Scalar> edge_metric_;

	cgogn::geometry::AABB<Vec3> bb_;

	std::unique_ptr<cgogn::rendering::VBO> vbo_pos_;
	std::unique_ptr<cgogn::rendering::VBO> vbo_scalar_;

	std::unique_ptr<cgogn::rendering::MapRender> map_render_;

	std::unique_ptr<cgogn::rendering::DisplayListDrawer> features_drawer_;
	std::unique_ptr<cgogn::rendering::DisplayListDrawer::Renderer> features_renderer_;

	std::unique_ptr<cgogn::rendering::DisplayListDrawer> lines_drawer_;
	std::unique_ptr<cgogn::rendering::DisplayListDrawer::Renderer> lines_renderer_;

	std::unique_ptr<cgogn::rendering::ShaderBoldLine::Param> param_edge_;
	std::unique_ptr<cgogn::rendering::ShaderPointSprite::Param> param_point_sprite_;
	std::unique_ptr<cgogn::rendering::ShaderScalarPerVertex::Param> param_scalar_;

	Scalar features_proximity;

	unsigned int nb_;
	bool map_rendering_;
	bool vertices_rendering_;
	bool edge_rendering_;
	bool feature_points_rendering_;
};
