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

#ifndef CGOGN_TOPOLOGY_FEATURES_H_
#define CGOGN_TOPOLOGY_FEATURES_H_

#include <cgogn/topology/algos/distance_field.h>
#include <cgogn/topology/algos/scalar_field.h>

namespace cgogn
{

namespace topology
{

/**
 * class FeaturesFinder : support the extraction of topological features of manifolds:
 * - extractions of maximal diameter
 * - extractions of topological features
 * - filtering of topological features
 */
template <typename Scalar, typename MAP>
class FeaturesFinder
{
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Face = typename MAP::Face;

	template<typename T>
	using VertexAttribute = typename MAP::template VertexAttribute<T>;
	template<typename T>
	using EdgeAttribute = typename MAP::template EdgeAttribute<T>;

	using VertexMarkerStore = typename cgogn::CellMarkerStore<MAP, Vertex::ORBIT>;
	using FaceMarkerStore = typename cgogn::CellMarkerStore<MAP, Face::ORBIT>;

	using ScalarField = cgogn::topology::ScalarField<Scalar,MAP>;

public:

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(FeaturesFinder);

	FeaturesFinder(MAP& map,
				   const AdjacencyCache<MAP>& cache,
				   const EdgeAttribute<Scalar>& weight) :
		map_(map),
		cache_(cache),
		edge_weight_(weight),
		distance_field_(map, cache, weight),
		intern_edge_weight_(false)
	{
		distance_to_A_ = map.template add_attribute<Scalar, Vertex>("__feature_A__");
		distance_to_B_ = map.template add_attribute<Scalar, Vertex>("__feature_B__");
		paths_ = map.template add_attribute<Vertex, Vertex>("__paths__");
	}

	FeaturesFinder(MAP& map,
				   const AdjacencyCache<MAP>& cache) :
		map_(map),
		cache_(cache),
		distance_field_(map, cache),
		intern_edge_weight_(true)
	{
		edge_weight_ = distance_field_.edge_weight();
		distance_to_A_ = map.template add_attribute<Scalar, Vertex>("__distance_to_A__");
		distance_to_B_ = map.template add_attribute<Scalar, Vertex>("__distance_to_B__");
		paths_ = map.template add_attribute<Vertex, Vertex>("__paths__");
	}

	~FeaturesFinder()
	{
		map_.remove_attribute(distance_to_A_);
		map_.remove_attribute(distance_to_B_);
		map_.remove_attribute(paths_);
	}

	/**
	 * @brief Search a maximal diameter made of two remote vertices
	 * @param[in] central_vertex a vertex from which the distance are computed
	 * @param[out] features the two vertices that define the maximal diameter
	 * @return the lenght of the maximal diameter
	 * The two furthest vertices from the center are searched first.
	 * Then their distance is locally maximized.
	 */
	Scalar get_maximal_diameter(Vertex central_vertex, std::vector<Vertex>& features)
	{
		// Init A with a vertex near the center of the mesh
		Vertex A = central_vertex;

		// Init B with the furthest vertice from A
		distance_field_.dijkstra_compute_paths({A}, distance_to_A_, paths_);
		Vertex B = distance_field_.find_maximum(distance_to_A_);
		Scalar max_distance = distance_to_A_[B];

		// Try to optimize these two vertices by locally maximizing their distance
		Scalar current_distance = Scalar(0);
		while (current_distance < max_distance)
		{
			// Store the actual distance
			current_distance = max_distance;

			// Move A to the furthest vertices from B
			distance_field_.dijkstra_compute_paths({B}, distance_to_B_, paths_);
			A = distance_field_.find_maximum(distance_to_B_);

			// Move B to the furthest vertices from A
			distance_field_.dijkstra_compute_paths({A}, distance_to_A_, paths_);
			B = distance_field_.find_maximum(distance_to_A_);
			max_distance = distance_to_A_[B];
		};

		features.push_back(A);
		features.push_back(B);
		return max_distance;
	}

	/**
	 * @brief Extract the basic features of the manifold.
	 * @param[in] central_vertex a vertex from which the distance are computed
	 * @param[out] the found features: Startig from a central vertex,
	 * compute the maxima of the distance_field build from this single source.
	 * The features are the maxima of this distance field.
	 */
	void basic_features(Vertex central_vertex, std::vector<Vertex>& features)
	{
		// Init A with a vertex near the center of the mesh
		Vertex A = central_vertex;

		// Build a distance field from A
		distance_field_.dijkstra_compute_paths({A}, distance_to_A_, paths_);

		// Get the maxima in the scalar field distance_to_A_
		ScalarField scalar_field_A(map_, cache_, distance_to_A_);
		scalar_field_A.critical_vertex_analysis();
		std::vector<Vertex> maxima_A = scalar_field_A.maxima();

		// Copy the maxima in the features set
				for (Vertex v: maxima_A)
					features.push_back(v);

		/* Another possible implementation
		 *
		// Select as features the maxima that are greater than their mean value
		Scalar mean_value = Scalar(0);
		uint32 count = 0;
		for (Vertex v: maxima_A)
		{
			mean_value += distance_to_A_[v];
			++count;
		}
		mean_value /= Scalar(count);

		for (Vertex v: maxima_A)
			if (distance_to_A_[v] > mean_value) features.push_back(v);
		*/
	}

	/**
	 * @brief Select a central vertex of an open 3-manifold
	 * @return the central vertex
	 * The method first computes the local maxima of the distance to boundary scalar field.
	 * Then, it selects the most central one, i.e. the vertex that generates a distance field
	 * with the lower maximum.
	 */
	Vertex central_vertex()
	{
		// Search the maxima in the distance to boundary field
		distance_field_.distance_to_boundary(distance_to_A_);
		ScalarField scalar_field(map_, cache_, distance_to_A_);
		scalar_field.critical_vertex_analysis();
		std::vector<Vertex> maxima = scalar_field.maxima();

		// Search the most central vertices in these maxima,
		// i.e. whose distance field has a minimal diameter
		Scalar min = std::numeric_limits<Scalar>::max();
		Vertex min_vertex = maxima.front();

		for (Vertex v : maxima)
		{
			distance_field_.distance_to_features({v}, distance_to_A_);
			Vertex max_vertex = distance_field_.find_maximum(distance_to_A_);
			Scalar max = distance_to_A_[max_vertex];
			if (min > max)
			{
				min = max;
				min_vertex = v;
			}
		}
		return min_vertex;
	}

private:

	/**
	 * Remove the vertices whose distance to local minima of the given scalar field
	 * is below the threshold and (if any) add the source of the closest to the filtered set
	 */
	void features_filter(std::vector<Vertex>& vertices,
						 std::vector<Vertex>& filtered,
						 Scalar threshold,
						 VertexAttribute<Scalar>& scalar_field,
						 VertexAttribute<Vertex>& path_to_sources)
	{
		if (vertices.empty()) return;
		std::vector<Vertex> vertices_to_keep;
		Vertex min_vertex = vertices.front();
		Scalar min_dist = scalar_field[min_vertex];

		for (Vertex v: vertices) {
			Scalar dist = scalar_field[v];
			// Search the vertex that is the closest from a source (i.e. with minimal value)
			if (dist < min_dist) {
				min_vertex = v;
				min_dist = dist;
			}
			// Select the vertices whose distance is greater than the threshold
			if (dist > threshold) vertices_to_keep.push_back(v);
		}
		// A vertex that is near a source has been found => lookup its source
		if (min_dist <= threshold)
		{
			Vertex source = min_vertex;
			while (path_to_sources[source].dart != source.dart)
				source = path_to_sources[source];
			filtered.push_back(source);
		}
		// Replace the initial vertices by the filtered ones
		vertices.clear();
		vertices.swap(vertices_to_keep);
	}

	/**
	 * @brief Compute the intersection of two sets of vertices.
	 * @param set_A the first set of vertices
	 * @param set_B the second set of vertices
	 * @param intersection the computed intersection
	 */
	void intersection(std::vector<Vertex>& set_A,
					  std::vector<Vertex>& set_B,
					  std::vector<Vertex>& intersection)
	{
		std::vector<Vertex> v1_not_in_v2;
		std::vector<Vertex> v2_not_in_v1;
		intersection.clear();

		for (Vertex v: set_A)
			for (Vertex u: set_B)
				if (u.dart == v.dart)
					intersection.push_back(u);
	}

public:

	/**
	 * @brief Extract the features of the manifold.
	 * @param[in] central_vertex a vertex from which the initial distances are computed
	 * @param[in] features_proximity the ratio, between 0 and 1, of the maximal diameter used to filter features
	 * @param[out] scalar_field the distance field to the seclected features
	 * @param[out] features the found features vertices
	 * First, compute the maximal diameter {A, B} of the manifold from the given central vertex.
	 * Secondly, search the maxima of the distance fields generated by A and B.
	 * Then, add in decreasing order of distance, the maxima that are common to these two sets
	 * and filter out the features that are too near from the already selected ones.
	 * The filtering distance is a ratio of the diameter : features_proximity * d(A,B).
	 * Experimentations give good results for a ratio between 0.25f and 0.4f for long features
	 * (like arms and legs), smaller ratio allow the detection of smaller features.
	 */
	void filtered_features(Vertex central_vertex,
						   Scalar features_proximity,
						   VertexAttribute<Scalar>& scalar_field,
						   std::vector<Vertex>& features)
	{
		// Get two Vertices A and B on a maximal diameter
		// as side effect this build the two scalar field distance_to_A_ and distance_to_B_
		Scalar max_distance = get_maximal_diameter(central_vertex, features);

		// Get the maxima in the scalar field distance_to_A_
		ScalarField scalar_field_A(map_, cache_, distance_to_A_);
		scalar_field_A.critical_vertex_analysis();
		std::vector<Vertex> maxima_A = scalar_field_A.maxima();

		// Get the maxima in the scalar field distance_to_B_
		ScalarField scalar_field_B(map_, cache_, distance_to_B_);
		scalar_field_B.critical_vertex_analysis();
		std::vector<Vertex> maxima_B = scalar_field_B.maxima();

		// Set the criteria to filter out near features
		Scalar filter_distance = features_proximity * max_distance;

		// Build a scalar field from the initial set of features = {A, B}
		// In this scalar field the selected features are sources of the distance field,
		// thus they hold a value of zero.
		distance_field_.dijkstra_compute_paths(features, scalar_field, paths_);

		// Initialize the sets of filtered maxima with the first features A and B
		std::vector<Vertex> filtered_maxima_A;
		std::vector<Vertex> filtered_maxima_B;
		filtered_maxima_A.push_back(features[0]); // vertex A
		filtered_maxima_B.push_back(features[1]); // vertex B

		// Filter maxima_A and maxima_B with the initial scalar field or in other words
		// remove the vertices of maxima_A and maxima_B that are not far enough from A and B.
		features_filter(maxima_A, filtered_maxima_A, filter_distance, scalar_field, paths_);
		features_filter(maxima_B, filtered_maxima_B, filter_distance, scalar_field, paths_);

		// Iteratively add features in the scalar field
		// and filter maxima_A and maxima_B with this new features
		// The added features is the furthest from all sources currently added to the scalar field
		// The loop ends when all possible maxima have been tried or when
		// the distance to the last added source is smaller than a target distance
		Scalar actual_distance = max_distance;
		while (filter_distance < actual_distance && !(maxima_A.empty() && maxima_B.empty())) {
			// Select the maximum as a new feature
			Vertex v = distance_field_.find_maximum(scalar_field);
			features.push_back(v);
			actual_distance = scalar_field[v];
			if (filter_distance < actual_distance)
			{
				// Rebuild the scalar field from the update set of features
				distance_field_.dijkstra_compute_paths(features, scalar_field, paths_);
				// Filter the remaing maxima with this scalar field or in other words
				// remove the vertices that are not far enough from the current features
				// and add v in the filtered sets if there is a maxima thas is close to v
				features_filter(maxima_A, filtered_maxima_A, filter_distance, scalar_field, paths_);
				features_filter(maxima_B, filtered_maxima_B, filter_distance, scalar_field, paths_);
			}
		}

		intersection(filtered_maxima_A, filtered_maxima_B, features);
		cgogn_log_info("get_filtered_features") << "Selected features " << features.size();
	}

private:

	MAP& map_;
	AdjacencyCache<MAP> cache_;
	VertexAttribute<Scalar> distance_to_A_;
	VertexAttribute<Scalar> distance_to_B_;
	VertexAttribute<Vertex> paths_;
	EdgeAttribute<Scalar> edge_weight_;
	DistanceField<Scalar, MAP> distance_field_;
	bool intern_edge_weight_;
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_TOPOLOGY_FEATURES_CPP_))
extern template class CGOGN_TOPLOGY_API FeaturesFinder<float32, CMap2>;
extern template class CGOGN_TOPLOGY_API FeaturesFinder<float64, CMap2>;
extern template class CGOGN_TOPLOGY_API FeaturesFinder<float32, CMap3>;
extern template class CGOGN_TOPLOGY_API FeaturesFinder<float64, CMap3>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_TOPOLOGY_FEATURES_CPP_))

} // namespace topology

} // namespace cgogn

#endif // CGOGN_TOPOLOGY_FEATURES_H_
