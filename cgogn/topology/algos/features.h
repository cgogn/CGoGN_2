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
		intern_edge_weight_(false),
		edge_weight_(weight),
		distance_field_(map, cache, weight)
	{
		map.add_attribute(distance_to_A_, "__feature_A__");
		map.add_attribute(distance_to_B_, "__feature_B__");
		map.add_attribute(paths_, "__paths__");
	}

	FeaturesFinder(MAP& map,
				   const AdjacencyCache<MAP>& cache) :
		map_(map),
		cache_(cache),
		intern_edge_weight_(true),
		distance_field_(map, cache)
	{
		edge_weight_ = distance_field_.edge_weight();
		map.add_attribute(distance_to_A_, "__distance_to_A__");
		map.add_attribute(distance_to_B_, "__distance_to_B__");
		map.add_attribute(paths_, "__paths__");
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
	 * @param[out] features the two vertices that define the maximal diameter
	 * Startig from a central vertex, compute the maxima of the distance_field
	 * build from this single source. Then select the maxima whose distance
	 * to the central vertex is greater than the average distance.
	 */
	void get_basic_features(Vertex central_vertex, std::vector<Vertex>& features)
	{
		// Init A with a vertex near the center of the mesh
		Vertex A = central_vertex;

		// Build a distance field from A
		distance_field_.dijkstra_compute_paths({A}, distance_to_A_, paths_);

		// Get the maxima in the scalar field distance_to_A_
		ScalarField scalar_field_A(map_, cache_, distance_to_A_);
		scalar_field_A.critical_vertex_analysis();
		std::vector<Vertex> maxima_A = scalar_field_A.get_maxima();

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

	void get_filtered_features(Vertex central_vertex,
							   VertexAttribute<Scalar>& scalar_field,
							   std::vector<Vertex>& features)
	{
		// Get two Vertices A and B on a maximal diameter
		// as side effect this build the two scalar field distance_to_A_ and distance_to_B_
		Scalar max_distance = get_maximal_diameter(central_vertex, features);

		// Get the maxima in the scalar field distance_to_A_
		ScalarField scalar_field_A(map_, cache_, distance_to_A_);
		scalar_field_A.critical_vertex_analysis();
		std::vector<Vertex> maxima_A = scalar_field_A.get_maxima();

		// Get the maxima in the scalar field distance_to_B_
		ScalarField scalar_field_B(map_, cache_, distance_to_B_);
		scalar_field_B.critical_vertex_analysis();
		std::vector<Vertex> maxima_B = scalar_field_B.get_maxima();

		// Set the criteria to stop the features filtering to 1/2 of the maximal diameter
		Scalar target_distance = max_distance / Scalar(2);
		// Set the criteria to remove too near features to 1/3 of the target distance
		Scalar filter_distance = target_distance / Scalar(3);

		// Build a scalar field from the initial set of features = {A, B}
		// In this scalar field the selected features are sources of the distance field,
		// so have a value of zero.
		distance_field_.dijkstra_compute_paths(features, scalar_field, paths_);

		// Initialize the sets of filtered maxima with the first features A and B
		std::vector<Vertex> filtered_maxima_A;
		std::vector<Vertex> filtered_maxima_B;
		filtered_maxima_A.push_back(features[0]);
		filtered_maxima_B.push_back(features[1]);

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
		//std::cout << "Distances: (" << max_distance << ") ";
		while (target_distance < actual_distance && !(maxima_A.empty() && maxima_B.empty())) {
			// Select the maximum as a new feature
			Vertex v = distance_field_.find_maximum(scalar_field);
			features.push_back(v);
			actual_distance = scalar_field[v];
			if (target_distance < actual_distance)
			{
				//std::cout << actual_distance << " ";
				// Rebuild the scalar field from the update set of features
				distance_field_.dijkstra_compute_paths(features, scalar_field, paths_);
				// Filter the remaing maxima with this scalar field or in other words
				// remove the vertices that are not far enough from the current features
				// and add v in the filtered sets if there is a maxima thas is close to v
				features_filter(maxima_A, filtered_maxima_A, filter_distance, scalar_field, paths_);
				features_filter(maxima_B, filtered_maxima_B, filter_distance, scalar_field, paths_);
			}
		}
		//std::cout << std::endl;

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
extern template class CGOGN_TOPLOGY_API FeaturesFinder<float32, CMap2<DefaultMapTraits>>;
extern template class CGOGN_TOPLOGY_API FeaturesFinder<float64, CMap2<DefaultMapTraits>>;
extern template class CGOGN_TOPLOGY_API FeaturesFinder<float32, CMap3<DefaultMapTraits>>;
extern template class CGOGN_TOPLOGY_API FeaturesFinder<float64, CMap3<DefaultMapTraits>>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_TOPOLOGY_FEATURES_CPP_))

} // namespace topology

} // namespace cgogn

#endif // CGOGN_TOPOLOGY_FEATURES_H_
