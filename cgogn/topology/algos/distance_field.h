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

#ifndef CGOGN_TOPOLOGY_DISTANCE_FIELD_H_
#define CGOGN_TOPOLOGY_DISTANCE_FIELD_H_

#include <cgogn/topology/types/adjacency_cache.h>

#include <cgogn/geometry/algos/centroid.h>

namespace cgogn
{

namespace topology
{

/**
 * class DistanceField : build, query and manage distance fields.
 * A distance field is a scalar field that associated each vertex with
 * its minimal distance to a given set of sources (a set of selected vertices).
 * Maps are interpreted as graphs containing vertices linked by edges.
 * The distances are computed as the sum of the edge weights along a path.
 */
template <typename Scalar, typename MAP>
class DistanceField
{
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;

	template<typename T>
	using VertexAttribute = typename MAP::template VertexAttribute<T>;
	template<typename T>
	using EdgeAttribute = typename MAP::template EdgeAttribute<T>;

public:

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(DistanceField);

	DistanceField(MAP& map,
				  const AdjacencyCache<MAP>& cache,
				  const EdgeAttribute<Scalar>& weight) :
		map_(map),
		cache_(cache),
		intern_edge_weight_(false),
		edge_weight_(weight)
	{
	}

	DistanceField(MAP& map,
				  const AdjacencyCache<MAP>& cache) :
		map_(map),
		cache_(cache),
		intern_edge_weight_(true)
	{
		edge_weight_ = map.template add_attribute<Scalar, Edge>("__edge_weight__");

		map.foreach_cell([&](Edge e)
		{
			edge_weight_[e] = Scalar(1);
		});
	}

	~DistanceField()
	{
		if (intern_edge_weight_)
			map_.remove_attribute(edge_weight_);
	}

	const EdgeAttribute<Scalar>& edge_weight()
	{
		return edge_weight_;
	}

private:

	/**
	 * Compute for each vertex of the map, the shortest path to the sources
	 * A path is a sequence of adjacent edges whose weights are summed along the paths
	 * The method returns the lengths of the shortest paths in a VertexAttribute.
	 * @param[in] sources the vertices from which the shortest paths are computed
	 * @param[out] distance_to_source : the sums of the edge weights in the shortest paths
	 */
	void dijkstra_compute_distances(
			const std::vector<Vertex>& sources,
			VertexAttribute<Scalar>& distance_to_source)
	{
		for (auto& d : distance_to_source)
			d = std::numeric_limits<Scalar>::max();

		// A pair contains a scalar (the actual distance) and a vertex (the index of one of its darts)
		using my_pair = std::pair<Scalar, uint32>;

		// The queue is used to manage the front of the dijkstra flood that consider
		// the vertices in the order of their estimated distance to a source
		using my_queue = std::priority_queue<my_pair, std::vector<my_pair>, std::greater<my_pair> >;
		my_queue vertex_queue;

		// Initialize the sources with a zero lenght path
		for (auto& source : sources)
		{
			vertex_queue.push(std::make_pair(Scalar(0), source.dart.index));
			distance_to_source[source] = Scalar(0);
		}

		// The dijkstra flood
		while (!vertex_queue.empty())
		{
			Scalar dist = vertex_queue.top().first;
			Vertex u = Vertex(Dart(vertex_queue.top().second));

			vertex_queue.pop();

			cache_.foreach_adjacent_vertex_through_edge(u, [&](Vertex v)
			{
				Scalar distance_through_u = dist + edge_weight_[Edge(v.dart)];
				if (distance_through_u < distance_to_source[v])
				{
					distance_to_source[v] = distance_through_u;
					vertex_queue.push(std::make_pair(distance_through_u, v.dart.index));
				}
			});
		}
	}

public:

	/**
	 * Compute for each vertex of the map, the shortest path to the sources
	 * A path is a sequence of adjacent edges whose weights are summed along the paths
	 * The method returns the shortest paths and their lengths in two VertexAttributes.
	 * @param[in] sources the vertices from which the shortest paths are computed
	 * @param[out] distance_to_source : the sums of the edge weights in the shortest paths
	 * @param[out] path_to_source : a reference to the previous vertex in the shortest path
	 */
	void dijkstra_compute_paths(
			const std::vector<Vertex>& sources,
			VertexAttribute<Scalar>& distance_to_source,
			VertexAttribute<Vertex>& path_to_source)
	{
		for (auto& d : distance_to_source)
			d = std::numeric_limits<Scalar>::max();

		for (auto& p : path_to_source)
			p = Vertex();

		// A pair contains a scalar (the actual distance) and a vertex (the index of one of its darts)
		using my_pair = std::pair<Scalar, uint32>;

		// The queue is used to manage the front of the dijkstra flood that consider
		// the vertices in the order of their estimated distance to a source
		using my_queue = std::priority_queue<my_pair, std::vector<my_pair>, std::greater<my_pair> >;
		my_queue vertex_queue;

		// Initialize the sources with a zero lenght path
		for (auto& source : sources)
		{
			vertex_queue.push(std::make_pair(Scalar(0), source.dart.index));
			distance_to_source[source] = Scalar(0);
			path_to_source[source] = source;
		}

		// The dijkstra flood
		while (!vertex_queue.empty())
		{
			Scalar dist = vertex_queue.top().first;
			Vertex u = Vertex(Dart(vertex_queue.top().second));

			vertex_queue.pop();

			cache_.foreach_adjacent_vertex_through_edge(u, [&](Vertex v)
			{
				Scalar distance_through_u = dist + edge_weight_[Edge(v.dart)];
				if (distance_through_u < distance_to_source[v])
				{
					distance_to_source[v] = distance_through_u;
					path_to_source[v] = u;
					vertex_queue.push(std::make_pair(distance_through_u, v.dart.index));
				}
			});
		}
	}

private:

	/**
	 * Compute a morse function from a scalar field.
	 * The morse function has disctinct values in each vertex.
	 * Its maxima are the same than those of the scalar field.
	 * Its minima are given.
	 * The morse function regularizes the other critical points of the scalar field.
	 * @param[in] minima the vertices that minimize the scalar field
	 * @param[in] scalar_field : the scalar field
	 * @param[out] morse_function : the values of the morse function
	 * The algorithm makes a dijkstra flood of the graph using the scalar field values
	 * in place of the shortest path distance computed in dijkstra_compute_distances().
	 * The vertices are traversed in increasing value of their scalar field.
	 */
	void dijkstra_to_morse_function(
			std::vector<Vertex> minima,
			const VertexAttribute<Scalar>& scalar_field,
			VertexAttribute<Scalar>& morse_function)
	{
		for(auto& d : morse_function)
			d = Scalar(0);							// To mark unvisited vertices

		using my_pair = std::pair<Scalar, unsigned int>;
		using my_queue = std::priority_queue<my_pair, std::vector<my_pair>, std::greater<my_pair> >;

		my_queue vertex_queue;

		for (auto source : minima)
			vertex_queue.push(std::make_pair(Scalar(0), source.dart.index));

		// Run dijkstra using distance_to_source in place of the estimated geodesic distances
		// and fill the morse function with i/n where i is index of distance_to_source
		// in an sorted array of the distances and n the number of vertices.
		const Scalar n = Scalar(map_.template nb_cells<Vertex::ORBIT>());
		uint32 i = 1;
		while(!vertex_queue.empty())
		{
			Vertex u = Vertex(Dart(vertex_queue.top().second));

			vertex_queue.pop();

			morse_function[u] = Scalar(i)/n;		// Set the final value
			++i;

			cache_.foreach_adjacent_vertex_through_edge(u, [&](Vertex v)
			{
				if(morse_function[v] == Scalar(0)) {	// If not visited
					vertex_queue.push(std::make_pair(scalar_field[v], v.dart.index));
					morse_function[v] = Scalar(1);		// Set as visited
				}
			});
		}
	}

public:

	/**
	 * @brief find the vertex that maximizes the distance field,
	 * or in other words the vertex that is the furthest from all sources.
	 * @param distance_to_source : the distance to sources computed during the generation of the distance field
	 * @return a pair made of the maximum of the distance field and the vertex that maximize it.
	 */
	Vertex find_maximum(const VertexAttribute<Scalar>& distance_to_source)
	{
		Scalar max_distance = Scalar(0);
		Vertex max_vertex = Vertex();

		map_.foreach_cell([&](Vertex v)
		{
			Scalar distance = distance_to_source[v];
			if(distance > max_distance) {
				max_distance = distance;
				max_vertex = v;
			}
		});

		return max_vertex;
	}

	/**
	 * @brief find the vertex that minilizes the distance field.
	 * @param scalar_field : the scalar_filed to analyse
	 * @return a vertex that minimize the scalar field
	 */
	Vertex find_minimum(const VertexAttribute<Scalar>& scalar_field)
	{
		Scalar min_distance = std::numeric_limits<Scalar>::max();
		Vertex min_vertex = Vertex();

		map_.foreach_cell([&](Vertex v)
		{
			Scalar distance = scalar_field[v];
			if(distance < min_distance) {
				min_distance = distance;
				min_vertex = v;
			}
		});

		return min_vertex;
	}

	/**
	 * @brief Build a scalar field that represent the lenght of a minimal path
	 * from a vertex to the boundary of the Map.
	 */
	void distance_to_boundary(VertexAttribute<Scalar>& scalar_field)
	{
		std::vector<Vertex> boundary_vertices;

		map_.foreach_cell([&](Vertex v)
		{
			if (map_.is_incident_to_boundary(v))
				boundary_vertices.push_back(v);
		});

		if (boundary_vertices.size() == 0u)
			cgogn_log_error("distance_to_boundary") << "No boundary found";
		else
			dijkstra_compute_distances(boundary_vertices, scalar_field);
	}

	/**
	 * @brief Return the vertex that is the furthest from all boundary.
	 * (i.e. the vertex that maximize the discance to boundary)
	 * and build a scalar field that represent the distance to that vertex.
	 */
	Vertex central_vertex(VertexAttribute<Scalar>& scalar_field)
	{
		distance_to_boundary(scalar_field);
		return find_maximum(scalar_field);
	}

	/**
	 * Build a scalar field that represent the distance of each vertex
	 * to a given set of features (selected vertices) of the Map.
	 * @param[in] features the vertices from which the shortest paths are computed
	 * @param[out] scalar_field : the computed distance field
	 */
	void distance_to_features(const std::vector<Vertex>& features,
							  VertexAttribute<Scalar>& scalar_field)
	{
		dijkstra_compute_distances(features, scalar_field);
	}

	/**
		 * Build a scalar field that represent the sum of the distance of each vertex
		 * to a given set of features (selected vertices) of the Map.
		 * @param[in] features the vertices from which the shortest paths are computed
		 * @param[out] scalar_field : the computed distance field
		 */
	void sum_of_distance_to_features(const std::vector<Vertex>& features,
									 VertexAttribute<Scalar>& scalar_field)
	{
		VertexAttribute<Scalar> distance_to_feature =
				map_.template add_attribute<Scalar, Vertex>("__distance_to_feature__");

		for (auto& s : scalar_field) s = Scalar(0);

		for (Vertex f : features)
		{
			dijkstra_compute_distances({f}, distance_to_feature);
			map_.foreach_cell([&](Vertex v)
			{
				scalar_field[v] += distance_to_feature[v];
			});
		}
		map_.remove_attribute(distance_to_feature);
	}

	/**
	 * Build a morse function from the shortest path distance to the boundary.
	 * @param[out] morse_function : the values of the morse function
	 */
	void morse_distance_to_boundary(VertexAttribute<Scalar>& morse_function)
	{
		VertexAttribute<Scalar> distance_to_boundary =
				map_.template add_attribute<Scalar, Vertex>("__distance_to_boundary__");

		std::vector<Vertex> boundary_vertices;

		map_.foreach_cell([&](Vertex v)
		{
			if (map_.is_incident_to_boundary(v))
				boundary_vertices.push_back(v);
		});

		if (boundary_vertices.size() == 0u)
		{
			cgogn_log_error("distance_to_boundary") << "No boundary found";
			return;
		}

		dijkstra_compute_distances(boundary_vertices, distance_to_boundary);

		dijkstra_to_morse_function(boundary_vertices, distance_to_boundary, morse_function);
		map_.remove_attribute(distance_to_boundary);
	}

	/**
	 * Build a morse function from the shortest path distances to the features.
	 * @param[in] features the vertices from which the shortest paths are computed
	 * @param[out] morse_function : the values of the morse function
	 */
	void morse_distance_to_features(const std::vector<Vertex>& features,
									VertexAttribute<Scalar>& morse_function)
	{
		VertexAttribute<Scalar> distance_to_features =
				map_.template add_attribute<Scalar, Vertex>("__distance_to_features__");

		// Compute the shortest paths to sources
		dijkstra_compute_distances(features, distance_to_features);

		// Search for the vertices that maximize the distance to the sources
		Vertex max_vertex = find_maximum(distance_to_features);
		Scalar max_distance = distance_to_features[max_vertex];

		// Inverse the distances so that the maxima are on the features
		for (auto& d : distance_to_features)
			d = max_distance - d;

		dijkstra_to_morse_function({max_vertex}, distance_to_features, morse_function);
		map_.remove_attribute(distance_to_features);
	}

private:

	MAP& map_;
	AdjacencyCache<MAP> cache_;
	EdgeAttribute<Scalar> edge_weight_;
	bool intern_edge_weight_;
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_TOPOLOGY_DISTANCE_FIELD_CPP_))
extern template class CGOGN_TOPLOGY_API DistanceField<float32, CMap2>;
extern template class CGOGN_TOPLOGY_API DistanceField<float64, CMap2>;
extern template class CGOGN_TOPLOGY_API DistanceField<float32, CMap3>;
extern template class CGOGN_TOPLOGY_API DistanceField<float64, CMap3>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_TOPOLOGY_DISTANCE_FIELD_CPP_))

} // namespace topology

} // namespace cgogn

#endif // CGOGN_TOPOLOGY_DISTANCE_FIELD_H_
