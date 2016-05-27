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

#ifndef CGOGN_TOPOLOGY_ALGOS_DIJKSTRA_H
#define CGOGN_TOPOLOGY_ALGOS_DIJKSTRA_H

//#include <map>
//#include <set>
//#include <vector>
//#include <algorithm>

namespace cgogn
{

namespace topology
{

template <typename Scalar, typename MAP>
class DistanceField
{
	using Vertex = typename MAP::Vertex;
	template<typename T>
	using VertexAttribute = typename MAP::template VertexAttribute<T>;

	using Edge = typename MAP::Edge;
	template<typename T>
	using EdgeAttribute = typename MAP::template EdgeAttribute<T>;

	using VertexArray = std::vector<Vertex>;

public:
	DistanceField(MAP& map, const EdgeAttribute<Scalar>& weight) :
		map_(map),
		intern_edge_weight_(false),
		edge_weight_(weight)
	{
	}

	DistanceField(MAP& map) :
		map_(map),
		intern_edge_weight_(true)
	{
		edge_weight_ = map.template add_attribute<Scalar, Edge::ORBIT>("__edge_weight__");

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

	/**
	 * Compute for each vertex of the map, the shortest path to the sources
	 * A path is a sequence of adjacent edges whose weights are summed along the paths
	 * The method returns the lengths of the shortest paths in a VertexAttribute.
	 * @param[in] sources the vertices from which the shortest paths are computed
	 * @param[out] distance_to_source : the sums of the edge weights in the shortest paths
	 */
	void dijkstra_compute_distances(
			const VertexArray& sources,
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

			map_.foreach_adjacent_vertex_through_edge(u, [&](Vertex v)
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

	/**
	 * Compute for each vertex of the map, the shortest path to the sources
	 * A path is a sequence of adjacent edges whose weights are summed along the paths
	 * The method returns the shortest paths and their lengths in two VertexAttributes.
	 * @param[in] sources the vertices from which the shortest paths are computed
	 * @param[out] path_to_source : a reference to the previous vertex in the shortest path
	 * @param[out] distance_to_source : the sums of the edge weights in the shortest paths
	 */
	void dijkstra_compute_paths(
			const VertexArray& sources,
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

			map_.foreach_adjacent_vertex_through_edge(u, [&](Vertex v)
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

	/**
	 * Compute a morse function from the shortest path distances to the sources.
	 * The morse function has disctinct values in each vertex.
	 * Its maxima are on the sources vertices and it has only one minima.
	 * @param[in] sources the vertices from which the shortest paths are computed
	 * @param[out] morse_function : the values of the morse function
	 */
	void dijkstra_to_morse_function(
			MAP& map,
			const VertexArray& sources,
			VertexAttribute<Scalar>& morse_function)
	{
		VertexAttribute<Scalar> distance_to_source = map.template add_attribute<Scalar, Vertex::ORBIT>("__dist_to_source__");

		// Compute the shortest paths to sources
		dijkstra_compute_distances(sources, distance_to_source);

		// Search for the vertices that maximize the disctance to the sources
		Scalar max_distance = Scalar(0);
		Vertex max_vertex = Vertex();

		map.foreach_cell([&] (Vertex v)
		{
			Scalar distance = distance_to_source[v];
			if(distance > max_distance)
			{
				max_vertex = v;
				max_distance = distance;
			}
		});

		// Inverse the distances so that the maxima are on the sources
		for (auto& d : distance_to_source)
			d = max_distance - d;

		for(auto& d : morse_function)
			d = Scalar(3);							// To mark unvisited vertices

		using my_pair = std::pair<Scalar, unsigned int>;
		using my_queue = std::priority_queue<my_pair, std::vector<my_pair>, std::greater<my_pair> >;

		my_queue vertex_queue;


		vertex_queue.push(std::make_pair(Scalar(0), max_vertex.dart.index));

		// Run dijkstra using distance_to_source in place of the estimated geodesic distances
		// and fill the morse function with i/n where i is index of distance_to_source
		// in an sorted array of the distances and n the number of vertices.
		Scalar n = map.template nb_cells<Vertex::ORBIT>();
		uint32 i = 0;
		while(!vertex_queue.empty())
		{
			Vertex u = Vertex(Dart(vertex_queue.top().second));

			vertex_queue.pop();

			morse_function[u] = Scalar(i)/n;		// Set the final value
			++i;

			map.foreach_adjacent_vertex_through_edge(u, [&](Vertex v)
			{
				if(morse_function[v] > Scalar(2)) {	// If not visited
					vertex_queue.push(std::make_pair(distance_to_source[v], v.dart.index));
					morse_function[v] = Scalar(1);	// Set as visited
				}
			});
		}
		map.remove_attribute(distance_to_source);
	}

public:
	void distance_to_boundary(
		MAP& map,
		VertexAttribute<Scalar>& scalar_field)
	{
		std::vector<Vertex> boundary_vertices;

		map.foreach_cell([&](Vertex v)
		{
			if (map.is_incident_to_boundary(v))
				boundary_vertices.push_back(v);
		});

		if (boundary_vertices.size() == 0u)
			cgogn_log_error("distance_to_boundary") << "No boundary found";
		else
			dijkstra_compute_distances(boundary_vertices, scalar_field);
	}

private:
	MAP& map_;
	EdgeAttribute<Scalar> edge_weight_;
	bool intern_edge_weight_;
};

} // namespace topology

} // namespace cgogn

#endif // CGOGN_TOPOLOGY_ALGOS_DIJKSTRA_H
