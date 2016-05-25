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

/**
 * Compute for each vertex of the map, the shortest path to the sources
 * A path is a sequence of adjacent edges whose weights are summed along the paths
 * The algorithm algo returns the computed data in two VertexAttributes :
 * - path_to_source : a reference to the previous vertex in the shortest path
 * - distance_to_source : the sums of the edge weights in the shortest paths
 */
template <typename Scalar, typename MAP, typename MASK>
void dijkstra_compute_paths(
        MAP& map,
        const typename MAP::template EdgeAttribute<Scalar>& weight,
        const std::vector<typename MAP::Vertex> sources,
        typename MAP::template VertexAttribute<Scalar>& distance_to_source,
        typename MAP::template VertexAttribute<typename MAP::Vertex>& path_to_source)
{
    using Vertex = typename MAP::Vertex;
    using Edge = typename MAP::Edge;

    for (auto& d : distance_to_source)
        d = std::numeric_limits<Scalar>::max();

    for (auto& p : path_to_source)
        p = Vertex();

    // A pair contains a scalar (the actual distance) and a vertex
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

        map.foreach_adjacent_vertex_through_edge(u, [&](Vertex v)
        {
            Scalar distance_through_u = dist + weight[Edge(v.dart)];
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
 * Compute for each vertex of the map, the shortest path to the sources
 * A path is a sequence of adjacent edges whose weights are summed along the paths
 * The algorithm algo returns the computed data in two VertexAttributes :
 * - path_to_source : a reference to the previous vertex in the shortest path
 * - distance_to_source : the sums of the edge weights in the shortest paths
 * The length of the paths are normalized so that the longest path has a unit length.
 */
template <typename Scalar, typename MAP>
void dijkstra_compute_normalized_paths(
        MAP& map,
        const typename MAP::template EdgeAttribute<T>& weight,
        const std::vector<typename MAP::Vertex> sources,
        typename MAP::template VertexAttribute<Scalar>& distance_to_source,
        typename MAP::template VertexAttribute<typename MAP::Vertex>& path_to_source)
{
    // Compute the shortest paths to sources
    dijkstra_compute_paths<Scalar>(map, weight, sources, distance_to_source, path_to_source);

    // Compute the maximal and minimal value of the scalar field
    Scalar max_d = std::numeric_limits<Scalar>::min();
    Scalar min_d = std::numeric_limits<Scalar>::max();

    for(auto& d : distance_to_source)
    {
        max_d = std::max(max_d, d);
        min_d = std::min(min_d, d);
    }
    Scalar min_max = max_d - min_d;

    // Normalize the scalar field
    for(auto& d : distance_to_source)
        d = (d - min_d) / min_max;
}

template <typename Scalar, typename MAP>
void argmin(
        const MAP& map,
        const typename MAP::template VertexAttribute<Scalar>& scalar_field,
        std::vector<typename MAP::Vertex>& result)
{
    using Vertex = typename MAP::Vertex;

    Scalar min_distance = std::numeric_limits<Scalar>::infinity();
    Vertex min_vertex;
    map.foreach_cell([&] (Vertex v)
    {
        Scalar distance = scalar_field[v];
        if(distance < min_distance)
        {
            min_vertex = v;
            min_distance = distance;
        }
    });

    map.foreach_cell([&] (Vertex v)
    {
        Scalar distance = scalar_field[v];
        if(distance == min_distance)
        {
            result.push_back(v);
        }
    });

    return result;
}

template <typename Scalar, typename MAP>
void dijkstra_to_morse_function(
        MAP& map,
        typename MAP::template VertexAttribute<Scalar>& scalar_field,
        typename MAP::template VertexAttribute<Scalar>& morse_function)
{
    using Vertex = typename MAP::Vertex;
    using Edge = typename MAP::Edge;

    Scalar n = map.template nb_cells<Vertex::ORBIT>();
    for(auto& d : morse_function)
        d = Scalar(3);							// To mark unvisited vertices

    using my_pair = std::pair<Scalar, unsigned int>;
    using my_queue = std::priority_queue<my_pair, std::vector<my_pair>, std::greater<my_pair> >;

    my_queue vertex_queue;

    uint32 i = 0;

    std::vector<Vertex> init;
    argmin<Scalar>(map, scalar_field, init);
    for (Vertex u: init) {
        vertex_queue.push(std::make_pair(scalar_field[u], u.dart.index));
        morse_function[u] = Scalar(0);
    }

    while(!vertex_queue.empty())
    {
        Vertex u = Vertex(Dart(vertex_queue.top().second));

        vertex_queue.pop();

        morse_function[u] = i/n;				// Set the final value
        ++i;

        map.foreach_adjacent_vertex_through_edge(u, [&](Vertex v)
        {
            if(morse_function[v] > Scalar(2)) {	// If not visited
                vertex_queue.push(std::make_pair(scalar_field[v], v.dart.index));
                morse_function[v] = Scalar(1);	// Set as visited
            }
        });
    }
}

} // namespace topology

} // namespace cgogn

#endif // CGOGN_TOPOLOGY_ALGOS_DIJKSTRA_H
