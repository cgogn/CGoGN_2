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
 * class Features : support the extractions of topological features on manifolds.
 */
template <typename Scalar, typename MAP>
class Features
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

public:
	Features(MAP& map, const EdgeAttribute<Scalar>& weight) :
		map_(map),
		intern_edge_weight_(false),
		edge_weight_(weight),
		distance_field_(map, weight)
	{
		distance_to_A_ = map.template add_attribute<Scalar, Vertex::ORBIT>("__feature_A__");
		distance_to_B_ = map.template add_attribute<Scalar, Vertex::ORBIT>("__feature_B__");
		paths_ = map.template add_attribute<Vertex, Vertex::ORBIT>("__paths__");
	}

	Features(MAP& map) :
		map_(map),
		intern_edge_weight_(true),
		distance_field_(map)
	{
		edge_weight_ = distance_field_.edge_weight_;
		distance_to_A_ = map.template add_attribute<Scalar, Vertex::ORBIT>("__distance_to_A__");
		distance_to_B_ = map.template add_attribute<Scalar, Vertex::ORBIT>("__distance_to_B__");
		paths_ = map.template add_attribute<Vertex, Vertex::ORBIT>("__paths__");
	}

	~Features()
	{
		if (intern_edge_weight_)
			map_.remove_attribute(edge_weight_);
	}

	Scalar maximal_diameter(Vertex central_vertex, std::vector<Vertex>& features)
	{
		// Init A with a vertex near the center of the mesh
		Vertex A = central_vertex;

		// Init B with the furthest vertice from A
		distance_field_.dijkstra_compute_paths({A}, distance_to_A_, paths_);
		Vertex B = distance_field_.find_maximum(distance_to_A_);
		Scalar max_distance = distance_to_A_[B];

		// Try to optimize these two vertices by maximizing their distance
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

	void get_features(Vertex central_vertex, std::vector<Vertex>& features)
	{
		// Get two Vertices A and B on a maximal diameter
		// as side effect this build the two scalar field distance_to_A_ and distance_to_B_
		Scalar max_distance = maximal_diameter(central_vertex, features);

		// Get the maxima in the scalar field distance_to_A_
		ScalarField<Scalar, MAP> scalar_field_A(map_, distance_to_A_);
		scalar_field_A.differential_analysis();
		std::vector<Vertex> maxima_A = scalar_field_A.get_maxima();
		std::cout << "FA size: " << maxima_A.size() << std::endl;

		// Get the maxima in the scalar field distance_to_B_
		cgogn::topology::ScalarField<Scalar, MAP> scalar_field_B(map_, distance_to_B_);
		scalar_field_B.differential_analysis();
		std::vector<Vertex> maxima_B = scalar_field_B.get_maxima();
		std::cout << "FB size: " << maxima_B.size() << std::endl;

		Scalar filter_distance = max_distance / Scalar(4);
	}

private:
	MAP& map_;
	VertexAttribute<Scalar> distance_to_A_;
	VertexAttribute<Scalar> distance_to_B_;
	VertexAttribute<Vertex> paths_;
	EdgeAttribute<Scalar> edge_weight_;
	DistanceField<Scalar, MAP> distance_field_;
	bool intern_edge_weight_;

};

} // namespace topology

} // namespace cgogn

#endif // CGOGN_TOPOLOGY_FEATURES_H_
