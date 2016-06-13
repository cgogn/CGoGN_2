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

#ifndef CGOGN_TOPOLOGY_SCALAR_FIELD_H_
#define CGOGN_TOPOLOGY_SCALAR_FIELD_H_

#include <cgogn/topology/types/adjacency_cache.h>
#include <cgogn/topology/types/critical_point.h>

namespace cgogn
{

namespace topology
{
/**
 * class ScalarField : support topological analysis of scalar fields defined on a manifold.
 *
 * Some definitions to understand the vertex classification :
 * - Star(v) is the star of vertex v : it contains all simplices incident to v.
 *   It is homeomorph to an open disk in 2d and an open sphere in 3d.
 * - Link(v) is the boundary of Star(v)\{v} : a closed cyle of edges in 2d and
 *   a closed surface homeomorph to the 2-sphere in 3d (except for boundary vertices).
 * Here Link(v) is considered as the vertices of the standart Link(v)
 * - Link+(v) and Link-(v) are the subsets of Link(v) whose scalar field values
 *   are greater / lower then the value at v.
 * The classification counts the number of connected components in Link+(v) and Link-(v).
 * Two vertices are say connected when they are incident to a same edge.
 *
 * Equality :
 * - Some algorithms suppose that all critical point in a scalar field have distincts values
 * - If this is not the case, we place equal vertices in the Link+(v).
 * That assures that the minima are always soundly analyzed, but be aware that
 * some artefacts may arise (extra critical point will be found)
 * => Building a Morse function resolve this issue.
 */
template <typename Scalar, typename MAP>
class ScalarField
{
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Face = typename MAP::Face;
	using Volume = typename MAP::Volume;

	template<typename T>
	using VertexAttribute = typename MAP::template VertexAttribute<T>;
	using VertexMarkerStore = typename cgogn::CellMarkerStore<MAP, Vertex::ORBIT>;
	using FaceMarkerStore = typename cgogn::CellMarkerStore<MAP, Face::ORBIT>;
	using VolumeMarkerStore = typename cgogn::CellMarkerStore<MAP, Volume::ORBIT>;

public:
	ScalarField(MAP& map,
				const AdjacencyCache<MAP>& cache,
				const VertexAttribute<Scalar>& scalar_field) :
		map_(map),
		cache_(cache),
		scalar_field_(scalar_field),
		vertex_type_computed_(false)
	{
		// To allow multiple ScalarField on a same map
		std::string name = "vertex_type_for_" + scalar_field.name();
		vertex_type_ = map.template add_attribute<CriticalPoint::Type, Vertex::ORBIT>(name);
	}

	~ScalarField()
	{
		map_.remove_attribute(vertex_type_);
	}

	const std::vector<Vertex>& get_maxima()
	{
		return maxima_;
	}

	const std::vector<Vertex>& get_minima()
	{
		return minima_;
	}

	const std::vector<Vertex>& get_saddles()
	{
		return saddles_;
	}

private:

	/**
	 * @brief Classify the vertex of a 2-manifold by differential analysis of the scalar field.
	 * @param v the vertex to analyze
	 * @return the vertex classification
	 * The algorithm traverse Link(v) and counts the number of times
	 * the scalar field values becomes greater or lower than the value in v.
	 */
	template <typename CONCRETE_MAP, typename std::enable_if<CONCRETE_MAP::DIMENSION == 2>::type* = nullptr>
	CriticalPoint critical_vertex_analysis(Vertex v)
	{
		Dart next = v.dart;
		Dart prev;
		Scalar center = scalar_field_[v];
		Scalar previous;
		int up = 0;
		int down = 0;

		// Find a vertex whose scalar field value is distinct from the center
		do
		{
			previous = scalar_field_[Vertex(map_.phi_1(next))];
			prev = next;
			next = map_.phi1(map_.phi2(next));
		} while (next != v.dart && previous == center);

		// If the whole LINK has been traversed, then all value are equal to the center
		if (next == v.dart)
			return CriticalPoint(CriticalPoint::Type::REGULAR);

		// Count the variation of the scalar field values around the vertex
		// i.e. how many times the value becomes greater or lower than the center
		next = prev;
		do
		{
			Scalar current = scalar_field_[Vertex(map_.phi2(next))];
			if (current < center && previous > center)
				++down;
			else if (current > center && previous < center)
				++up;
			// Skip the vertex whose value is equal to the center
			// (that alter the detection of variations)
			if (current != center) previous = current;

			next = map_.phi1(map_.phi2(next));
		} while (next != prev);

		// All values are greater than the center
		if (up == 0 && down == 0 && previous > center)
			return CriticalPoint(CriticalPoint::Type::MINIMUM);

		// All values are lower than the center
		if (up == 0 && down == 0 && previous < center)
			return CriticalPoint(CriticalPoint::Type::MAXIMUM);

		// A unique varation in both direction
		if (up == 1 && down == 1)
			return CriticalPoint(CriticalPoint::Type::REGULAR);

		// More than one varation in both direction
		if (up == down)
			return CriticalPoint(CriticalPoint::Type::SADDLE, up);

		cgogn_log_info("critical_vertex_analysis<dimension 2>") << "UNKNOW Critical Type";
		return CriticalPoint(CriticalPoint::Type::UNKNOWN);
	}

	/**
	 * @brief Count the number of connected components in Link+(v) or Link-(v)
	 * @param link the set of vertices that belong to the link
	 * @param vertex_marker a marker that should be set for the initial set of vertices
	 * @return the number of connected components
	 * The algorithm traverses the connected components of Link(v) through adjacent edges.
	 * The traversal are restricted to marked vertices (i.e. to Link+ or Link-).
	 */
	int nb_marked_cc_in_link(std::vector<Dart>& link, VertexMarkerStore& vertex_marker)
	{
		int nb_cc = 0;
		while (!link.empty())
		{
			// Search a marked vertex in the link
			// Such vertex belong to a not yet traversed connected component
			Dart d;
			do {
				d = link.back();
				link.pop_back();
			} while (!vertex_marker.is_marked(Vertex(d)) && !link.empty());

			// If a marked vertex has been found, its connected component is counted and unmarked
			if (vertex_marker.is_marked(Vertex(d)))
			{
				++nb_cc;
				// Traverse the connected component of d (marked vertices connected to d)
				std::vector<Dart> cc;
				cc.push_back(d);
				while (!cc.empty())
				{
					Dart e = cc.back();
					vertex_marker.unmark(Vertex(e));
					cc.pop_back();

					map_.foreach_incident_face(Edge(e), [&](Face f)
					{
						// The vertex of dart adj is adjacent to the vertex of e through an edge and
						// the dart phi2(adj) belongs to the central vertex C
						Dart adj = map_.phi2(map_.phi_1(map_.phi_1(f.dart)));
						if (vertex_marker.is_marked(Vertex(adj)))
							cc.push_back(adj);
					});
				}
			}
		}
		return nb_cc;
	}

	/**
	 * @brief Classify the vertex of a 3-manifold by differential analysis of the scalar field.
	 * @param v the vertex to analyze
	 * @return the vertex classification
	 * The algorithm traverse Link(v) and build Link+(v) and Link-(v) :
	 * Both sets are stored in vectors of Dart and theirs vertices marked.
	 * Then the vectors and markers are used to count the numbers of connected components.
	 */
	template <typename CONCRETE_MAP, typename std::enable_if<CONCRETE_MAP::DIMENSION == 3>::type* = nullptr>
	CriticalPoint critical_vertex_analysis(Vertex v)
	{
		VertexMarkerStore sup_vertex_marker(map_);
		VertexMarkerStore inf_vertex_marker(map_);

		std::vector<Dart> sup_link;
		std::vector<Dart> inf_link;

		// Mark and store the vertices that belong to the Link+(v) and Link-(v)
		Scalar center_value = scalar_field_[v];
		cache_.foreach_adjacent_vertex_through_edge(v, [&](Vertex u)
		{
			Scalar value = scalar_field_[u];
			if (value < center_value)
			{
				inf_vertex_marker.mark(u);
				inf_link.push_back(u.dart);
			}
			else
			{
				sup_vertex_marker.mark(u);
				sup_link.push_back(u.dart);
			}
		});

		// Count the number of connected components in the inf and sup links
		int nb_inf = nb_marked_cc_in_link(inf_link, inf_vertex_marker);
		int nb_sup = nb_marked_cc_in_link(sup_link, sup_vertex_marker);

		// All vertices of the Link are in Link-(v)
		if (nb_inf == 1 && nb_sup == 0)
			return CriticalPoint(CriticalPoint::Type::MAXIMUM);

		// All vertices of the Link are in Link+(v)
		if (nb_inf == 0 && nb_sup == 1)
			return CriticalPoint(CriticalPoint::Type::MINIMUM);

		// Link-(v) and Link+(v) are connected
		if (nb_inf == 1 && nb_sup == 1)
			return CriticalPoint(CriticalPoint::Type::REGULAR);

		// 1-Saddles
		if (nb_sup == 1 && nb_inf < 10)
			return CriticalPoint(CriticalPoint::Type::SADDLE1, 10+nb_inf);

		// 2-Saddles
		if (nb_inf == 1 && nb_sup < 10)
			return CriticalPoint(CriticalPoint::Type::SADDLE2, 10*nb_sup+1);

		cgogn_log_info("critical_vertex_analysis<dimension 3>") << "UNKNOW Critical Type"
																 << nb_inf << ", " << nb_sup;
		return CriticalPoint(CriticalPoint::Type::UNKNOWN);
	}

public:

	/**
	 * @brief Find and analyze all critical points of the scalar field
	 */
	void critical_vertex_analysis()
	{
		maxima_.clear();
		minima_.clear();
		saddles_.clear();

		map_.foreach_cell([&](Vertex v)
		{
			CriticalPoint type = critical_vertex_analysis<MAP>(v);
			vertex_type_[v] = type.v_;
			if (type.v_ == CriticalPoint::Type::MAXIMUM)
				maxima_.push_back(v);
			if (type.v_ == CriticalPoint::Type::MINIMUM)
				minima_.push_back(v);
			else if (type.v_ == CriticalPoint::Type::SADDLE  ||
					 type.v_ == CriticalPoint::Type::SADDLE1 ||
					 type.v_ == CriticalPoint::Type::SADDLE2)
				saddles_.push_back(v);
		});
		vertex_type_computed_ = true;
	}

private:

	/**
	 * @brief extract_level_sets
	 * @param level_lines
	 */
	template <typename CONCRETE_MAP, typename std::enable_if<CONCRETE_MAP::DIMENSION == 2>::type* = nullptr>
	void extract_level_sets(std::vector<Edge>& level_lines)
	{
		cgogn_message_assert(vertex_type_computed_,"Call critical_vertex_analysis() before this function");
		VertexMarkerStore vertex_marker(map_);
		FaceMarkerStore face_marker(map_);
		std::vector<Face> level_set_faces;

		using my_pair = std::pair<Scalar, unsigned int>;
		using my_queue = std::priority_queue<my_pair, std::vector<my_pair> >;

		my_queue level_sets_queue;
		my_queue vertex_queue;

		// Add all local maxima as level set sources and store the vertex classification
		for (Vertex v : maxima_)
			level_sets_queue.push(std::make_pair(scalar_field_[v], v.dart.index));

		// Build the level set of the next local maxima
		while (!level_sets_queue.empty()) {
			// Initialize a flood front for this level set
			my_pair p = level_sets_queue.top();
			level_sets_queue.pop();
			vertex_queue.push(p);

			Scalar saddle_value = Scalar(0);

			// While the flood front is not empty
			while (!vertex_queue.empty())
			{
				Vertex u = Vertex(Dart(vertex_queue.top().second));
				vertex_queue.pop();

				// We are still in the current level set
				if (!vertex_marker.is_marked(u) && scalar_field_[u] > saddle_value) {
					// We reach a saddle (the nearest one)
					if (vertex_type_[u] == CriticalPoint::Type::SADDLE) {
						saddle_value = scalar_field_[u];
					}
					else {
						vertex_marker.mark(u);
						// Extend the front of the level set
						cache_.foreach_adjacent_vertex_through_edge(u, [&] (Vertex v)
						{
							if (!vertex_marker.is_marked(v) && scalar_field_[v] > saddle_value)
							{
								vertex_queue.push(std::make_pair(scalar_field_[v], v.dart.index));
							}
						});
						map_.foreach_incident_face(u, [&] (Face f)
						{
							if (!face_marker.is_marked(f))
							{
								level_set_faces.push_back(f);
								face_marker.mark(f);
							}
						});
					}
				}
			}
			// The marked vertices and faces define the interior and closure of the level set
			for (Face f : level_set_faces)
			{
				map_.foreach_incident_edge(f, [&](Edge e)
				{
					std::pair<Vertex, Vertex> p = map_.vertices(e);
					if (!vertex_marker.is_marked(p.first) && !vertex_marker.is_marked(p.second))
						level_lines.push_back(e);
				});
			}
			level_set_faces.clear();
			vertex_marker.unmark_all();
			face_marker.unmark_all();
		}
	}

	/**
	 * @brief extract_level_sets
	 * @param level_lines
	 */
	template <typename CONCRETE_MAP, typename std::enable_if<CONCRETE_MAP::DIMENSION == 3>::type* = nullptr>
	void extract_level_sets(std::vector<Edge>& level_lines)
	{
		cgogn_message_assert(vertex_type_computed_,"Call critical_vertex_analysis() before this function");
		VertexMarkerStore vertex_marker(map_);
		VolumeMarkerStore volume_marker(map_);
		std::vector<Volume> level_set_volumes;

		using my_pair = std::pair<Scalar, unsigned int>;
		using my_queue = std::priority_queue<my_pair, std::vector<my_pair> >;

		my_queue level_sets_queue;
		my_queue vertex_queue;

		// Add all local maxima as level set sources and store the vertex classification
		for (Vertex v : maxima_)
			level_sets_queue.push(std::make_pair(scalar_field_[v], v.dart.index));

		// Build the level set of the next local maxima
		while (!level_sets_queue.empty()) {
			// Initialize a flood front for this level set
			my_pair p = level_sets_queue.top();
			level_sets_queue.pop();
			vertex_queue.push(p);

			Scalar saddle_value = Scalar(0);

			// While the flood front is not empty
			while (!vertex_queue.empty())
			{
				Vertex u = Vertex(Dart(vertex_queue.top().second));
				vertex_queue.pop();

				// We are still in the current level set
				if (!vertex_marker.is_marked(u) && scalar_field_[u] > saddle_value) {
					// We reach a saddle (the nearest one)
					if (vertex_type_[u] == CriticalPoint::Type::SADDLE1 ||
						vertex_type_[u] == CriticalPoint::Type::SADDLE2) {
						saddle_value = scalar_field_[u];
					}
					else {
						vertex_marker.mark(u);
						// Extend the front of the level set
						cache_.foreach_adjacent_vertex_through_edge(u, [&] (Vertex v)
						{
							if (!vertex_marker.is_marked(v) && scalar_field_[v] > saddle_value)
							{
								vertex_queue.push(std::make_pair(scalar_field_[v], v.dart.index));
							}
						});
						map_.foreach_incident_volume(u, [&] (Volume vol)
						{
							if (!volume_marker.is_marked(vol))
							{
								level_set_volumes.push_back(vol);
								volume_marker.mark(vol);
							}
						});
					}
				}
			}
			// The marked vertices and faces define the interior and closure of the level set
			for (Volume vol : level_set_volumes)
			{
				map_.foreach_incident_face(vol, [&](Face f)
				{
					bool boundary_face = true;
					map_.foreach_incident_vertex(f, [&] (Vertex v)
					{
						if (vertex_marker.is_marked(v)) boundary_face = false;
					});
					if (boundary_face)
						map_.foreach_incident_edge(f, [&] (Edge e)
						{
							level_lines.push_back(e);
						});
				});
			}
			level_set_volumes.clear();
			vertex_marker.unmark_all();
			volume_marker.unmark_all();
		}
	}

public:
	void extract_level_sets(std::vector<Edge>& level_lines)
	{
		extract_level_sets<MAP>(level_lines);
	}

	void extract_ascending_manifold(std::vector<Edge>& edges_set)
	{
		cgogn_message_assert(vertex_type_computed_,"Call critical_vertex_analysis() before this function");

		// Search for every 1-saddles the starts of the ascending 1-manifolds
		// that link the saddles to the minima
		// These starts are the minima of the connected components of the inf_link of the saddles
		VertexMarkerStore vertex_marker(map_);
		std::vector<Dart> inf_link;
		std::vector<Dart> saddles_to_minima;
		for (Vertex v : saddles_)
		{
			if ( vertex_type_[v] == CriticalPoint::Type::SADDLE ||
				 vertex_type_[v] == CriticalPoint::Type::SADDLE1 )
			{
				// Build the inf_link of v
				Scalar center_value = scalar_field_[v];
				cache_.foreach_adjacent_vertex_through_edge(v, [&](Vertex u)
				{
					Scalar value = scalar_field_[u];
					if (value < center_value)
					{
						vertex_marker.mark(u);
						inf_link.push_back(u.dart);
					}
				});
				// Foreach connected component of the link, search its local minima
				while (!inf_link.empty())
				{
					// Search a marked vertex in the link (initially selected in the link)
					Dart d;
					do {
						d = inf_link.back();
						inf_link.pop_back();
					} while (!vertex_marker.is_marked(Vertex(d)) && !inf_link.empty());

					// If a marked vertex has been found, its connected component is searched
					// for a minima and unmarked
					if (vertex_marker.is_marked(Vertex(d)))
					{
						std::vector<Dart> cc;
						Vertex min_cc = Vertex(d);
						Scalar min_value = scalar_field_[min_cc];
						cc.push_back(d);
						while (!cc.empty())
						{
							Vertex current(cc.back());
							cc.pop_back();
							vertex_marker.unmark(current);
							Scalar current_value = scalar_field_[current];
							if (current_value < min_value)
							{
								min_value = current_value;
								min_cc = current;
							}

							map_.foreach_incident_face(Edge(current.dart), [&](Face f)
							{
								// The vertex of dart adj is adjacent to the vertex of e through an edge and
								// the dart phi2(adj) belongs to the central vertex C
								Dart adj = map_.phi2(map_.phi_1(map_.phi_1(f.dart)));
								if (vertex_marker.is_marked(Vertex(adj)))
									cc.push_back(adj);
							});
						}
						saddles_to_minima.push_back(min_cc.dart);
					}
				}

			}
		};

		// For each found start descend its ascending 1-manifold
		while (!saddles_to_minima.empty())
		{
			Edge e(saddles_to_minima.back());
			saddles_to_minima.pop_back();
			edges_set.push_back(e);

			// Search for the next vertex in the descending path to the minimum
			Vertex min_vertex = Vertex(e.dart);
			Scalar min_value = scalar_field_[min_vertex];
			cache_.foreach_adjacent_vertex_through_edge(min_vertex, [&](Vertex u)
			{
				Scalar current_value = scalar_field_[u];
				if (current_value < min_value)
				{
					min_value = current_value;
					min_vertex = u;
				}
			});
			if (min_vertex.dart != e.dart) saddles_to_minima.push_back(min_vertex.dart);
		}
	}

	void extract_descending_manifold(std::vector<Edge>& edges_set)
	{
		cgogn_message_assert(vertex_type_computed_,"Call critical_vertex_analysis() before this function");

		// Search for every 1-saddles the starts of the descending 1-manifolds
		// that link the saddles to the maxima
		// These starts are the maxima of the connected components of the sup_link of the saddles
		VertexMarkerStore vertex_marker(map_);
		std::vector<Dart> sup_link;
		std::vector<Dart> saddles_to_maxima;
		for (Vertex v : saddles_)
		{
			if ( vertex_type_[v] == CriticalPoint::Type::SADDLE ||
				 vertex_type_[v] == CriticalPoint::Type::SADDLE2 )
			{
				// Build the inf_link of v
				Scalar center_value = scalar_field_[v];
				cache_.foreach_adjacent_vertex_through_edge(v, [&](Vertex u)
				{
					Scalar value = scalar_field_[u];
					if (value > center_value)
					{
						vertex_marker.mark(u);
						sup_link.push_back(u.dart);
					}
				});
				// Foreach connected component of the link, search its local minima
				while (!sup_link.empty())
				{
					// Search a marked vertex in the link (initially selected in the link)
					Dart d;
					do {
						d = sup_link.back();
						sup_link.pop_back();
					} while (!vertex_marker.is_marked(Vertex(d)) && !sup_link.empty());

					// If a marked vertex has been found, its connected component is searched
					// for a minima and unmarked
					if (vertex_marker.is_marked(Vertex(d)))
					{
						std::vector<Dart> cc;
						Vertex max_cc = Vertex(d);
						Scalar max_value = scalar_field_[max_cc];
						cc.push_back(d);
						while (!cc.empty())
						{
							Vertex current(cc.back());
							cc.pop_back();
							vertex_marker.unmark(current);
							Scalar current_value = scalar_field_[current];
							if (current_value > max_value)
							{
								max_value = current_value;
								max_cc = current;
							}

							map_.foreach_incident_face(Edge(current.dart), [&](Face f)
							{
								// The vertex of dart adj is adjacent to the vertex of e through an edge and
								// the dart phi2(adj) belongs to the central vertex C
								Dart adj = map_.phi2(map_.phi_1(map_.phi_1(f.dart)));
								if (vertex_marker.is_marked(Vertex(adj)))
									cc.push_back(adj);
							});
						}
						saddles_to_maxima.push_back(max_cc.dart);
					}
				}

			}
		};

		// For each found start descend its ascending 1-manifold
		while (!saddles_to_maxima.empty())
		{
			Edge e(saddles_to_maxima.back());
			saddles_to_maxima.pop_back();
			edges_set.push_back(e);

			// Search for the next vertex in the descending path to the minimum
			Vertex max_vertex = Vertex(e.dart);
			Scalar max_value = scalar_field_[max_vertex];
			cache_.foreach_adjacent_vertex_through_edge(max_vertex, [&](Vertex u)
			{
				Scalar current_value = scalar_field_[u];
				if (current_value > max_value)
				{
					max_value = current_value;
					max_vertex = u;
				}
			});
			if (max_vertex.dart != e.dart) saddles_to_maxima.push_back(max_vertex.dart);
		}
	}

private:
	MAP& map_;
	AdjacencyCache<MAP> cache_;
	VertexAttribute<Scalar> scalar_field_;
	VertexAttribute<CriticalPoint::Type> vertex_type_;
	bool vertex_type_computed_;

	std::vector<Vertex> maxima_;
	std::vector<Vertex> minima_;
	std::vector<Vertex> saddles_;

};

} // namespace topology

} // namespace cgogn

#endif // CGOGN_TOPOLOGY_SCALAR_FIELD_H_
