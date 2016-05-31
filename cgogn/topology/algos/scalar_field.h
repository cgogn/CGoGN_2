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
 *   a closed surface homeomorph to the 2-sphere in 3d
 * Here Link(v) is considered as the vertices of the standart Link(v)
 * - Link+(v) and Link-(v) are the subsets of Link(v) whose scalar field values
 *   are greater / lower then the value at v
 * The classification algorithm counts the number of connected components
 * in the Link+(v) and Link-(v).
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

	template<typename T>
	using VertexAttribute = typename MAP::template VertexAttribute<T>;
	using VertexMarkerStore = typename cgogn::CellMarkerStore<MAP, Vertex::ORBIT>;
	using FaceMarkerStore = typename cgogn::CellMarkerStore<MAP, Face::ORBIT>;

public:
	ScalarField(MAP& map, const VertexAttribute<Scalar>& scalar_field) :
		map_(map),
		scalar_field_(scalar_field)
	{
	}

	~ScalarField()
	{
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
	 * Classify the vertex of a 2-manifold by differential analysis of the scalar field.
	 * @return the vertex classification
	 * The algorithm traverse Link(v) and counts the number of times
	 * the scalar field values becomes greater or lower than the value in v.
	 */
	template <typename USEDMAP, typename std::enable_if<USEDMAP::DIMENSION == 2>::type* = nullptr>
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

		cgogn_log_error("critical_vertex_analysis<dimension 2>") << "UNKNOW Critical Type";
		return CriticalPoint(CriticalPoint::Type::UNKNOWN);
	}

	// The link vector contains selected vertices in Link+(v) or Link-(v)
	// For every dart d in this vector, phi2(d) belong to v
	int nb_marked_cc_in_link(std::vector<Dart>& link,	VertexMarkerStore& vertex_marker)
	{
		int nb_cc = 0;
		while (!link.empty())
		{
			// Search a marked vertex in the link (initially selected in the link)
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
	 * Classify the vertex of a 3-manifold by differential analysis of the scalar field.
	 * @return the vertex classification
	 * The algorithm traverse Link(v) and build Link+(v) and Link-(v) :
	 * they are stored in two vector of Dart and theirs vertices are marked.
	 * Then the marker are used to count their numbers of connected components.
	 */
	template <typename USEDMAP, typename std::enable_if<USEDMAP::DIMENSION == 3>::type* = nullptr>
	CriticalPoint critical_vertex_analysis(Vertex v)
	{
		VertexMarkerStore sup_vertex_marker(map_);
		VertexMarkerStore inf_vertex_marker(map_);

		std::vector<Dart> sup_link;
		std::vector<Dart> inf_link;

		// Mark and store the vertices that belong to the Link+(v) and Link-(v)
		Scalar center_value = scalar_field_[v];
		map_.foreach_adjacent_vertex_through_edge(v, [&](Vertex u)
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
			return CriticalPoint(CriticalPoint::Type::SADDLE, 10+nb_inf);

		// 2-Saddles
		if (nb_inf == 1 && nb_sup < 10)
			return CriticalPoint(CriticalPoint::Type::SADDLE, 10*nb_sup+1);

		//	if (nb_inf == 2 && nb_sup == 2)
		//		return CriticalPoint(CriticalPoint::Type::SADDLE, 22);

		cgogn_log_error("critical_vertex_analysis<dimension 3>") << "UNKNOW Critical Type" <<
																	nb_inf << ", " << nb_sup;
		return CriticalPoint(CriticalPoint::Type::UNKNOWN);
	}

public:

	/**
	 * @brief Find and analyze all critical points of the scalar field
	 */
	void differential_analysis()
	{
		maxima_.clear();
		minima_.clear();
		saddles_.clear();

		map_.foreach_cell([&](Vertex v)
		{
			CriticalPoint type = critical_vertex_analysis<MAP>(v);
			if (type.v_ == CriticalPoint::Type::MAXIMUM)
				maxima_.push_back(v);
			if (type.v_ == CriticalPoint::Type::MINIMUM)
				minima_.push_back(v);
			else if (type.v_ == CriticalPoint::Type::SADDLE)
				saddles_.push_back(v);
		});
	}

	void extract_level_sets(std::vector<Edge>& level_lines)
	{
		VertexMarkerStore vertex_marker(map_);
		FaceMarkerStore face_marker(map_);
		std::vector<Face> level_set_faces;

		VertexAttribute<uint32> vertex_type =
				map_.template add_attribute<uint32, Vertex::ORBIT>("__vertex_type__");

		using my_pair = std::pair<Scalar, unsigned int>;
		using my_queue = std::priority_queue<my_pair, std::vector<my_pair> >;

		my_queue level_sets_queue;
		my_queue vertex_queue;

		// Add all local maxima as level set sources and store the vertex classification
		map_.foreach_cell([&](Vertex v)
		{
			CriticalPoint type = critical_vertex_analysis<MAP>(v);
			vertex_type[v] = type.v_;
			if (type.v_ == CriticalPoint::Type::MAXIMUM)
			{
				level_sets_queue.push(std::make_pair(scalar_field_[v], v.dart.index));
			}
		});

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
					if (vertex_type[u] == CriticalPoint::Type::SADDLE) {
						saddle_value = scalar_field_[u];
					}
					else {
						vertex_marker.mark(u);
						// Extend the front of the level set
						map_.foreach_adjacent_vertex_through_edge(u, [&] (Vertex v)
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
		map_.remove_attribute(vertex_type);
	}

private:
	MAP& map_;
	VertexAttribute<Scalar> scalar_field_;
	std::vector<Vertex> maxima_;
	std::vector<Vertex> minima_;
	std::vector<Vertex> saddles_;

};

} // namespace topology

} // namespace cgogn

#endif // CGOGN_TOPOLOGY_SCALAR_FIELD_H_
