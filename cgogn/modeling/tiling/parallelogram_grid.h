#ifndef CGOGN_MODELING_TILING_PARALLELOGRAM_GRID_H
#define CGOGN_MODELING_TILING_PARALLELOGRAM_GRID_H

#include <cgogn/geometry/algos/bounding_box.h>
#include <cgogn/geometry/algos/length.h>

#include <cgogn/core/cmap/cmap2_builder.h>
#include <cgogn/modeling/tiling/tiling.h>

namespace cgogn
{

namespace modeling
{

template <typename MAP>

class ParallelogramGrid : public Tiling<MAP>
{
    using Map2 = cgogn::CMap2;

    using CDart = typename MAP::CDart;
    using Vertex = typename MAP::Vertex;
    using Edge = typename MAP::Edge;
    using Face = typename MAP::Face;
    using Volume = typename MAP::Volume;

    using Vec3 = Eigen::Vector3d;
    using Vec2 = Eigen::Vector2d;

    template <typename T>
    using VertexAttribute = Map2::VertexAttribute<T>;

    using MapBuilder = typename MAP::Builder;

public:
    /**
     * @brief ParallelogramGrid creates a mesh composed of a tiling of identical parallelograms.
     * Parallelograms are defined by two vectors (Ti,Tj).
     * The position of the left-bottom corner of one parallelogram is defined as being located at p.
     * The tiling is embedded geometrically and fills at least a rectangular area defined by the point RectOrigin and RectDiag.
     * @param map the CGoGN map that defines the mesh.
     * @param p origin of the reference parallelogram
     * @param Ti first vector definining a parallelogram
     * @param Tj second vector definining a parallelogram
     * @param RectOrigin origin of the rectangle to fill
     * @param RectDiag diagonal (starting from RectOrigin) of the rectangle to fill
     *
     * @pre p must lie in the rectangle defined by #RectOrigin and #RectDiag
     * @pre Ti[0] > 0
     * @pre Tj[0] >=0
     * @pre Tj[1] > Ti[1]
     */
    ParallelogramGrid(MAP& map, Vec2 p, Vec2 Ti, Vec2 Tj, Vec2 RectOrigin, Vec2 RectDiag):
        Tiling<MAP>(map)
    {
        map.template add_attribute<Vec2, Vertex>("position") ;
        //map.template add_attribute<uint32, Map2::Edge>("edgeType");
        //map.template add_attribute<int32, Map2::Edge>("edges");
        //map.template add_attribute<int32, Map2::Face>("faces");
        //map.template add_attribute<int32, Map2::Volume>("volumes");

        MapBuilder mbuild(map);
        // TODO reserve vertex table and face table if numbers known.

        //creation of quads and storing vertices
        std::vector<Edge> edges ; // list of edges to treat
        // Map2::CellMarker<Edge::ORBIT> border_edges(map) ; // list of border edges

        // Create face
        const Dart d0 = mbuild.add_face_topo_fp(4) ; // phi2 is fixed point
        const Vertex v0 = Vertex(d0) ;
        const Face f0 = Face(d0) ;


        // Create new orbit embeddings for all embedded orbits.
        // All VERTEX of this face
        const bool isVertexEmbedded = map.template is_embedded<Vertex>() ;
        map.foreach_incident_vertex(f0,
            [&] (Vertex v)
        {
            this->vertex_table_.push_back(v) ;
            if (isVertexEmbedded)
                mbuild.new_orbit_embedding(v) ;
        }) ;
        this->dart_ = this->vertex_table_[0].dart;

        // This FACE
        this->face_table_.push_back(f0) ;
        if(this->map_.template is_embedded<Face>())
            mbuild.new_orbit_embedding(f0) ;

        // All EDGE of this Face
        if(this->map_.template is_embedded<Edge>())
            this->map_.foreach_incident_edge(Face(this->dart_),
            [&](Edge e)
            {
                mbuild.new_orbit_embedding(e);
            });

        // Embed vertices of face
        VertexAttribute<Vec2> position = map.template get_attribute<Vec2, Vertex>("position");
        assert(position.is_valid());
        embed_parallelogram(map, position, f0, p, Ti, Tj) ;


        //

        // Mark boundary faces
        // mbuild.boundary_mark(f) ;

        // close boundary


        map.check_embedding_integrity() ;
    }

    /**
     * @brief embed_parallelogram
     * @param map
     * @param position
     * @param f
     * @param p0
     * @param Ti
     * @param Tj
     * @pre f needs to be a quadrilateral.
     * @pre position need be a valid vertex attribute.
     */
    void embed_parallelogram(MAP& map, typename MAP::template VertexAttribute<Vec2>& position, Face f, const Vec2& p0, const Vec2& Ti, const Vec2& Tj)
    {
        const Dart d0 = f.dart ;
        const Dart d1 = map.phi1(d0) ;
        const Dart d2 = map.phi1(d1) ;
        const Dart d3 = map.phi1(d2) ;
        assert(d0==map.phi1(d3)) ;

        position[Vertex(d0)] = p0 ;
        position[Vertex(d1)] = p0 + Ti ;
        position[Vertex(d2)] = position[d1] + Tj ;
        position[Vertex(d3)] = p0 + Tj ;

        position[Vertex(d0)] = Vec2(1,1) ;
        position[Vertex(d1)] = Vec2(2,2) ;
        position[Vertex(d2)] = Vec2(3,3) ;
        position[Vertex(d3)] = Vec2(0,4) ;

        const Vec2 a = position[Vertex(d0)] ;
        Vec2 b = position[Vertex(d1)] ;
        Vec2 c = position[Vertex(d2)] ;
        Vec2 d = position[Vertex(d3)] ;

        std::cout << &(position[Vertex(d0)][0]) << std::endl ;
        std::cout << &(position[Vertex(d1)][0]) << std::endl ;
        std::cout << &(position[Vertex(d2)][0]) << std::endl ;
        std::cout << &(position[Vertex(d3)][0]) << std::endl ;
    }
};

} //namespace modeling

} //namespace cgogn

#endif // CGOGN_MODELING_TILING_PARALLELOGRAM_GRID_H
