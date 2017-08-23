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

template <typename MAP,typename VEC>

class ParallelogramGrid : public Tiling<MAP>
{
    using Map2 = cgogn::CMap2;

    using CDart = typename MAP::CDart;
    using Vertex = typename MAP::Vertex;
    using Edge = typename MAP::Edge;
    using Face = typename MAP::Face;
    using Volume = typename MAP::Volume;

    template <typename T>
    using DartAttribute = Map2::CDartAttribute<T>;
    template <typename T>
    using VertexAttribute = Map2::VertexAttribute<T>;
    template <typename T>
    using FaceAttribute = Map2::FaceAttribute<T>;

    using MapBuilder = typename MAP::Builder;
    using ChunkArrayContainer = MapBaseData::ChunkArrayContainer<uint32>;

    enum Orient { S=0, E=1, N=2, W=3, INVALID_ORIENT} ;
    struct DartOrient
    {
        Orient value_;

        DartOrient() : value_(INVALID_ORIENT) {}

        friend std::ostream& operator<<(std::ostream& out, const DartOrient&) { return out; }
        friend std::istream& operator>>(std::istream& in, const DartOrient&) { return in; }
    };

protected:
    DartAttribute<DartOrient> dart_orientation_ ;
    VertexAttribute<VEC> vertex_position_ ;

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
     * @param fill_area rectangular area to fill withparallelograms
     *
     * @pre p must lie in the bounding box #fill_area
     * @pre Ti[0] > 0
     * @pre Tj[0] >=0
     * @pre Tj[1] > Ti[1]
     */
    ParallelogramGrid(MAP& map, const VEC& p, const VEC& Ti, const VEC& Tj, const geometry::AABB<VEC>& fill_area):
        Tiling<MAP>(map)
    {
        cgogn_assert(fill_area.contains(p)) ;

        vertex_position_ = this->map_.template add_attribute<VEC, Vertex>("position") ;
        dart_orientation_ = this->map_.template add_attribute<DartOrient, CDart>("dartOrient");

        MapBuilder mbuild(map);

        // Create face and update orbit embeddings
        std::array<Vertex,4> incidentVertices ;
        std::array<Face,4> adjacentFaces ;
        const Dart d0 = add_face_topo(mbuild, incidentVertices/*, dart_orientation*/) ;
        this->dart_ = d0;
        const Face f0 = Face(d0) ;

        // Embed vertices of face
        const geometry::AABB<VEC> f_bbox = embed_parallelogram(vertex_position_, f0, p, Ti, Tj) ;

        std::queue<Edge> edgeQueue ; // list of edges to treat
        queueFreeEdges(f0,edgeQueue, f_bbox, fill_area) ;

        while(!edgeQueue.empty())
        {
            // Pop front edge
            Edge e = edgeQueue.front() ;

            const Dart d = e.dart ;
            if (is_border(d)) // if it lacks a neighbor
            {
                // get Array of existing adjacent (neighboring) faces
                const Face provokingFace = Face(d) ;
                const Orient provokingFaceOrient = getOrientOfDart(d/*,provokingFace*/) ;
                adjacentFaces = getAdjacentFacesFromProvokingFace(/*dart_orientation,*/provokingFace,provokingFaceOrient/*dart_orientation[e.dart].value_*/) ;

                // Compute which incident vertices are already there
                incidentVertices = adjFaces2incVertices(adjacentFaces) ;

                // Add new face, re-using existing vertices if present
                const Dart newDart = add_face_topo(mbuild, incidentVertices/*, adjacentFaces, dart_orientation*/) ;
                const Face newFace = Face(newDart) ;

                // sew with neighbor faces
                sewFaceToAdjacentFaces(mbuild,newFace,adjacentFaces) ;

                // Embed vertices of face
                // Get embedding of previous face's reference vertex
                VEC p_ref = vertex_position_[getRefVertex(provokingFace)] ;

                // Define embedding of new face's reference vertex
                switch(provokingFaceOrient)
                {
                case(S):
                    p_ref -= Tj ;
                    break ;
                case(E):
                    p_ref += Ti ;
                    break ;
                case(N):
                    p_ref += Tj ;
                    break ;
                case(W):
                    p_ref -= Ti ;
                    break ;
                case(INVALID_ORIENT):
                    cgogn_log_error("INVALID_ORIENT in switch") ;
                    break ;
                }

                // Embed new face's geometry
                const geometry::AABB<VEC> f_bbox = embed_parallelogram(vertex_position_, newFace, p_ref, Ti, Tj) ;
                // Add edges that are free
                queueFreeEdges(newFace, edgeQueue, f_bbox, fill_area) ;
            }

            edgeQueue.pop() ;
        }

        // close boundary
        mbuild.close_map() ;
        cgogn_ensure(this->map_.check_embedding_integrity()) ;

//        uint nbV = this->map_.template nb_cells<Vertex::ORBIT>();
//        uint nbF = this->map_.template nb_cells<Face::ORBIT>();
//        std::cout << "Nb Vertices: " << nbV << std::endl ;
//        std::cout << "Nb Faces: " << nbF << std::endl ;
    }

//    Orient sew_FaceEdge(MapBuilder& mbuild, /*const DartAttribute<DartDir>& dart_orientation,*/ const Face& f0, const Edge& e0)
//    {
//        cgogn_ensure(this->map_.phi2(e0.dart) == e0.dart) ;

//        const DartDir& et = dart_orientation[e0] ;
//        Orient fSearchDartDir = INVALID_ORIENT ;
//        switch(et.value_)
//        {
//        case(S):
//            fSearchDartDir = N ;
//            break ;
//        case(E):
//            fSearchDartDir = W ;
//            break ;
//        case(N):
//            fSearchDartDir = S ;
//            break ;
//        case(W):
//            fSearchDartDir = E ;
//            break ;
//        case(INVALID_ORIENT):
//            cgogn_ensure(et.value_ != INVALID_ORIENT) ;
//            break ;
//        }

//        Edge to_sew ;
//        this->map_.foreach_incident_edge(f0,
//            [&] (Edge e) -> bool
//        {
//            to_sew = e ;
//            return (dart_orientation[e].value_ == fSearchDartDir) ;
//        }) ;

//        // sew to_sew and e0 by phi2
//        const Dart& d0 = to_sew.dart ;
//        const Dart& d1 = e0.dart ;
//        mbuild.phi2_sew(d0,d1) ;
//        // ensure embedding
//        mbuild.template set_embedding<Vertex>(d1, this->map_.embedding(Vertex(this->map_.phi1(d0))));

//        return et.value_ ;
//    }

// ------------------------ Mesh construction algorithm --------------------------------
    /**
     * @brief queueFreeEdges adds the edges of face #f that are border edges of the mesh to the queue #edgeQueue iff #f_bbox intersects with the area to fill (#fill_area).
     * @param f the face
     * @param edgeQueue the queue of edges to maintain
     * @param f_bbox bounding box of the current face
     * @param fill_area the area to fill
     * @return
     */
    uint queueFreeEdges(const Face& f, std::queue<Edge>& edgeQueue, const geometry::AABB<VEC>& f_bbox, const geometry::AABB<VEC>& fill_area)
    {
        uint counter = 0 ;
        // if current face touches the Rectangle
        if (fill_area.intersects(f_bbox))
        {
            // add all its edges to list of edges to treat
            this->map_.foreach_incident_edge(f,
                [&] (Edge e)
            {
                if (is_border(e.dart)) // if it hasn't been sewed
                {
                    edgeQueue.push(e);
                    ++counter ;
                }
            });
        }

        return counter ;
    }

// ------------------------ Tools: Position embedding --------------------------------
    /**
     * @brief embed_parallelogram embeds a quad into a parallelogram
     * @param position the Vertex position container
     * @param f the quad
     * @param p0 the reference vertex position
     * @param Ti the first vector
     * @param Tj the second vector
     * @pre f needs to be a quadrilateral and fully sewed
     * @pre position need be a valid vertex attribute.
     *
     * @return the axis-aligned bounding box of the embedded parallelogram
     */
    geometry::AABB<VEC> embed_parallelogram(
                             typename MAP::template VertexAttribute<VEC>& position,
                             //typename MAP::template DartAttribute<DartDir>& dart_orientation,
                            const Face& f, const VEC& p0, const VEC& Ti, const VEC& Tj)
    {
        const Dart d0 = f.dart ;
        const Dart d1 = this->map_.phi1(d0) ;
        const Dart d2 = this->map_.phi1(d1) ;
        const Dart d3 = this->map_.phi1(d2) ;
        cgogn_ensure(d0==this->map_.phi1(d3)) ;

        position[Vertex(d0)] = p0 ;
        position[Vertex(d1)] = p0 + Ti ;
        position[Vertex(d2)] = position[d1] + Tj ;
        position[Vertex(d3)] = p0 + Tj ;

        geometry::AABB<VEC> bbox(p0) ;
        bbox.add_point(position[d1]) ;
        bbox.add_point(position[d2]) ;
        bbox.add_point(position[d3]) ;

        return bbox ;
    }

// ------------------------ Tools: Constructing the mesh --------------------------------

    /**
     * @brief add_face_topo creates a new face and updates embeddings topological embedding pointers
     * @param mbuild
     * @param incidentVertices contains existing vertices to link to, if any (invalid ones otherwise)
     * @return
     */
    Dart add_face_topo(MapBuilder& mbuild, const std::array<Vertex,4>& incidentVertices/*, const std::array<Face,4>& adjacentFaces, DartAttribute<DartDir>& dart_orientation*/)
    {
        ChunkArrayContainer& dart_container = mbuild.template attribute_container<CDart::ORBIT>();
        ChunkArrayContainer& vertex_container = mbuild.template attribute_container<Vertex::ORBIT>();
        ChunkArrayContainer& face_container = mbuild.template attribute_container<Face::ORBIT>();

        // Create face
        const Dart d0 = mbuild.add_face_topo_fp(4u) ; // phi2 is fixed point
        const Dart d1 = this->map_.phi1(d0) ;
        const Dart d2 = this->map_.phi1(d1) ;
        const Dart d3 = this->map_.phi1(d2) ;

        // Create new orbit embeddings for all embedded orbits.
        // This FACE
        const Face f0 = Face(d0) ;
        if(this->map_.template is_embedded<Face>())
        {
            uint32 nf = face_container.insert_lines<1>();
            mbuild.template set_embedding<Face>(f0.dart,nf);
        }

        // All DART of this face
        if(this->map_.template is_embedded<CDart>())
        {
            this->map_.foreach_dart_of_orbit(f0, [&] (Dart d)
            {
                uint32 nd = dart_container.insert_lines<1>() ;
                mbuild.template set_embedding<CDart>(d,nd);
            });
        }

        // All VERTEX of this face (if not existing)
        if(this->map_.template is_embedded<Vertex>())
        {
            // if incident vertex exists, reuse, otherwise create new
            uint32 nv = incidentVertices[0].is_valid() ? this->map_.embedding(incidentVertices[0]) : vertex_container.insert_lines<1>() ;
            mbuild.template set_embedding<Vertex>(d0, nv);

            nv = incidentVertices[1].is_valid() ? this->map_.embedding(incidentVertices[1]) : vertex_container.insert_lines<1>() ;
            mbuild.template set_embedding<Vertex>(d1, nv);

            nv = incidentVertices[2].is_valid() ? this->map_.embedding(incidentVertices[2]) : vertex_container.insert_lines<1>() ;
            mbuild.template set_embedding<Vertex>(d2, nv);

            nv = incidentVertices[3].is_valid() ? this->map_.embedding(incidentVertices[3]) : vertex_container.insert_lines<1>() ;
            mbuild.template set_embedding<Vertex>(d3, nv);
        }

//        const bool isVertexEmbedded = this->map_.template is_embedded<Vertex>() ;
//        this->map_.foreach_incident_vertex(f0,
//                                    [&] (Vertex v)
//        {
//            if (isVertexEmbedded)
//            {
//                uint32 nv = vertex_container.insert_lines<1>(); // TODO, only if not yet existing
//                mbuild.template set_embedding<Vertex>(v.dart,nv);
//            }
//        }) ;

        // Label the darts
        dart_orientation_[d0].value_ = S ;
        dart_orientation_[d1].value_ = E ;
        dart_orientation_[d2].value_ = N ;
        dart_orientation_[d3].value_ = W ;

        return d0 ;
    }


    /**
     * @brief sewFaceToAdjacentFaces sews face #f0 to the existing non-sewed adjacent faces #adjacentFaces
     * @param mbuild
     * @param f0 the face to sew
     * @param adjacentFaces the adjacent faces, previously obtained by #getAdjacentFacesFromProvokingFace.
     */
    void sewFaceToAdjacentFaces(MapBuilder& mbuild,const Face& f0, const std::array<Face,4>& adjacentFaces)
    {
        Dart d = f0.dart ;
        for (uint o = S ; o <= W ; ++o)
        {
            const Face& fAdj = adjacentFaces[o] ;
            if (fAdj.is_valid()) // if there's a face to be sewed in that direction
            {
                // Take opposite orientation of that face
                const Orient oInv = Orient((o+2)%4) ;
                const Dart dSew = getDartByOrient(fAdj,oInv) ;//getEdgeOfFace(adjacentFaces[dir],/*dart_orientation,*/(dir+2)%4).dart ;
                mbuild.phi2_sew(d,dSew) ;

                // Set Edge embedding to that of the retrieved edge
                // mbuild.template set_embedding<Edge>(d, this->map_.embedding(eSew));
                // Set Vertex embedding to that of the retrieved Vertex
                //mbuild.template set_embedding<Vertex>(d, this->map_.embedding(Vertex(this->map_.phi1(dSew)))); // TODO CHECK IF ALREADY DONE. ITS RATHER AN OPTIMISATION AS DOING TWICE DOESNT HURT
            }
            d = this->map_.phi1(d) ; // go to next
        }
    }

    // ------------------------ Tools: Find unsewed neighboring topo items --------------------------------
private:
    /**
     * @brief getAdjacentFacesFromProvokingFace returns the existing unsewed south, east, north and west faces, respectively. If one of these (usually at least 2) is non-existing, the resturned faces are not valid.
     * @param map the map
     * @param dart_orientation the DartAttribute with edge type labels
     * @param provokingFace one of the adjacent faces, i.e., the one that initiated the creation of the current face
     * @param provokingDartDir the edge of #provokingFace to which the current face is neighboring
     * @return array of 4 Faces, respectively south, east, north and west of the new face, or invalid faces if they are inexistant.
     */
    std::array<Face,4> getAdjacentFacesFromProvokingFace(/*const DartAttribute<DartDir>& dart_orientation,*/ const Face& provokingFace, const Orient& provokingDartDir)
    {
        std::array<Face,4> neighbFaces ;

        switch(provokingDartDir)
        {
        case(S):
        {
            neighbFaces[2] = provokingFace ; // provokingFace is located north

            // Check to the West
            const Face fNW = getFaceInOrient(provokingFace,/*dart_orientation,*/W) ;
            if (fNW.is_valid())
            {
                const Face fW = getFaceInOrient(fNW,/*dart_orientation,*/S) ;
                if (fW.is_valid())
                {
                    neighbFaces[W] = fW ;
                }
            }

            // Check to the East
            const Face fNE = getFaceInOrient(provokingFace,/*dart_orientation,*/E) ;
            if (fNE.is_valid())
            {
                const Face fE = getFaceInOrient(fNE,/*dart_orientation,*/S) ;
                if (fE.is_valid())
                {
                    neighbFaces[E] = fE ;
                }
            }
        }
            break;
        case(E):
        {
            neighbFaces[3] = provokingFace ; // provokingFace is located west

            // Check to the North
            const Face fNW = getFaceInOrient(provokingFace,/*dart_orientation,*/N) ;
            if (fNW.is_valid())
            {
                const Face fN = getFaceInOrient(fNW,/*dart_orientation,*/E) ;
                if (fN.is_valid())
                {
                    neighbFaces[N] = fN ;
                }
            }

            // Check to the South
            const Face fSW = getFaceInOrient(provokingFace,/*dart_orientation,*/S) ;
            if (fSW.is_valid())
            {
                const Face fS = getFaceInOrient(fSW,/*dart_orientation,*/E) ;
                if (fS.is_valid())
                {
                    neighbFaces[S] = fS ;
                }
            }
        }
            break;
        case(N):
        {
            neighbFaces[0] = provokingFace ; // provokingFace is located south

            // Check to the West
            const Face fSW = getFaceInOrient(provokingFace,/*dart_orientation,*/W) ;
            if (fSW.is_valid())
            {
                const Face fW = getFaceInOrient(fSW,/*dart_orientation,*/N) ;
                if (fW.is_valid())
                {
                    neighbFaces[W] = fW ;
                }
            }

            // Check to the East
            const Face fSE = getFaceInOrient(provokingFace,/*dart_orientation,*/E) ;
            if (fSE.is_valid())
            {
                const Face fE = getFaceInOrient(fSE,/*dart_orientation,*/N) ;
                if (fE.is_valid())
                {
                    neighbFaces[E] = fE ;
                }
            }
        }
            break;
        case(W):
        {
            neighbFaces[1] = provokingFace ; // provokingFace is located east

            // Check to the North
            const Face fNE = getFaceInOrient(provokingFace,/*dart_orientation,*/N) ;
            if (fNE.is_valid())
            {
                const Face fN = getFaceInOrient(fNE,/*dart_orientation,*/W) ;
                if (fN.is_valid())
                {
                    neighbFaces[N] = fN ;
                }
            }

            // Check to the South
            const Face fSE = getFaceInOrient(provokingFace,/*dart_orientation,*/S) ;
            if (fSE.is_valid())
            {
                const Face fS = getFaceInOrient(fSE,/*dart_orientation,*/W) ;
                if (fS.is_valid())
                {
                    neighbFaces[S] = fS ;
                }
            }
        }
            break;
        case(INVALID_ORIENT):
            cgogn_log_error("INVALID_ORIENT in switch") ;
            break ;
        }

        return neighbFaces ;
    }

    /**
     * @brief adjFaces2incVertices returns an array of existing (or invalid) vertices for the new face, given its pre-existing adjacent faces
     * @param adjacentFaces the pre-existing adjacent faces
     * @return array of vertices pre-existing vertices to embed the current face with, or invalid vertices otherwise. In counter-clock numbering order, starting from the reference vertex.
     */
    std::array<Vertex,4> adjFaces2incVertices(const std::array<Face,4>& adjacentFaces) const
    {
        std::array<Vertex,4> incVertices ;

        if (adjacentFaces[0].is_valid()) // South
        {
            incVertices[0] = getVertexByNo(adjacentFaces[0],3) ; // Vx 3 of South face
            incVertices[1] = getVertexByNo(adjacentFaces[0],2) ; // Vx 2 of South face
        }
        if (adjacentFaces[1].is_valid()) // East
        {
            incVertices[1] = getVertexByNo(adjacentFaces[1],0) ; // Vx 0 of East face
            incVertices[2] = getVertexByNo(adjacentFaces[1],3) ; // Vx 3 of East face
        }
        if (adjacentFaces[2].is_valid()) // North
        {
            incVertices[2] = getVertexByNo(adjacentFaces[2],1) ; // Vx 1 of North face
            incVertices[3] = getVertexByNo(adjacentFaces[2],0) ; // Vx 0 of North face
        }
        if (adjacentFaces[3].is_valid()) // West
        {
            incVertices[3] = getVertexByNo(adjacentFaces[3],2) ; // Vx 2 of West face
            incVertices[0] = getVertexByNo(adjacentFaces[3],1) ; // Vx 1 of West face
        }

        return incVertices ;
    }

    // ------------------------ Tools: getR in a Face --------------------------------
private:
    /**
     * @brief getRefVertex returns the reference vertex of a given face
     * @param f the face
     * @return the reference vertex (i.e., vertex 0, by convention).
     */
    Vertex getRefVertex(const Face& f) const
    {
        return getVertexByNo(f,0) ;
    }

    //    Vertex getLeftBottomVertex(const VertexAttribute<VEC>& position, const Face& f)
    //    {
    //        Vertex lbv = Vertex(f.dart) ;
    //        this->map_.foreach_incident_vertex(f,
    //            [&] (Vertex v)
    //        {
    //            const VEC& p = position[v] ;
    //            if (p[1] < position[lbv][1])
    //            {
    //                lbv = v ;
    //            }
    //            else if (p[1] == position[lbv][1])
    //            {
    //                if (p[0] < position[lbv][0])
    //                {
    //                    lbv = v ;
    //                }
    //            }
    //        }) ;

    //        return lbv ;
    //    }

    /**
     * @brief getVertexByNo returns the nth vertex of a face, in counterclockwise numbering starting from the reference vertex
     * @param f the face
     * @param no the vertex to fetch
     * @return the requested vertex
     */
    Vertex getVertexByNo(const Face& f, uint no) const
    {
        assert(no<4) ;
        return Vertex(getDartByOrient(f,Orient(no))) ;
    }

    /**
     * @brief getDartByOrient returns the dart of a face oriented in a given target orientation
     * @param f the face
     * @param target_orient the orientation for which the dart needs to be returned
     * @return the dart corresponding to the requested orientation in face #f
     */
    const Dart getDartByOrient(const Face& f, const Orient& target_orient) const
    {
        Dart res ;
        this->map_.foreach_dart_of_orbit(f, [&] (Dart d) -> bool
        {
            if (dart_orientation_[d].value_ == target_orient)
            {
                res = d ;
                return false ;
            }
            return true ;
        });

        return res ;
//        this->map_.foreach_incident_edge(f,
//            [&] (Edge e)
//        Dart d = f.dart ;
//        for (uint i = 0 ; i < target_orient ; ++i)
//            d = this->map_.phi1(d) ;

//        return d ;
    }

    /**
     * @brief getOrientOfDart tests if a given dart is in face #f, and returns its orientation is yes, INVALID_ORIENT otherwise
     * @param d the dart
     * @param f the face
     * @return the orientation of the face, or INVALID_ORIENT if the dart is not in the face
     */
    Orient getOrientOfDart(const Dart& d/*, const Face& f*/) const
    {
        return dart_orientation_[d].value_ ;
        /*Orient res = INVALID_ORIENT ;

        Dart dd = f.dart ;
        for (uint i = S ; i < W ; ++i)
        {
            if (d == dd)
            {
                res = Orient(i) ;
                break ;
            }
            dd = this->map_.phi1(dd) ;
        }

        return res ;*/
    }
//    /**
//     * @brief getVertexOfFace returns the vertex number vertexNo of Face f.
//     * @param f
//     * @param vertexNo
//     * @return
//     */
//    const Vertex getVertexOfFace(const Face& f, uint vertexNo) const
//    {
//        Dart d = f.dart ;
//        for (uint i = 0 ; i < vertexNo ; ++i)
//            d = this->map_.phi1(d) ;

//        return Vertex(d) ;
//    }


    // ------------------------ Tools: getR for adjacent Faces --------------------------------
private:

    /**
     * @brief getFaceInOrient tests if a given face #f has a neighboring sewed face to its #DartDirection direction.
     * If yes, the Face returned is of the found face
     * If no, the Face returned is invalid.
     * @param f
     * @param dart_orientation
     * @param target_type
     * @return a valid Face to the face found if it exists, an invalid dart otherwise.
     */
    const Face getFaceInOrient(const Face& f/*, const DartAttribute<DartDir>& dart_orientation*/, const Orient& orientation) const
    {
        Face res ;

        const Dart& dTmp = getDartByOrient(f,orientation) ; /*getEdgeOfFace(f,dart_orientation,DartDirection).dart ;*/
        const Dart& dPhi2 = this->map_.phi2(dTmp) ;
        if (dTmp != dPhi2) // if there is a neighboring face there
        {
            res = Face(dPhi2) ;
        }

        return res ;
    }

    // ------------------------ Random Tools --------------------------------
    bool is_border(Dart d) const
    {
        return this->map_.phi2(d) == d ;
    }
};

} //namespace modeling

} //namespace cgogn

#endif // CGOGN_MODELING_TILING_PARALLELOGRAM_GRID_H
