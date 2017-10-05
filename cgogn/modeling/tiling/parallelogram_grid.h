#ifndef CGOGN_MODELING_TILING_PARALLELOGRAM_GRID_H
#define CGOGN_MODELING_TILING_PARALLELOGRAM_GRID_H

#include <cgogn/modeling/dll.h>
#include <cgogn/core/utils/numerics.h>
#include <cgogn/core/cmap/cmap2.h>
#include <cgogn/core/cmap/cmap2_builder.h>

#include <cgogn/geometry/algos/bounding_box.h>

namespace cgogn
{

namespace modeling
{

/**
 * @brief The Parallelogram class.
 * Defines a parallelogram by a point and two vectors.
 */
template <typename T, uint VECSIZE>
class Parallelogram
{
private:
	using VEC = Eigen::Matrix<T,VECSIZE,1> ;
	VEC p_ ;
	VEC Ti_ ;
	VEC Tj_ ;
public:
	/**
	 * @brief Parallelogram Default constructor: p(0,0), Ti(1,0) and Tj(0,1).
	 */
	Parallelogram()
	{
		p_ = VEC::Zero() ;
		Ti_ = VEC::Zero() ;
		Ti_[0] = T(1) ;
		Tj_ = VEC::Zero() ;
		Tj_[1] = T(1) ;
	}

	/**
	 * @brief Parallelogram constructor by exhaustively giving all points
	 * @param p0
	 * @param p1
	 * @param p2
	 * @param p3
	 */
	Parallelogram(const VEC& p0, const VEC& p1, const VEC& p2, const VEC& p3)
	{
		init(p0,p1-p0,p3-p0) ;
	}

	/**
	 * @brief Parallelogram constructor using a point and two vectors
	 * @param p the anchor point
	 * @param Ti vector 1
	 * @param Tj vector 2
	 */
	Parallelogram(const VEC& p, const VEC& Ti, const VEC& Tj)
	{
		init(p,Ti,Tj) ;
	}

	/**
	 * @brief Parallelogram constructor by giving the anchor point and the magnitudes and angles of the two vectors
	 * @param p anchor point
	 * @param mag_i magnitude of vector 1
	 * @param alpha_i angle of vector 1 (wrt vertical axis)
	 * @param mag_j magnitude of vector 2
	 * @param alpha_j angle of vector 2 (wrt vertical axis)
	 * @pre 0 < alpha_i < M_PI
	 * @pre alpha_i < alpha_j <= M_PI
	 */
	Parallelogram(const VEC& p, const T& mag_i, const T& alpha_i, const T& mag_j, const T& alpha_j )
	{
		VEC Ti = VEC::Zero() ,Tj = VEC::Zero() ;
		Ti[0] = mag_i*sin(alpha_i) ;
		Ti[1] = -mag_i*cos(alpha_i) ;
		Tj[0] = mag_j*sin(alpha_j) ;
		Tj[1] = -mag_j*cos(alpha_j) ;
		init(p,Ti,Tj) ;
	}

	/**
	 * @brief getRefPos returns the anchor point
	 * @return the anchor point
	 */
	const VEC& getRefPos() const
	{
		return p_ ;
	}

	/**
	 * @brief getTi returns the first vector
	 * @return the first vector in carthesian coordinates
	 */
	const VEC& getTi() const
	{
		return Ti_ ;
	}

	/**
	 * @brief getTj returns the second vector
	 * @return returns the second vector in carthesian coordinates
	 */
	const VEC& getTj() const
	{
		return Tj_ ;
	}

	/**
	 * @brief getMagI returns the magnitude of the first vector
	 * @return magnitude of the first vector
	 */
	T getMagI() const
	{
		return Ti_.norm() ;
	}

	/**
	 * @brief getMagJ returns the magnitude of the second vector
	 * @return magnitude of the second vector
	 */
	T getMagJ() const
	{
		return Tj_.norm() ;
	}

	/**
	 * @brief getAlpha returns the angle between the negative y-axis and the vector, in counterclockwise direction
	 * @param v the vector
	 * @return an angle in [[0 M_PI]]
	 */
	static T getAlpha(const VEC& v)
	{
		return acos(-v[1] / v.norm()) ;
	}

	/**
	 * @brief getAlphaI returns the angle of the first vector Ti using #getAlpha
	 * @return an angle in ]]0 M_PI[[
	 */
	T getAlphaI() const
	{
		return getAlpha(Ti_) ;
	}

	/**
	 * @brief getAlphaJ returns the angle of the second vector Tj using #getAlpha
	 * @return an angle in ]]0 M_PI]]
	 */
	T getAlphaJ() const
	{
		return getAlpha(Tj_) ;
	}

	/**
	 * @brief nb_scalars returns the amount of scalars needed for the internal representation of a parallelogram
	 * @return the amount of scalars needed for the internal representation of a parallelogram
	 */
	constexpr static uint nb_scalars()
	{
		return 3 * VEC::SizeAtCompileTime ;
	}

	static uint sample_parallelograms_uniform(std::queue<Parallelogram>& samples, uint angular_bins, uint magnitude_bins, uint pos_bins, uint diagonal)
	{
		const uint half_diagonal = diagonal / 2 ;

		const T angular_step = M_PI * (1.0 / angular_bins) ;
		magnitude_bins = std::min(magnitude_bins,half_diagonal) ;
		const uint mag_step = floor(half_diagonal / magnitude_bins) ;

		for (uint mag_i = mag_step ; mag_i <= half_diagonal ; mag_i += mag_step)
		{
			for (uint mag_j = mag_step ; mag_j <= half_diagonal ; mag_j += mag_step)
			{
				T start_alpha = M_PI/2.0 - floor(M_PI/2.0 / angular_step)*angular_step ; // M_PI/2 - n times angular_step so that smallest possible > 0
				for (T alpha_i = start_alpha ; alpha_i <= M_PI ; alpha_i += angular_step)
				{
					// if x coordinate of Ti will be 0
					if (mag_i * sin(alpha_i) < 1.0)
						continue ;

					for (T alpha_j = M_PI ; alpha_j > alpha_i ; alpha_j -= angular_step)
					{
						if (!almost_equal_absolute(alpha_i,alpha_j,std::min(angular_step,M_PI/16.0))) // If alpha_i and alpha_j are too close
						{
							const T move_i = T(mag_i-1)/pos_bins ;
							const T move_j = T(mag_j-1)/pos_bins ;
							if (move_i < 2 || move_j < 2 )
								continue ;

							VEC p0 = VEC::Zero();
							for (uint pi = 0 ; pi < pos_bins ; ++pi)
							{
								p0[1] = 0.0 ;
								for (uint pj = 0 ; pj < pos_bins ; ++pj)
								{
									Parallelogram<T,VECSIZE> p(p0,mag_i,alpha_i,mag_j,alpha_j) ;
									samples.push(p) ;
									p0[1] += move_j ;
								}
								p0[0] += move_i ;
							}
						}
					}
				}
			}
		}

		return samples.size() ;
	}

	inline friend std::ostream& operator<<(std::ostream& o, const Parallelogram<T,VECSIZE>& p)
	{
		o << "p0(" ;
		for (std::size_t i = 0ul ; i < VEC::SizeAtCompileTime - 1ul ; ++i )
			o << p.getRefPos()[i] << ",";
		o << p.p_[VEC::SizeAtCompileTime -1ul] << ")," ;

		o << "Ti(mag=" << p.getTi().norm() ;
		o << ",alpha=" << p.getAlphaI() << ")," ;
		o << "Tj(mag=" << p.getTj().norm() ;
		o << ",alpha=" << p.getAlphaJ() << ")";

		return o;
	}

	bool is_valid() const
	{
		bool res = true ;
		res &= Ti_[0] > T(0) && Tj_[0] >= T(0) ; // positive X coordinates
		res &= (res == (getAlphaJ() > getAlphaI())) ; // 2nd vector above 1st.
		//res &= !almost_equal_absolute(Ti_.norm(),T(0)) && !almost_equal_absolute(Tj_.norm(),T(0)) ; // non-zero vectors

		return res ;
	}

	void make_valid()
	{
//		// alpha2 > alpha1
//		const T& a1 = getAlphaI() ;
//		const T& a2 = getAlphaJ() ;
//		if (a2 )

//		// Ti positive x coords
//		Ti_[0] = clamp(Ti_[0],1) ;
//		Tj_[0] = clamp(Tj_[0],0) ;
		assert(false) ; // TODO
	}

private:
	void init(const VEC& p, const VEC& Ti, const VEC& Tj)
	{
		p_ = p ;
		Ti_ = Ti ;
		Tj_ = Tj ;
		if (!is_valid())
			cgogn_log_warning("Parallelogram::init") << "Creating an invalid Parallelogram.";
	}

};

/**
 * @brief The ParallelogramGrid class is a utility class to fill a rectangle with a tiling of parallelograms of a given size.
 */
template <typename MAP,typename VEC>
class ParallelogramGrid
{
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(ParallelogramGrid);
protected:
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
	MAP& map_ ;
	Dart dart_ ;
	const geometry::AABB<VEC>& area_ ;

	Parallelogram<typename VEC::Scalar,VEC::SizeAtCompileTime> p_ ;

	DartAttribute<DartOrient> dart_orientation_ ;
	VertexAttribute<VEC> vertex_position_ ;

public:
	/**
	 * @brief ParallelogramGrid is a utility class for creating a tiling of parallelograms.
	 * @param map the CGoGN map that defines the mesh
	 * @param fill_area the area to tile
	 */
	ParallelogramGrid(MAP& map, const geometry::AABB<VEC>& fill_area):
		map_(map),
		area_(fill_area)
	{}

	/**
	 * @brief embed_with_parallelograms creates a mesh composed of a tiling of identical parallelograms.
	 * Parallelograms are defined by two vectors (Ti,Tj).
	 * The position of the left-bottom corner of one parallelogram is defined as being located at p.
	 * The tiling is embedded geometrically and fills at least a rectangular area defined by the point RectOrigin and RectDiag.
	 * @param p origin of the reference parallelogram
	 * @param Ti first vector definining a parallelogram
	 * @param Tj second vector definining a parallelogram
	 *
	 * @pre p must lie in the area to fill (defined in the constructor #ParallelogramGrid)
	 * @pre Ti[0] > 0
	 * @pre Tj[0] >=0
	 * @pre Tj[1] > Ti[1]
	 *
	 * @post the map is embedded with a VertexAttribute called "position".
	 */
	Dart embed_with_parallelograms(const Parallelogram<typename VEC::Scalar, VEC::SizeAtCompileTime>& p)
	{
		cgogn_assert(area_.contains(p.getRefPos())) ;
		p_ = p ;

		vertex_position_ = map_.template get_attribute<VEC, Vertex>("position") ;
		if (!vertex_position_.is_valid())
			vertex_position_ = map_.template add_attribute<VEC, Vertex>("position") ;

		dart_orientation_ = map_.template get_attribute<DartOrient, CDart>("dartOrient");
		if (!dart_orientation_.is_valid())
			dart_orientation_ = map_.template add_attribute<DartOrient, CDart>("dartOrient");

		MapBuilder mbuild(map_);

		// Create face and update orbit embeddings
		std::array<Vertex,4> incidentVertices ;
		std::array<Face,4> adjacentFaces ;
		const Dart d0 = add_face_topo(mbuild, incidentVertices/*, dart_orientation*/) ;
		dart_ = d0;
		const Face f0 = Face(d0) ;

		// Embed vertices of face
		const geometry::AABB<VEC> f_bbox = embed_parallelogram(vertex_position_, f0, p_.getRefPos()) ;

		std::queue<Edge> edgeQueue ; // list of edges to treat
		queueFreeEdges(f0,edgeQueue,f_bbox) ;

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
				adjacentFaces = getAdjacentFacesFromProvokingFace(provokingFace,provokingFaceOrient/*dart_orientation[e.dart].value_*/) ;

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
						p_ref -= p_.getTj() ;
						break ;
					case(E):
						p_ref += p_.getTi() ;
						break ;
					case(N):
						p_ref += p_.getTj() ;
						break ;
					case(W):
						p_ref -= p_.getTi() ;
						break ;
					case(INVALID_ORIENT):
						cgogn_log_error("INVALID_ORIENT in switch") ;
						break ;
				}

				// Embed new face's geometry
				const geometry::AABB<VEC> f_bbox = embed_parallelogram(vertex_position_, newFace, p_ref) ;
				// Add edges that are free
				queueFreeEdges(newFace, edgeQueue, f_bbox) ;
			}

			edgeQueue.pop() ;
		}

		// close boundary
		mbuild.close_map() ;
		cgogn_ensure(map_.check_embedding_integrity()) ;

		//        uint nbV = map_.template nb_cells<Vertex::ORBIT>();
		//        uint nbF = map_.template nb_cells<Face::ORBIT>();
		//        std::cout << "Nb Vertices: " << nbV << std::endl ;
		//        std::cout << "Nb Faces: " << nbF << std::endl ;

		return dart_;
	}

	//    Orient sew_FaceEdge(MapBuilder& mbuild, /*const DartAttribute<DartDir>& dart_orientation,*/ const Face& f0, const Edge& e0)
	//    {
	//        cgogn_ensure(map_.phi2(e0.dart) == e0.dart) ;

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
	//        map_.foreach_incident_edge(f0,
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
	//        mbuild.template set_embedding<Vertex>(d1, map_.embedding(Vertex(map_.phi1(d0))));

	//        return et.value_ ;
	//    }

private:
	// ------------------------ Mesh construction algorithm --------------------------------
	/**
	 * @brief queueFreeEdges adds the edges of face #f that are border edges of the mesh to the queue #edgeQueue iff #f_bbox intersects with the area to fill (defined in the constructor).
	 * @param f the face
	 * @param edgeQueue the queue of edges to maintain
	 * @param f_bbox bounding box of the current face
	 * @return
	 */
	uint queueFreeEdges(const Face& f, std::queue<Edge>& edgeQueue, const geometry::AABB<VEC>& f_bbox)
	{
		uint counter = 0 ;
		// if current face touches the Rectangle
		if (area_.intersects(f_bbox))
		{
			// add all its edges to list of edges to treat
			map_.foreach_incident_edge(f,
									   [&] (Edge e)
			{
				if (is_border(e.dart)) // if it hasn't been sewed yet
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
	 * @pre f needs to be a quadrilateral and fully sewed
	 * @pre position need be a valid vertex attribute.
	 *
	 * @return the axis-aligned bounding box of the embedded parallelogram
	 */
	geometry::AABB<VEC> embed_parallelogram(
			typename MAP::template VertexAttribute<VEC>& position,
			//typename MAP::template DartAttribute<DartDir>& dart_orientation,
			const Face& f, const VEC& p0)
	{
		const Dart d0 = f.dart ;
		const Dart d1 = map_.phi1(d0) ;
		const Dart d2 = map_.phi1(d1) ;
		const Dart d3 = map_.phi1(d2) ;
		cgogn_ensure(d0==map_.phi1(d3)) ;

		position[Vertex(d0)] = p0 ;
		position[Vertex(d1)] = p0 + p_.getTi() ;
		position[Vertex(d2)] = position[d1] + p_.getTj() ;
		position[Vertex(d3)] = p0 + p_.getTj() ;



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
	Dart add_face_topo(MapBuilder& mbuild, const std::array<Vertex,4>& incidentVertices)
	{
		ChunkArrayContainer& dart_container = mbuild.template attribute_container<CDart::ORBIT>();
		ChunkArrayContainer& vertex_container = mbuild.template attribute_container<Vertex::ORBIT>();
		ChunkArrayContainer& face_container = mbuild.template attribute_container<Face::ORBIT>();

		// Create face
		const Dart d0 = mbuild.add_face_topo_fp(4u) ; // phi2 is fixed point
		const Dart d1 = map_.phi1(d0) ;
		const Dart d2 = map_.phi1(d1) ;
		const Dart d3 = map_.phi1(d2) ;

		// Create new orbit embeddings for all embedded orbits.
		// This FACE
		const Face f0 = Face(d0) ;
		if(map_.template is_embedded<Face>())
		{
			uint32 nf = face_container.insert_lines<1>();
			mbuild.template set_embedding<Face>(f0.dart,nf);
		}

		// All DART of this face
		if(map_.template is_embedded<CDart>())
		{
			map_.foreach_dart_of_orbit(f0, [&] (Dart d)
			{
				uint32 nd = dart_container.insert_lines<1>() ;
				mbuild.template set_embedding<CDart>(d,nd);
			});
		}

		// All VERTEX of this face (if not existing)
		if(map_.template is_embedded<Vertex>())
		{
			// if incident vertex exists, reuse, otherwise create new
			uint32 nv = incidentVertices[0].is_valid() ? map_.embedding(incidentVertices[0]) : vertex_container.insert_lines<1>() ;
			mbuild.template set_embedding<Vertex>(d0, nv);

			nv = incidentVertices[1].is_valid() ? map_.embedding(incidentVertices[1]) : vertex_container.insert_lines<1>() ;
			mbuild.template set_embedding<Vertex>(d1, nv);

			nv = incidentVertices[2].is_valid() ? map_.embedding(incidentVertices[2]) : vertex_container.insert_lines<1>() ;
			mbuild.template set_embedding<Vertex>(d2, nv);

			nv = incidentVertices[3].is_valid() ? map_.embedding(incidentVertices[3]) : vertex_container.insert_lines<1>() ;
			mbuild.template set_embedding<Vertex>(d3, nv);
		}

		//        const bool isVertexEmbedded = map_.template is_embedded<Vertex>() ;
		//        map_.foreach_incident_vertex(f0,
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
				const Dart dSew = getDartByOrient(fAdj,oInv) ;
				mbuild.phi2_sew(d,dSew) ;

				// Set Edge embedding to that of the retrieved edge
				// mbuild.template set_embedding<Edge>(d, map_.embedding(eSew));
				// Set Vertex embedding to that of the retrieved Vertex
				//mbuild.template set_embedding<Vertex>(d, map_.embedding(Vertex(map_.phi1(dSew)))); // TODO CHECK IF ALREADY DONE. ITS RATHER AN OPTIMISATION AS DOING TWICE DOESNT HURT
			}
			d = map_.phi1(d) ; // go to next
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
	std::array<Face,4> getAdjacentFacesFromProvokingFace(const Face& provokingFace, const Orient& provokingDartDir)
	{
		std::array<Face,4> neighbFaces ;

		switch(provokingDartDir)
		{
			case(S):
			{
				neighbFaces[2] = provokingFace ; // provokingFace is located north

				// Check to the West
				const Face fNW = getFaceInOrient(provokingFace,W) ;
				if (fNW.is_valid())
				{
					const Face fW = getFaceInOrient(fNW,S) ;
					if (fW.is_valid())
					{
						neighbFaces[W] = fW ;
					}
				}

				// Check to the East
				const Face fNE = getFaceInOrient(provokingFace,E) ;
				if (fNE.is_valid())
				{
					const Face fE = getFaceInOrient(fNE,S) ;
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
				const Face fNW = getFaceInOrient(provokingFace,N) ;
				if (fNW.is_valid())
				{
					const Face fN = getFaceInOrient(fNW,E) ;
					if (fN.is_valid())
					{
						neighbFaces[N] = fN ;
					}
				}

				// Check to the South
				const Face fSW = getFaceInOrient(provokingFace,S) ;
				if (fSW.is_valid())
				{
					const Face fS = getFaceInOrient(fSW,E) ;
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
				const Face fSW = getFaceInOrient(provokingFace,W) ;
				if (fSW.is_valid())
				{
					const Face fW = getFaceInOrient(fSW,N) ;
					if (fW.is_valid())
					{
						neighbFaces[W] = fW ;
					}
				}

				// Check to the East
				const Face fSE = getFaceInOrient(provokingFace,E) ;
				if (fSE.is_valid())
				{
					const Face fE = getFaceInOrient(fSE,N) ;
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
				const Face fNE = getFaceInOrient(provokingFace,N) ;
				if (fNE.is_valid())
				{
					const Face fN = getFaceInOrient(fNE,W) ;
					if (fN.is_valid())
					{
						neighbFaces[N] = fN ;
					}
				}

				// Check to the South
				const Face fSE = getFaceInOrient(provokingFace,S) ;
				if (fSE.is_valid())
				{
					const Face fS = getFaceInOrient(fSE,W) ;
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
	//        map_.foreach_incident_vertex(f,
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
		map_.foreach_dart_of_orbit(f, [&] (Dart d) -> bool
		{
			if (dart_orientation_[d].value_ == target_orient)
			{
				res = d ;
				return false ;
			}
			return true ;
		});

		return res ;
		//        map_.foreach_incident_edge(f,
		//            [&] (Edge e)
		//        Dart d = f.dart ;
		//        for (uint i = 0 ; i < target_orient ; ++i)
		//            d = map_.phi1(d) ;

		//        return d ;
	}

	/**
	 * @brief getOrientOfDart tests if a given dart is in face #f, and returns its orientation is yes, INVALID_ORIENT otherwise
	 * @param d the dart
	 * @param f the face
	 * @return the orientation of the face, or INVALID_ORIENT if the dart is not in the face
	 */
	Orient getOrientOfDart(const Dart& d) const
	{
		return dart_orientation_[d].value_ ;
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
	//            d = map_.phi1(d) ;

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
	const Face getFaceInOrient(const Face& f, const Orient& orientation) const
	{
		Face res ;

		const Dart& dTmp = getDartByOrient(f,orientation) ;
		const Dart& dPhi2 = map_.phi2(dTmp) ;
		if (dTmp != dPhi2) // if there is a neighboring face there
		{
			res = Face(dPhi2) ;
		}

		return res ;
	}

	// ------------------------ Random Tools --------------------------------
	bool is_border(Dart d) const
	{
		return map_.phi2(d) == d ;
	}
};

} //namespace modeling

} //namespace cgogn

#endif // CGOGN_MODELING_TILING_PARALLELOGRAM_GRID_H
