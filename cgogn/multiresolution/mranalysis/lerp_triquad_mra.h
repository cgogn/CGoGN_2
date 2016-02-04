#ifndef MULTIRESOLUTION_MRANALYSIS_LERP_TRI_QUAD_MRANALYSIS_H_
#define MULTIRESOLUTION_MRANALYSIS_LERP_TRI_QUAD_MRANALYSIS_H_

#include <core/basic/dart.h>

namespace cgogn {

template <typename MRMAP, typename VEC3>
class LerpTriQuadMRAnalysis
{

public:
    using Self = LerpTriQuadMRAnalysis<MRMAP, VEC3>;

    using VertexAttributeHandler = typename MRMAP::template VertexAttributeHandler<VEC3>;

protected:
    std::vector<std::function<void()>> synthesis_filters_;
    std::vector<std::function<void()>> analysis_filters_;

    MRMAP& map_;
    VertexAttributeHandler& va_;

public:
    LerpTriQuadMRAnalysis(MRMAP& map, VertexAttributeHandler& v):
        map_(map),
        va_(v)
    {
        synthesis_filters_.push_back(lerp_tri_quad_odd_synthesis_);
    }

protected:

    std::function<void()> lerp_tri_quad_odd_synthesis_ = [this] ()
    {
        map_.template foreach_cell<MRMAP::FACE>([&] (typename MRMAP::Face f)
        {
            if(map_.degree(f) != 3)
            {
                VEC3 vf(0.0);
                VEC3 ef(0.0);

                unsigned int count = 0;

                map_.template foreach_incident_edge(f, [&] (typename MRMAP::Edge e)
                {
                    vf += va_[e.dart];
                    map_.inc_current_level();
                    ef += va_[map_.phi1(e.dart)];
                    map_.dec_current_level();
                    ++count;
                });

                ef /= count;
                ef *= 2.0;

                vf /= count;

                map_.inc_current_level() ;
                Dart midF = map_.phi1(map_.phi1(f.dart));
                va_[midF] += vf + ef ;
                map_.dec_current_level() ;
            }
        });

        map_.template foreach_cell<MRMAP::EDGE>([&] (typename MRMAP::Edge e)
        {
            VEC3 ve = (va_[e.dart] + va_[map_.phi1(e)]) * 0.5;

            map_.inc_current_level() ;
            Dart midV = map_.phi1(e) ;
            va_[midV] += ve ;
            map_.dec_current_level() ;
        });
    };

public:

    void analysis()
    {
        cgogn_message_assert(map_.get_current_level() > 0, "analysis : called on level 0") ;

        map_.dec_current_level() ;

        for(unsigned int i = 0; i < analysis_filters_.size(); ++i)
            analysis_filters_[i]();
    }

    void synthesis()
    {
        cgogn_message_assert(map_.get_current_level() < map_.get_maximum_level(), "synthesis : called on max level") ;

        for(unsigned int i = 0; i < synthesis_filters_.size(); ++i)
            synthesis_filters_[i]();

        map_.inc_current_level();
    }

    void add_level()
    {
        map_.add_mixed_level();
    }

};

} //namespace cgogn

#endif // MULTIRESOLUTION_MRANALYSIS_LERP_TRI_QUAD_MRANALYSIS_H_

