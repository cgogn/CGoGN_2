#ifndef MRA_H
#define MRA_H


template <typename MRMAP, typename VEC3>
class LerpTriQuadMRAnalysis
{

    MRMAP& map_;
    typename MRMAP::VertexAttribute va_;

    LerpTriQuadMRAnalysis(MRMAP& map):
        map_(map)
    {
        synthesis_filters_.emplace_back(lerp_quad_odd_synthesis_);
    }

protected:
    std::vector<std::function<void()>> synthesis_filters_;
    std::vector<std::function<void()>> analysis_filters_;

protected:

    std::function<void()> lerp_tri_quad_odd_synthesis_ = [] ()
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
                        vf += v[e];
                        map_.incCurrentLevel();
                        ef += v[map_.phi1(e)];
                        map_.decCurrentLevel();
                        ++count;
                });

                ef /= count;
                ef *= 2.0;

                vf /= count;

                map_.incCurrentLevel() ;
                Dart midF = map_.phi1(map_.phi1(f));
                m_position[midF] += vf + ef ;
                map_.decCurrentLevel() ;
            }
        });

        map_.template foreach_cell<MRMAP::EDGE>([&] (typename MRMAP::Edge e)
        {
            VEC3 ve = (v[e] + v[map_.phi1(e)]) * typename VEC3::TYPE(0.5);

            map_.incCurrentLevel() ;
            Dart midV = map_.phi1(e) ;
            m_position[midV] += ve ;
            map_.decCurrentLevel() ;
        });
    };

public:

    inline set_attribute(typename MRMAP::VertexAttribute<VEC3>& v)
    {
        va_ = v;
    }

    void analysis()
    {
        cgogn_message_assert(map_.get_current_level() > 0, "analysis : called on level 0") ;

        map_.dec_current_level() ;

        for(unsigned int i = 0; i < analysis_filters_.size(); ++i)
                (*analysis_filters_[i])() ;
    }

    void synthesis()
    {
        cgogn_message_assert(map_.getCurrentLevel() < map_.getMaxLevel(), "synthesis : called on max level") ;

        for(unsigned int i = 0; i < synthesis_filters_.size(); ++i)
                (*synthesis_filters_[i])() ;

        map_.incCurrentLevel();
    }

    void add_level()
    {
        map_.add_mixed_level();
    }

};


#endif // MRA_H

