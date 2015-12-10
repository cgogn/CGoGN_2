#ifndef CORE_TRAVERSAL_GLOBAL_ITERATORS_H
#define CORE_TRAVERSAL_GLOBAL_ITERATORS_H

#include <core/basic/cell.h>
#include <core/basic/dart_marker.h>
#include <core/basic/cell_marker.h>

namespace cgogn
{

template <typename MAP, unsigned int ORBIT>
class AllCellIterator
{
public:

    typedef AllCellIterator<MAP, ORBIT> Self;
    typedef MAP Map;

    using   DartMarker = cgogn::DartMarker<Map>;
    using   CellMarker = cgogn::CellMarker<Map, ORBIT>;

protected:

    Map& map_;

public:

    AllCellIterator(MAP& map) :
        map_(map)
    {
    }

    ~AllCellIterator()
    {
    }

    class iterator
    {
    public:

        Map& map_;
        DartMarker* dm_;
        typename Map::const_iterator map_it_;

        inline iterator(Self& t) :
            map_(t.map_),
            dm_(nullptr),
            map_it_(t.map_.begin())
        {
            dm_ = new DartMarker(map_);
            if (map_it_ != map_.end()) {
                for (Dart d : InCellIterator<MAP,ORBIT>(map_, *map_it_, dm_))
                    dm_->mark(d);
            }
        }

        // TODO transformer en ajoutant un paramètre template booleen
        inline iterator(Self& t, typename MAP::const_iterator it) :
            map_(t.map_),
            dm_(nullptr),
            map_it_(it)
        {
            if (map_it_ != map_.end()) {
                std::cerr << "Warning : incorrect usage of end() constructor" << std::endl;
            }
        }

        ~iterator()
        {
            if(dm_) {
                dm_->unmark_all();
                delete dm_;
            }
        }

        inline iterator& operator++()
        {
            while (map_it_ != map_.end() && (dm_->is_marked(*map_it_)))
                ++map_it_;
            if (map_it_ != map_.end()) {
                for (Dart d : InCellIterator<MAP,ORBIT>(map_, *map_it_, dm_))
                    dm_->mark(d);
            }

            return *this;
        }

        inline Cell<ORBIT> operator*()
        {
            return Cell<ORBIT>(*map_it_);
        }

        inline bool operator!=(const iterator& it) const
        {
            return map_it_ != it.map_it_;
        }
    };

    inline iterator begin()
    {
        return iterator(*this);
    }

    inline iterator end()
    {
        return iterator(*this, map_.end());
    }
};

} // namespace cgogn

#endif // CORE_TRAVERSAL_GLOBAL_ITERATORS_H
