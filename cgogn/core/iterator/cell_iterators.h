/*******************************************************************************
* CGoGN: Combinatorial and Geometric modeling with Generic N-dimensional MAPs  *
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

#ifndef CORE_ITERATOR_CELL_ITERATORS_H
#define CORE_ITERATOR_CELL_ITERATORS_H

#include <core/basic/cell.h>
#include <core/basic/dart_marker.h>
#include <core/basic/cell_marker.h>

/* Les spécialisations pourraient aller dans les fichiers où sont déclarées
 * les classes de cartes pour ne pas devoir les inclure ici
 */
#include <core/map/cmap1.h>
#include <core/map/cmap2.h>

namespace cgogn
{

template <typename MAP, unsigned int ORBIT>
class InCellIterator
{
public:

    typedef InCellIterator<MAP, ORBIT> Self;
    using   DartMarker = cgogn::DartMarker<MAP>;

protected:

	MAP& map_;
	Dart cell_;
	bool outer_marker_;
    DartMarker* dm_;

public:

    InCellIterator(MAP& map, Dart d, DartMarker* dm = nullptr) :
		map_(map),
		cell_(d),
		outer_marker_(false),
		dm_(dm)
	{
		if (dm) outer_marker_ = true;

	}

    ~InCellIterator()
	{
        if (dm_ && !outer_marker_) {
            // TODO => unmark_all ?
			delete dm_;
        }
	}

    DartMarker* init_marker()
	{
		if (!dm_) {
            if (outer_marker_) {
                // TODO : paranoic test : est-ce possible ?
                std::cerr << "Warning: non optimal use of Iterator (duplicated Marker)" << std::endl;
                outer_marker_ = false;
            }
            dm_ = new DartMarker(map_);
		}
        DartMarker* tmp = dm_;
		dm_ = nullptr;
		return tmp;
	}

	class iterator
	{
	public:

        inline iterator(Self&, bool)
		{
            std::cout << "Not implemented (ORBIT=" << ORBIT << ")" << std::endl;
		}

		inline iterator& operator++()
		{
			return *this;
		}

		inline Dart operator*() const
		{
			return Dart();
		}

		inline bool operator!=(const iterator&) const
		{
			return false;
		}
	};

    // TODO faire du paramètre booleen un paramètre template ?
    inline iterator begin()
	{
        return iterator(*this, true);
	}

    inline iterator end()
	{
        return iterator(*this, false);
	}
};

template<> class InCellIterator<CMap1, VERTEX1>::iterator
{
public:

    typedef InCellIterator<CMap1, VERTEX1> Self;

	Dart current_;
	bool not_first_;

    inline iterator(Self& t, bool) :
		current_(t.cell_), not_first_(false) {}

	inline iterator& operator++()
	{
		not_first_ = true;
		return *this;
	}

	inline Dart operator*() const { return current_; }

	inline bool operator!=(const iterator& it) const
	{
		return !(current_ == it.current_ && not_first_);
	}
};

template<> class InCellIterator<CMap1, FACE2>::iterator
{
public:

    typedef InCellIterator<CMap1, FACE2> Self;

	CMap1& map_;
	Dart current_;
	bool not_first_;

    inline iterator(Self& t, bool) :
		map_(t.map_), current_(t.cell_), not_first_(false) {}

	inline iterator& operator++()
	{
		current_ = map_.phi1(current_);
		not_first_ = true;
		return *this;
	}

	inline Dart operator*() const { return current_; }

	inline bool operator!=(const iterator& it) const
	{
		return !(current_ == it.current_ && not_first_);
	}
};

template<> class InCellIterator<CMap1, VOLUME3>::iterator
{
public:

    typedef InCellIterator<CMap1, VOLUME3> Self;

	CMap1& map_;
	Dart current_;
	bool not_first_;

    inline iterator(Self& t, bool) :
        map_(t.map_), current_(t.cell_), not_first_(false) {}

	inline iterator& operator++()
	{
		current_ = map_.phi1(current_);
		not_first_ = true;
		return *this;
	}

	inline Dart operator*() const { return current_; }

	inline bool operator!=(const iterator& it) const
	{
		return !(current_ == it.current_ && not_first_);
	}
};

template<> class InCellIterator<CMap2, FACE2>::iterator
{
public:

    typedef InCellIterator<CMap2, FACE2> Self;

	CMap2& map_;
	Dart current_;
	bool not_first_;

    inline iterator(Self& t, bool) :
		map_(t.map_), current_(t.cell_), not_first_(false) {}

	inline iterator& operator++()
	{
		current_ = map_.phi1(current_);
		not_first_ = true;
		return *this;
	}

	inline Dart operator*() const { return current_; }

	inline bool operator!=(const iterator& it) const
	{
		return !(current_ == it.current_ && not_first_);
	}
};

template<> class InCellIterator<CMap2, VERTEX2>::iterator
{
public:

    typedef InCellIterator<CMap2, VERTEX2> Self;

	CMap2& map_;
	Dart current_;
	bool not_first_;

    inline iterator(Self& t, bool) :
		map_(t.map_), current_(t.cell_), not_first_(false) {}

	inline iterator& operator++()
	{
        current_ = map_.phi2(map_.phi_1(current_));
		not_first_ = true;
		return *this;
	}

	inline Dart operator*() const { return current_; }

	inline bool operator!=(const iterator& it) const
	{
		return !(current_ == it.current_ && not_first_);
	}
};

template<> class InCellIterator<CMap2, VOLUME3>::iterator
{
public:

    typedef InCellIterator<CMap2, VOLUME3> Self;
    using   DartMarker = cgogn::DartMarker<CMap2>;

	CMap2& map_;
    DartMarker* dm_;
	std::vector<Dart>* visited_darts_;
    std::size_t current_index_;
	Dart current_;
	bool not_first_;

    // is_begin est 'true' pour la construction de l'itérateur begin(), 'false' sinon
    inline iterator(Self& t, bool is_begin) :
		map_(t.map_),
        dm_(nullptr),
        visited_darts_(nullptr),
        current_index_(0u),
        current_(t.cell_),
		not_first_(false)
	{
        if (is_begin) {
            dm_ = t.init_marker();
            visited_darts_ = cgogn::get_dart_buffers()->get_buffer();
            visited_darts_->push_back(current_);
            dm_->mark(current_);
        }
	}

	inline ~iterator() {
		if (visited_darts_) {
			cgogn::get_dart_buffers()->release_buffer(visited_darts_);
		}
	}

	inline iterator& operator++()
	{
        // TODO : faire ce test uniquement en DEBUG ?
        if(dm_) {
            not_first_ = true;

            Dart d = map_.phi1(current_);
            if (!dm_->is_marked(d)) {
                visited_darts_->push_back(d);
                dm_->mark(d);
            }
            d = map_.phi2(current_);
            if (!dm_->is_marked(d)) {
                visited_darts_->push_back(d);
                dm_->mark(d);
            }
            ++current_index_;
            if (current_index_ < visited_darts_->size())
                current_ = (*visited_darts_)[current_index_];
            else
                current_ = (*visited_darts_)[0u];
        }
        else
            std::cerr << "Warning: ++ operator has no effect on the end() iterator";

		return *this;
	}

	inline Dart operator*() const
	{
		return current_;
	}

	inline bool operator!=(const iterator& it) const
	{
		return !(current_ == it.current_ && not_first_);
	}
};

} // namespace cgogn

#endif // CORE_ITERATOR_CELL_ITERATORS_H

