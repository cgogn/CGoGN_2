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
 * les classes de cartes pour ne pas les inclurent ici
 */
#include <core/map/cmap1.h>
#include <core/map/cmap2.h>

namespace cgogn
{

template <typename MAP, unsigned int ORBIT>
class CellIterator
{
public:

	typedef CellIterator<MAP, ORBIT> Self;
	using   DartMarker = cgogn::DartMarker<MAP>;

protected:

	MAP& map_;
	Dart cell_;
	bool outer_marker_;
	DartMarker* dm_;

public:

	CellIterator(MAP& map, Dart d, DartMarker* dm = nullptr) :
		map_(map),
		cell_(d),
		outer_marker_(false),
		dm_(dm)
	{
		if (dm) outer_marker_ = true;
	}

	~CellIterator()
	{
		if (dm_ && !outer_marker_)
			delete dm_;
	}

	DartMarker* init_marker()
	{
		if (!dm_) {
			if (outer_marker_)
				std::cerr << "Warning: unefficient use of Iterator (duplicated Marker)" << std::endl;
			dm_ = new DartMarker(map_);
		}
		DartMarker* tmp = dm_;
		dm_ = nullptr;
		return tmp;
	}

	class iterator
	{
	public:

		inline iterator(Self&)
		{
			std::cerr << "Not implemented" << std::endl;
		}

		inline iterator(Self&, Dart)
		{
			std::cerr << "Not implemented" << std::endl;
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

	inline iterator begin()
	{
		return iterator(*this);
	}

	inline iterator end()
	{
		return iterator(*this, Dart());
	}
};

template<> class CellIterator<CMap1, VERTEX1>::iterator
{
public:

	typedef CellIterator<CMap1, VERTEX1> Self;

	Dart current_;
	bool not_first_;

	inline iterator(Self& t) :
		current_(t.cell_), not_first_(false) {}

	inline iterator(Self& t, Dart) :
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

template<> class CellIterator<CMap1, FACE2>::iterator
{
public:

	typedef CellIterator<CMap1, FACE2> Self;

	CMap1& map_;
	Dart current_;
	bool not_first_;

	inline iterator(Self& t) :
		map_(t.map_), current_(t.cell_), not_first_(false) {}

	inline iterator(Self& t, Dart) :
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

template<> class CellIterator<CMap1, VOLUME3>::iterator
{
public:

	typedef CellIterator<CMap1, VOLUME3> Self;

	CMap1& map_;
	Dart current_;
	bool not_first_;

	inline iterator(Self& t) :
		map_(t.map_), current_(t.cell_), not_first_(false) {}

	inline iterator(Self& t, Dart) :
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

template<> class CellIterator<CMap2, FACE2>::iterator
{
public:

	typedef CellIterator<CMap2, FACE2> Self;

	CMap2& map_;
	Dart current_;
	bool not_first_;

	inline iterator(Self& t) :
		map_(t.map_), current_(t.cell_), not_first_(false) {}

	inline iterator(Self& t, Dart) :
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

template<> class CellIterator<CMap2, VERTEX2>::iterator
{
public:

	typedef CellIterator<CMap2, VERTEX2> Self;

	CMap2& map_;
	Dart current_;
	bool not_first_;

	inline iterator(Self& t) :
		map_(t.map_), current_(t.cell_), not_first_(false) {}

	inline iterator(Self& t, Dart) :
		map_(t.map_), current_(t.cell_), not_first_(false) {}

	inline iterator& operator++()
	{
		current_ = map_.phi_1(map_.phi2(current_));
		not_first_ = true;
		return *this;
	}

	inline Dart operator*() const { return current_; }

	inline bool operator!=(const iterator& it) const
	{
		return !(current_ == it.current_ && not_first_);
	}
};

template<> class CellIterator<CMap2, VOLUME3>::iterator
{
public:

	typedef CellIterator<CMap2, VOLUME3> Self;
	using   DartMarker = cgogn::DartMarker<CMap2>;

	CMap2& map_;
	DartMarker* dm_;
	std::vector<Dart>* visited_darts_;
	std::size_t visited_current_;
	Dart current_;
	bool not_first_;

	inline iterator(Self& t) :
		map_(t.map_),
		visited_current_(0u),
		current_(t.cell_),
		not_first_(false)
	{
		dm_ = t.init_marker();
		visited_darts_ = cgogn::get_dart_buffers()->get_buffer();
		visited_darts_->push_back(current_);
		dm_->mark(current_);
	}

	inline iterator(Self& t, Dart) :
		map_(t.map_),
		visited_darts_(nullptr),
		visited_current_(0u),
		current_(t.cell_),
		not_first_(false)
	{
	}

	inline ~iterator() {
		if (visited_darts_) {
			cgogn::get_dart_buffers()->release_buffer(visited_darts_);
		}
	}

	inline iterator& operator++()
	{
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
		++visited_current_;
		if (visited_current_ < visited_darts_->size())
			current_ = (*visited_darts_)[visited_current_];
		else
			current_ = (*visited_darts_)[0u];
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

