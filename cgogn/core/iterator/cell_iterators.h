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

#ifndef CORE_TRAVERSAL_CELL_ITERATORS_H
#define CORE_TRAVERSAL_CELL_ITERATORS_H

#include <core/basic/cell.h>
#include <core/basic/dart_marker.h>
#include <core/basic/cell_marker.h>

namespace cgogn
{

template <typename MAP, unsigned int ORBIT>
class CellIterator {
public:

	typedef CellIterator<MAP, ORBIT> Self;
	typedef MAP Map;

	using   DartMarker = cgogn::DartMarker<Map>;

protected:

	Map& map_;
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

	Dart current_;
	bool is_first_;

	inline iterator(Self& t) :
		current_(t.cell_), is_first_(true) {}

	inline iterator(Self& t, Dart) :
		current_(t.cell_), is_first_(true)	{}

	inline iterator& operator++()
	{
		is_first_ = false;
		return *this;
	}

	inline Dart operator*() const { return current_; }

	inline bool operator!=(const iterator& it) const
	{
		return !(current_ == it.current_ && !is_first_);
	}
};

template<> class CellIterator<CMap1, FACE2>::iterator
{
public:

	Map& map_;
	Dart current_;
	bool is_first_;

	inline iterator(Self& t) :
		map_(t.map_), current_(t.cell_), is_first_(true) {}

	inline iterator(Self& t, Dart) :
		map_(t.map_), current_(t.cell_), is_first_(true)	{}

	inline iterator& operator++()
	{
		current_ = map_.phi1(current_);
		is_first_ = false;
		return *this;
	}

	inline Dart operator*() const { return current_; }

	inline bool operator!=(const iterator& it) const
	{
		return !(current_ == it.current_ && !is_first_);
	}
};

template<> class CellIterator<CMap1, VOLUME3>::iterator
{
public:

	Map& map_;
	Dart current_;
	bool is_first_;

	inline iterator(Self& t) :
		map_(t.map_), current_(t.cell_), is_first_(true) {}

	inline iterator(Self& t, Dart) :
		map_(t.map_), current_(t.cell_), is_first_(true)	{}

	inline iterator& operator++()
	{
		current_ = map_.phi1(current_);
		is_first_ = false;
		return *this;
	}

	inline Dart operator*() const { return current_; }

	inline bool operator!=(const iterator& it) const
	{
		return !(current_ == it.current_ && !is_first_);
	}
};

template<> class CellIterator<CMap2, FACE2>::iterator
{
public:

	Map& map_;
	Dart current_;
	bool is_first_;

	inline iterator(Self& t) :
		map_(t.map_), current_(t.cell_), is_first_(true) {
	}

	inline iterator(Self& t, Dart) :
		map_(t.map_), current_(t.cell_), is_first_(true)	{
	}

	inline iterator& operator++()
	{
		current_ = map_.phi1(current_);
		is_first_ = false;
		return *this;
	}

	inline Dart operator*() const { return current_; }

	inline bool operator!=(const iterator& it) const
	{
		return !(current_ == it.current_ && !is_first_);
	}
};

template<> class CellIterator<CMap2, VERTEX2>::iterator
{
public:

	Map& map_;
	Dart current_;
	bool is_first_;

	inline iterator(Self& t) :
		map_(t.map_), current_(t.cell_), is_first_(true) {}

	inline iterator(Self& t, Dart) :
		map_(t.map_), current_(t.cell_), is_first_(true)	{}

	inline iterator& operator++()
	{
		current_ = map_.phi_1(map_.phi2(current_));
		is_first_ = false;
		return *this;
	}

	inline Dart operator*() const { return current_; }

	inline bool operator!=(const iterator& it) const
	{
		return !(current_ == it.current_ && !is_first_);
	}
};

template<> class CellIterator<CMap2, VOLUME3>::iterator
{
	using   DartMarker = cgogn::DartMarker<Map>;

public:

	Map& map_;
	DartMarker* dm_;
	std::vector<Dart>* visited_darts_;
	std::vector<Dart>::iterator visited_current_;
	Dart current_;

	inline iterator(Self& t) :
		map_(t.map_),
		current_(t.cell_)
	{
		std::cout << "iterator begin() in " << current_ << std::endl;
		dm_ = t.init_marker();
		visited_darts_ = cgogn::get_dart_buffers()->get_buffer();
		visited_darts_->push_back(current_);
		dm_->mark(current_);
		visited_current_ = visited_darts_->begin();
		std::cout << "iterator begin() out " << std::endl;
	}

	// TODO : le brin donné par Vector<Dart>.end() est 0u, mais pas NIL c'est ok ?
	// TODO : renvoyer le premier brin à la fin et ajouter le booleen is_first
	inline iterator(Self& t, Dart) :
		map_(t.map_),
		visited_darts_(nullptr),
		current_(0u)
	{
	}

	inline ~iterator() {
		if (visited_darts_)
			cgogn::get_dart_buffers()->release_buffer(visited_darts_);
	}

	inline iterator& operator++()
	{
		Dart d = map_.phi1(*visited_current_);
		if (!dm_->is_marked(d)) {
			visited_darts_->push_back(d);
			dm_->mark(d);
		}
		d = map_.phi2(*visited_current_);
		if (!dm_->is_marked(d)) {
			visited_darts_->push_back(d);
			dm_->mark(d);
		}
		++visited_current_;
		current_ = *visited_current_;
		return *this;
	}

	inline Dart operator*() const
	{
		return current_;
	}

	// TODO : faire le test sur visited_current_ == end() et ignorer it ?
	inline bool operator!=(const iterator& it) const
	{
		std::cout << "iterator " << current_ << " != " << it.current_ << std::endl;
		return current_ != it.current_;
	}
};



} // namespace cgogn

#endif // CORE_TRAVERSAL_CELL_ITERATORS_H

