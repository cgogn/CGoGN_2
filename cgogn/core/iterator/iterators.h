#ifndef CORE_TRAVERSAL_GLOBAL_ITERATORS_H
#define CORE_TRAVERSAL_GLOBAL_ITERATORS_H

#include <core/basic/cell.h>
#include <core/basic/dart_marker.h>
#include <core/basic/cell_marker.h>

namespace cgogn
{

enum TraversorStrategy
{
	AUTO = 0,
	FORCE_DART_MARKING,
	FORCE_CELL_MARKING,
	FORCE_TOPO_CACHE
};

template <typename MAP, unsigned int ORBIT, TraversorStrategy STRATEGY = AUTO>
class MapIterator
{
public:

	typedef MapIterator<MAP, ORBIT, STRATEGY> Self;
	typedef MAP Map;

	using   DartMarker = cgogn::DartMarker<Map>;
	using   CellMarker = cgogn::CellMarker<Map, ORBIT>;

protected:

	Map& map_;
	DartMarker* dm_;
	CellMarker* cm_;

public:

	MapIterator(MAP& map) :
		map_(map),
		dm_(nullptr),
		cm_(nullptr)
	{
		switch (STRATEGY)
		{
			case FORCE_DART_MARKING :
				dm_ = new DartMarker(map_);
				break;
			case FORCE_CELL_MARKING :
				cm_ = new CellMarker(map_);
				break;
			case FORCE_TOPO_CACHE :
				cgogn_assert_not_reached("FORCE_TOPO_CACHE not implemented yet");
				break;
			case AUTO :
				if (map_.template is_orbit_embedded<ORBIT>())
					cm_ = new CellMarker(map_);
				else
					dm_ = new DartMarker(map_);
				break;
		}
	}

	virtual ~MapIterator()
	{
		if(dm_) delete dm_;
		if(cm_) delete cm_;
	}

	class iterator
	{
	public:

		Self& traversor_;
		typename MAP::const_iterator map_it_;

		inline iterator(Self& t) :
			traversor_(t),
			map_it_(t.map_.begin())
		{
			switch (STRATEGY)
			{
				case FORCE_DART_MARKING :
					if (map_it_ != traversor_.map_.end())
						traversor_.dm_->template mark_orbit<ORBIT>(*map_it_);
					break;
				case FORCE_CELL_MARKING :
					if (map_it_ != traversor_.map_.end())
						traversor_.cm_->mark(*map_it_);
					break;
				case FORCE_TOPO_CACHE :
					cgogn_assert_not_reached("FORCE_TOPO_CACHE not implemented yet");
					break;
				case AUTO :
					if (map_it_ != traversor_.map_.end())
					{
						if (traversor_.dm_)
							traversor_.dm_->template mark_orbit<ORBIT>(*map_it_);
						else
							traversor_.cm_->mark(*map_it_);
					}
					break;
			}

		}

		inline iterator(Self& t, typename MAP::const_iterator it) :
			traversor_(t),
			map_it_(it)
		{
//			unsigned int dim = map_.dimension();
//			while(map_it_ != map_.end() && map_.is_boundary_marked(dim, *map_it_))
//				++map_it_;

			switch (STRATEGY)
			{
				case FORCE_DART_MARKING :
					if (map_it_ != traversor_.map_.end())
						traversor_.dm_->template mark_orbit<ORBIT>(*map_it_);
					break;
				case FORCE_CELL_MARKING :
					if (map_it_ != traversor_.map_.end())
						traversor_.cm_->mark(*map_it_);
					break;
				case FORCE_TOPO_CACHE :
					cgogn_assert_not_reached("FORCE_TOPO_CACHE not implemented yet");
					break;
				case AUTO :
					if (map_it_ != traversor_.map_.end())
					{
						if (traversor_.dm_)
							traversor_.dm_->template mark_orbit<ORBIT>(*map_it_);
						else
							traversor_.cm_->mark(*map_it_);
					}
					break;
			}
		}

		inline iterator& operator++()
		{
			cgogn_message_assert(map_it_ != traversor_.map_.end(), "MapIterator: iterator ++ after end");

			switch (STRATEGY)
			{
				case FORCE_DART_MARKING :
					while (map_it_ != traversor_.map_.end() && (traversor_.dm_->is_marked(*map_it_) /*|| traversor_.map_.is_boundary_marked(dim, *it)*/))
						++map_it_;
					if (map_it_ != traversor_.map_.end())
						traversor_.dm_->template mark_orbit<ORBIT>(*map_it_);
					break;
				case FORCE_CELL_MARKING :
					while (map_it_ != traversor_.map_.end() && (traversor_.cm_->is_marked(*map_it_) /*|| traversor_.map_.is_boundary_marked(dim, *it)*/))
						++map_it_;
					if (map_it_ != traversor_.map_.end())
						traversor_.cm_->mark(*map_it_);
					break;
				case FORCE_TOPO_CACHE :
					cgogn_assert_not_reached("FORCE_TOPO_CACHE not implemented yet");
					break;
				case AUTO :
					if (traversor_.dm_)
					{
						while (map_it_ != traversor_.map_.end() && (traversor_.dm_->is_marked(*map_it_) /*|| traversor_.map_.is_boundary_marked(dim, *it)*/))
							++map_it_;
						if (map_it_ != traversor_.map_.end())
							traversor_.dm_->template mark_orbit<ORBIT>(*map_it_);
					}
					else
					{
						while (map_it_ != traversor_.map_.end() && (traversor_.cm_->is_marked(*map_it_) /*|| traversor_.map_.is_boundary_marked(dim, *it)*/))
							++map_it_;
						if (map_it_ != traversor_.map_.end())
							traversor_.cm_->mark(*map_it_);
					}
					break;
			}

			return *this;
		}

		inline Cell<ORBIT> operator*()
		{
			return Cell<ORBIT>(*map_it_);
		}

		inline bool operator!=(iterator it) const
		{
			cgogn_assert(&traversor_ == &(it.traversor_));
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
