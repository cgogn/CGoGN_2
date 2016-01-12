#ifndef UTILS_ALLOCATOR_H
#define UTILS_ALLOCATOR_H
#include <cstddef>
#include <cassert>
namespace cgogn {

struct Chunk
{
    typedef unsigned char uchar;
    void init(std::size_t blockSize, uchar blocks)
    {
        assert(blocks > 0);
        // Overflow check
        const ::std::size_t allocSize = blockSize * blocks;
        assert( allocSize / blockSize == blocks);
        _pData = new uchar[blockSize * blocks];
        _firstAvailableBlock = 0;
        _blocksAvailable = blocks;
        uchar i{0};
        uchar* p = _pData;
        for(; i != blocks ; p+=blockSize)
        {
            *p = ++i;
        }
    }

    inline bool isFilled() const
    {
        return _blocksAvailable == uchar(0);
    }

    inline bool hasAvailable( unsigned char numBlocks ) const
    {
        return ( _blocksAvailable == numBlocks );
    }

    void* allocate(std::size_t blockSize)
    {
        if (isFilled())
            return nullptr;

        uchar* result = _pData + (_firstAvailableBlock * blockSize);
        //update firstavailableblock to point to the next block
        _firstAvailableBlock = *result;
        --_blocksAvailable;
        return result;
    }

    void deallocate(void* p, std::size_t blockSize)
    {
        assert( p >= _pData );
        uchar* toRelease = static_cast<uchar*>(p);
        // Alignment check
        assert((toRelease - _pData) % blockSize == 0);
        const unsigned char index = static_cast< unsigned char >(( toRelease - _pData ) / blockSize);
#if defined(DEBUG) || defined(_DEBUG)
    // Check if block was already deleted.  Attempting to delete the same
    // block more than once causes Chunk's linked-list of stealth indexes to
    // become corrupt.  And causes count of blocksAvailable_ to be wrong.
    if ( 0 < _blocksAvailable )
        assert( _firstAvailableBlock != index );
#endif
        *toRelease = _firstAvailableBlock;
        _firstAvailableBlock = static_cast<uchar>( index );
        ++_blocksAvailable;
    }

    void release()
    {
        assert( _pData != nullptr);
        delete[] _pData;

    }

    unsigned char* _pData;
    unsigned char _firstAvailableBlock;
    unsigned char _blocksAvailable;
};
}

#endif // UTILS_ALLOCATOR_H

