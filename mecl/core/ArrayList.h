//--------------------------------------------------------------------------
/// @file ArrayList.h
/// @brief Contains the implementation for a simple vector-like array implementation.
///
/// ArrayList is a replacement to use instead of std::vector.
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Michael Schulte (michael.schulte@magna.com)
///
//  --------------------------------------------------------------------------


#ifndef ARRAY_LIST_H_
#define ARRAY_LIST_H_

#include "MeclTypes.h"
#include "Helpers.h"
#include "MemArrayList.h"

namespace mecl
{
namespace core
{

// for convenience the typedef "type" or "iterator" is always named type or iterator
// PRQA S 1507 EOF

/// As alternative to the simple Array, the ArrayList counts the number
/// of data elements that has been added to the ArrayList.
/// It actually wraps the Array class adding some list operations and
/// restricting access to the array contents.
template<typename T, uint32_t MaxSize>
class ArrayList: public MemArrayList<T, MaxSize>
{
public:
    ArrayList(void);
    virtual ~ArrayList(void);

private:
    /// Do not allow (the usually accidental) copying of the ArrayList.
    ArrayList(const ArrayList<T, MaxSize> &);
    /// Do not allow (the usually accidental) copying by operator=.
    ArrayList<T, MaxSize> &operator =(const ArrayList<T, MaxSize> &);
    /// Wrapped array.
    T items_o[MaxSize];
};

/// ArrayList that stores the pointer instead of the actual type.
template<typename T, uint32_t MaxSize>
struct PtrArrayList: public ArrayList<T*, MaxSize> {};

// ////////////////////////////////////////////
// ArrayList implementation
// ////////////////////////////////////////////
template<typename T, uint32_t MaxSize>
ArrayList<T, MaxSize>::ArrayList() :
        MemArrayList<T, MaxSize>(&items_o[0])
{
}

template<typename T, uint32_t MaxSize>
ArrayList<T, MaxSize>::~ArrayList(void)
{
}

}
}

#endif // ARRAY_LIST_H_
