//--------------------------------------------------------------------------
/// @file MemArrayList.h
/// @brief Contains the implementation to directly access memory regions as List.
///
/// The MemArrayList directly operates on the RAM with checks included
/// to operate only on defined memory regions. It is the counterpart to ArrayList.
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

#ifndef MEM_ARRAY_LIST_H_
#define MEM_ARRAY_LIST_H_

#include "MeclTypes.h"
#include "Helpers.h"
#include "MemArray.h"

namespace mecl
{
namespace core
{

// for convenience the typedef "type" or "iterator" is always named type or iterator
// PRQA S 1507 EOF

/// Iterates over the elements of the MemArrayList.
template<typename T, uint32_t MaxSize>
class ConstArrayListIterator;

/// As alternative to the simple Array, the MemArrayList counts the number
/// of data elements that has been added to the MemArrayList.
/// It actually wraps the Array class adding some list operations and
/// restricting access to the array contents.
template<typename T, uint32_t MaxSize>
class MemArrayList: public Collection<T, MaxSize, ConstArrayListIterator<T, MaxSize>, true >
{
public:
    /// The actual iterator which may be used when looping - the same used as for Array.
    typedef ConstArrayListIterator<T, MaxSize> const_iterator;
    typedef typename Collection<T, MaxSize, ConstArrayListIterator<T, MaxSize>, true >::IteratorType_t iterator;
    typedef typename ConstRefType<T>::type ConstRefType_t;

    explicit MemArrayList(T* const i_MemArea_po);
    virtual ~MemArrayList(void);

    /// Copying of MemArrayList is allowed as there are no actual resources copied or freed here.
    /// It is also only allowed to copy an array of the same size.
    explicit MemArrayList(const MemArrayList<T, MaxSize>& i_CopyFrom_ro);
    /// Copying of MemArrayList is allowed as there are no actual resources copied or freed here.
    /// It is also only allowed to copy an array of the same size.
    MemArrayList<T,MaxSize>& operator= (const MemArrayList<T, MaxSize>& i_CopyFrom_ro);

    // adds an element to the end of the list
    void pushBack_v(ConstRefType_t i_Item_ro);
    T& popBack_ro();

    /// Returns the last element in the list (if not empty)
    T& back_ro() { return operator[](currentSize_u32-1); }
    /// Constant access to the last element in the list (if not empty)
    const T& back_ro() const { return operator[](currentSize_u32-1); }
    /// Returns the first element in the list (if not empty)
    T& front_ro() { return operator[](0); }
    /// Constant access to the first element in the list (if not empty)
    const T& front_ro() const { return operator[](0); }
    /// Increases the size of the internal list and returns the last item.
    T& addItem_ro() { ++currentSize_u32; return back_ro(); }

    virtual uint32_t size_u32() const { return currentSize_u32; }

    void clear_v() { items_o.init_v(); currentSize_u32 = 0U; }
    /// Checked read-only access to the array contents.
    /// The index has to be in the range of the list.
    const T& operator[](uint32_t index_u32) const;
    /// Checked RW-access of the array contents.
    /// Data may be changed with this iterator but may not be added - use pushBack_v for adding data.
    T& operator[](uint32_t index_u32);
    virtual bool_t isEmpty_b() const { return currentSize_u32 == 0U; }

    virtual const_iterator begin_o() const
    {
        return const_iterator(*this, items_o.begin_o());
    }

    virtual const_iterator end_o() const
    {
        return const_iterator(*this, items_o.iteratorAt_o(currentSize_u32));
    }

    friend class ConstArrayListIterator<T, MaxSize> ;

    /// Safe access to the underlying array.
    const MemArray<T, MaxSize, true>& getArray_o() const { return items_o; }

    /// Copies another list into this one.
    /// @param i_CopyFrom_ro the other list to copy data from
    /// @param i_CopyFromStartIndex_u32 the first index to copy from the other list
    /// @param i_DestStartIndex_u32 the first index to copy the data to in this list
    /// @param i_Length_u32 the number of bytes to copy
    template<uint32_t SizeOther>
    void copy_v(const MemArrayList<T, SizeOther>& i_CopyFrom_ro,
                uint32_t i_CopyFromStartIndex_u32,
                uint32_t i_DestStartIndex_u32,
                uint32_t i_Length_u32)
    {
        items_o.copy_v(i_CopyFrom_ro.items_o, i_CopyFromStartIndex_u32, i_DestStartIndex_u32, i_Length_u32);
        currentSize_u32 = i_CopyFrom_ro.currentSize_u32;
    }

    /// Copies another list into this one.
    /// @param i_CopyFrom_ro the other list to copy data from
    /// @param i_Length_u32 the number of bytes to copy
    template<uint32_t SizeOther>
    void copy_v(const MemArrayList<T, SizeOther>& i_CopyFrom_ro,
                uint32_t i_Length_u32)
    {
        copy_v(i_CopyFrom_ro, 0U, 0U, i_Length_u32);
    }

    /// Copies another list completely into this one.
    /// @param i_CopyFrom_ro the other list to copy data from
    void copyAll_v(const MemArrayList<T, MaxSize>& i_CopyFrom_ro)
    {
        items_o.copyAll_v(i_CopyFrom_ro.items_o);
        currentSize_u32 = i_CopyFrom_ro.currentSize_u32;
    }

private:

    /// the current number of elements that were added using pushBack_v.
    /// Should not exceed the maximum size.
    uint32_t currentSize_u32;
    /// Wrapped memory area.
    MemArray<T, MaxSize, true> items_o;
};

/// This actually only wraps the ConstArrayIterator and adds an additional check not to access
/// beyond the current list size.
template<typename T, uint32_t MaxSize>
class ConstArrayListIterator
{
public:
    const T& operator*() const;
    bool_t operator ==(const ConstArrayListIterator<T, MaxSize>& i_Other_ro) const;
    bool_t operator !=(const ConstArrayListIterator<T, MaxSize>& i_Other_ro) const;

    /// ++-iterator should also work on const lists.
    /// Hence the iterator is defined as const.
    const ConstArrayIterator<T, MaxSize, true>& operator++() const;
    /// the iterator can only be created from the outer type
    friend class MemArrayList<T, MaxSize> ;
private:
    ConstArrayListIterator();
    ConstArrayListIterator(const MemArrayList<T, MaxSize> &i_ArrayList_ro,
                      const ConstArrayIterator<T, MaxSize, true> &i_ArrayIterator_ro);
    const MemArrayList<T, MaxSize> &arrayList_ro;
    const ConstArrayIterator<T, MaxSize, true> arrayIterator_ro;
};

// ////////////////////////////////////////////
// MemArrayList implementation
// ////////////////////////////////////////////
template<typename T, uint32_t MaxSize>
MemArrayList<T, MaxSize>::MemArrayList(T* const i_MemArea_po) :
        Collection<T, MaxSize, ConstArrayListIterator<T, MaxSize> >(),
        currentSize_u32(0U),
        items_o(i_MemArea_po, MaxSize)
{
}

template<typename T, uint32_t MaxSize>
MemArrayList<T, MaxSize>::~MemArrayList(void)
{
}


template<typename T, uint32_t MaxSize>
MemArrayList<T, MaxSize>::MemArrayList(const MemArrayList<T, MaxSize>& i_CopyFrom_ro)
    : currentSize_u32(i_CopyFrom_ro.currentSize_u32), items_o(i_CopyFrom_ro.items_o)
{
}

template<typename T, uint32_t MaxSize>
MemArrayList<T,MaxSize>& MemArrayList<T, MaxSize>::operator= (const MemArrayList<T, MaxSize>& i_CopyFrom_ro)
{
    currentSize_u32 = i_CopyFrom_ro.currentSize_u32;
    items_o = i_CopyFrom_ro.items_o;
    return *this;
}


template<typename T, uint32_t MaxSize>
const T& MemArrayList<T, MaxSize>::operator[](uint32_t index_u32) const
{
    Assert(index_u32 < currentSize_u32);
    return items_o[index_u32];
}

template<typename T, uint32_t MaxSize>
T& MemArrayList<T, MaxSize>::operator[](uint32_t index_u32)
{
    Assert(index_u32 < currentSize_u32);
    return items_o[index_u32];
}


template<typename T, uint32_t MaxSize>
void MemArrayList<T, MaxSize>::pushBack_v(ConstRefType_t i_Item_ro)
{
    Assert(currentSize_u32 != MaxSize);
    uint32_t currentInsertPosition_u32 = currentSize_u32;
    ++currentSize_u32;
    // a little complicated call because of MISRA-C++ Rule 14-6-2
    // - the copy operator could be defined later - an explicit call makes sure the real operator is inserted by the compiler
    // for primitive types the usual call to "=" is called.
    this->copyItem_v(items_o[currentInsertPosition_u32], i_Item_ro);
    // This is what it actually does: items_o[currentInsertPosition_u32] = i_Item_ro
}

template<typename T, uint32_t MaxSize>
T& MemArrayList<T, MaxSize>::popBack_ro()
{
    Assert(currentSize_u32 != 0U);
    --currentSize_u32;
    return items_o[currentSize_u32];
}

// ////////////////////////////////////////////
// ConstArrayListIterator implementation
// ////////////////////////////////////////////

template<typename T, uint32_t MaxSize>
const T& ConstArrayListIterator<T, MaxSize>::operator*() const
{
    Assert(arrayIterator_ro.getIndex_u32() < arrayList_ro.currentSize_u32);
    return *arrayIterator_ro;
}

template<typename T, uint32_t MaxSize>
bool_t ConstArrayListIterator<T, MaxSize>::operator ==(const ConstArrayListIterator<T, MaxSize>& i_Other_ro) const
{
    return arrayIterator_ro.operator==(i_Other_ro.arrayIterator_ro);
}

template<typename T, uint32_t MaxSize>
bool_t ConstArrayListIterator<T, MaxSize>::operator !=(
        const ConstArrayListIterator<T, MaxSize>& i_Other_ro) const
{
    return !(*this == i_Other_ro);
}

template<typename T, uint32_t MaxSize>
const ConstArrayIterator<T, MaxSize, true>& ConstArrayListIterator<T, MaxSize>::operator++() const
{
    ++arrayIterator_ro;
    return arrayIterator_ro;
}

template<typename T, uint32_t MaxSize>
ConstArrayListIterator<T, MaxSize>::ConstArrayListIterator(
        const MemArrayList<T, MaxSize> &i_ArrayList_ro,
        const ConstArrayIterator<T, MaxSize, true> &i_ArrayIterator_ro) :
        arrayList_ro(i_ArrayList_ro),
        arrayIterator_ro(i_ArrayIterator_ro)
{
}

}
}

#endif // MEM_ARRAY_LIST_H_
