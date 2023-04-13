//--------------------------------------------------------------------------
/// @file CircularBuffer.h
/// @brief Contains the implementation for a simple circular buffer.
///
/// CircularBuffer is used as a vector-like buffer.
/// As difference to ArrayList the CircularBuffer will remove the oldest elements
/// in a FIFO principle when elements are added to an already filled buffer.
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
/// \addtogroup mecl
/// @{
/// \addtogroup core
/// @{

#ifndef CIRCULARBUFFER_H_
#define CIRCULARBUFFER_H_

#include "MeclTypes.h"
#include "Helpers.h"
#include "Array.h"

namespace mecl
{
namespace core
{

// for convenience the typedef "type" or "iterator" is always named type or iterator
// PRQA S 1507 EOF
// operators are only used in a conventional manner here
// PRQA S 2083, 2094 EOF
// It is allowed to define template functions in the class
// PRQA S 2106, 2131, 2135 EOF

/// Iterates the CircularBuffer.
template<typename T, uint32_t MaxSize>
class ConstCircularBufferIterator;

// PRQA S 1051 38
/// As alternative to the simple Array, the CircularBuffer counts the number
/// of data elements that has been added to the CircularBuffer.
/// It actually wraps the Array class adding some list operations and
/// restricting access to the array contents.
/// Instead of throwing exceptions as the ArrayList does, CircularBuffer
/// simply overwrites the oldest element when a new element is added and the
/// buffer is full.
/// @par Example
/// @code{.cpp}
/// #include "mecl/core/CircularBuffer.h"
///
/// void test()
/// {
///   // use as stack
///   mecl::core::CircularBuffer<uint32_t, 16> v_Stack_o;
///   v_Stack_o.pushBack_v(11);
///   v_Stack_o.pushBack_v(22);
///   v_Stack_o.pushBack_v(33);
///   while (v_Stack_o.size_u32() > 0U)
///   {
///     // popBack_ro removes the last inserted elements first
///     log_printf("%d\n", static_cast<printInt_t>(v_Stack_o.popBack_ro()));
///   }
///   // prints 33, 22, 11
///
///   // use as queue - sole difference is using popFront instead of popBack
///   mecl::core::CircularBuffer<uint16_t, 2> v_Queue_o;
///   v_Queue_o.pushBack_v(0x11);
///   v_Queue_o.pushBack_v(0x22);
///   // removes the first element "0x11" as the size of the queue is only 2 elements
///   v_Queue_o.pushBack_v(0x33);
///   while (v_Queue_o.size_u32() > 0U)
///   {
///     // popFront_ro removes the first inserted elements first
///     log_printf("0x%02x\n", static_cast<printInt_t>(v_Queue_o.popFront_ro()));
///   }
///   // prints 0x22, 0x33
///
///   // this won't compile as the size has to be a power of 2
///   // mecl::core::CircularBuffer<uint16_t, 12> v_QueueCompileError_o;
/// }
/// @endcode
///
/// @tparam T the stored type
/// @tparam MaxSize maximum number of elements stored.
/// It should be a power of 2 for performance reasons - then
/// the compiler is able to replace the %-operations by &-operations (as x % pow(2,n) is the same as x & pow(2,n)-1).
/// However in ARM environments the %-operation with constant values seems to be efficient enough so that there is
/// no difference measurable between using % and & operators.
template<typename T, uint32_t MaxSize>
class CircularBuffer: public Collection<T, MaxSize, ConstCircularBufferIterator<T, MaxSize>, true >
{
public:
    /// The actual iterator which may be used when looping - the same used as for Array.
    typedef ConstCircularBufferIterator<T, MaxSize> const_iterator;
    typedef typename Collection<T, MaxSize, ConstCircularBufferIterator<T, MaxSize>, true >::IteratorType_t iterator;
    typedef typename ConstRefType<T>::type ConstRefType_t;

    CircularBuffer(void);
    virtual ~CircularBuffer(void);

    /// Adds an element to the end of the list
    void pushBack_v(ConstRefType_t i_Item_ro);

    /// Removes the element at the end of the list.
    /// Use this with pushBack_v when needing a stack or LIFO.
    T& popBack_ro();

    /// Removes the element at the beginning of the list.
    /// Use this with pushBack_v when needing a queue or FIFO.
    T& popFront_ro();

    /// Returns the current size of the list.
    virtual uint32_t size_u32() const
    {
        return pushIndex_u32 - popIndex_u32; // PRQA S 3084 // pushIndex_u32 >= popIndex_u32
    }

    /// Clears the list - sets it back to its original state.
    void clear_v()
    {
        // just reset the indices
        popIndex_u32 = 0U;
        pushIndex_u32 = 0U;
    }

    /// Checked read-only access to the array contents.
    /// The index has to be in the range of the list.
    const T& operator[](uint32_t index_u32) const;
    /// Checked RW-access of the array contents.
    /// Data may be changed with this iterator but may not be added - use pushBack_v for adding data.
    /// The index has to be in the range of the list.
    T& operator[](uint32_t index_u32);

    /// Checks if elements are contained in the list.
    virtual bool_t isEmpty_b() const
    {
        return popIndex_u32 == pushIndex_u32;
    }

    /// Returns iterator element that starts at the beginning of the buffer
    virtual const_iterator begin_o() const
    {
        return ConstCircularBufferIterator<T, MaxSize>(*this, 0U);
    }
    /// End iterator needed for comparison in loops.
    virtual const_iterator end_o() const
    {
        return ConstCircularBufferIterator<T, MaxSize>(*this, size_u32());
    }

    /// The iterator class needs access to member elements in this class.
    friend class ConstCircularBufferIterator<T, MaxSize>;

private:
    /// Do not allow (the usually accidental) copying of the CircularBuffer.
    CircularBuffer(const CircularBuffer<T, MaxSize> &);
    /// Do not allow (the usually accidental) copying by operator=.
    CircularBuffer<T, MaxSize> &operator =(const CircularBuffer<T, MaxSize> &);
    uint32_t pushIndex_u32;
    uint32_t popIndex_u32;
    // Wrap the array
    Array<T, MaxSize> items_o;

    void checkIndices_v();
};

/// CircularBuffer that stores the pointer instead of the actual type.
template<typename T, uint32_t MaxSize>
struct PtrCircularBuffer: public CircularBuffer<T*, MaxSize> {};

/// Iterator definition for CircularBuffer type.
/// This actually only wraps the ArrayIterator and adds an additional check not to access
/// beyond the current list size.
template<typename T, uint32_t MaxSize>
class ConstCircularBufferIterator
{
public:
    virtual ~ConstCircularBufferIterator() {}
    ConstCircularBufferIterator(const ConstCircularBufferIterator& i_Other_ro)
        : circularBuffer_ro(i_Other_ro.circularBuffer_ro), // PRQA S 2528 // OK, see comment below
          arrayIndex_u32(i_Other_ro.arrayIndex_u32)
    {
    }
    const T& operator*() const;
    bool_t operator !=(const ConstCircularBufferIterator<T, MaxSize>& i_Other_ro) const
    {
        return arrayIndex_u32 != i_Other_ro.arrayIndex_u32;
    }
    /// Prefix ++ operator implementation.
    const ConstCircularBufferIterator<T, MaxSize>& operator++() const
    {
        ++arrayIndex_u32;
        return *this;
    }
    /// the iterator can only be created from the outer type
    friend class CircularBuffer<T, MaxSize> ;
private:
    ConstCircularBufferIterator();
    /// Creates the iterator.
    /// Constructor is only accessible by the CircularBuffer, which the iterator references.
    /// Hence it is not possible, that the referenced buffer gets destructed before the iterator.
    /// @param i_CircularBuffer_ro The referenced buffer
    /// @param i_ArrayIndex_u32 The array index position in the buffer.
    ///
    ConstCircularBufferIterator(const CircularBuffer<T, MaxSize> &i_CircularBuffer_ro, uint32_t i_ArrayIndex_u32) :
            circularBuffer_ro(i_CircularBuffer_ro), // PRQA S 2528 // OK, because referenced buffer creates this object
            arrayIndex_u32(i_ArrayIndex_u32)
    {
    }
    /// Do not allow (the usually accidental) copying by operator=.
    ConstCircularBufferIterator<T, MaxSize> &operator =(const ConstCircularBufferIterator<T, MaxSize> &);

    const CircularBuffer<T, MaxSize> &circularBuffer_ro;
    mutable uint32_t arrayIndex_u32;
};

// ////////////////////////////////////////////
// CircularBuffer implementation
// ////////////////////////////////////////////
template<typename T, uint32_t MaxSize>
CircularBuffer<T, MaxSize>::CircularBuffer() :
        Collection<T, MaxSize, ConstCircularBufferIterator<T, MaxSize> >(),
        pushIndex_u32(0U),
        popIndex_u32(0U)
{
    // MaxSize is needed to be a power of 2
    // power of 2 = 100...0, minus one is 011...1, the AND result is 0
    StaticAssert(IsPowerOfTwo<MaxSize>::value, "MaxSize must be power of 2");
}

template<typename T, uint32_t MaxSize>
CircularBuffer<T, MaxSize>::~CircularBuffer(void)
{
}


template<typename T, uint32_t MaxSize>
const T& CircularBuffer<T, MaxSize>::operator[](uint32_t index_u32) const
{
    Assert(!isEmpty_b());
    return items_o[(index_u32 + popIndex_u32) % MaxSize];
}

template<typename T, uint32_t MaxSize>
T& CircularBuffer<T, MaxSize>::operator[](uint32_t index_u32)
{
    Assert(!isEmpty_b());
    return items_o[(index_u32 + popIndex_u32) % MaxSize]; // PRQA S 3084 // unlikely overflow leads to same result
}


template<typename T, uint32_t MaxSize>
void CircularBuffer<T, MaxSize>::checkIndices_v()
{
    Assert(pushIndex_u32 >= popIndex_u32);
    while (popIndex_u32 >= MaxSize)
    {
        // PRQA S 3084 2 // checked unsigned arithmetic is OK here
        popIndex_u32 -= MaxSize; // this is actually the same as using %-operator, but popIndex_u32 must be >= pushIndex_u32
        pushIndex_u32 -= MaxSize;
    }
}

template<typename T, uint32_t MaxSize>
void CircularBuffer<T, MaxSize>::pushBack_v(ConstRefType_t i_Item_ro)
{
    bool_t wasFull_b = this->isFull_b();
    // actually the oldest item will be overwritten here, so it is allowed to
    // add elements to a buffer that is already full
    this->copyItem_v(items_o[pushIndex_u32 % MaxSize], i_Item_ro);
    ++pushIndex_u32;
    // The read index has to be adapted in case the buffer was full.
    // Here the oldest element will be overwritten!
    if (wasFull_b)
    {
        ++popIndex_u32;
    }
    checkIndices_v();
}

template<typename T, uint32_t MaxSize>
T& CircularBuffer<T, MaxSize>::popFront_ro()
{
    Assert(!isEmpty_b());
    T& ret_ro = items_o[popIndex_u32 % MaxSize];
    ++popIndex_u32;
    checkIndices_v();
    return ret_ro;
}

template<typename T, uint32_t MaxSize>
T& CircularBuffer<T, MaxSize>::popBack_ro()
{
    // actually pushIndex_u32 has always to be > 0 when the buffer is not empty
    // because the implementation makes sure that pushIndex_u32 >= popIndex_u32 at all times and
    // popIndex_u32 >= 0 and hence pushIndex > 0 if not empty.
    Assert(!isEmpty_b() && pushIndex_u32 > 0U);
    --pushIndex_u32;
    T& ret_ro = items_o[pushIndex_u32 % MaxSize]; // select the previous element
    checkIndices_v();
    return ret_ro;
}


template<typename T, uint32_t MaxSize>
const T& ConstCircularBufferIterator<T, MaxSize>::operator*() const
{
    Assert(arrayIndex_u32 < circularBuffer_ro.size_u32());
    return circularBuffer_ro.items_o[arrayIndex_u32 % MaxSize];
}

}
}


#endif // CIRCULARBUFFER_H_
/// @}
/// @}
