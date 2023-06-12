/*
 * Array.h
 *
 *  Created on: 10.06.2014
 *      Author: MEE_MiSch
 */

#ifndef TSC_ARRAY_H_
#define TSC_ARRAY_H_

#if 0
#include "Collection.h"
#include <cstring>

namespace core
{

typedef uint16_t ArraySize_t;

/// Forward declaration of iterator used in Array
template<typename T, ArraySize_t MaxSize>
class ArrayIterator;

/// Replaces the usual C-array implementation.
/// Uses checks to ensure the array bounds are not overwritten.
/// It also can be used as a Collection to iterate over the items.
/// For multidimensional Arrays use Array2D, Array3D,...
/// The arrays may be used as usual - integer, floating-point and pointer-arrays
/// are initialized with 0 automatically.
/// @example
/// @code{
/// for example Array<uint16_t, 3> v_arr_o;
/// for example v_arr[0] = 1U;
/// for example v_arr[1] = 2U;
/// for example v_arr[2] = 3U;
/// for example Array3D<float_32t, 10, 11, 12> v_arr3D_o;
/// for example v_arr3D_o[9][10][11] = 3.1f;
/// for example std::cout << v_arr3D_o[0][0][0] << std::endl; // should print 0
/// for example std::cout << v_arr3D_o[9][10][11] << std::endl; // should print 3.1
/// for example }
/// When writing methods that should deal with Arrays of arbitrary type or size
/// it is necessary to define the methods as template.
/// @example
/// @code{
/// template <typename T, ArraySize_t MaxSize>
/// for example  uint32_t calcSum_u32(const Array<T, MaxSize>& arr_ro)
/// for example  {
/// for example      uint32_t sum_u32 = 0UL;
/// for example      for (const typename Array<T, MaxSize>::iterator it_o = arr_ro.begin_o(); it_o != arr_ro.end_o(); ++it_o)
/// for example      {
/// for example              sum_u32 += *it_o;
/// for example      }
/// for example      return sum_u32;
/// for example   }
/// for example}



template<typename T, ArraySize_t MaxSize>
class Array
{
public:
    Array();
    ~Array();
    /// Checked writable access to the array contents.
    T& operator[](ArraySize_t index_u);
    /// Checked read-only access to the array contents.
    const T& operator[](ArraySize_t index_u) const;

    /// The actual iterator which may be used when looping
    typedef ArrayIterator<T, MaxSize> iterator;
    const iterator begin_o() const;
    const iterator end_o() const;
    /// Non-const access to the collection by iterator. Simply wraps the const version by default.
    //PRQA S 3083 2
	iterator begin_o() { return const_cast<const Array*>(this)->begin_o(); }
    /// Non-const access to end of the collection. Simply wraps the const version by default
	//PRQA S 3083 2
	iterator end_o() { return const_cast<const Array*>(this)->end_o(); }
    const iterator iteratorAt_o(ArraySize_t position_u) const;
    void init_v();

	ArraySize_t size_u() const { return MaxSize; }
    /// Copies another array into this one.
    /// @param copyFrom_ro the other array to copy data from
    /// @param copyFromStartIndex_u the first index to copy from the other array
    /// @param destStartIndex_u the first index to copy the data to in this array
    /// @param length_u the number of bytes to copy
    template<ArraySize_t SizeOther>
    void copy_v(const Array<T, SizeOther>& copyFrom_ro, ArraySize_t copyFromStartIndex_u = 0, ArraySize_t destStartIndex_u = 0,
            ArraySize_t length_u = MaxSize);

    friend class ArrayIterator<T, MaxSize> ; // let the iterator access the item-array

protected:
    // do not allow (the usually accidental) copying by operator=
    Array(const Array<T, MaxSize> &);
    Array<T, MaxSize> &operator =(const Array<T, MaxSize> &);

    // This is the plain old C-Array
    T items_ao[MaxSize];
};


/// Iterator definition for Array type.
template<typename T, ArraySize_t MaxSize>
class ArrayIterator
{
public:
    T& operator*();
    const T& operator*() const;
    bool operator !=(const ArrayIterator<T, MaxSize>& i_other_ro) const;
    const ArrayIterator<T, MaxSize>& operator++() const;
    inline ArraySize_t getIndex_u() const { return index_u; }
    /// the iterator can only be created from the outer type
    friend class Array<T, MaxSize>;
private:
    ArrayIterator(Array<T, MaxSize>* i_parentArray_po, int32_t i_startIndex);
    Array<T, MaxSize> *parentArray_po;
    mutable ArraySize_t index_u;
};

// ////////////////////////////////////////////
// Array implementation
// ////////////////////////////////////////////
// PRQA S 4054 2
template<typename T, ArraySize_t MaxSize>
Array<T, MaxSize>::Array()
{
}

template<typename T, ArraySize_t MaxSize>
Array<T, MaxSize>::~Array(void)
{
}

template<typename T, ArraySize_t MaxSize>
void Array<T, MaxSize>::init_v()
{
    if (IsIntegral<T>::value || IsPointer<T>::value || IsFloatingPoint<T>::value)
    {
        memset(items_ao, 0, sizeof(T) * MaxSize);
    }
    else
    {
        // (currently) nothing to do
    }
}

template<typename T, ArraySize_t MaxSize>
T& Array<T, MaxSize>::operator[](ArraySize_t index_u)
{
    assert(index_u < MaxSize);
    return items_ao[index_u];
}

template<typename T, ArraySize_t MaxSize>
const T& Array<T, MaxSize>::operator[](ArraySize_t index_u) const
{
    assert(index_u < MaxSize);
    return items_ao[index_u];
}

template<typename T, ArraySize_t MaxSize>
const ArrayIterator<T, MaxSize> Array<T, MaxSize>::begin_o() const
{
    // the const return value of this function ensures that only
    // the const-version of the *-operator can be called on the iterator.
    // Hence the contents of the array are not changed by the returned iterator.
    //PRQA S 3083 2
    return iterator(const_cast<Array<T, MaxSize>*>(this), 0);
}

template<typename T, ArraySize_t MaxSize>
const ArrayIterator<T, MaxSize> Array<T, MaxSize>::end_o() const
{
    // see comment above in begin_o
    //PRQA S 3083 2
    return iterator(const_cast<Array<T, MaxSize>*>(this), MaxSize);
}

template<typename T, ArraySize_t MaxSize>
const ArrayIterator<T, MaxSize> Array<T, MaxSize>::iteratorAt_o(ArraySize_t position_u) const
{
    // see comment above in begin_o
    //PRQA S 3083 2
    return iterator(const_cast<Array<T, MaxSize>*>(this), position_u);
}


template<typename T, ArraySize_t MaxSize>
template<ArraySize_t SizeOther>
void Array<T, MaxSize>::copy_v(const Array<T, SizeOther>& copyFrom_ro, ArraySize_t copyFromStartIndex_u,
        ArraySize_t destStartIndex_u, ArraySize_t length_u)
{
    assert(
            (((destStartIndex_u + length_u)) <= this->size_u())
                    && ((copyFromStartIndex_u + length_u) <= copyFrom_ro.size_u()));
    for (ArraySize_t idx_u = 0U; idx_u < length_u; idx_u++)
    {
        items_ao[idx_u + destStartIndex_u] = copyFrom_ro.items_ao[idx_u + copyFromStartIndex_u];
    }
}

// ////////////////////////////////////
// iterator implementation
// ////////////////////////////////////
template<typename T, ArraySize_t MaxSize>
T& ArrayIterator<T, MaxSize>::operator*()
{
    assert(index_u < MaxSize);
    return (parentArray_po->items_ao[index_u]);
}

template<typename T, ArraySize_t MaxSize>
const T& ArrayIterator<T, MaxSize>::operator*() const
{
    assert(index_u < MaxSize);
    return (parentArray_po->items_ao[index_u]);
}

template<typename T, ArraySize_t MaxSize>
bool ArrayIterator<T, MaxSize>::operator!=(const ArrayIterator<T, MaxSize>& i_other_ro) const
{
    return index_u != i_other_ro.index_u;
}

template<typename T, ArraySize_t MaxSize>
const ArrayIterator<T, MaxSize>& ArrayIterator<T, MaxSize>::operator++() const
{
    ++index_u; // index == SIZE is allowed here - corresponds to end-element
    return *this;
}

template<typename T, ArraySize_t MaxSize>
ArrayIterator<T, MaxSize>::ArrayIterator(Array<T, MaxSize>* i_parentArray_po, int32_t i_startIndex) :
        parentArray_po(i_parentArray_po), index_u(i_startIndex)
{
}

}
#endif
#endif /* ARRAY_H_ */
