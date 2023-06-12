#ifndef TSC_ARRAY_LIST_H
#define TSC_ARRAY_LIST_H


#if 0

#include <stdint.h>
#include <assert.h>
#include <string.h>
#include "Helpers.h"
#include "Array.h"

//PRQA S 3083 ++

namespace core
{
typedef ArraySize_t ArrayListSize_t;

template<typename T, ArrayListSize_t MaxSize>
class ArrayListIterator;

/// As alternative to the simple Array, the ArrayList counts the number
/// of data elements that has been added to the ArrayList.
/// It actually wraps the Array class adding some list operations and
/// restricting access to the array contents.
template <typename T, ArrayListSize_t MaxSize>
class ArrayList
{
public:
	ArrayList(void);
	~ArrayList(void);

    // Constant access to the last element of the array list.
    const T& back() const { return items_o[currentSize_u-1]; }
    // Non-constant access to the last element.
    T& back() { return items_o[currentSize_u-1]; }
    // Constant access to the first element of the array list.
    const T& front() const { return items_o[0]; }
    // Non-constant access to the first element.
    T& front() { return items_o[0]; }
    // Extend the current size of the array list by one element. 
    T& addItem();

    // adds an element to the end of the list
    void pushBack_v(T i_item_ro);
    T& popBack_ro();
    ArrayListSize_t size_u() const { return currentSize_u; }

    void clear_v() { items_o.init_v(); currentSize_u = 0U; }
    /// Checked read-only access to the array contents.
    /// The index has to be in the range of the list.
    /// Write-access by [] is not allowed with ArrayList.
    const T& operator[](ArrayListSize_t index_u) const;
    // Allow write-access
    T& operator[](ArrayListSize_t index_u);// { const_cast<const ArrayList*>(this)->operator[](index_u); }

    /// The actual iterator which may be used when looping - the same used as for Array.
    typedef ArrayListIterator<T, MaxSize> iterator;
    const iterator begin_o() const;
    const iterator end_o() const;
    /// Non-const access to the collection by iterator. Simply wraps the const version by default.
	iterator begin_o() { return const_cast<const ArrayList*>(this)->begin_o(); }
    /// Non-const access to end of the collection. Simply wraps the const version by default.
	iterator end_o() { return const_cast<const ArrayList*>(this)->end_o(); }
	friend class ArrayListIterator<T, MaxSize>;

	/// Safe access to the underlying array.
	inline const Array<T,MaxSize>& getArray_o() const { return items_o; }
    
    // Copy function
    void copy_v(const ArrayList<T, MaxSize>& copyFrom_ro, ArrayListSize_t length_u);

 protected:
     // do not allow (the usually accidental) copying by operator=
     ArrayList(const ArrayList<T, MaxSize> &);
     ArrayList<T, MaxSize> &operator =(const ArrayList<T, MaxSize> &);
     /// the current number of elements that were added using pushBack_v.
     /// Should not exceed the maximum size.
  	 ArrayListSize_t currentSize_u;
  	 // Wrap the array
  	 Array<T,MaxSize> items_o;
};

template <typename T, ArrayListSize_t MaxSize>
struct PtrArrayList : public ArrayList<T*,MaxSize>
{
};

/// Iterator definition for ArrayList type.
/// This actually only wraps the ArrayIterator and adds an additional check not to access
/// beyond the current list size.
template<typename T, ArrayListSize_t MaxSize>
class ArrayListIterator
{
public:
	inline T& operator*() { assert(arrayIterator_ro.getIndex_u() < arrayList_ro.currentSize_u); return *const_cast<ArrayIterator<T,MaxSize>&>(arrayIterator_ro);}
	inline const T& operator*() const { assert(arrayIterator_ro.getIndex_u() < arrayList_ro.currentSize_u); return *arrayIterator_ro; }
	//PRQA S 3336 1
	inline bool operator !=(const ArrayListIterator<T, MaxSize>& i_other_ro) const { return arrayIterator_ro != i_other_ro.arrayIterator_ro; }
	inline const ArrayIterator<T, MaxSize>& operator++() const { return (++arrayIterator_ro); }
    /// the iterator can only be created from the outer type
    friend class ArrayList<T, MaxSize>;
private:
	ArrayListIterator(const ArrayList<T, MaxSize> &i_arrayList_ro, const ArrayIterator<T,MaxSize> &i_arrayIterator_ro) : arrayList_ro(i_arrayList_ro), arrayIterator_ro(i_arrayIterator_ro) {}
	const ArrayList<T, MaxSize> &arrayList_ro;
	const ArrayIterator<T,MaxSize> arrayIterator_ro;
};


// ////////////////////////////////////////////
// ArrayList implementation
// ////////////////////////////////////////////
template <typename T, ArrayListSize_t MaxSize>
ArrayList<T, MaxSize>::ArrayList()
	: currentSize_u(0U)
{
}

template <typename T, ArrayListSize_t MaxSize>
ArrayList<T, MaxSize>::~ArrayList(void)
{
}

template <typename T, ArrayListSize_t MaxSize>
const typename ArrayList<T, MaxSize>::iterator ArrayList<T, MaxSize>::begin_o() const
{ 
	return typename ArrayListIterator<T,MaxSize>::ArrayListIterator(*this, items_o.begin_o());
}

template <typename T, ArrayListSize_t MaxSize>
const typename ArrayList<T, MaxSize>::iterator ArrayList<T, MaxSize>::end_o() const
{ 
	return typename ArrayListIterator<T,MaxSize>::ArrayListIterator(*this, items_o.iteratorAt_o(currentSize_u));
}


template <typename T, ArrayListSize_t MaxSize>
const T& ArrayList<T, MaxSize>::operator[](ArrayListSize_t index_u) const
{
    assert(index_u < currentSize_u);
    return items_o[index_u];
}

template <typename T, ArrayListSize_t MaxSize>
T& ArrayList<T, MaxSize>::operator[](ArrayListSize_t index_u)
{
    assert(index_u < currentSize_u);
    return items_o[index_u];
}

template <typename T, ArrayListSize_t MaxSize>
T& ArrayList<T, MaxSize>::addItem()
{
    assert(currentSize_u != MaxSize);
	ArrayListSize_t currentGetPosition_u = currentSize_u;
	++currentSize_u;
	return items_o[currentGetPosition_u];
}

template <typename T, ArrayListSize_t MaxSize>
void ArrayList<T, MaxSize>::pushBack_v(T i_item_ro)
{
    assert(currentSize_u != MaxSize);
	ArrayListSize_t currentInsertPosition_u = currentSize_u;
	//PRQA S 2088 1
	items_o[currentInsertPosition_u] = i_item_ro;
	++currentSize_u;
}

template <typename T, ArrayListSize_t MaxSize>
T& ArrayList<T, MaxSize>::popBack_ro()
{
    assert(currentSize_u != 0U);
    --currentSize_u;
    return items_o[currentSize_u];
}

template<typename T, ArrayListSize_t MaxSize>
void ArrayList<T, MaxSize>::copy_v(const ArrayList<T, MaxSize>& copyFrom_ro, ArrayListSize_t length_u)
{
    assert(length_u <= MaxSize);
    // Works whether length_u is greater/smaller than this->size_u()
    for (ArrayListSize_t idx_u = 0U; idx_u < length_u; idx_u++)
    {
        //PRQA S 2088 1
        items_o[idx_u] = copyFrom_ro.items_o[idx_u];
    }
    // Adjust currentSize_u
    currentSize_u = length_u;
}

}
#endif

#endif // ARRAY_LIST_H
