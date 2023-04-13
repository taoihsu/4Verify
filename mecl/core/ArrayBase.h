//--------------------------------------------------------------------------
/// @file ArrayBase.h
/// @brief Contains the abstract base implementation of the array template.
///
/// This contains the major functionality for accessing memory areas with an array
/// with checking the access for validity.
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

#ifndef ARRAYBASE_H_
#define ARRAYBASE_H_

#include "MeclTypes.h"
#include "Collection.h"
#include "MeclAssert.h"
#include "Helpers.h"
#include <cstring> // for memset
namespace mecl
{
namespace core
{
// for convenience the typedef "type" or "iterator" is always named type or iterator
// PRQA S 1507 EOF
// see comment in class -> the user must ensure the given memory area and the MaxSize are consistent with each other
// and that access out of memory bounds is not possible. The usual way to use this class is by its derived class Array.
// PRQA S 3706 EOF
// For convenience reasons and to suit user expectations, operators - especially the []-operator - are allowed here.
// PRQA S 2083 EOF
// For convenience it is allowed to define templates inside the class definition
// PRQA S 2106 EOF
// There is no function implicitly overriding a base class function - but lots of wrong detected warnings here
// PRQA S 2135 EOF

/// Forward declaration of iterator used in ArrayBase
template<typename T, uint32_t MaxSize, bool_t MutableCollection>
class ConstArrayIterator;

/// Implementation of C-Array-like access to a memory area.
/// @attention The user must ensure the memory area is used within its bounds by setting MaxSize correctly.
template<typename T, uint32_t MaxSize, bool_t Mutable = true>
class ArrayBase: public Collection<T, MaxSize, ConstArrayIterator<T, MaxSize, Mutable>, Mutable>
{
public:
  friend class ArrayBase<T, MaxSize, !Mutable> ;
  typedef typename ConditionalType<Mutable, T*, const T*>::type PtrType_t;

  // this non-static member function does not change data, but gives access to data that may be changed.
  // -> turn off warning about this function not changing any data
  // PRQA S 4211 ++
  /// The actual iterator which may be used when looping
  typedef typename Collection<T, MaxSize, ConstArrayIterator<T, MaxSize, Mutable>, Mutable>::IteratorType_t iterator;
  typedef ConstArrayIterator<T, MaxSize, Mutable> const_iterator;
  /// Iterator to the first element in the array.
  virtual const_iterator begin_o() const;
  /// Iterator to the invalid element behind the array.
  virtual const_iterator end_o() const;

  /// Explicit call to (re-)initialize the area with 0.
  /// This is only executed for primitive and pointer type arrays.
  /// @param i_ArraySize_u32 actual number of elements to initialize
  void init_v(uint32_t i_ArraySize_u32);

  void init_v()
  {
    init_v(this->size_u32());
  }

  /// Explicit call to set everything to the given value using the copy assignment operator or memcpy for PODs.
  /// @param[in] i_InitValue_ro should be of the underlying type which is used in the array.
  /// @param[in] i_ArraySize_u32 Number of elements to set.
  template<typename ElemType>
  void initWith_v(const ElemType &i_InitValue_ro,
                  uint32_t i_ArraySize_u32);

  template<typename ElemType>
  void initWith_v(const ElemType &i_InitValue_ro)
  {
    initWith_v(i_InitValue_ro, this->size_u32());
  }

  void memset_v(uint8_t i_InitValue_u8,
                uint32_t i_ArraySize_u32);

  /// Explicit call to set everything bytewise to the given value (normally 0 or 0xFF).
  /// Calling this method for a non-POD element type will yield an assertion.
  /// @param i_InitValue_u8 the byte to set.
  void memset_v(uint8_t i_InitValue_u8)
  {
    memset_v(i_InitValue_u8, this->size_u32());
  }

  /// Is always false for an array.
  virtual bool_t isEmpty_b() const
  {
    return false;
  }

  /// Copies another array into this one.
  /// @param i_CopyFrom_ro the other array to copy data from
  /// @param i_CopyFromStartIndex_u32 the first index to copy from the other array
  /// @param i_DestStartIndex_u32 the first index to copy the data to in this array
  /// @param i_Length_u32 the number of bytes to copy
  template<uint32_t SizeOther, bool_t MutableOther>
  void copy_v(const ArrayBase<T, SizeOther, MutableOther>& i_CopyFrom_ro,
              uint32_t i_CopyFromStartIndex_u32,
              uint32_t i_DestStartIndex_u32,
              uint32_t i_Length_u32);

  /// Copies another array completely into this one.
  /// @param i_CopyFrom_ro the other array to copy data from
  template<bool_t MutableOther>
  void copyAll_v(const ArrayBase<T, MaxSize, MutableOther>& i_CopyFrom_ro);

  /// Calls the memcmp function on the contents of this array and the compared array.
  /// In case of multi-dimensional arrays the first non-null value or 0 is returned.
  /// @param i_CompareWith_ro the array to compare this array with.
  template<bool_t MutableOther>
  sint32_t memcmp_s32(const ArrayBase<T, MaxSize, MutableOther>& i_CompareWith_ro) const;

  friend class ConstArrayIterator<T, MaxSize, Mutable> ; // let the iterator access the item-array

  // The functions should be inlined for performance reasons
  // Hence the warning for and code insight of the class are
  // temporarily disabled.
  // PRQA S 2134 ++
  // PRQA S 2106 ++
  /// Checked writable access to the array contents - if Mutable - otherwise readonly.
  typename ConditionalType<Mutable, T&, const T&>::type operator[](uint32_t index_u32)
  {
    Assert(index_u32 < this->size_u32());
    // PRQA S 4024 1 // it is intended to give direct access to the contents here
    return memArea_po[index_u32];
  }
  // PRQA S 4211 --
  /// Checked read-only access to the array contents.
  const T& operator[](uint32_t index_u32) const
  {
    Assert(index_u32 < this->size_u32());
    return memArea_po[index_u32];
  }
  /// Specialized iterator for a given position. Should only be used internally.
  const_iterator iteratorAt_o(uint32_t position_u32) const
  {
    // see comment in implementation of begin_o
    return const_iterator(this, position_u32);
  }

  /// Returns an immutable const reference of this ArrayBase object
  const ArrayBase<T, MaxSize, false>& asConst_ro() const
  {
    // This works because the difference with the "Mutable" flag is the implementation of the iterator.
    // The non-const iterator is derived from the const version, hence the non-const can always be casted back
    // to the const version, but not the other way.
    return reinterpret_cast<const ArrayBase<T, MaxSize, false>&>(*this);
  }

  // PRQA S 2106 --
  // PRQA S 2134 --

protected:
  /// Initializes the memory access.
  /// @param i_MemArea_po points to the beginning of the memory area
  explicit ArrayBase(PtrType_t i_MemArea_po);

  /// Initializes the memory access with constant initial data.
  /// @param i_MemArea_po points to the beginning of the memory area
  /// @param i_ArraySize_u32 the actual length of the array
  /// @param i_MemInit_po pointer to constant data. This should be at least of the same size as the used memory area.
  ArrayBase(PtrType_t i_MemArea_po,
            uint32_t i_ArraySize_u32,
            const T* const i_MemInit_po);

  /// Destructor for inheritance - does nothing.
  virtual ~ArrayBase();
  /// Copying of ArrayBase is allowed as there are no actual resources copied or freed here.
  /// It is also only allowed to copy an array of the same MaxSize.
  ArrayBase(const ArrayBase<T, MaxSize, Mutable>& i_CopyFrom_ro);
  /// Copy and assignment of ArrayBase is allowed as there are no actual resources copied or freed here.
  /// It is also only allowed to assign an array of the same MaxSize.
  ArrayBase& operator=(const ArrayBase& i_CopyFrom_ro);

  /// Helper method for derived classes to access the memory area
  PtrType_t getPointerAccess_x() // PRQA S 4211,4267 // get access to internal data. Function should be non-const for non-const access.
  {
    return memArea_po;
  }
  const T* getConstPointerAccess_x() const
  {
    return memArea_po;
  }

private:
  ArrayBase();

  /// This is the plain old C-Array.
  volatile PtrType_t memArea_po;

};

// remove QACPP warnings about implicit copy constructors
// as members will not be copied here other than in the implicit implementation
// hence no implicit implementation is needed.
// PRQA S 2113 ++
// PRQA S 1702 ++
/// Iterator definition for ArrayBase type.
template<typename T, uint32_t MaxSize, bool_t MutableCollection>
class ConstArrayIterator
{
public:
  /// const version working on const array types which returns
  /// read-only access to the internal data.
  const T& operator*() const;
  /// Compares two iterators for equality.
  /// This is for JSF compliance.
  bool_t operator ==(const ConstArrayIterator<T, MaxSize, MutableCollection>& i_Other_ro) const;
  /// Compares two iterators for non-equality.
  /// This is used (only) by the iterator check for the end in for loops.
  bool_t operator !=(const ConstArrayIterator<T, MaxSize, MutableCollection>& i_Other_ro) const;
  /// This 'const' version of the 'operator++' only affects the
  /// mutable index, but does not change the internal data of the array.
  const ConstArrayIterator<T, MaxSize, MutableCollection>& operator++() const;
  /// This is actually for library-internal use only.
  /// Returns the current index position of the iterator.
  uint32_t getIndex_u32() const;
  /// the iterator can only be created from the outer type
  friend class ArrayBase<T, MaxSize, MutableCollection> ;
protected:
  ConstArrayIterator();
  ConstArrayIterator(const ArrayBase<T, MaxSize, MutableCollection>* i_ParentArray_po,
                     uint32_t i_StartIndex);
private:
  /// Pointer to the underlying array.
  const ArrayBase<T, MaxSize, MutableCollection>* parentArray_po;
  /// This is mutable to allow a constant operator++ to.
  /// This allows const iterators that can only access the content readonly
  /// but still can be iterated.
  mutable uint32_t index_u32;
};

// PRQA S 2113 -- // PRQA S 1702 --

// Helper macros to be used for automatic array initialization
// PRQA S 1063 ++ // wrong detection of unused class in QACPP
// PRQA S 2640 ++ // should not be relevant here for static _const_ objects
// PRQA S 1514 ++ // is not relevant here as well
// PRQA S 2106 ++ // for better readability the implementation is just defined in the short template definitions
// PRQA S 2122 ++ // It is OK here to define public static functions
/// Contains helper templates for array implementation.
namespace array
{

struct SubArrayFunctions
{
  /// Initialize a the elements of this array that are itself array elements.
  /// This specialization calls the actual init method.
  template<typename T2, uint32_t MaxSize2>
  static void initSub_v(ArrayBase<T2, MaxSize2>* const v_Arr_po)
  {
    v_Arr_po->init_v();
  }
  /// Initialize a the elements of this array that are itself array elements.
  /// This is an empty dummy-implementation for non-array types.
  static void initSub_v(void* const)
  {
    // other complex type: nothing to do (yet)
  }
  /// Initialize a the elements of this array that are itself array elements.
  /// This specialization calls the actual init method.
  template<typename T2, uint32_t MaxSize2>
  static void memset_v(ArrayBase<T2, MaxSize2>* const v_Arr_po,
                       uint8_t i_InitValue_u8,
                       uint32_t i_ArraySize_u32)
  {
    v_Arr_po->memset_v(i_InitValue_u8, i_ArraySize_u32);
  }
  /// Initialize a the elements of this array that are itself array elements.
  /// This is an empty dummy-implementation for non-array types.
  static void memset_v(void * const,
                       uint8_t,
                       uint32_t)
  {
    Assert(false); // memset must not be called for non-POD types!
  }
  template<typename T, uint32_t MaxSize, bool_t MutableCollection>
  static sint32_t memcmp_s32(const ArrayBase<T, MaxSize, MutableCollection>* const v_Arr_po,
                             const ArrayBase<T, MaxSize, MutableCollection>* const v_CopyArr_po)
  {
    return v_Arr_po->memcmp_s32(*v_CopyArr_po);
  }
  static sint32_t memcmp_s32(void * const,
                             const void* const)
  {
    Assert(false);
    return 0;
  }

};

}
// PRQA S 1063 -- // PRQA S 2640 -- // PRQA S 1514 -- // PRQA S 2106 --

// ////////////////////////////////////////////
// ArrayBase implementation
// ////////////////////////////////////////////

template<typename T, uint32_t MaxSize, bool_t Mutable>
ArrayBase<T, MaxSize, Mutable>::ArrayBase(PtrType_t i_MemArea_po)
    : Collection<T, MaxSize, ConstArrayIterator<T, MaxSize, Mutable>, Mutable>(),
      memArea_po(i_MemArea_po)
{
}

template<typename T, uint32_t MaxSize, bool_t Mutable>
ArrayBase<T, MaxSize, Mutable>::ArrayBase(PtrType_t i_MemArea_po,
                                          uint32_t i_ArraySize_u32,
                                          const T* const i_MemInit_po)
    : Collection<T, MaxSize, ConstArrayIterator<T, MaxSize, Mutable>, Mutable>(),
      memArea_po(i_MemArea_po)
{
  Assert(i_ArraySize_u32 <= MaxSize);
  memcpy(i_MemArea_po, i_MemInit_po, sizeof(T) * i_ArraySize_u32);
}

template<typename T, uint32_t MaxSize, bool_t Mutable>
ArrayBase<T, MaxSize, Mutable>::ArrayBase(const ArrayBase<T, MaxSize, Mutable>& i_CopyFrom_ro)
    : Collection<T, MaxSize, ConstArrayIterator<T, MaxSize, Mutable>, Mutable>(),
      memArea_po(i_CopyFrom_ro.memArea_po)
{
}

template<typename T, uint32_t MaxSize, bool_t Mutable>
ArrayBase<T, MaxSize, Mutable>::~ArrayBase()
{
}

template<typename T, uint32_t MaxSize, bool_t Mutable>
ArrayBase<T, MaxSize, Mutable>& ArrayBase<T, MaxSize, Mutable>::operator=(const ArrayBase<T, MaxSize, Mutable>& i_CopyFrom_ro)
{
  memArea_po = i_CopyFrom_ro.memArea_po;
  return *this;
}

// turn off warning about const_cast as data will be returned const and will not be modified
// PRQA S 2121 ++ // wrong detection by QACPP
template<typename T, uint32_t MaxSize, bool_t Mutable>
typename ArrayBase<T, MaxSize, Mutable>::const_iterator ArrayBase<T, MaxSize, Mutable>::begin_o() const
{
  return const_iterator(this, 0);
}

template<typename T, uint32_t MaxSize, bool_t Mutable>
typename ArrayBase<T, MaxSize, Mutable>::const_iterator ArrayBase<T, MaxSize, Mutable>::end_o() const
{
  return const_iterator(this, this->size_u32());
}
// PRQA S 2121 --

template<typename T, uint32_t MaxSize, bool_t Mutable>
template<uint32_t SizeOther, bool_t MutableOther>
void ArrayBase<T, MaxSize, Mutable>::copy_v(const ArrayBase<T, SizeOther, MutableOther>& i_CopyFrom_ro,
                                            uint32_t i_CopyFromStartIndex_u32,
                                            uint32_t i_DestStartIndex_u32,
                                            uint32_t i_Length_u32)
{
  Assert(
      (((i_DestStartIndex_u32 + i_Length_u32)) <= this->size_u32()) && ((i_CopyFromStartIndex_u32 + i_Length_u32)
          <= i_CopyFrom_ro.size_u32()));
  for (uint32_t idx_u32 = 0U; idx_u32 < i_Length_u32; ++idx_u32)
  {
    this->copyItem_v(memArea_po[idx_u32 + i_DestStartIndex_u32],
        i_CopyFrom_ro.memArea_po[idx_u32 + i_CopyFromStartIndex_u32]);
    // This is what it actually does: memArea_po[idx_u32 + destStartIndex_u32]
    //                                       ## = copyFrom_ro.memArea_po[idx_u32 + copyFromStartIndex_u32];
  }
}

namespace array
{
/// Helper template to determine if the given type T is of an arbitrary type ArrayBase<T2,MaxSize2>.
/// This template depends on uint32_t and uint8_t being of different sizes.
template<typename T>
class IsArray // PRQA S 2195 // PRQA S 2108 // cannot start with public here - value uses typedefs etc.
{
private:
  enum Sizes_e
  {
    YesSize = 1U, NoSize = 2U
  };
  typedef uint8_t IsAnArray_t[YesSize]; // PRQA S 2411 // typedef used internally only
  typedef uint8_t IsNotAnArray_t[NoSize]; // PRQA S 2411 // typedef used internally only

  static IsNotAnArray_t& testCall(const void* const); // declared, but not defined

  template<typename T2, uint32_t MaxSize2>
  static IsAnArray_t& testCall(const ArrayBase<T2, MaxSize2>* const); // declared, but not defined

public:
  // PRQA S 2640 4 // template has a static - but const - object
  // PRQA S 1504 3 // PRQA S 1514 3 // PRQA S 1533 3 // irrelevant here (only used in one translation unit)
  // PRQA S 4636 2 // program won't terminate here
  /// The actual result of the IsArray<T> check.
  static const bool value = ((sizeof(testCall(static_cast<T*>(0)))) == (sizeof(IsAnArray_t)));
};

/// Common copy template for items that are neither POD nor derived from ArrayBase.
template<typename T, uint32_t MaxSize, bool_t Mutable, bool_t MutableOther, bool_t IsPodCheck, bool_t IsArrayCheck>
struct CopyAll
{
  static void copyAll_v(T* const memArea_px,
                          const T* const copyFrom_px,
                          uint32_t v_Length_u32)
  {
    if (Mutable)
    {
      for (uint32_t v_Idx_u32 = 0U; v_Idx_u32 < v_Length_u32; ++v_Idx_u32)
      {
        (memArea_px[v_Idx_u32]).operator=(copyFrom_px[v_Idx_u32]);
      }
    }
  }
};

/// Template specialization for PODs
template<typename T, uint32_t MaxSize, bool_t MutableOther>
struct CopyAll<T, MaxSize, true, MutableOther, true, false>
{
  static void copyAll_v(T* const memArea_px,
                        const T* const copyFrom_px,
                        uint32_t v_Length_u32)
  {
    memcpy(memArea_px, copyFrom_px, v_Length_u32 * sizeof(T));
  }
};

/// Template specialization for Arrays.
template<typename T, uint32_t MaxSize, bool_t MutableOther>
struct CopyAll<T, MaxSize, true, MutableOther, false, true>
{
  /// delegates to copyAll_v for each item
  static void copyAll_v(ArrayBase<T, MaxSize, true>& i_This_ro,
                        const ArrayBase<T, MaxSize, MutableOther>& i_CopyFrom_ro,
                        uint32_t v_Length_u32)
  {
    for (uint32_t v_Idx_u32 = 0U; v_Idx_u32 < v_Length_u32; ++v_Idx_u32)
    {
      i_This_ro[v_Idx_u32].copyAll_v(i_CopyFrom_ro[v_Idx_u32]);
    }
  }
};

/// Common initialization template for items that are neither POD nor derived from ArrayBase.
template<typename T, uint32_t MaxSize, bool_t Mutable, bool_t IsPodCheck, bool_t IsPrimitiveCheck, bool_t IsArrayCheck>
struct InitElem
{
  static void initElem_v(ArrayBase<T, MaxSize, Mutable>& i_Array_ro,
                         typename ConstRefType<T>::type i_Item_ro,
                         uint32_t v_ArraySize_u32)
  {
    if (Mutable)
    {
      for (uint32_t v_idx_u32 = 0U; v_idx_u32 < v_ArraySize_u32; ++v_idx_u32)
      {
        i_Array_ro[v_idx_u32].operator=(i_Item_ro);
      }
    }
  }
};

/// Template specialization for PODs
template<typename T, uint32_t MaxSize, bool_t Mutable>
struct InitElem<T, MaxSize, Mutable, true, false, false>
{
  static void initElem_v(ArrayBase<T, MaxSize, Mutable>& i_Array_ro,
                         typename ConstRefType<T>::type i_Item_ro,
                         uint32_t v_ArraySize_u32)
  {
    if (Mutable)
    {
      for (uint32_t v_idx_u32 = 0U; v_idx_u32 < v_ArraySize_u32; ++v_idx_u32)
      {
        memcpy(&i_Array_ro[v_idx_u32], i_Item_ro, sizeof(T));
      }
    }
  }
};

/// Template specialization for primitive Types
template<typename T, uint32_t MaxSize, bool_t Mutable>
struct InitElem<T, MaxSize, Mutable, true, true, false>
{
  static void initElem_v(ArrayBase<T, MaxSize, Mutable>& i_Array_ro,
                         T i_Item_ro,
                         uint32_t v_ArraySize_u32)
  {
    if (Mutable)
    {
      for (uint32_t v_idx_u32 = 0U; v_idx_u32 < v_ArraySize_u32; ++v_idx_u32)
      {
        i_Array_ro[v_idx_u32] = i_Item_ro;
      }
    }
  }
};

/// Template specialization for Arrays.
template<typename T, uint32_t MaxSize, bool_t Mutable>
struct InitElem<T, MaxSize, Mutable, false, false, true>
{
  template<typename TElem>
  static void initElem_v(ArrayBase<T, MaxSize, Mutable>& i_Array_ro,
                         const TElem& i_Item_ro,
                         uint32_t v_ArraySize_u32)
  {
    if (Mutable)
    {
      for (uint32_t v_idx_u32 = 0U; v_idx_u32 < v_ArraySize_u32; ++v_idx_u32)
      {
        i_Array_ro[v_idx_u32].initWith_v(i_Item_ro);
      }
    }
  }
};
}

template<typename T, uint32_t MaxSize, bool_t Mutable>
template<bool_t MutableOther>
void ArrayBase<T, MaxSize, Mutable>::copyAll_v(const ArrayBase<T, MaxSize, MutableOther>& i_CopyFrom_ro)
{
  Assert(this->size_u32() == i_CopyFrom_ro.size_u32());
  array::CopyAll<T, MaxSize, Mutable, MutableOther, IsSimplePod<T>::value, array::IsArray<T>::value>::copyAll_v(
      memArea_po, i_CopyFrom_ro.memArea_po, this->size_u32());
}

template<typename T, uint32_t MaxSize, bool_t Mutable>
template<typename ElemType>
void ArrayBase<T, MaxSize, Mutable>::initWith_v(const ElemType &i_InitValue_ro,
                                                uint32_t i_ArraySize_u32)
{
  array::InitElem<T, MaxSize, Mutable, IsSimplePod<T>::value, IsPrimitive<T>::value, array::IsArray<T>::value>::initElem_v(
      *this, i_InitValue_ro, i_ArraySize_u32);
}

template<typename T, uint32_t MaxSize, bool_t Mutable>
template<bool_t MutableOther>
sint32_t ArrayBase<T, MaxSize, Mutable>::memcmp_s32(const ArrayBase<T, MaxSize, MutableOther>& i_CompareWith_ro) const
{
  sint32_t v_Result_s32 = this->size_u32() - i_CompareWith_ro.size_u32();
  if (0 == v_Result_s32)
  {
	if (IsSimplePod<T>::value)
	{
	  v_Result_s32 = memcmp(memArea_po, i_CompareWith_ro.memArea_po, this->size_u32() * sizeof(T));
	}
	else
	{
	  uint32_t v_Idx_u32 = 0U;
	  bool_t v_HasResult = false;
	  const uint32_t end_u32 = this->size_u32();
	  while ((v_Idx_u32 < end_u32) && (!v_HasResult))
	  {
	    v_Result_s32 = array::SubArrayFunctions::memcmp_s32(&memArea_po[v_Idx_u32],
	  	  &i_CompareWith_ro.memArea_po[v_Idx_u32]);
  	    v_Idx_u32++;
	    v_HasResult = (0U != v_Result_s32);
	  }
	}
  }
  return v_Result_s32;
}

// This non-const member function actually does modify data (in memset and also in initSub_v).
// Hence turn off the unnecessary warning by QACPP.
// PRQA S 4211 ++
/// Initialize the array: sets everything to 0 for basic types only (int, float, pointer)
/// MISRA: this is the MISRA-workaround for using a virtual override for init_v: use templates instead.
/// This is actually a bit harder to read, but the advantage is that multidimensional arrays are a lot
/// easier to implement.
template<typename T, uint32_t MaxSize, bool_t Mutable>
void ArrayBase<T, MaxSize, Mutable>::init_v(uint32_t i_ArraySize_u32)
{
  // The template evaluation of IsPrimitive and IsArray result depends on the given type during compile time.
  // This is as intended here and QACPP cannot determine the right cause here.
  // Hence the warning of constant expression being checked is removed here
  // PRQA S 4090 ++
  if (IsPrimitive<T>::value) // set int/float or pointer to 0
  {
    memset_v(0U, i_ArraySize_u32);
  }
  else
  {
    for (uint32_t idx = 0U; idx < i_ArraySize_u32; ++idx)
    {
      array::SubArrayFunctions::initSub_v(&memArea_po[idx]);
    }
  }
  // PRQA S 4090 --
}

template<typename T, uint32_t MaxSize, bool_t Mutable>
void ArrayBase<T, MaxSize, Mutable>::memset_v(uint8_t i_InitValue_u8,
                                              uint32_t i_ArraySize_u32)
{
  // unfortunately we cannot call StaticAssert here as we would have compile errors
  // even if this function wasn't called
  // We must ensure however that the memset does not overwrite a virtual function table here
  // The template evaluation of IsPrimitive and IsArray result depends on the given type during compile time.
  // This is as intended here and QACPP cannot determine the right cause here.
  // Hence the warning of constant expression being checked is removed here
  // PRQA S 4090 ++
  if (IsSimplePod<T>::value)
  {
    memset(memArea_po, i_InitValue_u8, sizeof(T) * i_ArraySize_u32);
  }
  else
  {
    for (uint32_t v_idx_u32 = 0U; v_idx_u32 < i_ArraySize_u32; ++v_idx_u32)
    {
      array::SubArrayFunctions::memset_v(&memArea_po[v_idx_u32], i_InitValue_u8, i_ArraySize_u32);
    }
  }
  // PRQA S 4090 --
}

// PRQA S 4211 --

// ////////////////////////////////////
// iterator implementation
// ////////////////////////////////////

template<typename T, uint32_t MaxSize, bool_t MutableCollection>
const T& ConstArrayIterator<T, MaxSize, MutableCollection>::operator*() const
{
  Assert(index_u32 < parentArray_po->size_u32());
  return (parentArray_po->memArea_po[index_u32]);
}

template<typename T, uint32_t MaxSize, bool_t MutableCollection>
bool_t ConstArrayIterator<T, MaxSize, MutableCollection>::operator==(const ConstArrayIterator<T, MaxSize,
    MutableCollection>& i_Other_ro) const
{
  return index_u32 == i_Other_ro.index_u32;
}

template<typename T, uint32_t MaxSize, bool_t MutableCollection>
bool_t ConstArrayIterator<T, MaxSize, MutableCollection>::operator!=(const ConstArrayIterator<T, MaxSize,
    MutableCollection>& i_Other_ro) const
{
  return !(this->operator==(i_Other_ro));
}

// This special const implementation returns a const reference - which is fine here
// PRQA S 2622 6
template<typename T, uint32_t MaxSize, bool_t MutableCollection>
const ConstArrayIterator<T, MaxSize, MutableCollection>& ConstArrayIterator<T, MaxSize, MutableCollection>::operator++() const
{
  ++index_u32; // index == SIZE is allowed here - corresponds to end-element
  return *this;
}

template<typename T, uint32_t MaxSize, bool_t MutableCollection>
ConstArrayIterator<T, MaxSize, MutableCollection>::ConstArrayIterator(const ArrayBase<T, MaxSize, MutableCollection>* i_ParentArray_po,
                                                                      uint32_t i_StartIndex)
    : parentArray_po(i_ParentArray_po),
      index_u32(i_StartIndex)
{
}

template<typename T, uint32_t MaxSize, bool_t MutableCollection>
uint32_t ConstArrayIterator<T, MaxSize, MutableCollection>::getIndex_u32() const
{
  return index_u32;
}
}
}

// PRQA S 2122 --

#endif // ARRAYBASE_H_
