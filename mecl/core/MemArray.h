//--------------------------------------------------------------------------
/// @file MemArray.h
/// @brief Contains the implementation to directly access
/// memory regions.
///
/// The MemArray directly operates on the RAM with checks included
/// to operate only on defined memory regions.
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

#ifndef MEMARRAY_H_
#define MEMARRAY_H_

#include "ArrayBase.h"
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
const uint32_t MEM_ARRAY_MAXSIZE = 0xFFFFFFFFUL;

/// Implementation of C-Array-like access to a memory area.
/// @attention The user must ensure the memory area is used within its bounds by setting i_MaxSize_u32 in the constructor correctly.
template<typename T, uint32_t MaxSize = MEM_ARRAY_MAXSIZE, bool_t Mutable = true>
class MemArray: public ArrayBase<T, MaxSize, Mutable>
{
public:
  typedef typename ConditionalType<Mutable, T*, const T*>::type PtrType_t;
  /// Initializes the memory access.
  /// @param i_MemArea_po points to the beginning of the memory area
  /// @param i_MaxSize_u32 real size of the accessed memory area (in number of T-items)
  MemArray(PtrType_t i_MemArea_po, uint32_t i_MaxSize_u32);
  /// Initializes the memory access with constant initial data.
  /// @param i_MemArea_po points to the beginning of the memory area
  /// @param i_MaxSize_u32 real size of the accessed memory area (in number of T-items)
  /// @param i_MemAreaInit_po pointer to constant data. This should be at least of the same size as the used memory area.
  MemArray(PtrType_t i_MemArea_po, uint32_t i_MaxSize_u32, const T* const i_MemAreaInit_po);
  /// Copying of MemArray is allowed as there are no actual resources copied or freed here.
  /// It is also only allowed to copy an array of the same size.
  MemArray(const MemArray<T, MaxSize, Mutable>& i_CopyFrom_ro);
  /// Destructor - does nothing.
  virtual ~MemArray() {}
  /// Copying of MemArray is allowed as there are no actual resources copied or freed here.
  /// It is also only allowed to copy an array of the same size.
  MemArray<T, MaxSize, Mutable>& operator= (const MemArray<T, MaxSize, true>& i_CopyFrom_ro);

  MemArray<T, MaxSize, Mutable>& operator= (const MemArray<T, MaxSize, false>& i_CopyFrom_ro);

  virtual uint32_t size_u32() const { return arraySize_u32;}

private:
  MemArray();
  uint32_t arraySize_u32;
};

// ////////////////////////////////////////////
// MemArray implementation
// ////////////////////////////////////////////

template<typename T, uint32_t MaxSize, bool_t Mutable>
MemArray<T, MaxSize, Mutable>::MemArray(PtrType_t i_MemArea_po, uint32_t i_MaxSize_u32)
: ArrayBase<T, MaxSize, Mutable>(i_MemArea_po)
, arraySize_u32(i_MaxSize_u32)
{
}

template<typename T, uint32_t MaxSize, bool_t Mutable>
MemArray<T, MaxSize, Mutable>::MemArray(PtrType_t i_MemArea_po, uint32_t i_MaxSize_u32, const T* const i_MemAreaInit_po)
: ArrayBase<T, MaxSize, Mutable>(i_MemArea_po, i_MaxSize_u32, i_MemAreaInit_po)
, arraySize_u32(i_MaxSize_u32)
{
}

template<typename T, uint32_t MaxSize, bool_t Mutable>
MemArray<T, MaxSize, Mutable>::MemArray(const MemArray<T, MaxSize, Mutable>& i_CopyFrom_ro)
: ArrayBase<T,MaxSize,Mutable>(i_CopyFrom_ro)
, arraySize_u32(i_CopyFrom_ro.arraySize_u32)
{
}

template<typename T, uint32_t MaxSize, bool_t Mutable>
MemArray<T, MaxSize, Mutable>& MemArray<T,MaxSize, Mutable>::operator= (const MemArray<T, MaxSize, true>& i_CopyFrom_ro)
{
  StaticAssert(Mutable, "MemArray must be mutable on left side of assignment operator");
  arraySize_u32 = i_CopyFrom_ro.size_u32();
  ArrayBase<T, MaxSize>::operator=(i_CopyFrom_ro);
  return *this;
}

template<typename T, uint32_t MaxSize, bool_t Mutable>
MemArray<T, MaxSize, Mutable>& MemArray<T,MaxSize, Mutable>::operator= (const MemArray<T, MaxSize, false>& i_CopyFrom_ro)
{
  StaticAssert(Mutable, "MemArray must be mutable on left side of assignment operator");
  Assert(arraySize_u32 >= i_CopyFrom_ro.size_u32());
  ArrayBase<T, MaxSize>::copy_v(i_CopyFrom_ro, 0U, 0U, i_CopyFrom_ro.size_u32());
  arraySize_u32 = i_CopyFrom_ro.size_u32();
  return *this;
}

}
}
#endif // MEMARRAY_H_
