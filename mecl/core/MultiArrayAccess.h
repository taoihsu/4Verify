//--------------------------------------------------------------------------
/// @file MultiArrayAccess.h
/// @brief Contains the implementation to access memory regions as a multidimensional array.
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

#ifndef MECL_COLL_MULTIARRAYACCESS_H_
#define MECL_COLL_MULTIARRAYACCESS_H_

#include "ArrayBase.h"

namespace mecl
{
namespace core
{
// for templates it is allowed to define functions inside the class definition
// PRQA S 2106 EOF
// As the classes defined here do not introduce additional data, the copy-constructor, assignment operator
// and virtual destructor are left out.
// Static const data members cannot be defined in cpp here
// PRQA S 1504, 1514 EOF
// The definition of ... maybe delayed in this path - cannot be here. All is in order.
// PRQA S 1065 EOF
// Overloading an operator is here allowed ([] for array)
// PRQA S 2083 EOF
// Unsigned arithmetic is used here only with uint32_t and only with + and * - this should not pose any problems
// PRQA S 3084 EOF
// static objects here are const
// PRQA S 2640 EOF
// StaticAssert should always compile - hence used expressions in them are never false
// PRQA S 3272 EOF
// These are all detected falsely (all functions do have the virtual keyword that override base class functions
// especially destructors).
// PRQA S 2135 EOF
// Explicit copy constructors are the same as the implicit ones here.
// PRQA S 2634 EOF
/// Access to arrays in a multidimensional manner with index checks.
/// @tparam T the underlying elementary type
/// @tparam Size1 size of the first dimension
/// @tparam Size2 size of the 2nd dimension
/// @tparam Size3 size of the 3rd dimension
/// @tparam Size4 size of the 4th dimension
/// @tparam Mutable whether the content is mutable or not. Default is mutable.
/// The class MultiArrayAccessBase may be used if non-const access is desired.
template<typename T, uint32_t Size1, uint32_t Size2 = 1U, uint32_t Size3 = 1U, uint32_t Size4 = 1U, bool_t Mutable =
        false>
class MultiArrayAccessBase: public ArrayBase<T, Size1 * Size2 * Size3 * Size4, Mutable>
{
public:
    /// Offset for calculating the array access
    static const uint32_t Offset = Size2 * Size3 * Size4;
protected:
    MultiArrayAccessBase(const MultiArrayAccessBase& i_Copy_ro)
            : ArrayBase<T, Size, Mutable>(i_Copy_ro)
    {
    }

    virtual ~MultiArrayAccessBase()
    {
    }
    explicit MultiArrayAccessBase(const T* i_Cont_px)
            // pointer to mutable data is needed in ArrayBase
            : ArrayBase<T, Size, Mutable>(const_cast<T*>(i_Cont_px)) // PRQA S 3081, 3083
    {
        // mutable array access shall not be used for constant data!
        StaticAssert((Size > 0U) && (false == Mutable), "Size must not be zero and content must be mutable");
    }
    explicit MultiArrayAccessBase(T* i_Cont_px)
            : ArrayBase<T, Size, Mutable>(i_Cont_px)
    {
        StaticAssert(Size > 0U, "Size must not be zero");
    }
    /// Creates an array copying the data from the given constant array.
    /// @param i_Cont_px pointer to the array data
    /// @param i_InitElements_px the constant array defining the initial content.
    MultiArrayAccessBase(T* i_Cont_px,
                         const T* i_InitElements_px)
            : ArrayBase<T, Size, Mutable>(i_Cont_px, Size, i_InitElements_px)
    {
        StaticAssert(Size > 0U, "Size must not be zero");
    }

    /// Total size of the used data.
    static const uint32_t Size = Size1 * Offset;
private:
    MultiArrayAccessBase();
    MultiArrayAccessBase& operator=(const MultiArrayAccessBase&);
    MultiArrayAccessBase& operator=(const T&);
};

/// Access to arrays in a multidimensional manner with index checks for constant data.
/// @tparam T the underlying elementary type
/// @tparam Size1 size of the first dimension
/// @tparam Size2 size of the 2nd dimension
/// @tparam Size3 size of the 3rd dimension
/// @tparam Size4 size of the 4th dimension
template<typename T, uint32_t Size1, uint32_t Size2 = 1U, uint32_t Size3 = 1U, uint32_t Size4 = 1U>
class ConstMultiArrayAccess: public MultiArrayAccessBase<T, Size1, Size2, Size3, Size4, false>
{
public:
    explicit ConstMultiArrayAccess(const T* i_Cont_px)
            : MultiArrayAccessBase<T, Size1, Size2, Size3, Size4, false>(i_Cont_px)
    {
    }
    ConstMultiArrayAccess(const ConstMultiArrayAccess& i_Copy_ro)
            : MultiArrayAccessBase<T, Size1, Size2, Size3, Size4, false>(i_Copy_ro)
    {
    }
    virtual ~ConstMultiArrayAccess()
    {
    }
    virtual uint32_t size_u32() const
    {
        return MultiArrayAccessBase<T, Size1, Size2, Size3, Size4, false>::Size;
    }
    ConstMultiArrayAccess<T, Size2, Size3, Size4, 1U> operator[](uint32_t i_Index_u32) const
    {
        return elementAt_ro(i_Index_u32);
    }
    ConstMultiArrayAccess<T, Size2, Size3, Size4, 1U> operator[](uint32_t i_Index_u32) // PRQA S 4211
    {
        return elementAt_ro(i_Index_u32);
    }
private:
    ConstMultiArrayAccess<T, Size2, Size3, Size4, 1U> elementAt_ro(uint32_t i_Index_u32) const
    {
        Assert(i_Index_u32 < Size1);
        // This is checked pointer arithmetic here // PRQA S 3705 3
        ConstMultiArrayAccess<T, Size2, Size3, Size4, 1U> v_Ret_o(
                this->getConstPointerAccess_x() + (MultiArrayAccessBase<T, Size1, Size2, Size3, Size4, false>::Offset
                        * i_Index_u32));
        return v_Ret_o;
    }
    ConstMultiArrayAccess();
    ConstMultiArrayAccess& operator=(const ConstMultiArrayAccess&);
};

/// Specialization for 1-dimensional array with readonly-access to the data.
/// The []-operator to access the data is implemented in the base class ArrayBase.
template<typename T, uint32_t Size1>
class ConstMultiArrayAccess<T, Size1, 1U, 1U, 1U> : public MultiArrayAccessBase<T, Size1, 1U, 1U, 1U, false>
{
public:
    explicit ConstMultiArrayAccess(const T* i_Cont_px)
            : MultiArrayAccessBase<T, Size1, 1U, 1U, 1U, false>(i_Cont_px)
    {
    }
    ConstMultiArrayAccess(const ConstMultiArrayAccess& i_Copy_ro)
            : MultiArrayAccessBase<T, Size1, 1U, 1U, 1U, false>(i_Copy_ro)
    {
    }
    virtual ~ConstMultiArrayAccess()
    {
    }
    virtual uint32_t size_u32() const
    {
        return Size1;
    }
private:
    ConstMultiArrayAccess();
    ConstMultiArrayAccess& operator=(const ConstMultiArrayAccess&);
};

/// Access to arrays in a multidimensional manner with index checks for writable data.
/// @tparam T the underlying elementary type
/// @tparam Size1 size of the first dimension
/// @tparam Size2 size of the 2nd dimension
/// @tparam Size3 size of the 3rd dimension
/// @tparam Size4 size of the 4th dimension
template<typename T, uint32_t Size1, uint32_t Size2 = 1U, uint32_t Size3 = 1U, uint32_t Size4 = 1U>
class MultiArrayAccess: public MultiArrayAccessBase<T, Size1, Size2, Size3, Size4, true>
{
public:
    explicit MultiArrayAccess(T* i_Cont_px)
            : MultiArrayAccessBase<T, Size1, Size2, Size3, Size4, true>(i_Cont_px)
    {
    }
    MultiArrayAccess(T* i_Cont_px,
                     const T* i_InitialData_px)
            : MultiArrayAccessBase<T, Size1, Size2, Size3, Size4, true>(i_Cont_px, i_InitialData_px)
    {
    }
    MultiArrayAccess(const MultiArrayAccess& i_Copy_ro)
            : MultiArrayAccessBase<T, Size1, Size2, Size3, Size4, true>(i_Copy_ro)
    {
    }
    virtual ~MultiArrayAccess()
    {
    }
    virtual uint32_t size_u32() const
    {
        return MultiArrayAccessBase<T, Size1, Size2, Size3, Size4, true>::Size;
    }
    // This non-const method does not modify member data but returns a writable reference to it // PRQA S 4211 1
    MultiArrayAccess<T, Size2, Size3, Size4, 1U> operator[](uint32_t i_Index_u32)
    {
        // The const_cast is OK here // PRQA S 3081, 3083 1
        MultiArrayAccess<T, Size2, Size3, Size4, 1U> v_Ret_o(const_cast<T*>(pointerAt_po(i_Index_u32)));
        return v_Ret_o;
    }
private:
    const T* pointerAt_po(uint32_t i_Index_u32) const
    {
        Assert(i_Index_u32 < Size1);
        // This is checked pointer arithmetic here
        return this->getConstPointerAccess_x() + // PRQA S 3705
        (MultiArrayAccessBase<T, Size1, Size2, Size3, Size4, false>::Offset * i_Index_u32);
    }
    MultiArrayAccess();
    MultiArrayAccess& operator=(const MultiArrayAccess&);
};

/// Specialization for 1-dimensional array with rw-access to the data.
/// The []-operator to access the data is implemented in the base class ArrayBase.
template<typename T, uint32_t Size1>
class MultiArrayAccess<T, Size1, 1U, 1U, 1U> : public MultiArrayAccessBase<T, Size1, 1U, 1U, 1U, true>
{
public:
    explicit MultiArrayAccess(T* i_Cont_px)
                : MultiArrayAccessBase<T, Size1, 1U, 1U, 1U, true>(i_Cont_px)
    {
    }
    MultiArrayAccess(T* i_Cont_px,
                     const T* i_InitialData_px)
            : MultiArrayAccessBase<T, Size1, 1U, 1U, 1U, true>(i_Cont_px, i_InitialData_px)
    {
    }
    MultiArrayAccess(const MultiArrayAccess& i_Copy_ro)
            : MultiArrayAccessBase<T, Size1, 1U, 1U, 1U, true>(i_Copy_ro)
    {
    }
    virtual ~MultiArrayAccess()
    {
    }
    virtual uint32_t size_u32() const
    {
        return Size1;
    }
private:
    MultiArrayAccess();
    MultiArrayAccess& operator=(const MultiArrayAccess&);
};

}
}

#endif // MECL_COLL_MULTIARRAYACCESS_H_
