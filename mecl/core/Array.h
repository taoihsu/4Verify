// --------------------------------------------------------------------------
/// @file Array.h
/// @brief Contains the template to easily create arrays with checked access.
///
/// Array is a replacement to use instead of C-arrays.
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Michael Schulte (michael.schulte@magna.com)
///
// --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup core
/// @{

#ifndef ARRAY_H_
#define ARRAY_H_

#include "MeclTypes.h"
#include "Collection.h"
#include "MeclAssert.h"
#include "ArrayN.h"
#include <cstring> // for memset

namespace mecl
{
namespace core
{

// for convenience the typedef "type" or "iterator" is always named type or iterator
// PRQA S 1507 EOF
// Examples in the comments should be OK
// PRQA S 1051 EOF
// for templates it is allowed to define functions inside the class definition
// PRQA S 2106 EOF

// --------------------------------------------------------------------------
/// @brief Array template class
///
/// Replaces the usual C-array implementation.
/// Uses checks to ensure the array bounds are not overwritten.
/// It also can be used as a Collection to iterate over the items.
/// For multidimensional Arrays use Array2D, Array3D,...
/// The arrays may be used as usual - integer, floating-point and pointer-arrays
/// are initialized with 0 automatically.
///
/// @par Example usage:
/// @code{.cpp}
/// Array<uint16_t, 3> v_arr_o;
/// v_arr_o[0U] = 1U;
/// v_arr_o[1U] = 2U;
/// v_arr_o[2U] = 3U;
///
/// Array3D<float32_t, 10, 11, 12> v_arr3D_o;
/// v_arr3D_o[9U][10U][11U] = 3.1F;
/// std::cout << v_arr3D_o[0U][0U][0U]   << std::endl; // should print 0.0
/// std::cout << v_arr3D_o[9U][10U][11U] << std::endl; // should print 3.1
/// @endcode
///
/// @par Template Method
/// When writing methods that should deal with Arrays of arbitrary type or size
/// it is necessary to define the methods as template.
/// It is advisable to define the method on the most basic interface.
/// So for array this could either be the ArrayBase - which also serves MemArray implementations -
/// or the Collection, which serves any Collection-type. In case of the Collection only the iterator
/// can be used to iterate the Collection - the array access via '[]' is not available then.
///
/// @code{.cpp}
/// template<typename T, uint32_t MaxSize>
/// uint32_t calcSum_u32(const ArrayBase<T, MaxSize>& v_Arr_ro)
/// {
///     uint32_t v_sum_u32 = 0UL;
///     for (uint32_t v_idx_u32 = 0UL; v_idx_u32 < v_Arr_ro.size_u32(); ++v_idx_u32)
///     {
///         v_sum_u32 += v_Arr_ro[v_idx_u32];
///     }
///     return v_sum_u32;
/// }
///
/// template<typename T, uint32_t MaxSize, typename IteratorType>
/// uint32_t calcSumColl_u32(const Collection<T, MaxSize, IteratorType>& coll_ro)
/// {
///     uint32_t v_sum_u32 = 0UL;
///     for (const IteratorType v_it_o = coll_ro.begin_o(); v_it_o != coll_ro.end_o(); ++v_it_o)
///     {
///         v_sum_u32 += *v_it_o;
///     }
///     return v_sum_u32;
/// }
/// @endcode
///
// --------------------------------------------------------------------------
template<typename T, uint32_t MaxSize>
class Array: public ArrayN<T, MaxSize>
{
public:
    // --------------------------------------------------------------------------
    /// @brief Create an array and if necessary initialize its contents.
    ///
    /// @par Example use:
    /// @snippet CoreTester.cpp Array_Constructor1
    ///
    /// @param[in] i_InitArea_b if true and the array contains primitive types or pointers,
    /// the array is initialized with null - else the array contents are left as is.
    // --------------------------------------------------------------------------
    explicit Array(bool_t i_InitArea_b = true) : ArrayN<T, MaxSize>(i_InitArea_b)
    {
        if (i_InitArea_b)
        {
            ArrayBase<T, MaxSize>::init_v(MaxSize);
        }
    }

    // --------------------------------------------------------------------------
    /// @brief Creates an array copying the data from the given constant array.
    ///
    /// @par Example use: 
    /// @snippet CoreTester.cpp Array_Constructor2
    ///
    /// @param[in] i_InitElements_ax the constant array defining the initial content.
    // --------------------------------------------------------------------------
    explicit Array(const T i_InitElements_ax[MaxSize])
            : ArrayN<T, MaxSize>(i_InitElements_ax)
    {
    }

    // --------------------------------------------------------------------------
    /// @brief Destructor
    // --------------------------------------------------------------------------
    virtual ~Array() {}
private:
    // --------------------------------------------------------------------------
    /// @brief Copy constructor
    ///
    /// The constructor copies the contents of \p Array into this Array.
    ///
    /// @param[in]    Array Array object from which to copy contents
    /// @return       Array pre-initialized with specific content
    // --------------------------------------------------------------------------
    Array(const Array&);

    // --------------------------------------------------------------------------
    /// @brief Array assignment operator
    ///
    /// Operator sets value of all elements from this Array to values of input
    /// argument \p Array.
    ///
    /// @param[in]    Array Array object from which to copy contents
    /// @return       Reference to this matrix
    // --------------------------------------------------------------------------
    Array& operator = (const Array&);
};

}
}
#endif // ARRAY_H_
/// @}
/// @}
