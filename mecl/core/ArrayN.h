// PRQA S 1051 EOF // Code example is not commented out code
//--------------------------------------------------------------------------
/// @file ArrayN.h
/// @brief Contains the implementation to access memory regions as a multidimensional array.
///
/// @par Usage
/// @code{cpp}
/// typedef mecl::core::Array3D<uint32_t,2,3,4> MyArray;
/// static const uint32_t initialData_au32[2][3][4] =
///         {{{1U,2U,11U,22U}, {3U,4U,33U,44U}, {5U,6U,55U,66U}},
///         {{7U,8U,77U,88U}, {10U,20U,30U,40U},{50U,60U,70U,80}}};
///
/// MyArray myArray_o(initialData_au32);
/// uint32_t v_X_u32 = myArray_o[0][1][2];
///
/// EXPECT_EQ(33U, v_X_u32) << "Array value should be same as initial value";
/// myArray_o[0][1][2] = 333U;
/// EXPECT_EQ(333U, myArray_o[0][1][2]) << "Array value should have changed";
///
/// ASSERT_DEATH({v_X_u32 = myArray_o[1][2][4];},
///            "Assertion failed: index_u32 < .*, file .*, line .*") << "should fail on access beyond the end";
/// @endcode
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

#ifndef ARRAYN_H_
#define ARRAYN_H_

#include "MultiArrayAccess.h"
#include "Array.h"

// for templates it is allowed to define functions inside the class definition
// PRQA S 2106 EOF
// Subscript operator used here is OK
// PRQA S 3706 EOF

namespace mecl
{
namespace core
{


// --------------------------------------------------------------------------
/// @class ArrayN 
/// @brief N-dimensional array 
/// 
/// Implements multidimensional array interface for 1D, 2D, 3D and 4D 
/// array types.
/// 
/// Supports use of initialization with constant multidimensional C-array, checking of bounds and
/// access with multiple []-access operators. 
// --------------------------------------------------------------------------
template<typename T, uint32_t SizeX, uint32_t SizeY = 1U, uint32_t SizeZ = 1U, uint32_t SizeA = 1U>
class ArrayN : public MultiArrayAccess<T, SizeX, SizeY, SizeZ, SizeA>
{
public:
    /// Number of array elements in each dimension
    static const uint32_t Size = MultiArrayAccess<T, SizeX, SizeY, SizeZ, SizeA>::Size; 


    // --------------------------------------------------------------------------
    /// @brief Default constructor
    ///
    /// Create a multidimensional array and if necessary initialize its contents.
    ///
    /// @par Example use:
    /// @snippet CoreTester.cpp ArrayN_Constructor1
    ///
    /// @param[in] initArray_b if true and the array contains primitive types or pointers,
    /// the array is initialized with null - else the array contents are left as is.
    // --------------------------------------------------------------------------
    explicit ArrayN(bool_t initArray_b = true) : MultiArrayAccess<T, SizeX, SizeY, SizeZ, SizeA>(&items_ax[0])
    {
        if (initArray_b)
        {
            ArrayBase<T, Size>::init_v(Size);
        }
    }
    
    // --------------------------------------------------------------------------
    /// @brief Creates an array copying the data from the given constant array.
    ///
    /// @par Example use:
    /// @snippet CoreTester.cpp ArrayN_Constructor2
    ///
    /// @param[in] i_InitElements_px the constant array defining the initial content.
    // --------------------------------------------------------------------------
    explicit ArrayN(const T* i_InitElements_px)
            : MultiArrayAccess<T, SizeX, SizeY, SizeZ, SizeA>(&items_ax[0], i_InitElements_px)
    {
    }

    // --------------------------------------------------------------------------
    /// @brief Destructor
    // --------------------------------------------------------------------------
    virtual ~ArrayN() {}
private:
    // --------------------------------------------------------------------------
    /// @brief ArrayN assignment operator
    ///
    /// Operator sets value of all elements from this ArrayN to values of input
    /// argument \p ArrayN.
    ///
    /// @param[in]    ArrayN ArrayN object from which to copy contents
    /// @return       Reference to this ArrayN
    // --------------------------------------------------------------------------
    ArrayN& operator=(const ArrayN&);

    // --------------------------------------------------------------------------
    /// @brief Copy constructor
    ///
    /// The constructor copies the contents of \p ArrayN into this ArrayN.
    ///
    /// @param[in]    ArrayN ArrayN object from which to copy contents
    /// @return       ArrayN pre-initialized with specific content
    // --------------------------------------------------------------------------
    ArrayN(const ArrayN&);

    T items_ax[Size]; ///< Memory array for value storage
};

// n-dimensional arrays are declared for convenience here
// It would be quite inconvenient to create own header files for them.


// PRQA S 1062 ++
// PRQA S 1063 ++
// --------------------------------------------------------------------------
/// @class Array2D
/// @brief 2-dimensional array
/// 
/// Supports use of initialization with constant multidimensional C-array, checking of bounds and
/// access with [][]-access operators. 
// --------------------------------------------------------------------------
template<typename T, uint32_t SizeX, uint32_t SizeY>
class Array2D : public ArrayN<T, SizeX, SizeY>
{
public:
    // --------------------------------------------------------------------------
    /// @brief Default constructor
    ///
    /// Create a 2-dimensional array and if necessary initialize its contents.
    ///
    /// @par Example use:
    /// @snippet CoreTester.cpp Array2D_Constructor1
    ///
    /// @param[in] i_InitArray_b if true and the array contains primitive types or pointers,
    /// the array is initialized with null - else the array contents are left as is.
    // --------------------------------------------------------------------------
    explicit Array2D(bool_t i_InitArray_b = true) : ArrayN<T, SizeX, SizeY>(i_InitArray_b) {}

    // --------------------------------------------------------------------------
    /// @brief Creates an array copying the data from the given constant array.
    ///
    /// @par Example use:
    /// @snippet CoreTester.cpp Array2D_Constructor2
    ///
    /// @param[in] i_InitElements_ax the constant array defining the initial content.
    // --------------------------------------------------------------------------
    explicit Array2D(const T i_InitElements_ax[SizeX][SizeY])
            : ArrayN<T, SizeX, SizeY>(&i_InitElements_ax[0][0])
    {
    }
private:
    // --------------------------------------------------------------------------
    /// @brief Array2D assignment operator
    ///
    /// Operator sets value of all elements from this Array2D to values of input
    /// argument \p Array2D.
    ///
    /// @param[in]    Array2D Array2D object from which to copy contents
    /// @return       Reference to this Array2D
    // --------------------------------------------------------------------------
    Array2D& operator=(const Array2D&);

    // --------------------------------------------------------------------------
    /// @brief Copy constructor
    ///
    /// The constructor copies the contents of \p Array2D into this Array2D.
    ///
    /// @param[in]    Array2D Array2D object from which to copy contents
    /// @return       Array2D pre-initialized with specific content
    // --------------------------------------------------------------------------
    Array2D(const Array2D&);
};

// --------------------------------------------------------------------------
/// @class Array3D 
/// @brief 3-dimensional array
/// 
/// Supports use of initialization with constant multidimensional C-array, checking of bounds and
/// access with [][][]-access operators. 
// --------------------------------------------------------------------------
template<typename T, uint32_t SizeX, uint32_t SizeY, uint32_t SizeZ>
class Array3D : public ArrayN<T, SizeX, SizeY, SizeZ>
{
public:
    // --------------------------------------------------------------------------
    /// @brief Default constructor
    ///
    /// Create a 3-dimensional array and if necessary initialize its contents.
    ///
    /// @par Example use:
    /// @snippet CoreTester.cpp Array3D_Constructor1
    ///
    /// @param[in] i_InitArray_b if true and the array contains primitive types or pointers,
    /// the array is initialized with null - else the array contents are left as is.
    // --------------------------------------------------------------------------
    explicit Array3D(bool_t i_InitArray_b = true) : ArrayN<T, SizeX, SizeY, SizeZ>(i_InitArray_b) {}
    
    // --------------------------------------------------------------------------
    /// @brief Creates an array copying the data from the given constant array.
    ///
    /// @par Example use:
    /// @snippet CoreTester.cpp Array3D_Constructor2
    ///
    /// @param[in] i_InitElements_ax the constant array defining the initial content.
    // --------------------------------------------------------------------------
    explicit Array3D(const T i_InitElements_ax[SizeX][SizeY][SizeZ])
            : ArrayN<T, SizeX, SizeY, SizeZ>(&i_InitElements_ax[0][0][0])
    {
    }
private:
    // --------------------------------------------------------------------------
    /// @brief Array3D assignment operator
    ///
    /// Operator sets value of all elements from this Array3D to values of input
    /// argument \p Array3D.
    ///
    /// @param[in]    Array3D Array3D object from which to copy contents
    /// @return       Reference to this Array3D
    // --------------------------------------------------------------------------
    Array3D& operator=(const Array3D&);
    
    // --------------------------------------------------------------------------
    /// @brief Copy constructor
    ///
    /// The constructor copies the contents of \p Array3D into this Array3D.
    ///
    /// @param[in]    Array3D Array3D object from which to copy contents
    /// @return       Array3D pre-initialized with specific content
    // --------------------------------------------------------------------------
    Array3D(const Array3D&);
};

// --------------------------------------------------------------------------
/// @class Array4D
/// @brief 4-dimensional array
/// 
/// Supports use of initialization with constant multidimensional C-array, checking of bounds and
/// access with [][][][]-access operators. 
// --------------------------------------------------------------------------
template<typename T, uint32_t SizeX, uint32_t SizeY, uint32_t SizeZ, uint32_t SizeW>
class Array4D : public ArrayN<T, SizeX, SizeY, SizeZ, SizeW>
{
public:
    // --------------------------------------------------------------------------
    /// @brief Default constructor
    ///
    /// Create a 4-dimensional array and if necessary initialize its contents.
    ///
    /// @par Example use:
    /// @snippet CoreTester.cpp Array4D_Constructor1
    ///
    /// @param[in] i_InitArray_b if true and the array contains primitive types or pointers,
    /// the array is initialized with null - else the array contents are left as is.
    // --------------------------------------------------------------------------
    explicit Array4D(bool_t i_InitArray_b = true) : ArrayN<T, SizeX, SizeY, SizeZ, SizeW>(i_InitArray_b) {}
    
    // --------------------------------------------------------------------------
    /// @brief Creates an array copying the data from the given constant array.
    ///
    /// @par Example use:
    /// @snippet CoreTester.cpp Array4D_Constructor2
    ///
    /// @param[in] i_InitElements_ax the constant array defining the initial content.
    // --------------------------------------------------------------------------
    explicit Array4D(const T i_InitElements_ax[SizeX][SizeY][SizeZ][SizeW])
            : ArrayN<T, SizeX, SizeY, SizeZ, SizeW>(&i_InitElements_ax[0][0][0][0])
    {
    }
private:
    // --------------------------------------------------------------------------
    /// @brief Array4D assignment operator
    ///
    /// Operator sets value of all elements from this Array4D to values of input
    /// argument \p Array4D.
    ///
    /// @param[in]    Array4D Array4D object from which to copy contents
    /// @return       Reference to this Array4D
    // --------------------------------------------------------------------------
    Array4D& operator=(const Array4D&);
    
    // --------------------------------------------------------------------------
    /// @brief Copy constructor
    ///
    /// The constructor copies the contents of \p Array4D into this Array4D.
    ///
    /// @param[in]    Array4D Array4D object from which to copy contents
    /// @return       Array4D pre-initialized with specific content
    // --------------------------------------------------------------------------
    Array4D(const Array4D&);
};
// PRQA S 1062 --
// PRQA S 1063 --

}
}

#endif /* ARRAYN_H_ */

/// @}
/// @}
