//--------------------------------------------------------------------------
/// @file RotationMatrix.h
/// @brief This is the short description of the template module
///
/// Here may follow a longer description for the template module with examples.
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Helmut Zollner (helmut.zollner@magna.com)
/// @date 22.02.2015
///
//  --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup core
/// @{

#ifndef MECL_CORE_ROTATIONMATRIX_H_
#define MECL_CORE_ROTATIONMATRIX_H_

#include "Matrix3x3.h"
#include "model/ModelTypes.h"
#include "math/Math.h"

namespace mecl
{
namespace core
{

// --------------------------------------------------------------------------
/// @class RotationMatrix
/// @brief Rotation matrix implementation based on the Matrix3x3 class.
///
/// The RotationMatrix implements rotation in 3D around x-, y- and z-axis 
/// based on Euler angles.
// --------------------------------------------------------------------------
template<typename T>
class RotationMatrix : public Matrix3x3<T>
{
public:

  // --------------------------------------------------------------------------
  //! @struct EulerAngles_s
  //! @brief Euler angles data structure
  // --------------------------------------------------------------------------
  struct EulerAngles_s
  {
    T pitch_x;              ///< image plane rotation around x-axis
    T yaw_x;                ///< image plane rotation around y-axis
    T roll_x;               ///< image plane rotation around z-axis
  };

  // --------------------------------------------------------------------------
  /// @brief Default constructor
  /// The default construtor initializes all cell elements to zero.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp RotationMatrix_Constructor1
  ///
  /// @return       RotationMatrix
  // --------------------------------------------------------------------------
  RotationMatrix()
  : Matrix3x3<T>()
  {}

  // --------------------------------------------------------------------------
  /// @brief Constructor with initialization value
  ///
  /// The constructor initializes all cell elements to the input argument \p value.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp RotationMatrix_Constructor2
  ///
  /// @param[in] value  Initialization value for all elements in RotationMatrix
  /// @return       RotationMatrix pre-initialized with specific values
  // --------------------------------------------------------------------------
  RotationMatrix(T value)
  : Matrix3x3<T>(value)
  {}

  // --------------------------------------------------------------------------
  /// @brief Copy constructor
  ///
  /// The constructor copies the contents of \p M into this rotation matrix.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp RotationMatrix_Constructor3
  /// @snippet CoreTester.cpp RotationMatrix_Constructor4
  ///
  /// @param[in]    M RotationMatrix object from which to copy contents
  /// @return       RotationMatrix pre-initialized with specific content
  // --------------------------------------------------------------------------
  RotationMatrix(const Matrix3x3<T>& M)
  : Matrix3x3<T>(M)
  {}

  // --------------------------------------------------------------------------
  /// @brief Copy constructor with initialization by Euler angle configuration
  ///
  /// The constructor initializes cell contents using Euler angle configuration
  /// supplied in \p i_EulerAngles_s.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp RotationMatrix_Constructor3
  ///
  /// @param[in]    i_EulerAngles_s Euler angle configuration
  /// @param[in]    i_PreRoll90_rb If true, Rotation matrix is rotated 90 degrees 
  /// around Z-axis.
  /// @return       RotationMatrix pre-initialized with specific angle configuration
  // --------------------------------------------------------------------------
  RotationMatrix(const EulerAngles_s i_EulerAngles_s, const bool_t& i_PreRoll90_rb = false)
  : Matrix3x3<T>()
  {
    if(false == i_PreRoll90_rb)
    {
      this->init_v(i_EulerAngles_s);
    }
    else
    {
      this->initPreRoll90_v(i_EulerAngles_s);
    }
  }

  // --------------------------------------------------------------------------
  /// @brief Destructor
  // --------------------------------------------------------------------------
  virtual ~RotationMatrix() {}

  // --------------------------------------------------------------------------
  /// @brief Rotation matrix for 0 degree roll
  ///
  /// Function generates a rotation matrix incorporating a 0 degree roll
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp RotationMatrix_preRoll0
  ///
  /// @return Rotation matrix with 0 degree roll
  // --------------------------------------------------------------------------
  static RotationMatrix preRoll0(void)
  {
    return static_cast< RotationMatrix<T> >(Matrix<T, 3, 3>::eye_x());
  }
    
  // --------------------------------------------------------------------------
  /// @brief Rotation matrix for 90 degree roll
  ///
  /// Function generates a rotation matrix incorporating a 90 degree roll
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp RotationMatrix_preRoll90
  ///
  /// @return Rotation matrix with 90 degree roll
  // --------------------------------------------------------------------------
  static RotationMatrix preRoll90(void)
  {
    static typename Matrix<T, 3, 3>::Config_s s_MatCfg_x = {
        math::constants<T>::zero_x(), math::constants<T>::minusOne_x(), math::constants<T>::zero_x(),
        math::constants<T>::one_x(),  math::constants<T>::zero_x(),     math::constants<T>::zero_x(),
        math::constants<T>::zero_x(), math::constants<T>::zero_x(),     math::constants<T>::one_x()
    };
    return static_cast< RotationMatrix<T> >(Matrix<T, 3, 3>(s_MatCfg_x));
  }
    
  // --------------------------------------------------------------------------
  /// @brief Rotation matrix for 180 degree roll
  ///
  /// Function generates a rotation matrix incorporating a 180 degree roll
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp RotationMatrix_preRoll180
  ///
  /// @return Rotation matrix with 180 degree roll
  // --------------------------------------------------------------------------
  static RotationMatrix preRoll180(void)
  {
     static typename Matrix<T, 3, 3>::Config_s s_MatCfg_x = {
        math::constants<T>::minusOne_x(), math::constants<T>::zero_x(),      math::constants<T>::zero_x(),
        math::constants<T>::zero_x(),     math::constants<T>::minusOne_x(),  math::constants<T>::zero_x(),
        math::constants<T>::zero_x(),     math::constants<T>::zero_x(),      math::constants<T>::one_x()
    };
    return static_cast< RotationMatrix<T> >(Matrix<T, 3, 3>(s_MatCfg_x));
  }
  
  // --------------------------------------------------------------------------
  /// @brief Rotation matrix for 270 degree roll
  ///
  /// Function generates a rotation matrix incorporating a 270 degree roll
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp RotationMatrix_preRoll270
  ///
  /// @return Rotation matrix with 270 degree roll
  // --------------------------------------------------------------------------
  static RotationMatrix preRoll270(void)
  {
    static typename Matrix<T, 3, 3>::Config_s s_MatCfg_x = {
        math::constants<T>::zero_x(),     math::constants<T>::one_x(),   math::constants<T>::zero_x(),
        math::constants<T>::minusOne_x(), math::constants<T>::zero_x(),  math::constants<T>::zero_x(),
        math::constants<T>::zero_x(),     math::constants<T>::zero_x(),  math::constants<T>::one_x()
    };
    return static_cast< RotationMatrix<T> >(Matrix<T, 3, 3>(s_MatCfg_x));
  }

  // --------------------------------------------------------------------------
  /// @brief Initialize rotation matrix
  ///
  /// The function initializes rotation matrix elements based on Euler angle
  /// configuration.
  /// 
  /// @attention Euler angles define camera orientation as see from world coordinate system 
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp RotationMatrix_init_v
  ///
  /// @param[in]    i_EulerAngles_rs Euler angle configuration
  /// @return       void
  // --------------------------------------------------------------------------
  void init_v(const EulerAngles_s& i_EulerAngles_rs)
  {
    T v_SinAngX_x = math::trigonometry<T>::sin_x(i_EulerAngles_rs.pitch_x);
    T v_CosAngX_x = math::trigonometry<T>::cos_x(i_EulerAngles_rs.pitch_x);
    T v_SinAngY_x = math::trigonometry<T>::sin_x(i_EulerAngles_rs.yaw_x  );
    T v_CosAngY_x = math::trigonometry<T>::cos_x(i_EulerAngles_rs.yaw_x  );
    T v_SinAngZ_x = math::trigonometry<T>::sin_x(i_EulerAngles_rs.roll_x );
    T v_CosAngZ_x = math::trigonometry<T>::cos_x(i_EulerAngles_rs.roll_x );

    (*this)(0,0) =  v_CosAngZ_x * v_CosAngY_x;
    (*this)(0,1) = -v_SinAngZ_x * v_CosAngX_x + v_CosAngZ_x * v_SinAngY_x * v_SinAngX_x;
    (*this)(0,2) =  v_SinAngZ_x * v_SinAngX_x + v_CosAngZ_x * v_SinAngY_x * v_CosAngX_x;
    (*this)(1,0) =  v_SinAngZ_x * v_CosAngY_x;
    (*this)(1,1) =  v_CosAngZ_x * v_CosAngX_x + v_SinAngZ_x * v_SinAngY_x * v_SinAngX_x;
    (*this)(1,2) = -v_CosAngZ_x * v_SinAngX_x + v_SinAngZ_x * v_SinAngY_x * v_CosAngX_x;
    (*this)(2,0) = -v_SinAngY_x;
    (*this)(2,1) =  v_CosAngY_x * v_SinAngX_x;
    (*this)(2,2) =  v_CosAngY_x * v_CosAngX_x;

    return;
  }

  // --------------------------------------------------------------------------
  /// @brief Initialize rotation matrix and apply 90 degree roll
  ///
  /// The function initializes rotation matrix elements based on Euler angle
  /// configuration and applies 90 degree rotation around Z-axis.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp RotationMatrix_initPreRoll90_v
  ///
  /// @param[in]    i_EulerAngles_rs Euler angle configuration
  /// @return       void
  // --------------------------------------------------------------------------
  void initPreRoll90_v(const EulerAngles_s& i_EulerAngles_rs)
  {
    T v_SinAngX_x = math::trigonometry<T>::sin_x(i_EulerAngles_rs.pitch_x);
    T v_CosAngX_x = math::trigonometry<T>::cos_x(i_EulerAngles_rs.pitch_x);
    T v_SinAngY_x = math::trigonometry<T>::sin_x(i_EulerAngles_rs.yaw_x  );
    T v_CosAngY_x = math::trigonometry<T>::cos_x(i_EulerAngles_rs.yaw_x  );
    T v_SinAngZ_x = math::trigonometry<T>::sin_x(i_EulerAngles_rs.roll_x );
    T v_CosAngZ_x = math::trigonometry<T>::cos_x(i_EulerAngles_rs.roll_x );

    (*this)(0,0) = -v_SinAngZ_x * v_CosAngX_x + v_CosAngZ_x * v_SinAngY_x * v_SinAngX_x;
    (*this)(0,1) = -v_CosAngZ_x * v_CosAngY_x;
    (*this)(0,2) =  v_SinAngZ_x * v_SinAngX_x + v_CosAngZ_x * v_SinAngY_x * v_CosAngX_x;
    (*this)(1,0) =  v_CosAngZ_x * v_CosAngX_x + v_SinAngZ_x * v_SinAngY_x * v_SinAngX_x;
    (*this)(1,1) = -v_SinAngZ_x * v_CosAngY_x;
    (*this)(1,2) = -v_CosAngZ_x * v_SinAngX_x + v_SinAngZ_x * v_SinAngY_x * v_CosAngX_x;
    (*this)(2,0) =  v_CosAngY_x * v_SinAngX_x;
    (*this)(2,1) =  v_SinAngY_x;
    (*this)(2,2) =  v_CosAngY_x * v_CosAngX_x;

    return;
  }

  // --------------------------------------------------------------------------
  /// @brief Calculate Euler angles
  ///
  /// The function calculates Euler angles based on the rotation matrix. The
  /// function does not take into account that multiple solutions to the problem
  /// may exist.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp RotationMatrix_calcEA_s
  ///
  /// @return       EulerAngles_s Euler angle configuration
  // --------------------------------------------------------------------------
  EulerAngles_s calcEA_s() const
  {
    EulerAngles_s v_EulerAngles_s = {0,0,0};
    v_EulerAngles_s.pitch_x = math::trigonometry<T>::atan2_x((*this)(2,1), (*this)(2,2));

    T c_Temp_x = sqrt((*this)(2,1)*(*this)(2,1)+ (*this)(2,2)* (*this)(2,2));
    v_EulerAngles_s.yaw_x = math::trigonometry<T>::atan2_x( -(*this)(2,0), c_Temp_x);

    v_EulerAngles_s.roll_x = math::trigonometry<T>::atan2_x((*this)(1,0), (*this)(0,0));
    normalizeEA_v(v_EulerAngles_s);
    return v_EulerAngles_s;
  }

  // --------------------------------------------------------------------------
  /// @brief Calculate Euler angles
  ///
  /// The function calculates Euler angles based on the rotation matrix. If two
  /// solutions exists, both are returned, otherwise the design value (pixel = 0)
  /// solution is returned.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp RotationMatrix_calcEASafe_b
  ///
  /// @param[out] o_Solution1_rs Euler angle configuration solution 1
  /// @param[out] o_Solution2_rs Euler angle configuration solution 2
  /// @return     Boolean, True = two solutions exist, False = infinite solutions exist
  // --------------------------------------------------------------------------
  bool_t calcEASafe_b(EulerAngles_s& o_Solution1_rs, EulerAngles_s& o_Solution2_rs) const
  {
    bool_t twoSolutions_b = math::constants<T>::one_x() != math::abs_x<T>((*this)(2,0));

    if(false == twoSolutions_b)
    {
       // infinite number of solutions, pitch = 0 as design value
      o_Solution1_rs.roll_x = math::constants<T>::zero_x();

      T v_Delta_x = math::trigonometry<T>::atan2_x((*this)(0,1), (*this)(0,2));

      if(math::constants<T>::minusOne_x() == (*this)(2,0))
      {
        o_Solution1_rs.yaw_x = math::constants<T>::pi_x() / 2;
        o_Solution1_rs.pitch_x = o_Solution1_rs.roll_x + v_Delta_x;
      }
    }
    else
    { 
      // a pair of solutions exists
      o_Solution1_rs.yaw_x = - math::trigonometry<T>::asin_x((*this)(2,0));
      o_Solution2_rs.yaw_x =   math::constants<T>::pi_x() - o_Solution1_rs.yaw_x;

      T v_CosYaw1_x = math::trigonometry<T>::cos_x(o_Solution1_rs.yaw_x);
      T v_CosYaw2_x = math::trigonometry<T>::cos_x(o_Solution2_rs.yaw_x);

      o_Solution1_rs.pitch_x = math::trigonometry<T>::atan2_x((*this)(2,1) / v_CosYaw1_x, (*this)(2,2) / v_CosYaw1_x);
      o_Solution2_rs.pitch_x = math::trigonometry<T>::atan2_x((*this)(2,1) / v_CosYaw2_x, (*this)(2,2) / v_CosYaw2_x);

      o_Solution1_rs.roll_x  = math::trigonometry<T>::atan2_x((*this)(1,0) / v_CosYaw1_x, (*this)(0,0) / v_CosYaw1_x);
      o_Solution2_rs.roll_x  = math::trigonometry<T>::atan2_x((*this)(1,0) / v_CosYaw2_x, (*this)(0,0) / v_CosYaw2_x);
    }

    normalizeEA_v(o_Solution1_rs);
    normalizeEA_v(o_Solution2_rs);

    return twoSolutions_b;
  }

  // --------------------------------------------------------------------------
  /// @brief Normalize Euler angles
  ///
  /// The function ensures none of the angles contained within the Euler angle
  /// structure exceeds 2 * pi.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp RotationMatrix_normalizeEA_v
  ///
  /// @param[out] b_EulerAngles_rs Normalized Euler angle configuration
  /// @return     void
  // --------------------------------------------------------------------------
  static void normalizeEA_v(EulerAngles_s& b_EulerAngles_rs, AngleUnit_e i_AngleUnit_e = e_Radians)
  {
    b_EulerAngles_rs.pitch_x = math::getNormalizedAngle_x(b_EulerAngles_rs.pitch_x, i_AngleUnit_e);
    b_EulerAngles_rs.yaw_x   = math::getNormalizedAngle_x(b_EulerAngles_rs.yaw_x, i_AngleUnit_e);
    b_EulerAngles_rs.roll_x =  math::getNormalizedAngle_x(b_EulerAngles_rs.roll_x, i_AngleUnit_e);
    return;
  }

  // --------------------------------------------------------------------------
  /// @brief Print Euler angles
  ///
  /// The function calculates, normalizes and prints Euler angles in either 
  /// radian or degrees depending on the value of \p i_InDegrees_b. The
  /// function does not take into account that multiple solutions to the problem
  /// may exist.
  ///
  /// Function used only for debugging purposes.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp RotationMatrix_printEA_v
  ///
  /// @param[in]  i_InDegrees_b Boolean, True = degrees, False = radian
  /// @return     void
  // --------------------------------------------------------------------------
  void printEA_v(bool_t i_InDegrees_b = true) const
  {
    EulerAngles_s v_Solution_s = this->calcEA_s();
    if (true == i_InDegrees_b) {
      log_printf("\nEA (pitch, yaw, roll): (%f, %f, %f)\n",
              math::toDegrees_x(v_Solution_s.pitch_x),
              math::toDegrees_x(v_Solution_s.yaw_x),
              math::toDegrees_x(v_Solution_s.roll_x) );
    } else {
      log_printf("\nEA (pitch, yaw, roll): (%f, %f, %f)\n",
              v_Solution_s.pitch_x, v_Solution_s.yaw_x, v_Solution_s.roll_x);
    }
    return;
  }

  // --------------------------------------------------------------------------
  /// @brief Print Euler angles
  ///
  /// The function calculates, normalizes and prints Euler angles in either 
  /// radian or degrees depending on the value of \p i_InDegrees_b. If two
  /// solutions exists, both are returned, otherwise the design value (pixel = 0)
  /// solution is returned.
  ///
  /// Function used only for debugging purposes.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp RotationMatrix_printEASafe_v
  ///
  /// @param[in]  i_InDegrees_b Boolean, True = degrees, False = radian
  /// @return     void
  // --------------------------------------------------------------------------
  void printEASafe_v(bool_t i_InDegrees_b = true) const
  {
    EulerAngles_s v_Solution1_s = {0,0,0};
    EulerAngles_s v_Solution2_s = {0,0,0};

    bool_t twoSolutions_b = this->calcEASafe_b(v_Solution1_s, v_Solution2_s);
    log_printf("Euler Angles: (pitch [Rx], yaw [Ry] , roll[Ry])\n");
      if (i_InDegrees_b) {
        log_printf("Solution 1: (%f, %f, %f)\n",
            math::toDegrees_x(v_Solution1_s.pitch_x),
            math::toDegrees_x(v_Solution1_s.yaw_x),
            math::toDegrees_x(v_Solution1_s.roll_x) );
        log_printf("Solution 2: (%f, %f, %f)\n",
            math::toDegrees_x(v_Solution2_s.pitch_x),
            math::toDegrees_x(v_Solution2_s.yaw_x),
            math::toDegrees_x(v_Solution2_s.roll_x) );
      } else {
        log_printf("Solution 1: (%f, %f, %f)\n",
                    v_Solution1_s.pitch_x,
                    v_Solution1_s.yaw_x,
                    v_Solution1_s.roll_x);
        log_printf("Solution 2: (%f, %f, %f)\n",
                    v_Solution2_s.pitch_x,
                    v_Solution2_s.yaw_x,
                    v_Solution2_s.roll_x);
      }
    if (!twoSolutions_b) log_printf("Solutions 1 and 2 are equal.");

    return;
  }
};

} // namespace core
} // namespace mecl

#endif // MECL_CORE_ROTATIONMATRIX_H_
/// @}
/// @}

