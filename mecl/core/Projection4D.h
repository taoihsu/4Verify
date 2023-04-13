//  --------------------------------------------------------------------------
/// @file Matrix4x4.h
/// @brief 4x4 matrix storage class (static memory)
///
/// Here may follow a longer description for the template module with examples.
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Mark Reichert (Mark.Reichert2@magna.com)
///
//  --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup core
/// @{

#ifndef MECL_CORE_MATRIX4X4_H_
#define MECL_CORE_MATRIX4X4_H_

#include "Matrix.h"
#include "Matrix3x3.h"

#include "Point.h"

namespace mecl
{
namespace core
{

template <typename T>
class Projection4D : public core::Matrix<T, 4, 4>
{
public:
  typedef typename Matrix<T, 4, 4>::Config_s Config_s;  ///< Configuration data set
  
  // --------------------------------------------------------------------------
  /// @brief Default constructor
  /// The default constructor creates an Identity Matrix
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Matrix4x4_Constructor1
  ///
  /// @return       Matrix4x4
  // --------------------------------------------------------------------------
  Projection4D()
     : Matrix<T, 4, 4>()
  {
    this->setEye();
  }

  // --------------------------------------------------------------------------
  /// @brief Constructor with initialization value
  ///
  /// The constructor initializes all cell elements to the input argument \p value.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Matrix4x4_Constructor2
  ///
  /// @param[in] value  Initialization value for all elements in Matrix
  /// @return       Matrix4x4 pre-initialized with specific values
  // --------------------------------------------------------------------------
  Projection4D(T i_Value)
     : Matrix<T, 4, 4>(i_Value)
  {}
  
  // --------------------------------------------------------------------------
  /// @brief Copy constructor
  ///
  /// The constructor copies the contents of \p i_Matrix_rx into this matrix.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Matrix4x4_Constructor3
  ///
  /// @param[in]    i_Matrix_rx Matrix object from which to copy contents
  /// @return       Matrix pre-initialized with specific content
  // --------------------------------------------------------------------------
  Projection4D(const Matrix<T, 4, 4>& i_Matrix_rx)
    : Matrix<T, 4, 4>(i_Matrix_rx)
  {}
  
  // --------------------------------------------------------------------------
  /// @brief Copy constructor with initialization by configuration
  ///
  /// The constructor copies the contents of \p i_Config_rs into cell elements.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Matrix4x4_Constructor4
  ///
  /// @param[in]    i_Config_rs Configuration to be copied into cell elements in Matrix
  /// @return       Matrix pre-initialized with specific configuration
  // --------------------------------------------------------------------------
  Projection4D(const Config_s& i_Config_rs)
     : Matrix<T, 4, 4>(i_Config_rs)
  {}
  
  // --------------------------------------------------------------------------
  /// @brief Create Translation Matrix
  ///
  /// Produces a 4x4 translation by x, y, z
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Matrix4x4_CreateTranslation
  ///
  /// @param[in]    i_X x coordinate of the translation vector
  /// @param[in]    i_Y y coordinate of the translation vector
  /// @param[in]    i_Z z coordinate of the translation vector

  /// @return       a 4x4 Matrix that contains the Translation
  // --------------------------------------------------------------------------
  static Projection4D createTranslation_o(T i_X, T i_Y, T i_Z)
  {
    Projection4D v_M_o;

    v_M_o(3, 0) = i_X;
    v_M_o(3, 1) = i_Y;
    v_M_o(3, 2) = i_Z;
  
    return v_M_o;
  }

  // --------------------------------------------------------------------------
  /// @brief Create uniform Scale Matrix
  ///
  /// Produces a 4x4 uniform scale matrix
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Matrix4x4_CreateScaleUniform
  ///
  /// @param[in]    i_Scale indicates the desired scale factor along each of the three axes.
  ///
  /// @return       a 4x4 Matrix that contains the uniform scale
  // --------------------------------------------------------------------------
  static Projection4D createScale_o(T i_Scale)
  {
    Projection4D v_M_o;
  
    v_M_o(0, 0) = i_Scale;
    v_M_o(1, 1) = i_Scale;
    v_M_o(2, 2) = i_Scale;
  
    return v_M_o;
  }
  
  // --------------------------------------------------------------------------
  /// @brief Create non-uniform Scale Matrix
  ///
  /// Produces a 4x4 non-uniform scale matrix by x, y, z
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Matrix4x4_CreateScale
  ///
  /// @param[in]    i_X scale factor along x axis
  /// @param[in]    i_Y scale factor along y axis
  /// @param[in]    i_Z scale factor along z axis
  ///
  /// @return       a 4x4 Matrix that contains the non-uniform scale
  // --------------------------------------------------------------------------
  static Projection4D createScale_o(T i_X, T i_Y, T i_Z)
  {
    Projection4D v_M_o;
  
    v_M_o(0, 0) = i_X;
    v_M_o(1, 1) = i_Y;
    v_M_o(2, 2) = i_Z;
  
    return v_M_o;
  }
  
  // --------------------------------------------------------------------------
  /// @brief Create Rotation Matrix 
  ///
  /// Produces a 4x4 a rotation of i_Radians around the X axis
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Matrix4x4_CreateRotationX
  ///
  /// @param[in]    i_Radians specifies the angle of rotation in radians
  ///
  /// @return       a 4x4 Matrix that contains the X Rotation
  // --------------------------------------------------------------------------
  static Projection4D createRotationX_o(T i_Radians)
  {
    Projection4D v_M_o;
  
    v_M_o(0,0) = 1; v_M_o(0,1) = 0;              v_M_o(0,2) = 0;               v_M_o(0,3) = 0;
    v_M_o(1,0) = 0; v_M_o(1,1) = cos(i_Radians); v_M_o(1,2) = -sin(i_Radians); v_M_o(1,3) = 0;
    v_M_o(2,0) = 0; v_M_o(2,1) = sin(i_Radians); v_M_o(2,2) =  cos(i_Radians); v_M_o(2,3) = 0;
    v_M_o(3,0) = 0; v_M_o(3,1) = 0;              v_M_o(3,2) = 0;               v_M_o(3,3) = 1;
  
    return v_M_o;
  }
  
  // --------------------------------------------------------------------------
  /// @brief Create Rotation Matrix 
  ///
  /// Produces a 4x4 a rotation of i_Radians around the Y axis
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Matrix4x4_CreateRotationY
  ///
  /// @param[in]    i_Radians specifies the angle of rotation in radians
  ///
  /// @return       a 4x4 Matrix that contains the Y Rotation
  // --------------------------------------------------------------------------
  static Projection4D createRotationY_o(T i_Radians)
  {
    Projection4D out;
  
    out(0,0) = cos( i_Radians);   out(0,1) = 0; out(0,2) = sin( i_Radians); out(0,3) = 0;
    out(1,0) = 0;                 out(1,1) = 1; out(1,2) = 0;               out(1,3) = 0;
    out(2,0) = -sin( i_Radians);  out(2,1) = 0; out(2,2) = cos( i_Radians); out(2,3) = 0;
    out(3,0) = 0;                 out(3,1) = 0; out(3,2) = 0;               out(3,3) = 1;
  
    return out;
  }

  // --------------------------------------------------------------------------
  /// @brief Create Rotation Matrix 
  ///
  /// Produces a 4x4 a rotation of i_Radians around the Z axis
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Matrix4x4_CreateRotationZ
  ///
  /// @param[in]    i_Radians specifies the angle of rotation in radians
  ///
  /// @return       a 4x4 Matrix that contains the Z Rotation
  // --------------------------------------------------------------------------
  static Projection4D createRotationZ_o(T i_Radians)
  {
    Projection4D out;

    out(0,0) = cos(i_Radians);  out(0,1) = -sin(i_Radians);   out(0,2) = 0; out(0,3) = 0;
    out(1,0) = sin(i_Radians);  out(1,1) =  cos(i_Radians);   out(1,2) = 0; out(1,3) = 0;
    out(2,0) = 0;               out(2,1) = 0;                 out(2,2) = 1; out(2,3) = 0;
    out(3,0) = 0;               out(3,1) = 0;                 out(3,2) = 0; out(3,3) = 1;

    return out;
  }

  // --------------------------------------------------------------------------
  /// @brief TODO: implement in Point class
  // --------------------------------------------------------------------------
  static Point3D<T> normalizeP3(const Point3D<T>& point)
  {
    T length = point.norm();

    Point3D<T> out(point);
    if (length > math::constants<T>::zero_x())
    {
      T invLength = static_cast<T>(1) / length;
      out *= invLength;
    }

    return out;
  }

  // --------------------------------------------------------------------------
  /// @brief Create Left Handed View Matrix
  ///
  /// Produces a 4x4 Left Handed view Transformation
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Matrix4x4_CreateViewLH
  ///
  /// @param[in]    i_From_ro specifies the position of the eye point
  /// @param[in]    i_At_ro   specifies the position of the reference point
  /// @param[in]    i_Up_ro   specifies the direction of the up vector
  ///
  /// @return       a 4x4 Matrix that contains the view Transformation
  // --------------------------------------------------------------------------
  static Projection4D createViewLH(const Point3D<T>& i_From_ro, const Point3D<T>& i_At_ro, const Point3D<T>& i_Up_ro)
  {
    Point3D<T> viewDir = normalizeP3((Point3D<T>)(i_At_ro - i_From_ro));
    Point3D<T> right = normalizeP3(i_Up_ro.cross(viewDir));
    Point3D<T> up = viewDir.cross(right);
  
    Projection4D view;

    view(0, 0) = right.getPosX();
    view(1, 0) = right.getPosY();
    view(2, 0) = right.getPosZ();
  
    view(0, 1) = up.getPosX();
    view(1, 1) = up.getPosY();
    view(2, 1) = up.getPosZ();
  
    view(0, 2) = viewDir.getPosX();
    view(1, 2) = viewDir.getPosY();
    view(2, 2) = viewDir.getPosZ();
    
    view(3, 0) = -right.dot(i_From_ro); 
    view(3, 1) = -up.dot(i_From_ro);
    view(3, 2) = -viewDir.dot(i_From_ro);
  
    return view;
  }

  // --------------------------------------------------------------------------
  /// @brief Create Right Handed View Matrix
  ///
  /// Produces a 4x4 Right Handed view Transformation
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Matrix4x4_CreateViewRH
  ///
  /// @param[in]    i_From_ro specifies the position of the eye point
  /// @param[in]    i_At_ro   specifies the position of the reference point
  /// @param[in]    i_Up_ro   specifies the direction of the up vector
  ///
  /// @return       a 4x4 Matrix that contains the view Transformation
  // --------------------------------------------------------------------------
  static Projection4D createViewRH(const Point3D<T>& i_From_ro, const Point3D<T>& i_At_ro, const Point3D<T>& i_Up_ro)
  {
    Point3D<T> viewDir  = normalizeP3((Point3D<T>)(i_At_ro - i_From_ro));
    Point3D<T> right    = normalizeP3(viewDir.cross(i_Up_ro));
    Point3D<T> up       = right.cross(viewDir);

    Projection4D view;

    view(0, 0) = right.getPosX();
    view(1, 0) = right.getPosY();
    view(2, 0) = right.getPosZ();

    view(0, 1) = up.getPosX();
    view(1, 1) = up.getPosY();
    view(2, 1) = up.getPosZ();

    view(0, 2) = -viewDir.getPosX();
    view(1, 2) = -viewDir.getPosY();
    view(2, 2) = -viewDir.getPosZ();

    view(3, 0) = -right.dot(i_From_ro); 
    view(3, 1) = -up.dot(i_From_ro);
    view(3, 2) =  viewDir.dot(i_From_ro);

    return view;
  }

  // --------------------------------------------------------------------------
  /// @brief Creates a Left Handed Perspective Projection Matrix
  ///
  /// Produces a viewing frustum into the world coordinate system.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Matrix4x4_createPerspectiveLH
  ///
  /// @param[in]    i_FovY    specifies the field of view angle(radians) in the y direction
  /// @param[in]    i_Aspect  specifies the aspect ratio that determines the fov in x direction (width / height)
  /// @param[in]    zNear     distance from the viewer to the near clipping plane (always positive)
  /// @param[in]    zFar      distance from the viewer to the far clipping plane  (always positive)
  ///
  /// @return       a 4x4 Matrix that contains the view Transformation
  // --------------------------------------------------------------------------
  static Projection4D createPerspectiveLH(T i_FovY, T i_Aspect, T zNear, T zFar)
  {
    T const v_TanHalfFovY = math::trigonometry<T>::tan_x(i_FovY / static_cast<T>(2));
  
    Projection4D v_Out_o;
    v_Out_o.setZeros();
    
    v_Out_o(0, 0) = math::constants<T>::one_x() / (i_Aspect * v_TanHalfFovY);
		v_Out_o(1, 1) = math::constants<T>::one_x() / (v_TanHalfFovY);
    v_Out_o(2, 2) = (zFar + zNear) / (zFar - zNear);
		v_Out_o(2, 3) = math::constants<T>::one_x();    
		v_Out_o(3, 2) = -(static_cast<T>(2) * zFar * zNear) / (zFar - zNear);

    return v_Out_o;
  }

  // --------------------------------------------------------------------------
  /// @brief Creates a Right Handed Perspective Projection Matrix
  ///
  /// Produces a viewing frustum into the world coordinate system.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Matrix4x4_createPerspectiveRH
  ///
  /// @param[in]    i_FovY    specifies the field of view angle(radians) in the y direction
  /// @param[in]    i_Aspect  specifies the aspect ratio that determines the fov in x direction (width / height)
  /// @param[in]    zNear     distance from the viewer to the near clipping plane (always positive)
  /// @param[in]    zFar      distance from the viewer to the far clipping plane  (always positive)
  ///
  /// @return       a 4x4 Matrix that contains the view Transformation
  // --------------------------------------------------------------------------
  static Projection4D createPerspectiveRH(T i_FovY, T i_Aspect, T zNear, T zFar)
  {
    T const v_TanHalfFovY = math::trigonometry<T>::tan_x(i_FovY / static_cast<T>(2));
  
    Projection4D v_Out_o;
    v_Out_o.setZeros();

    v_Out_o(0, 0) = static_cast<T>(1) / (i_Aspect * v_TanHalfFovY);
    v_Out_o(1, 1) = static_cast<T>(1) / (v_TanHalfFovY);
    v_Out_o(2, 2) = - (zFar + zNear) / (zFar - zNear);
    v_Out_o(2, 3) = - static_cast<T>(1);
    v_Out_o(3, 2) = - (static_cast<T>(2) * zFar * zNear) / (zFar - zNear);

    return v_Out_o;
  }

  // --------------------------------------------------------------------------
  /// @brief Creates an orthographic projection Matrix
  ///
  /// Describes a transformation that produces a parallel projection.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Matrix4x4_createOrtho
  ///
  /// @param[in]    i_Left    specifies the coordinates for the left clipping plane
  /// @param[in]    i_Right   specifies the coordinates for the right clipping plane
  /// @param[in]    i_Bottom  specifies the coordinates for the bottom clipping plane
  /// @param[in]    i_Top     specifies the coordinates for the top clipping plane
  /// @param[in]    zNear     distance from the viewer to the near clipping plane (negative if the plane is to be behind the viewer)
  /// @param[in]    zFar      distance from the viewer to the far clipping plane  (negative if the plane is to be behind the viewer)
  ///
  /// @return       a 4x4 Matrix that contains the view Transformation
  // --------------------------------------------------------------------------
  static Projection4D createOrtho(T i_Left, T i_Right, T i_Bottom, T i_Top, T i_zNear, T i_zFar)
  {
    Projection4D v_Out_o;

    v_Out_o(0, 0) =   static_cast<T>(2)  / (i_Right - i_Left);
    v_Out_o(1, 1) =   static_cast<T>(2)  / (i_Top - i_Bottom);
    v_Out_o(2, 2) = - static_cast<T>(2)  / (i_zFar - i_zNear);
    v_Out_o(3, 0) = - (i_Right + i_Left) / (i_Right - i_Left);
    v_Out_o(3, 1) = - (i_Top + i_Bottom) / (i_Top - i_Bottom);
    v_Out_o(3, 2) = - (i_zFar + i_zNear) / (i_zFar - i_zNear);

    return v_Out_o;
  }

  // --------------------------------------------------------------------------
  /// @brief Creates a 4x4 View Matrix 
  ///
  /// Produces a 4x4  view Transformation from extrinsic parameters (3x3 rotation matrix and translation vector)
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Matrix4x4_extrinsicToView
  ///
  /// @param[in]    i_Rotation      3x3 Matrix that contains the camera rotation
  /// @param[in]    i_Translation   specifies the camera translation vector
  ///
  /// @return       a 4x4 Matrix that contains the view Transformation
  // --------------------------------------------------------------------------
  static Projection4D extrinsicToView(const Matrix3x3<T>& i_Rotation, const Point3D<T>& i_Translation)
  {
    Projection4D out;

    out(0, 0) = i_Rotation(0, 0);
    out(0, 1) = i_Rotation(0, 1);
    out(0, 2) = i_Rotation(0, 2);
    out(0, 3) = 0.0F; 

    out(1, 0) = i_Rotation(1, 0);
    out(1, 1) = i_Rotation(1, 1);  //out(1, 1) = -i_Rotation(1, 1); 
    out(1, 2) = i_Rotation(1, 2);
    out(1, 3) = 0.0F;

    out(2, 0) = i_Rotation(2, 0);
    out(2, 1) = i_Rotation(2, 1);
    out(2, 2) = i_Rotation(2, 2);
    out(2, 3) = 0.0F;

    out(3, 0) = i_Translation.getPosX();
    out(3, 1) = i_Translation.getPosY();
    out(3, 2) = i_Translation.getPosZ();
    out(3, 3) = 1.0F;

    return out;
  }

  // --------------------------------------------------------------------------
  /// @brief Creates a 4x4 Perspective Projection Matrix from Intrinsic parameters
  ///
  /// Produces a viewing frustum into the world coordinate system.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Matrix4x4_intrinsicToProjection
  ///
  /// @param[in]    i_K       The K Matrix that contains the intrinsic parameters
  /// @param[in]    i_X0      principal point x
  /// @param[in]    i_Y0      principal point y
  /// @param[in]    i_Width   specifies the width in image coordinates
  /// @param[in]    i_Height  specifies the height in image coordinates
  /// @param[in]    zNear     distance from the viewer to the near clipping plane 
  /// @param[in]    zFar      distance from the viewer to the far clipping plane  
  /// @return       a 4x4 Matrix that contains the view Transformation
  // --------------------------------------------------------------------------
  static Projection4D intrinsicToProjection(const Matrix3x3<T>& i_K, T i_X0, T i_Y0, T i_Width, T i_Height, T i_ZNear, T i_ZFar)
  {
    T v_Two = static_cast<T>(2);
    T v_Depth = i_ZFar - i_ZNear;
    
    Projection4D out;
    // yDown convention
    out(0, 0) =   v_Two * i_K(0, 0) / i_Width;                              // focal length x 
    out(1, 0) =  -v_Two * i_K(0, 1) / i_Width;                              // shear x
    out(2, 0) = (-v_Two * i_K(0, 2) + i_Width + v_Two * i_X0) / i_Width;    // principal point x
    out(3, 0) = math::constants<T>::zero_x();

    out(1, 0) = math::constants<T>::zero_x();                                // shear y?          
    out(1, 1) =  -v_Two * i_K(1, 1) / i_Height;                              // focal length y    
    out(1, 2) = (-v_Two * i_K(1, 2) + i_Height + v_Two * i_Y0) / i_Height;   // principal point y 
    out(1, 3) = math::constants<T>::zero_x();

    out(2, 0) = math::constants<T>::zero_x();
    out(2, 1) = math::constants<T>::zero_x();
    out(2, 2) = -(i_ZFar + i_ZNear) / v_Depth;
    out(2, 3) = -v_Two * (i_ZFar + i_ZNear) / v_Depth;

    out(3, 0) = math::constants<T>::zero_x();
    out(3, 1) = math::constants<T>::zero_x();
    out(3, 2) = math::constants<T>::minusOne_x();
    out(3, 3) = math::constants<T>::zero_x();

    return out;
  }

};

} // namespace core

} // namespace mecl

#endif // MECL_CORE_MATRIX4X4_H_
