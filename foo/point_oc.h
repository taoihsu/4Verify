// ----------------------------------------------------------------------------
// --- Written by Ehsan Parvizi [26-Jul-2013]
// --- Copyright (c) Magna Vectrics (MEVC) 2013
// ----------------------------------------------------------------------------
#ifndef __POINT_H_
#define __POINT_H_
// ----------------------------------------------------------------------------
#include <math.h>
#include "mecl/mecl.h"
// ----------------------------------------------------------------------------

namespace fc
{
    // 2D Point
    template <typename T> class PointTsc2D
    {
    public:
        PointTsc2D(): x_x( 0 ), y_x( 0 )
        {
            // Using initialization lists
        }

        PointTsc2D( const T i_X_x, const T i_Y_x ): x_x( i_X_x ), y_x( i_Y_x )
        {
            // Using initialization lists
        }

        PointTsc2D( const PointTsc2D < T >& i_Pt_rt ): x_x( i_Pt_rt.x_x ), y_x( i_Pt_rt.y_x )
        {
            // Using initialization lists
        }

        PointTsc2D < T >& operator = ( const PointTsc2D < T >& i_Pt_rt )
        {
            x_x = i_Pt_rt.x_x;
            y_x = i_Pt_rt.y_x;
            return  *this;
        }

        float64_t distance( const PointTsc2D < T >& i_Pt_rt )
        {
            float64_t dx = static_cast < float64_t >( x_x - i_Pt_rt.x_x );
            float64_t dy = static_cast < float64_t >( y_x - i_Pt_rt.y_x );
            return /*std::*/sqrt( dx * dx + dy * dy );
        }

        float64_t norm_f64()
        {
            return /*std::*/sqrt( static_cast < float64_t >( x_x * x_x + y_x * y_x ) );
        }

        //PRQA S 2100 2 //TODO. get and set methods not implemented yet
        T x_x;
        T y_x;
    };

    template <typename T> static inline PointTsc2D<T>& operator += ( PointTsc2D<T>& lhs, const PointTsc2D<T>& rhs )
    {
        lhs.x_x = static_cast<T>( lhs.x_x + rhs.x_x );
        lhs.y_x = static_cast<T>( lhs.y_x + rhs.y_x );
        return lhs;
    }

    template <typename T> static inline PointTsc2D<T>& operator -= ( PointTsc2D<T>& lhs, const PointTsc2D<T>& rhs )
    {
        lhs.x_x = static_cast<T>( lhs.x_x - rhs.x_x );
        lhs.y_x = static_cast<T>( lhs.y_x - rhs.y_x );
        return lhs;
    }

    template <typename T> static inline PointTsc2D<T> operator + ( const PointTsc2D<T>& lhs, PointTsc2D<T>& rhs )
    {
        return PointTsc2D<T> ( static_cast<T>( lhs.x_x + rhs.x_x ), static_cast<T>( lhs.y_x + rhs.y_x ) );
    }

    template <typename T> static inline PointTsc2D<T> operator - ( const PointTsc2D<T>& lhs, PointTsc2D<T>& rhs )
    {
        return PointTsc2D<T> ( static_cast<T>( lhs.x_x - rhs.x_x ), static_cast<T>( lhs.y_x - rhs.y_x ) );
    }

    template <typename T> static inline bool operator == ( const PointTsc2D<T>& lhs, const PointTsc2D<T>& rhs )
    {
        return lhs.x_x == rhs.x_x && lhs.y_x == rhs.y_x;
    }

    template <typename T> static inline bool operator != ( const PointTsc2D<T>& lhs, const PointTsc2D<T>& rhs )
    {
        return lhs.x_x != rhs.x_x || lhs.y_x != rhs.y_x;
    }

    // 3D Point

    template <typename T> class PointTsc3D
    {
    public:
        PointTsc3D(): x_x( 0 ), y_x( 0 ), z_x( 0 )
        {
            // Using initialization lists
        }

        PointTsc3D( const T _x, const T _y, const T _z ): x_x( _x ), y_x( _y ), z_x( _z )
        {
            // Using initialization lists
        }

        PointTsc3D( const PointTsc3D < T >& i_Pt_rt ): x_x( i_Pt_rt.x_x ), y_x( i_Pt_rt.y_x ), z_x( i_Pt_rt.z_x )
        {
            // Using initialization lists
        }

        explicit PointTsc3D( const PointTsc2D < T >& i_Pt_rt ): x_x( i_Pt_rt.x_x ), y_x( i_Pt_rt.y_x ), z_x( T() )
        {
            // Using initialization lists
        }

        PointTsc3D < T >& operator = ( const PointTsc3D < T >& i_Pt_rt )
        {
            x_x = i_Pt_rt.x_x;
            y_x = i_Pt_rt.y_x;
            z_x = i_Pt_rt.z_x;
            return  *this;
        }

        float64_t distance( const PointTsc3D < T >& i_Pt_rt )
        {
            float64_t dx = static_cast < float64_t >( x_x - i_Pt_rt.x_x );
            float64_t dy = static_cast < float64_t >( y_x - i_Pt_rt.y_x );
            float64_t dz = static_cast < float64_t >( z_x - i_Pt_rt.z_x );
            return /*std::*/sqrt( dx * dx + dy * dy + dz * dz );
        }

        float64_t norm_f64()
        {
            return /*std::*/sqrt( static_cast < float64_t >( x_x * x_x + y_x * y_x + z_x * z_x ) );
        }

        //PRQA S 2100 3 //TODO. get and set methods not implemented yet
        T x_x;
        T y_x;
        T z_x;
    };

    template <typename T> static inline PointTsc3D<T>& operator += ( PointTsc3D<T>& lhs, const PointTsc3D<T>& rhs )
    {
        lhs.x_x = static_cast<T>( lhs.x_x + rhs.x_x );
        lhs.y_x = static_cast<T>( lhs.y_x + rhs.y_x );
        lhs.z_x = static_cast<T>( lhs.z_x + rhs.z_x );
        return lhs;
    }

    template <typename T> static inline PointTsc3D<T>& operator -= ( PointTsc3D<T>& lhs, const PointTsc3D<T>& rhs )
    {
        lhs.x_x = static_cast<T>( lhs.x_x - rhs.x_x );
        lhs.y_x = static_cast<T>( lhs.y_x - rhs.y_x );
        lhs.z_x = static_cast<T>( lhs.z_x - rhs.z_x );
        return lhs;
    }

    template <typename T> static inline PointTsc3D<T> operator + ( const PointTsc3D<T>& lhs, PointTsc3D<T>& rhs )
    {
        return PointTsc3D<T> ( static_cast<T>( lhs.x_x + rhs.x_x ), static_cast<T>( lhs.y_x + rhs.y_x ), static_cast<T>( lhs.z_x + rhs.z_x ) );
    }

    template <typename T> static inline PointTsc3D<T> operator - ( const PointTsc3D<T>& lhs, PointTsc3D<T>& rhs )
    {
        return PointTsc3D<T> ( static_cast<T>( lhs.x_x - rhs.x_x ), static_cast<T>( lhs.y_x - rhs.y_x ), static_cast<T>( lhs.z_x - rhs.z_x ) );
    }

    template <typename T> static inline bool operator == ( const PointTsc3D<T>& lhs, const PointTsc3D<T>& rhs )
    {
        return lhs.x_x == rhs.x_x && lhs.y_x == rhs.y_x && lhs.z_x == rhs.z_x;
    }

    template <typename T> static inline bool operator != ( const PointTsc3D<T>& lhs, const PointTsc3D<T>& rhs )
    {
        return lhs.x_x != rhs.x_x || lhs.y_x != rhs.y_x || lhs.z_x != rhs.z_x;
    }

    typedef PointTsc2D < float64_t > Pointd;
    typedef PointTsc2D < sint32_t > Point;
    typedef PointTsc3D < float64_t > Point3d;
    typedef PointTsc3D < sint32_t > Point3;
    typedef PointTsc2D < float32_t > Pointf;
}

#endif
