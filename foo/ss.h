#ifndef test
#define test
// C++ program to implement
// the above approach
#include <iostream>
#include <stdio.h>
#include <vector>
#include "tscAlg.h"

using namespace std;


class regression
{
    // Dynamic array which is going
    // to contain all (i-th x)
    vector<float> x;

    // Dynamic array which is going
    // to contain all (i-th y)
    vector<float> y;

    // Store the coefficient/slope in
    // the best fitting line
    float coeff;

    // Store the constant term in
    // the best fitting line
    float constTerm;

    // Contains sum of product of
    // all (i-th x) and (i-th y)
    float sum_xy;

    // Contains sum of all (i-th x)
    float sum_x;

    // Contains sum of all (i-th y)
    float sum_y;

    // Contains sum of square of
    // all (i-th x)
    float sum_x_square;

    // Contains sum of square of
    // all (i-th y)
    float sum_y_square;

public:
    // Constructor to provide the default
    // values to all the terms in the
    // object of class regression
    regression()
    {
        coeff = 0;
        constTerm = 0;
        sum_y = 0;
        sum_y_square = 0;
        sum_x_square = 0;
        sum_x = 0;
        sum_xy = 0;
    }

    // Function that calculate the coefficient/
    // slope of the best fitting line
    void calculateCoefficient()
    {
        float N = x.size();
        float numerator
            = ( N * sum_xy - sum_x * sum_y );
        float denominator
            = ( N * sum_x_square - sum_x * sum_x );
        coeff = numerator / denominator;
    }

    // Member function that will calculate
    // the constant term of the best
    // fitting line
    void calculateConstantTerm()
    {
        float N = x.size();
        float numerator
            = ( sum_y * sum_x_square - sum_x * sum_xy );
        float denominator
            = ( N * sum_x_square - sum_x * sum_x );
        constTerm = numerator / denominator;
    }

    // Function that return the number
    // of entries (xi, yi) in the data set
    int sizeOfData()
    {
        return x.size();
    }

    // Function that return the coeffecient/
    // slope of the best fitting line
    float coefficient()
    {
        if( coeff == 0 )
        {
            calculateCoefficient();
        }

        return coeff;
    }

    // Function that return the constant
    // term of the best fitting line
    float constant()
    {
        if( constTerm == 0 )
        {
            calculateConstantTerm();
        }

        return constTerm;
    }

    // Function that print the best
    // fitting line
    float PrintBestFittingLine()
    {
        if( coeff == 0 && constTerm == 0 )
        {
            calculateCoefficient();
            calculateConstantTerm();
        }

        cout << "The best fitting line is y = "
             << coeff << "x + " << constTerm << endl;
        return coeff + constTerm;
    }

    // Function to take input from the dataset
    void takeInput( fc::Pointf* array, int no )
    {
        for( size_t i = 0; i < no; i++ )
        {
            sum_x += array[i].x_x;
            sum_y += array[i].y_x;
            sum_xy += array[i].x_x * array[i].y_x;
            sum_x_square += array[i].x_x * array[i].x_x;
            sum_y_square += array[i].y_x * array[i].y_x;
            x.push_back( array[i].x_x );
            y.push_back( array[i].y_x );
        }
    }
    void reset()
    {
        x.clear();
        y.clear();
        sum_x, sum_y, sum_xy, sum_x_square, sum_y_square = 0;
    }
    // Function to show the data set
    void showData()
    {
        for( int i = 0; i < 62; i++ )
        {
            printf( "_" );
        }

        printf( "\n\n" );
        printf( "|%15s%5s %15s%5s%20s\n",
                "X", "", "Y", "", "|" );

        for( int i = 0; i < x.size(); i++ )
        {
            printf( "|%20f %20f%20s\n",
                    x[i], y[i], "|" );
        }

        for( int i = 0; i < 62; i++ )
        {
            printf( "_" );
        }

        printf( "\n" );
    }

    // Function to predict the value
    // corresponding to some input
    float predict( float x )
    {
        return coeff * x + constTerm;
    }

    // Function that returns overall
    // sum of square of errors
    float errorSquare()
    {
        float ans = 0;

        for( int i = 0;
             i < x.size(); i++ )
        {
            ans += ( ( predict( x[i] ) - y[i] )
                     * ( predict( x[i] ) - y[i] ) );
        }

        return ans;
    }

    // Functions that return the error
    // i.e the difference between the
    // actual value and value predicted
    // by our model
    float errorIn( float num )
    {
        for( int i = 0;
             i < x.size(); i++ )
        {
            if( num == x[i] )
            {
                return ( y[i] - predict( x[i] ) );
            }
        }

        return 0;
    }
};

// Driver code
/*int main()
{
freopen("input.txt", "r",
stdin);
regression reg;

// Number of pairs of (xi, yi)
// in the dataset
int n;
cin >> n;

// Calling function takeInput to
// take input of n pairs
reg.takeInput(n);

// Printing the best fitting line
reg.PrintBestFittingLine();
cout << "Predicted value at 2060 = "
<< reg.predict(2060) << endl;
cout << "The errorSquared = "
<< reg.errorSquare() << endl;
cout << "Error in 2050 = "
<< reg.errorIn(2050) << endl;
}*/

std::vector<std::vector<float>> Eul2Quat( fc::Pointf* yaw, fc::Pointf* pitch, fc::Pointf* roll, int no )
{
    std::vector<std::vector<float>> Quat;
    std::vector<float> w, x, y, z;

    for( size_t i = 0; i < no; i++ )
    {
        float cy = cos( yaw[i].y_x * 0.5 );
        float sy = sin( yaw[i].y_x * 0.5 );
        float cp = cos( pitch[i].y_x * 0.5 );
        float sp = sin( pitch[i].y_x * 0.5 );
        float cr = cos( roll[i].y_x * 0.5 );
        float sr = sin( roll[i].y_x * 0.5 );
        //w.push_back(cr*cp*cy + sr*sp*sy);
        //x.push_back(sr*cp*cy - cr*sp*sy);
        //y.push_back(cr*sp*cy + sr*cp*sy);
        //z.push_back(cr*cp*sy - sr*sp*cy);
        float v_SinHalfX = sin( 0.0174532925 * pitch[i].y_x / 2 );
        float v_CosHalfX = cos( 0.0174532925 * pitch[i].y_x / 2 );
        float v_SinHalfY = sin( 0.0174532925 * yaw[i].y_x / 2 );
        float v_CosHalfY = sin( 0.0174532925 * yaw[i].y_x / 2 );
        float v_SinHalfZ = sin( 0.0174532925 * roll[i].y_x / 2 );
        float v_CosHalfZ = cos( 0.0174532925 * roll[i].y_x / 2 ); //pi/180 = 0.0174532925 deg to rad
        x.push_back( v_CosHalfY * v_CosHalfZ * v_SinHalfX + v_SinHalfY * v_SinHalfZ * v_CosHalfX );
        y.push_back( v_CosHalfY * v_SinHalfZ * v_SinHalfX + v_SinHalfY * v_CosHalfZ * v_CosHalfX );
        z.push_back( v_CosHalfY * v_SinHalfZ * v_CosHalfX - v_SinHalfY * v_CosHalfZ * v_SinHalfX );
        w.push_back( v_CosHalfY * v_CosHalfZ * v_CosHalfX - v_SinHalfY * v_SinHalfZ * v_SinHalfX );
    }

    Quat.push_back( w );
    Quat.push_back( x );
    Quat.push_back( y );
    Quat.push_back( z );
    return Quat;
}

float* Quat2Eul( float* q )
{
    float eul[3] = { 0, 0, 0 };
    double sinr_cosp = 2 * ( q[0] * q[1] + q[2] * q[3] );
    double cosr_cosp = 1 - 2 * ( q[1] * q[1] + q[2] * q[2] );
    eul[2] = std::atan2( sinr_cosp, cosr_cosp );
    // pitch (y-axis rotation)
    double sinp = 2 * ( q[0] * q[2] - q[3] * q[1] );

    if( std::abs( sinp ) >= 1 )
    {
        eul[1] = std::copysign( M_PI / 2, sinp );  // use 90 degrees if out of range
    }
    else
    {
        eul[1] = std::asin( sinp );
    }

    // yaw (z-axis rotation)
    double siny_cosp = 2 * ( q[0] * q[3] + q[1] * q[2] );
    double cosy_cosp = 1 - 2 * ( q[2] * q[2] + q[3] * q[3] );
    eul[0] = std::atan2( siny_cosp, cosy_cosp );
    return eul;
}
#if 0
std::vector<std::vector<float>> Quat = Eul2Quat( v_YawPairs_at, v_PitchPairs_at, v_RollPairs_at, c_IGs_px->size_u32() );
float w = 0, x = 0, y = 0, z = 0;

for( size_t i = 0; i < Quat[0].size(); i++ )
{
    w = w + ( Quat[0][i] ); //180/pi rad to deg
    x = x + ( Quat[1][i] );
    y = y + ( Quat[2][i] );
    z = z + ( Quat[3][i] );
}

w = w / Quat[0].size();
x = x / Quat[0].size();
y = y / Quat[0].size();
z = z / Quat[0].size();
float quat[4] = { w, x, y, z };
float* eul = Quat2Eul( quat );
initialGuess_o.PutYaw( 57.2958 * ( eul[0] ) );
initialGuess_o.PutPitch( 57.2958 * ( eul[1] ) );
initialGuess_o.PutRoll( 57.2958 * ( eul[2] ) );
regression r3;
r3.takeInput( v_ZPairs_at, c_IGs_px->size_u32() );
float zz = r3.PrintBestFittingLine();
initialGuess_o.PutZ( zz );
#endif
#endif // !test

