/* @file fusion.h
* This file contains the implementation of a Kalman filter for the location system
*/
//------------------------------------------------------------------------------

#include "imu_fusion.h"
#include <math.h>

//------------------------------------------------------------------------------

ImuFusion::ImuFusion()
{
    //

    // Process noise covariance (assume states are uncorrelated)
    // We approximate the process noise using a small constant 0.0001
    this->setQ(0, 0, .0001);
    this->setQ(1, 1, .0001);
    this->setQ(2, 2, .0001);
    this->setQ(3, 3, .0001);
    this->setQ(4, 4, .0001);
    this->setQ(5, 5, .0001);

    // Same for measurement noise
    this->setR(0, 0, .0001);
    this->setR(1, 1, .0001);
    this->setR(2, 2, .0001);

}

void ImuFusion::model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta])
{
    // process model is f(x)
    fx[0] = this->x[0] + this->x[3]*dt;
    fx[1] = this->x[1] + this->x[4]*dt;
    fx[2] = this->x[2] + this->x[5]*dt;
    fx[3] = u1*(-1/2)*cos(theta) + u2*(1/2)*cos(theta);
    fx[4] = u1*(-1/2)*sin(theta) + u2*(1/2)*sin(theta);
    fx[5] = (u1 - u2)/(2*L) ;

    // the process model Jacobian
    F[0][0] = 1;
    F[0][3] = dt;
    F[1][1] = 1;
    F[1][4] = dt;
    F[2][2] = 1;
    F[2][5] = dt;

    // Measurement function
    hx[0] = this->x[3]*cos(theta)/r + this->x[4]*sin(theta)/r + this->x[5]*(L/r);  // left wheel angular velocity from current state
    hx[1] = this->x[3]*(-cos(theta))/r + this->x[4]*(-sin(theta))/r + this->x[5]*(L/r); // right wheel angular velocity from current state
    hx[2] = this->x[2];  // car(gyro) angle of the current state

    // Jacobian of measurement function
    H[0][3] = cos(theta)/r;
    H[0][4] = sin(theta)/r ;
    H[0][5] = L/r ;
    H[1][3] = -cos(theta)/r;
    H[1][4] = -sin(theta)/r ;
    H[1][5] = L/r ;
    H[2][2] = 1;

    theta = this->x[2];

}
