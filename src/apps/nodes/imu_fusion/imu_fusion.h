#ifndef IMUFUSION_H
#define IMUFUSION_H

// These must be defined before including TinyEKF.h
#define Nsta 6     // Four state values: X, Y, Heading, X', Y', W
#define Mobs 3     // Third measurements:
                   //                     Encoders - left velocity
                   //                     Encoders - right velocity
                   //                     IMU Heading

#include "TinyEKF/TinyEKF.h"

//------------------------------------------------------------------------------
/** TODO
*
*   TODO
*
*/

class ImuFusion : public TinyEKF {

    public:
        ImuFusion();

    protected:
        void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]);

public:
        double L;
        double r;
        double theta;
        double dt;
        double u1;
        double u2;

};

#endif
