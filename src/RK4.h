#ifndef SIMINTERFACE_RK4_H
#define SIMINTERFACE_RK4_H

#include "Eigen/Dense"
#include "SystemInterface.h"




class RK4 {

public:
    static Eigen::VectorXf Step(SystemInterface& system, float t, Eigen::VectorXf& x);

};


#endif //SIMINTERFACE_RK4_H
