#ifndef SIMINTERFACE_RK4_H
#define SIMINTERFACE_RK4_H

#include "Eigen/Dense"
#include "DynamicSystem.h"




class RK4 {

public:
    static Eigen::VectorXf Step(DynamicSystem& system, float dt, float t, Eigen::VectorXf& x);
};


#endif //SIMINTERFACE_RK4_H
