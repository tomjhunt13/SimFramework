#ifndef SIMINTERFACE_MASS1D_H
#define SIMINTERFACE_MASS1D_H

#include "Eigen/Dense"
#include "../src/Framework.h"



class Mass1D : public SimFramework::DynamicSystem {

public:

    Mass1D(SimFramework::Signal* inputSpringForce, SimFramework::Signal* outputStates);

    // Block functions
    void Update(float t) override;

    // Dynamic system functions
    Eigen::VectorXf Gradient(float t, Eigen::VectorXf x) override;

private:
    // Physical Properties
    float mass = 1;
};


#endif //SIMINTERFACE_MASS1D_H
