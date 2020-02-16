#ifndef SIMINTERFACE_SPRINGDAMPER1D_H
#define SIMINTERFACE_SPRINGDAMPER1D_H

#include "Eigen/Dense"
#include "Framework.h"

class SpringDamper1D : public SimFramework::Function {

public:

    SpringDamper1D(SimFramework::Signal* inputConnection1,
                   SimFramework::Signal* inputConnection2,
                   SimFramework::Signal* outputForce);

    // Block functions
    void Update(float t) override;

private:

    // Physical Properties
    float k = 1.f;
    float c = 0.f;

};


#endif //SIMINTERFACE_SPRINGDAMPER1D_H
