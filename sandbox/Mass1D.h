#ifndef SIMINTERFACE_MASS1D_H
#define SIMINTERFACE_MASS1D_H

//#include <vector>

#include "Eigen/Dense"

#include "../src/Block.h"
#include "../src/Signal.h"
#include "../src/DynamicSystem.h"
#include "../src/ForwardEuler.h"



class Mass1D : public SimFramework::Block, public SimFramework::DynamicSystem<Eigen::Vector2f> {

private:

    // Signals
    SimFramework::Signal<float>* inputSpringForce = nullptr;
    SimFramework::Signal<Eigen::Vector2f>* outputStates = nullptr;

    // Physical Properties
    float mass = 1;

    // States;
    float u = 0;
    float t_n = 0;
    Eigen::Vector2f states;


public:

    Mass1D(SimFramework::Signal<float>& inputSpringForce, SimFramework::Signal<Eigen::Vector2f> & outputStates);

    // Block functions
    void Read() override;
    void Write() override;
    void Update(float t) override;

    // Dynamic system functions
    Eigen::Vector2f Gradient(float t, Eigen::Vector2f x) override;

};


#endif //SIMINTERFACE_MASS1D_H
