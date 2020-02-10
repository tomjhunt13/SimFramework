#ifndef SIMINTERFACE_MASS1D_H
#define SIMINTERFACE_MASS1D_H

#include <vector>

#include "../src/Block.h"
#include "../src/Signal.h"
#include "../src/DynamicSystem.h"
#include "../src/ForwardEuler.h"



class Mass1D : public SimInterface::Block, public SimInterface::DynamicSystem<std::vector<float>> {

private:

    // Signals
    SimInterface::Signal<float>* inputSpringForce = nullptr;
    SimInterface::Signal<std::vector<float>>* outputStates = nullptr;

    // Physical Properties
    float mass = 1;

    // States;
    float u = 0;
    float t_n = 0;
    std::vector<float> states = {0, 0};


public:

    Mass1D(SimInterface::Signal<float>& inputSpringForce, SimInterface::Signal<std::vector<float>> & outputStates);

    // Block functions
    void Read() override;
    void Write() override;
    void Update(float t) override;

    // Dynamic system functions
    std::vector<float> Gradient(float t, std::vector<float> x) override;


};


#endif //SIMINTERFACE_MASS1D_H
