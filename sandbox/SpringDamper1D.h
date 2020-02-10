#ifndef SIMINTERFACE_SPRINGDAMPER1D_H
#define SIMINTERFACE_SPRINGDAMPER1D_H

#include <vector>

#include "../src/Block.h"
#include "../src/Signal.h"

class SpringDamper1D : public SimInterface::Block {

private:

    // Signals
    SimInterface::Signal<std::vector<float>>* inputConnection1 = nullptr;
    SimInterface::Signal<std::vector<float>>* inputConnection2 = nullptr;
    SimInterface::Signal<float>* outputForce = nullptr;

    // Physical Properties
    float k = 1;
    float c = 0.25;

    // States
    std::vector<float> connection1;
    std::vector<float> connection2;
    float force;


public:

    SpringDamper1D(SimInterface::Signal<std::vector<float>>& inputConnection1,
                   SimInterface::Signal<std::vector<float>>& inputConnection2,
                   SimInterface::Signal<float>& outputForce);

    // Block functions
    void Read() override;
    void Write() override;
    void Update(float t) override;
};


#endif //SIMINTERFACE_SPRINGDAMPER1D_H
