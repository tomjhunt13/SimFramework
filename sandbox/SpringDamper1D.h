#ifndef SIMINTERFACE_SPRINGDAMPER1D_H
#define SIMINTERFACE_SPRINGDAMPER1D_H

#include "Eigen/Dense"

#include "../src/Block.h"
#include "../src/Signal.h"

class SpringDamper1D : public SimInterface::Block {

private:

    // Signals
    SimInterface::Signal<Eigen::Vector2f>* inputConnection1 = nullptr;
    SimInterface::Signal<Eigen::Vector2f>* inputConnection2 = nullptr;
    SimInterface::Signal<float>* outputForce = nullptr;

    // Physical Properties
    float k = 10.f;
    float c = 15.f;

    // States
    Eigen::Vector2f connection1;
    Eigen::Vector2f connection2;
    float force = 2.f;


public:

    SpringDamper1D(SimInterface::Signal<Eigen::Vector2f>& inputConnection1,
                   SimInterface::Signal<Eigen::Vector2f>& inputConnection2,
                   SimInterface::Signal<float>& outputForce);

    // Block functions
    void Read() override;
    void Write() override;
    void Update(float t) override;
};


#endif //SIMINTERFACE_SPRINGDAMPER1D_H
