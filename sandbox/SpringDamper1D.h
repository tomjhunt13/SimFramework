#ifndef SIMINTERFACE_SPRINGDAMPER1D_H
#define SIMINTERFACE_SPRINGDAMPER1D_H

#include "Eigen/Dense"

#include "../src/Block.h"
#include "../src/Signal.h"

class SpringDamper1D : public SimFramework::Block {

private:

    // Signals
    SimFramework::Signal<Eigen::Vector2f>* inputConnection1 = nullptr;
    SimFramework::Signal<Eigen::Vector2f>* inputConnection2 = nullptr;
    SimFramework::Signal<float>* outputForce = nullptr;

    // Physical Properties
    float k = 1.f;
    float c = 0.f;

    // States
    Eigen::Vector2f connection1;
    Eigen::Vector2f connection2;
    float force = 0.f;


public:

    SpringDamper1D(SimFramework::Signal<Eigen::Vector2f>& inputConnection1,
                   SimFramework::Signal<Eigen::Vector2f>& inputConnection2,
                   SimFramework::Signal<float>& outputForce);

    // Block functions
    void Read() override;
    void Write() override;
    void Update(float t) override;
};


#endif //SIMINTERFACE_SPRINGDAMPER1D_H
