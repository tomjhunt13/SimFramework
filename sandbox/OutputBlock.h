#ifndef SIMINTERFACE_OUTPUTBLOCK_H
#define SIMINTERFACE_OUTPUTBLOCK_H

#include <iostream>

#include "Eigen/Dense"

#include "../src/Block.h"
#include "../src/Signal.h"

class OutputBlock : public SimInterface::Block {

private:

    SimInterface::Signal<Eigen::Vector2f>* massStates;
    SimInterface::Signal<float>* force;

    Eigen::Vector2f states;
    float forceVal;

public:
    OutputBlock(SimInterface::Signal<Eigen::Vector2f>& massStates, SimInterface::Signal<float>& force);

    // Block functions
    void Read() override;
    void Update(float t) override;
    void Write() override {};

};


#endif //SIMINTERFACE_OUTPUTBLOCK_H
