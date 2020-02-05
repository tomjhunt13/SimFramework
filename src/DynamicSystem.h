#ifndef SIMINTERFACE_DYNAMICSYSTEM_H
#define SIMINTERFACE_DYNAMICSYSTEM_H

#include "Eigen/Dense"
#include "Block.h"


class DynamicSystem : public Block {
public:
    DynamicSystem(std::vector<Signal*>& inputSignals, Signal* outputSignal) : Block(inputSignals, outputSignal) {};

    virtual Eigen::VectorXf Gradient(float t, Eigen::VectorXf x);
    virtual void Update(float finalTime);
};


#endif //SIMINTERFACE_DYNAMICSYSTEM_H
