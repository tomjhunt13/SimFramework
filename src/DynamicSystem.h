#ifndef SIMINTERFACE_DYNAMICSYSTEM_H
#define SIMINTERFACE_DYNAMICSYSTEM_H

#include "Eigen/Dense"
#include "Block.h"


//class DynamicSystem : public Block {
class DynamicSystem {
public:
//    DynamicSystem(std::vector<Signal*>& inputSignals, Signal* outputSignal) : Block(inputSignals, outputSignal) {};

    virtual Eigen::VectorXf Gradient(float t, Eigen::VectorXf x) = 0;
    virtual void Update(float finalTime) = 0;
};


#endif //SIMINTERFACE_DYNAMICSYSTEM_H
