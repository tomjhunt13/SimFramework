#ifndef SIMINTERFACE_DYNAMICSYSTEM_H
#define SIMINTERFACE_DYNAMICSYSTEM_H


#include <vector>
#include <cmath>

#include "Eigen/Dense"

#include "Block.h"
#include "TimeManager.h"


class DynamicSystem : public Block {
public:

    DynamicSystem(TimeManager& manager) : Block(manager) {};

    virtual Eigen::VectorXf Gradient(float t, Eigen::VectorXf x) = 0;


    static std::vector<float> TimeSteps(float tMin, float tMax, float dt);
};


#endif //SIMINTERFACE_DYNAMICSYSTEM_H
