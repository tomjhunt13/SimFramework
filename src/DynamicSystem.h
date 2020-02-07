#ifndef SIMINTERFACE_DYNAMICSYSTEM_H
#define SIMINTERFACE_DYNAMICSYSTEM_H


#include <vector>
#include <cmath>

#include "Eigen/Dense"

#include "Block.h"

namespace SimInterface {

    class DynamicSystem : public Block {
    public:
        virtual Eigen::VectorXf Gradient(float t, Eigen::VectorXf x) = 0;

    };

}

#endif //SIMINTERFACE_DYNAMICSYSTEM_H
