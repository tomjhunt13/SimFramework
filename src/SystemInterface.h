#ifndef SIMINTERFACE_SYSTEMINTERFACE_H
#define SIMINTERFACE_SYSTEMINTERFACE_H

#include "Eigen/Dense"


class SystemInterface {

public:
    virtual Eigen::VectorXf Gradient(float t, Eigen::VectorXf x) = 0;
};


#endif //SIMINTERFACE_SYSTEMINTERFACE_H
