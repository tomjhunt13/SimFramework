#ifndef SIMINTERFACE_OUTPUTBLOCK_H
#define SIMINTERFACE_OUTPUTBLOCK_H

#include <iostream>

#include "Eigen/Dense"
#include "Framework.h"



class OutputBlock : public SimFramework::Sink {

public:
    OutputBlock(SimFramework::Signal* massStates, SimFramework::Signal* force);

    // Block functions
    void Update(float t_np1) override;

private:

};


#endif //SIMINTERFACE_OUTPUTBLOCK_H
