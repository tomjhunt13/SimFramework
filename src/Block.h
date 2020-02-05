#ifndef SIMINTERFACE_BLOCK_H
#define SIMINTERFACE_BLOCK_H

#include "Signal.h"
#include "TimeManager.h"

class Block {
public:

    Block(TimeManager& manager);


    virtual void Update(float finalTime) = 0;
};


#endif //SIMINTERFACE_BLOCK_H
