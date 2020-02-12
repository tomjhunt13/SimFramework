#ifndef SIMINTERFACE_BLOCK_H
#define SIMINTERFACE_BLOCK_H

#include <vector>

#include "SystemManager.h"
#include "Signal.h"


namespace SimFramework {

    class Block {
    public:

        // TODO: should enforce that initial states are written to outputs
        Block();

        virtual void Read() = 0;
        virtual void Write() = 0;
        virtual void Update(float finalTime) = 0;


    };


} // namespace SimFramework



#endif //SIMINTERFACE_BLOCK_H
