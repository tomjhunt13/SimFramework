#ifndef SIMINTERFACE_BLOCK_H
#define SIMINTERFACE_BLOCK_H

#include <vector>

#include "SystemManager.h"
#include "Signal.h"


namespace SimInterface {

    class Block {
    public:
        Block();

        virtual void Read() = 0;
        virtual void Write() = 0;
        virtual void Update(float finalTime) = 0;
    };


} // namespace SimInterface



#endif //SIMINTERFACE_BLOCK_H
