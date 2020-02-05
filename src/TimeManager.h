//
// Created by Thomas Hunt on 05/02/2020.
//

#ifndef SIMINTERFACE_TIMEMANAGER_H
#define SIMINTERFACE_TIMEMANAGER_H

#include <vector>

#include "Block.h"

class TimeManager {

private:
    std::vector<Block*> m_Blocks;

public:
    void RegisterBlock(Block* block);

};


#endif //SIMINTERFACE_TIMEMANAGER_H
