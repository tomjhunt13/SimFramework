#ifndef SIMINTERFACE_BLOCK_H
#define SIMINTERFACE_BLOCK_H

#include <vector>
#include "Signal.h"

class BlockManager;

class Block {
public:

    Block(BlockManager& manager);


    virtual void Update(float finalTime) = 0;
};


class BlockManager {

private:
    std::vector<Block*> m_Blocks;

public:
    void RegisterBlock(Block* block);

};

#endif //SIMINTERFACE_BLOCK_H
